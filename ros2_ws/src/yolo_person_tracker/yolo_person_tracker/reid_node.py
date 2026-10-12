"""Independent ReID observer. No target lock, height-prior transfer or motion output."""
import copy
from concurrent.futures import ThreadPoolExecutor
import math
import time
import message_filters
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from person_interfaces.msg import PersonStateArray, PersonIdentity, PersonIdentityArray
from .input_contract import stamp_seconds
from .reid import IdentityConfig, IdentityManager
from .reid_backend import OSNetBackend, appearance_crop


class ReIDNode(Node):
    def __init__(self, backend=None, **kwargs):
        super().__init__('reid_observer', **kwargs)
        defaults=dict(enabled=False, model_path='', color_topic='/camera/color/image_rect',
                      expected_frame='camera_color_optical_frame', max_age_s=.5,
                      sample_hz=5., max_people=8, min_crop_height=80, min_crop_width=32,
                      blur_min=20.)
        defaults.update(vars(IdentityConfig()))
        for name, value in defaults.items():
            self.declare_parameter(name,value)
        self.cfg={k:self.get_parameter(k).value for k in defaults}
        c=self.cfg
        self.manager=IdentityManager(IdentityConfig(**{k:c[k] for k in vars(IdentityConfig())}))
        if not isinstance(c['enabled'],bool):
            raise ValueError('enabled must be boolean')
        for name in ('max_age_s','sample_hz','blur_min'):
            if not math.isfinite(c[name]) or c[name]<=0:
                raise ValueError(f'{name} must be positive finite')
        for name in ('max_people','min_crop_height','min_crop_width'):
            if type(c[name]) is not int or c[name]<1:
                raise ValueError(f'{name} must be a positive integer')
        self.bridge=CvBridge();self.backend=backend;self.error=''
        self.pool=ThreadPoolExecutor(max_workers=1,thread_name_prefix='reid')
        self.loading=None;self.future=None;self.generation=0
        self.last_submit=-math.inf;self.last_stamp=None;self.last_invalid=-math.inf
        self.was_valid=False
        self.pub=self.create_publisher(PersonIdentityArray,'person_identities',10)
        if c['enabled']:
            self.pose_sub=message_filters.Subscriber(self,PersonStateArray,'person_states')
            self.color_sub=message_filters.Subscriber(self,Image,c['color_topic'],qos_profile=qos_profile_sensor_data)
            self.sync=message_filters.TimeSynchronizer([self.pose_sub,self.color_sub],queue_size=20)
            self.sync.registerCallback(self.on_pair)
            self.create_subscription(PersonStateArray,'person_states',self.on_pose_gate,10)
        self.create_timer(.05,self.tick)
        if c['enabled'] and self.backend is None:
            self.loading=self.pool.submit(OSNetBackend,c['model_path'])

    def invalid(self,detail):
        msg=PersonIdentityArray(enabled=self.cfg['enabled'],valid=False,detail=detail)
        msg.processing_ms=msg.observation_age_s=math.nan
        self.pub.publish(msg)
        self.was_valid=False;self.last_invalid=time.monotonic()

    def on_pose_gate(self,msg):
        if not msg.valid or msg.is_simulated:
            self.generation+=1;self.manager.suspend()
            self.invalid('Invalid or simulated pose input')

    def on_pair(self,persons,color):
        if not self.cfg['enabled'] or self.backend is None or self.error:
            return
        stamp=stamp_seconds(color.header.stamp)
        age=self.get_clock().now().nanoseconds*1e-9-stamp
        if (not persons.valid or persons.is_simulated or stamp<=0 or not 0<=age<=self.cfg['max_age_s']
                or color.header!=persons.header or color.header.frame_id!=self.cfg['expected_frame']
                or len(persons.persons)>self.cfg['max_people']):
            self.generation+=1;self.manager.suspend();self.invalid('Input time/frame/count/validity rejected')
            return
        if self.future is not None:
            return  # One worker, no stale inference backlog.
        if time.monotonic()-self.last_submit<1./self.cfg['sample_hz']:
            return
        self.last_submit=time.monotonic()
        self.future=self.pool.submit(self.process,persons,color,self.generation)

    def process(self,persons,color,generation):
        start=time.monotonic()
        image=self.bridge.imgmsg_to_cv2(color,'bgr8')
        tracks=[p.track_id for p in persons.persons]
        if any(not t for t in tracks) or len(set(tracks))!=len(tracks):
            raise ValueError('Missing or duplicate track identity')
        observations={t:None for t in tracks};reasons={};crops=[];ids=[]
        for person in persons.persons:
            crop,reason=appearance_crop(image,person,persons.persons,
                self.cfg['min_crop_height'],self.cfg['min_crop_width'],self.cfg['blur_min'])
            reasons[person.track_id]=reason
            if crop is not None:
                crops.append(crop);ids.append(person.track_id)
        if crops:
            features=self.backend.extract(crops)
            if len(features)!=len(ids):
                raise ValueError('Descriptor count differs from tracked crops')
            observations.update(zip(ids,features))
        return persons.header,observations,reasons,(time.monotonic()-start)*1000.,generation

    def tick(self):
        if self.loading is not None and self.loading.done():
            try:
                self.backend=self.loading.result()
            except Exception as exc:
                self.error=f'ReID model unavailable: {type(exc).__name__}: {exc}'
                self.get_logger().error(self.error)
            self.loading=None
        if self.future is not None and self.future.done():
            try:
                header,observations,reasons,elapsed,generation=self.future.result()
                stamp=stamp_seconds(header.stamp)
                age=self.get_clock().now().nanoseconds*1e-9-stamp
                if generation==self.generation and 0<=age<=self.cfg['max_age_s']:
                    results=self.manager.update(stamp,observations)
                    msg=PersonIdentityArray(header=copy.deepcopy(header),enabled=True,valid=True,
                        detail='Appearance hypotheses only; no target lock or prior transfer',
                        processing_ms=float(elapsed),observation_age_s=float(age))
                    verified=set()
                    for result in results:
                        ok=bool(result.person_id)
                        if ok:verified.add(result.person_id)
                        msg.persons.append(PersonIdentity(track_id=result.track_id,person_id=result.person_id,
                            verified=ok,visible=True,state=result.state,appearance_distance=float(result.distance),
                            last_seen_age_s=0. if ok else math.nan,
                            detail=result.detail+'; '+reasons[result.track_id]))
                    for identity,gallery in self.manager.galleries.items():
                        if identity not in verified:
                            msg.persons.append(PersonIdentity(person_id=identity,state='LOST',
                                appearance_distance=math.nan,last_seen_age_s=float(stamp-gallery.last_seen),
                                detail='Retained identity only; no current verified track or position'))
                    self.pub.publish(msg);self.was_valid=True;self.last_stamp=stamp
                else:
                    self.manager.suspend();self.invalid('Expired or invalidated inference result')
            except Exception as exc:
                self.manager.suspend()
                self.invalid(f'ReID inference rejected: {type(exc).__name__}: {exc}')
            self.future=None
        fresh=(self.last_stamp is not None and 0<=self.get_clock().now().nanoseconds*1e-9-self.last_stamp<=self.cfg['max_age_s'])
        if not fresh:
            if self.was_valid:
                self.generation+=1;self.manager.suspend()
                self.invalid('Stale appearance observations')
            if time.monotonic()-self.last_invalid>=.5:
                self.invalid(self.error or ('ReID disabled' if not self.cfg['enabled'] else 'Waiting for fresh matching RGB/Pose'))

    def destroy_node(self):
        self.pool.shutdown(wait=True,cancel_futures=True)
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args);node=ReIDNode()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
