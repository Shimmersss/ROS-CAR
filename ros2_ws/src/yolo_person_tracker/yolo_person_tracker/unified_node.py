"""Independent unified ground-contact output; original RGB-D and mono interfaces remain."""
import copy
import math
import time
import rclpy
from person_interfaces.msg import FloorPlane, PersonGround, PersonGroundArray
from visualization_msgs.msg import MarkerArray
from .ground_node import GroundLocalizer
from .input_contract import stamp_seconds
from .unified import UnifiedConfig, UnifiedTrack, candidates, mount_std
from .body_contact import BodyContactConfig, BodyContact, body_anchor


class UnifiedLocalizer(GroundLocalizer):
    NODE_NAME = 'unified_localizer'
    PERSON_TOPIC = 'person_positions'
    MARKER_TOPIC = 'position_markers'
    TARGET_TOPIC = 'target_state_unified'
    SOURCE = 'yolo_unified'
    DESCRIPTION = 'Bilateral ankle midpoint on ground; mapped body/depth/mono fusion, initial uncertainty model'

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        values={}
        for key,value in vars(UnifiedConfig()).items():
            self.declare_parameter('unified_'+key,value)
            values[key]=self.get_parameter('unified_'+key).value
        self.unified_config=UnifiedConfig(**values)
        self.tracks={}
        body_values={}
        for key,value in vars(BodyContactConfig()).items():
            self.declare_parameter('body_contact_'+key,value)
            body_values[key]=self.get_parameter('body_contact_'+key).value
        self.body_config=BodyContactConfig(**body_values)
        self.body_maps={}
        # Optional: this frame's stable depth-fitted floor replaces the calibrated plane for ankles.
        self.declare_parameter('floor_plane_enabled',False)
        self.floor_enabled=self.get_parameter('floor_plane_enabled').value
        self.floors={}
        if self.floor_enabled:
            self.create_subscription(FloorPlane,'floor_plane',self.on_floor,10)

    def on_floor(self,msg):
        if (not msg.valid or not msg.stable or msg.header.frame_id!=self.cfg['expected_source_frame']
                or not all(math.isfinite(v) for v in (msg.up.x,msg.up.y,msg.up.z,msg.height_m,msg.rms_m))):
            return
        key=(msg.header.stamp.sec,msg.header.stamp.nanosec)
        self.floors[key]=((msg.up.x,msg.up.y,msg.up.z),float(msg.height_m),float(msg.rms_m))
        while len(self.floors)>30:self.floors.pop(next(iter(self.floors)))

    def on_persons(self,msg):
        stamp=stamp_seconds(msg.header.stamp)
        reason,_=self.gate(msg,self.get_clock().now().nanoseconds*1e-9)
        ids=[p.track_id for p in msg.persons]
        if len(ids)>128 or len(ids)!=len(set(ids)) or any(not i for i in ids):
            msg=copy.deepcopy(msg);msg.valid=False;msg.detail='Invalid or excessive track identities'
            reason=msg.detail
        if reason:
            self.tracks.clear()
        else:
            self.tracks={k:v for k,v in self.tracks.items() if k in ids and
                         v.previous_stamp is not None and 0 < stamp-v.previous_stamp <= self.unified_config.max_gap_s}
        self.body_maps={k:v for k,v in self.body_maps.items() if k in self.tracks}
        self.obs_stamps={k:v for k,v in self.obs_stamps.items() if k in self.tracks}
        super().on_persons(msg)

    def locate(self,person,stamp,stamp_msg,transform):
        result=PersonGround(track_id=person.track_id)
        result.position.x=result.position.y=result.position.z=math.nan
        result.horizontal_distance_m=result.bearing_rad=math.nan
        result.std_m=result.confidence=result.measurement_age_s=math.nan
        track=self.tracks.setdefault(person.track_id,UnifiedTrack(self.unified_config))
        try:
            # Exact same-frame stamp only; a floor from another frame is never substituted.
            floor=(self.floors.get((stamp_msg.sec,stamp_msg.nanosec)) if self.floor_enabled else None)
            depth,mono,hard,detail=candidates(person,self.model,transform,self.config,self.unified_config,floor)
            anchor=body_anchor(person,self.model,transform,self.config,self.unified_config,self.body_config)
            mapping=self.body_maps.setdefault(person.track_id,BodyContact(self.body_config))
            foot=depth if depth is not None and depth.method=='depth_ankles' else mono
            # Do not calibrate against disagreeing sources.
            source_conflict=(depth is not None and mono is not None and
                math.dist(depth.point[:2],mono.point[:2])>self.unified_config.source_agreement_m)
            noise=(math.hypot(self.unified_config.body_projection_std_m,mount_std(anchor,self.unified_config))
                   if anchor is not None else math.inf)
            body,mapping_reason=mapping.observe(stamp,anchor,foot,noise,self.config.ground_z_m,
                self.config.max_std_m,hard or source_conflict)
            estimate,reason=track.update(stamp,depth,mono,hard,body,self.config.max_std_m)
            detail+='; '+mapping_reason
            if body is not None:detail+='; '+body.detail
        except (ValueError,TypeError,IndexError) as exc:
            estimate,reason=None,str(exc);detail='Invalid geometry'
            self.tracks.pop(person.track_id,None);self.body_maps.pop(person.track_id,None)
        result.detail=f'{reason}; {detail}'
        if estimate:
            result.valid=True;result.method=estimate.method
            result.std_m=estimate.std_m;result.confidence=estimate.confidence
            result.measurement_age_s=0.
            self.obs_stamps[person.track_id]=copy.deepcopy(stamp_msg)
            self.fill_position(result,estimate.point)
        else:
            self.obs_stamps.pop(person.track_id,None)
        return result

    def tick(self):
        if self.latest is not None and self.latest.valid and (
                time.monotonic()-self.latest_at>self.cfg['max_age_s'] or
                not 0 <= self.get_clock().now().nanoseconds*1e-9-stamp_seconds(self.latest.header.stamp) <= self.cfg['max_age_s']):
            out=PersonGroundArray();out.header.frame_id=self.cfg['target_frame']
            out.detail='Expired unified observations'
            self.tracks.clear();self.body_maps.clear();self.obs_stamps.clear()
            self.finish(out,MarkerArray(),set())
        return super().tick()


def main(args=None):
    rclpy.init(args=args);node=UnifiedLocalizer()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
