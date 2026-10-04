"""C route extends B outputs without changing target selection or motion interfaces."""
import copy
import math
import time
from collections import deque
import cv2
import numpy as np
import rclpy
from rclpy.parameter import Parameter
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
from person_interfaces.msg import PersonState, PersonStateArray, RuntimeMetrics, PersonIdentityArray
from .node import TrackerNode, serialized
from .pose import PoseConfig, EDGES, LABELS, points2d, joints3d
from .input_contract import stamp_seconds
from .fusion import FusionConfig, fuse_depth
from .identity_selection import IdentitySelection
from .pose3d import Pose3DConfig, EnhancedPostureTracker


class PoseTrackerNode(TrackerNode):
    def __init__(self, **kwargs):
        overrides = list(kwargs.pop('parameter_overrides', []))
        overrides = [p for p in overrides if p.name != 'model_task']
        overrides.append(Parameter('model_task', value='pose'))
        super().__init__(parameter_overrides=overrides, **kwargs)
        self.declare_parameter('reid_lock_enabled', False)
        self.declare_parameter('reid_lock_confirm_frames', 3)
        self.declare_parameter('reid_lock_max_age_s', .5)
        self.reid_lock_enabled = self.get_parameter('reid_lock_enabled').value
        if self.reid_lock_enabled:
            self.selection = IdentitySelection(
                self.get_parameter('reid_lock_confirm_frames').value,
                self.get_parameter('reid_lock_max_age_s').value)
            self.create_subscription(PersonIdentityArray, 'person_identities',
                                     self.on_identities, 10, callback_group=self.state_group)
        values = {}
        for name, value in vars(PoseConfig()).items():
            self.declare_parameter('pose_'+name, value)
            values[name] = self.get_parameter('pose_'+name).value
        self.pose_config = PoseConfig(**values)
        fusion_values = {}
        for name, value in vars(FusionConfig()).items():
            self.declare_parameter('fusion_'+name, value)
            fusion_values[name] = self.get_parameter('fusion_'+name).value
        self.fusion_config = FusionConfig(**fusion_values)
        self.declare_parameter('performance_enabled', True)
        self.performance_enabled=self.get_parameter('performance_enabled').value
        spatial_values={}
        for name,value in vars(Pose3DConfig()).items():
            self.declare_parameter('pose3d_'+name,value)
            spatial_values[name]=self.get_parameter('pose3d_'+name).value
        self.postures = EnhancedPostureTracker(self.pose_config,Pose3DConfig(**spatial_values))
        self.person_pub = self.create_publisher(PersonStateArray, 'person_states', 10)
        self.skeleton_pub = self.create_publisher(MarkerArray, 'skeleton_markers', 10)
        self.metrics_pub = self.create_publisher(RuntimeMetrics, 'performance', 10)
        self.samples = deque(maxlen=120)
        self.output_samples = deque(maxlen=120)
        self.pose_epoch = None
        self.active_markers = set()
        self.pose_valid = False
        self.pose_frame = ''
        self.last_invalid_at = -math.inf
        self.previous_joints = {}
        if self.performance_enabled:
            self.create_timer(1., self.publish_metrics, callback_group=self.state_group)

    def measure_frame(self, metres, detections, intrinsics):
        if not self.fusion_config.enabled:
            return super().measure_frame(metres, detections, intrinsics)
        observations = {
            d.track_id: fuse_depth(
                metres, d.box, d.keypoints, intrinsics,
                confidence=self.pose_config.confidence,
                min_fraction=self.cfg['depth_min_fraction'],
                other_boxes=[other.box for index, other in enumerate(detections) if index != i],
                config=self.fusion_config)
            for i, d in enumerate(detections) if d.track_id is not None
        }
        return ({identity: result.target for identity, result in observations.items()}, observations)

    def process_pair(self, *args):
        start=time.monotonic()
        result=super().process_pair(*args)
        if self.performance_enabled:
            self.samples.append((time.monotonic(), (time.monotonic()-start)*1000.))
        return result

    @serialized
    def on_identities(self, msg):
        self.selection.observe(
            msg.header.stamp.sec*1000000000+msg.header.stamp.nanosec,
            msg.header.frame_id,
            [(p.track_id, p.person_id, p.verified, p.visible) for p in msg.persons],
            self.get_clock().now().nanoseconds,
            valid=msg.enabled and msg.valid and not self.error
                  and self.snapshot is not None and self.fresh_snapshot())

    @serialized
    def tick(self):
        if self.reid_lock_enabled and (self.error or self.snapshot is None or not self.fresh_snapshot()):
            self.selection.invalidate()
        super().tick()
        if self.error or self.snapshot is None or not self.fresh_snapshot():
            self.postures.reset()
            self.previous_joints.clear()
            now=time.monotonic()
            if self.pose_valid or self.active_markers or now-self.last_invalid_at >= 1.:
                msg=PersonStateArray()
                msg.header.frame_id=self.pose_frame
                msg.detail=self.error or 'Stale RGB-D inference'
                self.person_pub.publish(msg)
                self.delete_markers(set(), self.get_clock().now().to_msg(), self.pose_frame)
                self.last_invalid_at=now
            self.pose_valid=False

    def delete_markers(self, current, stamp, frame):
        markers=MarkerArray()
        for identity in self.active_markers-current:
            marker=Marker(); marker.header.stamp=stamp; marker.header.frame_id=frame
            marker.ns=identity; marker.id=0; marker.action=Marker.DELETE
            markers.markers.append(marker)
        if markers.markers: self.skeleton_pub.publish(markers)
        self.active_markers=current

    def publish_image(self, color, image, detections):
        if self.reid_lock_enabled:
            self.selection.record_frame(
                color.header.stamp.sec*1000000000+color.header.stamp.nanosec,
                color.header.frame_id)
        if self.pose_epoch != self.selection.epoch:
            self.postures.reset(); self.previous_joints.clear(); self.pose_epoch=self.selection.epoch
        msg=PersonStateArray(); msg.header=copy.deepcopy(color.header); msg.valid=True
        self.pose_frame=color.header.frame_id
        msg.detail='COCO17; camera-relative heuristic; initial thresholds'
        annotated=image.copy(); markers=MarkerArray(); current=set()
        stamp=stamp_seconds(color.header.stamp)
        next_joints={}
        for detection in detections:
            if detection.track_id is None:
                continue
            identity=f'{self.selection.epoch}:{detection.track_id}'
            person=PersonState(); person.track_id=identity
            person.box=list(map(float,detection.box)); person.confidence=float(detection.confidence)
            points, valid=points2d(detection.keypoints, self.pose_config.confidence)
            person.keypoints_2d=[Point(x=float(p[0]), y=float(p[1]), z=0.) if ok
                                 else Point(x=math.nan,y=math.nan,z=math.nan)
                                 for p,ok in zip(points,valid)]
            person.keypoint_confidences=[float(p[2]) if np.isfinite(p[2]) else 0. for p in points]
            xyz=np.full((17,3),np.nan)
            observation = self.snapshot[6].get(detection.track_id)
            person.body_depth_m=math.nan
            if self.cfg['depth_registered']:
                body=(observation.target if observation is not None else None)
                source=observation.source if observation is not None else ''
                if not self.fusion_config.enabled and detection.track_id not in self.held_position_ids:
                    body=self.positions.get(detection.track_id);source='legacy_regions_filtered'
                if body is not None and math.isfinite(body[2]):
                    person.body_depth_valid=True;person.body_depth_m=float(body[2]);person.body_depth_source=source
            if self.cfg['depth_registered'] and self.fusion_config.enabled:
                if observation is not None:
                    xyz = observation.joints.copy()
            elif self.cfg['depth_registered']:
                # Never use the predicted/held target position as fresh joint evidence.
                reference=self.positions.get(detection.track_id) if detection.track_id not in self.held_position_ids else None
                xyz=joints3d(self.snapshot[1],detection.keypoints,self.snapshot[3],self.pose_config.confidence,reference)
            previous=self.previous_joints.get(identity)
            next_joints[identity]=(stamp,xyz.copy())
            if previous and 0 < stamp-previous[0] <= self.pose_config.max_gap_s:
                jump=np.linalg.norm(xyz-previous[1],axis=1)>self.pose_config.joint_jump_m
                xyz[jump]=np.nan
            person.keypoints_3d=[Point(x=float(p[0]),y=float(p[1]),z=float(p[2])) for p in xyz]
            person.keypoints_3d_valid=list(map(bool,np.isfinite(xyz).all(axis=1)))
            person.posture,person.fall_stage,person.detail=self.postures.update(identity,stamp,detection.box,detection.keypoints,xyz)
            if self.fusion_config.enabled:
                reason = (f'depth={observation.source}; {observation.reason}' if observation is not None
                          else 'depth=invalid; registration unconfirmed')
                person.detail += '; ' + reason
            msg.persons.append(person)
            tint=(0,0,255) if person.fall_stage else (255,255,0)
            for a,b in EDGES:
                if valid[a] and valid[b]:
                    cv2.line(annotated,tuple(map(int,points[a,:2])),tuple(map(int,points[b,:2])),tint,2)
            x,y=map(int,detection.box[:2])
            cv2.putText(annotated,f'{LABELS[person.posture]} fall={person.fall_stage} {person.detail.split(chr(59),1)[0]}',(x,max(30,y+18)),cv2.FONT_HERSHEY_SIMPLEX,.5,tint,1)
            marker=Marker(); marker.header=copy.deepcopy(color.header)
            marker.ns=identity; marker.id=0; marker.type=Marker.LINE_LIST; marker.action=Marker.ADD
            marker.pose.orientation.w=1.; marker.scale.x=.025
            marker.color.r=1. if person.fall_stage else 0.; marker.color.g=1.; marker.color.a=1.
            marker.lifetime.nanosec=500000000
            for a,b in EDGES:
                if person.keypoints_3d_valid[a] and person.keypoints_3d_valid[b]:
                    marker.points.extend([person.keypoints_3d[a],person.keypoints_3d[b]])
            if marker.points: markers.markers.append(marker); current.add(identity)
        self.person_pub.publish(msg); self.pose_valid=True
        self.previous_joints=next_joints
        self.delete_markers(current,color.header.stamp,color.header.frame_id)
        if markers.markers: self.skeleton_pub.publish(markers)
        age=(self.get_clock().now().nanoseconds*1e-9-stamp)*1000.
        if self.performance_enabled:
            self.output_samples.append((time.monotonic(),age))
        super().publish_image(color,image,detections,overlay_image=annotated)

    @serialized
    def publish_metrics(self):
        now=time.monotonic(); inputs=[v for t,v in list(self.samples) if now-t<=1.]
        outputs=[v for t,v in list(self.output_samples) if now-t<=1.]
        msg=RuntimeMetrics(); msg.header.stamp=self.get_clock().now().to_msg(); msg.source='yolo_pose'; msg.window_s=1.
        msg.input_count=len(inputs); msg.output_count=len(outputs)
        msg.input_fps=float(len(inputs)); msg.output_fps=float(len(outputs))
        msg.processing_ms=float(np.mean(inputs)) if inputs else math.nan
        msg.processing_p95_ms=float(np.percentile(inputs,95)) if inputs else math.nan
        msg.rgbd_ms=msg.processing_ms; msg.rgbd_p95_ms=msg.processing_p95_ms
        msg.observation_age_ms=float(np.mean(outputs)) if outputs else math.nan
        msg.observation_age_p95_ms=float(np.percentile(outputs,95)) if outputs else math.nan
        msg.control_latency_ms=msg.control_latency_p95_ms=math.nan
        self.metrics_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args); node=PoseTrackerNode(); executor=MultiThreadedExecutor(num_threads=2); executor.add_node(node)
    try: executor.spin()
    except KeyboardInterrupt: pass
    finally:
        executor.shutdown(); node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
