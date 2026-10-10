"""C route extends B outputs without changing target selection or motion interfaces."""
import copy
import math
import time
from collections import deque
from dataclasses import replace
import cv2
import numpy as np
import rclpy
from rclpy.parameter import Parameter
from rclpy.executors import MultiThreadedExecutor
from rclpy.time import Time
from geometry_msgs.msg import Point
from tf2_ros import Buffer, TransformListener, TransformException
from visualization_msgs.msg import Marker, MarkerArray
from person_interfaces.msg import (FloorPlane, PersonState, PersonStateArray, RuntimeMetrics,
                                   PersonIdentityArray)
from .node import TrackerNode, serialized
from .pose import PoseConfig, EDGES, LABELS, JointJumpGate, points2d, joints3d
from .input_contract import stamp_seconds
from .fusion import FALLBACK_SOURCES, FusionConfig, fuse_depth
from .identity_selection import IdentitySelection
from .pose3d import Pose3DConfig, EnhancedPostureTracker
from .floor_plane import FloorConfig, FloorTracker, camera_angles, fit_floor
from .height_fall import HeightFallConfig, HeightFallTracker, body_heights, skeleton_veto
from .pose import FALLEN, SITTING_CROUCHING, STANDING
from .ground import rotation_matrix


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
        self.spatial_config=Pose3DConfig(**spatial_values)
        self.postures = EnhancedPostureTracker(self.pose_config,self.spatial_config)
        # Optional RANSAC floor evidence; off by default and never a confirmation by itself.
        self.declare_parameter('floor_fit_enabled', False)
        self.floor_enabled=self.get_parameter('floor_fit_enabled').value
        floor_values={}
        for name,value in vars(FloorConfig()).items():
            self.declare_parameter('floor_'+name,value)
            floor_values[name]=self.get_parameter('floor_'+name).value
        self.floor_config=FloorConfig(**floor_values)
        self.floor_tracker=FloorTracker(self.floor_config)
        self.pending_floor=None
        if self.spatial_config.up_source=='floor' and not self.floor_enabled:
            raise ValueError("pose3d_up_source='floor' requires floor_fit_enabled")
        # 'tf' source: gravity frame z is up (e.g. base_footprint driven by the gimbal IMU).
        self.declare_parameter('gravity_frame','base_footprint')
        self.declare_parameter('gravity_ground_z_m',0.)
        self.gravity_frame=self.get_parameter('gravity_frame').value
        self.gravity_ground_z=float(self.get_parameter('gravity_ground_z_m').value)
        if self.spatial_config.up_source=='tf':
            self.tf_buffer=Buffer();self.tf_listener=TransformListener(self.tf_buffer,self)
        self.floor_pub=self.create_publisher(FloorPlane,'floor_plane',10)
        # Optional skeleton-free fall cue: body top height from depth (see height_fall.py).
        self.declare_parameter('height_fall_enabled',False)
        self.height_enabled=self.get_parameter('height_fall_enabled').value
        height_values={}
        for name,value in vars(HeightFallConfig()).items():
            self.declare_parameter('height_'+name,value)
            height_values[name]=self.get_parameter('height_'+name).value
        self.height_tracker=HeightFallTracker(HeightFallConfig(**height_values))
        self.height_boxes={}
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

    def measurement_is_fallback(self, track_id, evidence):
        observation = evidence.get(track_id)
        return observation is not None and observation.source in FALLBACK_SOURCES

    def mark_rejected(self, evidence, rejected):
        # Keep joints (they pass their own gate) but withdraw the gated body depth.
        return {key: replace(value, target=None, reason=value.reason+'; withheld by temporal depth gate (inconsistent or unconfirmed)')
                if key in rejected and value.target is not None else value
                for key, value in evidence.items()}

    def process_pair(self, *args):
        start=time.monotonic()
        result=super().process_pair(*args)
        if self.floor_enabled and self.cfg['depth_registered']:
            snapshot,detections=result[0],result[1]
            # Worker-owned; one in-flight future, consumed by publish_image for the same stamp.
            self.pending_floor=(stamp_seconds(snapshot[0].header.stamp),
                fit_floor(snapshot[1],snapshot[3],[d.box for d in detections],
                          (self.spatial_config.up_x,self.spatial_config.up_y,self.spatial_config.up_z),
                          self.floor_config))
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
            self.floor_tracker.reset()
            self.height_tracker.reset();self.height_boxes.clear()
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
            self.height_tracker.reset(); self.height_boxes.clear()
        msg=PersonStateArray(); msg.header=copy.deepcopy(color.header); msg.valid=True
        self.pose_frame=color.header.frame_id
        msg.detail='COCO17; camera-relative heuristic; initial thresholds'
        annotated=image.copy(); markers=MarkerArray(); current=set()
        stamp=stamp_seconds(color.header.stamp)
        gravity=self.frame_gravity(color,stamp)
        # Falls often break the tracker ID; a lost track may hand over to one new track (pose_handover_s).
        present={f'{self.selection.epoch}:{d.track_id}':d.box for d in detections if d.track_id is not None}
        self.postures.handover(stamp,present)
        if self.height_enabled and self.pose_config.handover_s>0:
            self.height_tracker.handover(stamp,present,self.height_boxes,self.pose_config.handover_s)
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
            gate=self.previous_joints.get(identity) or JointJumpGate(
                self.pose_config.joint_jump_m,self.pose_config.joint_jump_speed_mps,self.pose_config.max_gap_s)
            next_joints[identity]=gate
            xyz,_=gate.apply(xyz,stamp)
            person.keypoints_3d=[Point(x=float(p[0]),y=float(p[1]),z=float(p[2])) for p in xyz]
            person.keypoints_3d_valid=list(map(bool,np.isfinite(xyz).all(axis=1)))
            person.posture,person.fall_stage,person.detail=self.postures.update(
                identity,stamp,detection.box,detection.keypoints,xyz,gravity)
            if self.height_enabled:
                self.apply_height_fall(person,identity,stamp,detection.box,gravity)
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

    def apply_height_fall(self, person, identity, stamp, box, gravity):
        """Raise fall_stage from the body-top height cue; a skeleton standing/crouching posture
        in a tall box vetoes it. Without confirmed gravity and ground it can only report suspected."""
        s=self.spatial_config
        top=None
        if self.cfg['depth_registered']:
            if s.up_source=='static':
                up,height=np.array([s.up_x,s.up_y,s.up_z]),s.camera_height_m
            elif gravity is not None and gravity[1] is not None:
                up,height=gravity
            else:
                up=None
            if up is not None:
                measured=body_heights(self.snapshot[1],box,self.snapshot[3],up,height,self.height_tracker.cfg)
                top=None if measured is None else measured[0]
        self.height_boxes[identity]=tuple(map(float,box))
        phase,reason=self.height_tracker.update(identity,stamp,top,
            upright_hint=skeleton_veto(person.posture in (STANDING,SITTING_CROUCHING),box,self.height_tracker.cfg))
        if phase==2 and not (s.gravity_confirmed and s.ground_confirmed):
            phase,reason=1,reason+'; gravity/ground unconfirmed: suspected only'
        if phase>person.fall_stage:
            person.fall_stage=phase
            if phase==2:person.posture=FALLEN
        person.detail+=f'; height_fall={phase} ({reason})'

    def frame_gravity(self, color, stamp):
        """Per-frame (up, camera height) in the optical frame, or None; never a stale value."""
        floor=None
        if self.floor_enabled:
            pending=self.pending_floor;self.pending_floor=None
            msg=FloorPlane();msg.header=copy.deepcopy(color.header)
            if pending is None or pending[0]!=stamp:
                self.floor_tracker.reset();msg.detail='No floor fit for this frame'
            else:
                fit=pending[1];stable,msg.detail=self.floor_tracker.update(stamp,fit)
                msg.valid=fit.valid;msg.stable=bool(stable and fit.valid)
                msg.up.x,msg.up.y,msg.up.z=fit.up;msg.height_m=fit.height_m
                msg.pitch_up_deg,msg.roll_deg=camera_angles(fit.up)
                msg.inliers=fit.inliers;msg.inlier_fraction=fit.inlier_fraction;msg.rms_m=fit.rms_m
                if msg.stable:floor=(np.array(fit.up),fit.height_m)
            self.floor_pub.publish(msg)
        source=self.spatial_config.up_source
        if source=='floor':
            return floor
        if source=='tf':
            try:
                # Observation-time lookup only; a missing transform makes this frame 2D.
                t=self.tf_buffer.lookup_transform(self.gravity_frame,color.header.frame_id,
                                                  Time.from_msg(color.header.stamp)).transform
            except TransformException:
                return None
            r=rotation_matrix((t.rotation.x,t.rotation.y,t.rotation.z,t.rotation.w))
            return (r.T@np.array([0.,0.,1.]),t.translation.z-self.gravity_ground_z)
        return None

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
