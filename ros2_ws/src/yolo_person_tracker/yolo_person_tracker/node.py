"""Real RGB-D tracking, with one in-flight inference and explicit target selection."""
import copy
import math
import time
from functools import wraps
from threading import RLock
from concurrent.futures import ThreadPoolExecutor

import cv2
import numpy as np
import message_filters
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo, CompressedImage
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker
from person_interfaces.msg import OwnerState, TargetState
from person_interfaces.srv import ClaimOwner
from vision_msgs.msg import Detection2DArray
from astra_body_adapter.detections import detection_array
from .backend import YoloBackend
from .depth import DepthGate, DepthTrackFilter, measure
from .state import Selection
from .sot import ArbiterConfig, BridgeArbiter, BridgeConfig, TargetBridge, VitTrack, overlaps_anyone
from .input_contract import stamp_seconds, validate_pair
from .registration import load_registration, rectified_camera_info, validate_raw_pair
from .visualization import draw_labels
from .owner_lifecycle import OwnerLifecycle


def serialized(method):
    """Keep selection/snapshot/future transitions atomic across callback groups."""
    @wraps(method)
    def call(self, *args, **kwargs):
        with self.state_lock:
            return method(self, *args, **kwargs)
    return call


class TrackerNode(Node):
    def __init__(self, backend=None, sot_tracker=None, **kwargs):
        super().__init__('yolo_person_tracker', **kwargs)
        defaults = {
            'model_path': '', 'device': 'cpu', 'image_size': 640, 'nms_free': True,
            'model_task': 'detect',
            'color_topic': '/camera/color/image_rect',
            'depth_topic': '/camera/aligned_depth_to_color/image_raw',
            'camera_info_topic': '/camera/color/camera_info',
            'depth_registered': False, 'sync_slop_s': .06, 'max_age_s': .5,
            'visualization_fps': 10.0, 'visualization_scale': 0.5,
            'auto_lock_single': False,
            'owner_lost_timeout_s': 10.0,
            'track_hold_s': .8,
            'reacquire_s': 1.2,
            'reacquire_center_fraction': .25,
            'kalman_measurement_std_m': .08,
            'kalman_acceleration_std_mps2': 2.0,
            'depth_jump_reset_m': .8,
            # Gate replaces jump re-initialization; false restores the legacy filter.
            'depth_gate_enabled': True,
            'depth_gate_base_m': DepthGate.base_m,
            'depth_gate_speed_mps': DepthGate.speed_mps,
            'depth_gate_confirm_s': DepthGate.confirm_s,
            'depth_fallback_gate_scale': DepthGate.fallback_scale,
            'depth_fallback_noise_scale': DepthGate.fallback_noise_scale,
            'depth_min_fraction': .08,
            'position_hold_s': .25,
            # Opt-in single-object bridge (VitTrack) that arbitrates target continuity.
            'sot_enabled': False, 'sot_model_path': '', 'sot_backend': 'auto',
            'sot_min_score': BridgeConfig.min_score, 'sot_max_bridge_s': BridgeConfig.max_bridge_s,
            'sot_depth_rejects': BridgeConfig.depth_rejects,
            'sot_conflict_frames': ArbiterConfig.conflict_frames,
            'sot_handover_frames': ArbiterConfig.handover_frames,
            'sot_mask_refresh': True,
            # Opt-in in-process Gemini registration: subscribe native RGB-D, rectify/align in the
            # worker and publish the rectified CameraInfo, replacing the gemini_registration node.
            'registration_calibration_file': '', 'registration_expected_serial': '',
            'registration_backend': 'numpy',
            'registration_color_frame': 'camera_color_optical_frame',
            'registration_depth_frame': 'camera_depth_optical_frame',
            'registration_rectified_color_topic': '/camera/color/image_rect',
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.cfg = {name: self.get_parameter(name).value for name in defaults}
        self.owner = OwnerLifecycle(self.cfg['owner_lost_timeout_s'])
        self.selection_generation = self.pending_generation = self.arbiter_generation = 0
        if not 0 < self.cfg['sync_slop_s'] < self.cfg['max_age_s']:
            raise ValueError('Require 0 < sync_slop_s < max_age_s')
        if self.cfg['image_size'] < 32:
            raise ValueError('image_size must be at least 32')
        if not math.isfinite(self.cfg['track_hold_s']) or self.cfg['track_hold_s'] < 0:
            raise ValueError('track_hold_s must be finite and non-negative')
        if not math.isfinite(self.cfg['reacquire_s']) or self.cfg['reacquire_s'] < 0:
            raise ValueError('reacquire_s must be finite and non-negative')
        if (not math.isfinite(self.cfg['reacquire_center_fraction'])
                or self.cfg['reacquire_center_fraction'] <= 0):
            raise ValueError('reacquire_center_fraction must be finite and positive')
        for name in ('kalman_measurement_std_m', 'kalman_acceleration_std_mps2'):
            if not math.isfinite(self.cfg[name]) or self.cfg[name] <= 0:
                raise ValueError(f'{name} must be finite and positive')
        if (not math.isfinite(self.cfg['depth_jump_reset_m'])
                or self.cfg['depth_jump_reset_m'] <= 0):
            raise ValueError('depth_jump_reset_m must be finite and positive')
        self.depth_gate = DepthGate(
            base_m=self.cfg['depth_gate_base_m'], speed_mps=self.cfg['depth_gate_speed_mps'],
            confirm_s=self.cfg['depth_gate_confirm_s'],
            fallback_scale=self.cfg['depth_fallback_gate_scale'],
            fallback_noise_scale=self.cfg['depth_fallback_noise_scale'],
        ) if self.cfg['depth_gate_enabled'] else None
        if (not math.isfinite(self.cfg['depth_min_fraction'])
                or not 0 < self.cfg['depth_min_fraction'] <= 1):
            raise ValueError('depth_min_fraction must be finite and in (0, 1]')
        if not math.isfinite(self.cfg['position_hold_s']) or self.cfg['position_hold_s'] < 0:
            raise ValueError('position_hold_s must be finite and non-negative')
        if (not math.isfinite(self.cfg['visualization_fps'])
                or not 0 <= self.cfg['visualization_fps'] <= 30):
            raise ValueError('visualization_fps must be finite and in [0, 30]')
        if (not math.isfinite(self.cfg['visualization_scale'])
                or not 0 < self.cfg['visualization_scale'] <= 1):
            raise ValueError('visualization_scale must be finite and in (0, 1]')
        self.registration = None
        if self.cfg['registration_calibration_file']:
            self.registration = load_registration(self.cfg['registration_calibration_file'],
                                                  self.cfg['registration_expected_serial'],
                                                  self.cfg['registration_backend'])
        self.state_lock = RLock()
        self.input_group = MutuallyExclusiveCallbackGroup()
        self.state_group = MutuallyExclusiveCallbackGroup()
        self.positions = {}
        self.position_ages = {}
        self.position_stamps = {}
        self.held_position_ids = set()
        self.depth_filters = {}
        self.bridge = CvBridge()
        if not isinstance(self.cfg['sot_enabled'], bool):
            raise ValueError('sot_enabled must be boolean')
        self.arbiter = None
        if self.cfg['sot_enabled']:
            # Explicit local model; a missing or unloadable model refuses to start, no fallback.
            self.arbiter = BridgeArbiter(
                TargetBridge(sot_tracker or VitTrack(self.cfg['sot_model_path'], self.cfg['sot_backend']),
                             BridgeConfig(min_score=self.cfg['sot_min_score'],
                                          max_bridge_s=self.cfg['sot_max_bridge_s'],
                                          depth_rejects=self.cfg['sot_depth_rejects']),
                             gate=self.depth_gate),
                ArbiterConfig(conflict_frames=self.cfg['sot_conflict_frames'],
                              handover_frames=self.cfg['sot_handover_frames'],
                              mask_refresh=self.cfg['sot_mask_refresh']))
        self.arbiter_target = None  # worker-owned: target the arbiter state belongs to
        self.pending_target = None  # selection target at submission time
        self.sot_result = None  # (target_id, Decision) from the last worker run
        self.sot_bridged = None  # target currently reported from the bridge alone
        self.sot_paused = None  # target whose output is paused by a bridge/track conflict
        self.sot_detail = ''
        self.selection = Selection(
            # With the bridge, a vanished target is bridged or lost, never shown as its stale box.
            hold_s=0. if self.arbiter is not None else self.cfg['track_hold_s'],
            reacquire_s=self.cfg['reacquire_s'],
            reacquire_center_fraction=self.cfg['reacquire_center_fraction'],
            nearest_reacquire=self.arbiter is None)
        self.auto_lock_suppressed = False
        self.info = None
        self.snapshot = None
        self.future = None
        self.last_key = None
        self.last_stamp = None
        self.last_input_at = -math.inf
        self.stream_reset_pending = False
        self.error = ''
        self.last_visualization_at = -math.inf
        self.backend = backend
        self.pool = ThreadPoolExecutor(max_workers=1, thread_name_prefix='yolo')
        self.pub = self.create_publisher(TargetState, 'target_state', 10)
        self.owner_state_pub = self.create_publisher(OwnerState, 'owner_state', 10)
        self.marker_pub = self.create_publisher(Marker, 'target_marker', 10)
        self.detections_pub = self.create_publisher(Detection2DArray, 'detections', 10)
        self.image_pub = self.create_publisher(Image, 'detections_image', 2)
        self.color_preview_pub = self.create_publisher(
            CompressedImage, 'color_preview/compressed', 2)
        self.detections_preview_pub = self.create_publisher(
            CompressedImage, 'detections_preview/compressed', 2)
        self.create_service(Trigger, 'lock_target', self.lock_target, callback_group=self.state_group)
        self.create_service(Trigger, 'release_target', self.release_target, callback_group=self.state_group)
        self.create_service(ClaimOwner, 'claim_owner', self.claim_owner, callback_group=self.state_group)
        self.rectified_color_pub = None
        if self.registration is None:
            self.create_subscription(CameraInfo, self.cfg['camera_info_topic'],
                                     self.on_info, qos_profile_sensor_data, callback_group=self.input_group)
        else:
            # Same rectified CameraInfo the standalone node published, for the localizers.
            self.info_pub = self.create_publisher(CameraInfo, self.cfg['camera_info_topic'], 10)
            rectified_topic = self.cfg['registration_rectified_color_topic']
            if self.resolve_topic_name(rectified_topic) == self.resolve_topic_name(self.cfg['color_topic']):
                raise ValueError('Rectified color output must differ from raw color input')
            self.rectified_color_pub = self.create_publisher(Image, rectified_topic, 2)
        image_qos = QoSProfile(depth=1, reliability=qos_profile_sensor_data.reliability,
                               durability=qos_profile_sensor_data.durability)
        self.color_sub = message_filters.Subscriber(
            self, Image, self.cfg['color_topic'], qos_profile=image_qos, callback_group=self.input_group)
        self.depth_sub = message_filters.Subscriber(
            self, Image, self.cfg['depth_topic'], qos_profile=image_qos, callback_group=self.input_group)
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.color_sub, self.depth_sub], queue_size=5, slop=self.cfg['sync_slop_s'])
        self.sync.registerCallback(self.on_pair)
        self.loading = None
        if backend is None:
            if not self.cfg['model_path']:
                self.error = 'Configure an existing local model_path; no automatic weight download'
            else:
                self.loading = self.pool.submit(YoloBackend, self.cfg['model_path'],
                                                self.cfg['device'], self.cfg['image_size'],
                                                self.cfg['nms_free'], self.cfg['model_task'])
        self.timer = self.create_timer(.05, self.tick, callback_group=self.state_group)

    @serialized
    def on_info(self, msg):
        self.info = msg

    @serialized
    def lock_target(self, request, response):
        return self.claim_owner(ClaimOwner.Request(), response)

    @serialized
    def claim_owner(self, request, response):
        # The serialized lifecycle is authoritative, including when an old track was
        # withdrawn or its epoch reset. A claim must not restart a reserved owner's timer.
        if self.owner.state != 'UNCLAIMED':
            response.success, response.message = False, 'Owner reserved; wait for UNCLAIMED or call release_target first'
            return response
        fresh = self.snapshot is not None and self.fresh_snapshot()
        track_id = None
        if request.target_id:
            try:
                epoch, track_id = map(int, request.target_id.split(':'))
                if epoch != self.selection.epoch or track_id < 0:
                    raise ValueError()
            except ValueError:
                response.success, response.message = False, 'Require a current epoch:track_id'
                return response
        if self.error or not self.cfg['depth_registered'] or not fresh:
            response.success, response.message = False, 'No fresh valid inference'
        else:
            response.success, response.message = self.selection.lock(max_age=self.cfg['max_age_s'], track_id=track_id)
            if response.success:
                self.owner.claim()
                self.selection_generation += 1
                self.auto_lock_suppressed = False
                self.sot_detail = ''
        return response

    @serialized
    def release_target(self, request, response):
        self.selection.release()
        self.owner.release()
        self.selection_generation += 1
        self.sot_detail = ''
        self.auto_lock_suppressed = True
        response.success, response.message = True, 'Target released'
        return response

    def fresh(self, color, received_at):
        stamp = stamp_seconds(color.header.stamp)
        age = self.get_clock().now().nanoseconds*1e-9 - stamp
        return (stamp > 0 and 0 <= age <= self.cfg['max_age_s']
                and time.monotonic()-received_at <= self.cfg['max_age_s'])

    def fresh_snapshot(self):
        return (self.fresh(self.snapshot[0], self.snapshot[2])
                and self.fresh(self.snapshot[5], self.snapshot[2]))

    @serialized
    def on_pair(self, color, depth):
        if self.backend is None:
            return
        received_at = time.monotonic()
        try:
            now = self.get_clock().now().nanoseconds*1e-9
            if self.registration is None:
                info = self.info
                intrinsics = validate_pair(color, depth, info, now,
                                           self.cfg['max_age_s'], self.cfg['sync_slop_s'])
            else:
                intrinsics = validate_raw_pair(
                    color, depth, now, self.cfg['max_age_s'], self.cfg['sync_slop_s'],
                    self.cfg['registration_color_frame'], self.cfg['registration_depth_frame'],
                    self.registration)
                info = rectified_camera_info(self.registration, color.header)
                self.info_pub.publish(info)
            stamp = stamp_seconds(color.header.stamp)
            key = (color.header.frame_id, color.width, color.height, tuple(info.p))
            reset = self.last_key is not None and (
                key != self.last_key or stamp <= self.last_stamp
                or stamp-self.last_stamp > self.cfg['max_age_s']
                or received_at-self.last_input_at > self.cfg['max_age_s'])
            # Continuity is about valid incoming RGB-D, including frames skipped
            # while inference is busy. A slow worker is not a camera restart.
            self.stream_reset_pending |= reset
            self.last_key, self.last_stamp, self.last_input_at = key, stamp, received_at
            if self.future is not None:
                return
            reset = self.stream_reset_pending
            self.stream_reset_pending = False
            if reset:
                self.selection.reset_stream()
                self.depth_filters.clear()
            self.error = ''
            self.pending_target = None if reset else self.selection.target_id
            self.pending_generation = self.selection_generation
            self.future = self.pool.submit(self.process_pair, color, depth, received_at, intrinsics, reset)
        except (ValueError, TypeError, cv2.error, CvBridgeError) as exc:
            self.error = str(exc)
            self.snapshot = None
            self.selection.candidates = []

    def measure_frame(self, metres, detections, intrinsics):
        """Worker-owned measurements and optional per-frame subclass evidence."""
        return ({d.track_id: measure(metres, d.box, intrinsics,
                                    min_fraction=self.cfg['depth_min_fraction'])
                 for d in detections if d.track_id is not None}, {})

    def measurement_is_fallback(self, track_id, evidence):
        """Whether a measurement lacks torso corroboration; plain region depth never does."""
        return False

    def mark_rejected(self, evidence, rejected):
        """Subclass hook: gated depth must not be republished as current evidence."""
        return evidence

    def process_pair(self, color, depth, received_at, intrinsics, reset):
        # One worker owns all backend calls; no callback can reset/update ByteTrack concurrently.
        bridge = CvBridge()
        image = bridge.imgmsg_to_cv2(color, desired_encoding='bgr8').copy()
        metres = bridge.imgmsg_to_cv2(depth, desired_encoding='passthrough').astype(np.float32)
        if depth.encoding == '16UC1':
            metres *= .001
        if self.registration is not None:
            # Raw native pair: rectify color and project depth into it, as gemini_registration did.
            image, metres = self.registration.apply(image, metres)
        if reset:
            self.backend.reset()
        detections = self.backend.infer(image)
        observation_stamp = stamp_seconds(color.header.stamp)
        # Keep tracks with recent accepted or still-pending (unconfirmed) evidence.
        self.depth_filters = {key: filt for key, filt in self.depth_filters.items()
                              if filt.evidence_stamp is not None
                              and 0 <= observation_stamp-filt.evidence_stamp <= self.cfg['max_age_s']}
        positions = {}
        position_ages = {}
        position_stamps = {}
        held_position_ids = set()
        evidence = {}
        rejected = set()
        if self.cfg['depth_registered']:
            measurements, evidence = self.measure_frame(metres, detections, intrinsics)
            for d in detections:
                if d.track_id is None:
                    continue
                filt = self.depth_filters.setdefault(
                    d.track_id,
                    DepthTrackFilter(
                        measurement_std_m=self.cfg['kalman_measurement_std_m'],
                        acceleration_std_mps2=self.cfg['kalman_acceleration_std_mps2'],
                        jump_reset_m=self.cfg['depth_jump_reset_m'],
                        reset_gap_s=self.cfg['max_age_s'], gate=self.depth_gate))
                measured = measurements.get(d.track_id)
                xyz = filt.update(measured, observation_stamp,
                                  self.measurement_is_fallback(d.track_id, evidence))
                if filt.last_rejected:
                    rejected.add(d.track_id)
                if xyz is not None:
                    positions[d.track_id] = xyz
                    position_ages[d.track_id] = 0.0
                    position_stamps[d.track_id] = observation_stamp
                elif measured is None or filt.last_rejected:
                    # Rejected depth keeps the last real measurement age, like a gap.
                    held = filt.hold(observation_stamp, self.cfg['position_hold_s'])
                    if held is not None:
                        positions[d.track_id], position_ages[d.track_id] = held
                        position_stamps[d.track_id] = observation_stamp-position_ages[d.track_id]
                        held_position_ids.add(d.track_id)
            evidence = self.mark_rejected(evidence, rejected)
            self.run_arbiter(image, metres, intrinsics, observation_stamp, detections, positions,
                             position_ages, position_stamps, held_position_ids)
        return ((color, metres, received_at, intrinsics, image, depth, evidence), detections,
                positions, position_ages, position_stamps, held_position_ids)

    def run_arbiter(self, image, metres, intrinsics, stamp, detections, positions,
                    position_ages, position_stamps, held_position_ids):
        """Worker-owned: bridge the selected target; never chooses a person by itself."""
        self.sot_result = None
        if self.arbiter is None:
            return
        target = self.pending_target
        if target != self.arbiter_target or self.arbiter_generation != self.pending_generation:
            self.arbiter.reset(); self.arbiter_target = target
            self.arbiter_generation = self.pending_generation
        if target is None:
            return
        tracked = [d for d in detections if d.track_id is not None]
        current = next((d for d in tracked if d.track_id == target[1]), None)
        decision = self.arbiter.update(image, metres, intrinsics, stamp, current, tracked)
        if decision.action == 'handover':
            self.arbiter_target = (target[0], decision.track_id)
        elif decision.action == 'lost':
            self.arbiter_target = None
        elif decision.action == 'fill' and current is None:
            # Bridged box depth enters only the target's own filter, as tighter-gated fallback.
            filt = self.depth_filters.get(target[1])
            if filt is not None:
                xyz = filt.update(measure(metres, decision.box, intrinsics,
                                          min_fraction=self.cfg['depth_min_fraction']), stamp, True)
                if xyz is not None:
                    positions[target[1]], position_ages[target[1]] = xyz, 0.0
                    position_stamps[target[1]] = stamp
                    held_position_ids.discard(target[1])
        self.sot_result = (target, decision)

    def apply_arbiter(self):
        """Main-thread: apply the worker decision if the selection did not change meanwhile."""
        self.sot_bridged = self.sot_paused = None
        result, self.sot_result = self.sot_result, None
        if (result is None or result[0] != self.selection.target_id
                or self.pending_generation != self.selection_generation):
            if hasattr(self.selection, 'check_absence') and self.snapshot is not None:
                self.selection.check_absence(self.snapshot[2], bridging=False)
            return
        decision = result[1]
        self.sot_detail = decision.reason
        identity = hasattr(self.selection, 'note_bridge')  # ReID identity lock
        if hasattr(self.selection, 'check_absence'):  # bridge-primary owner following
            self.apply_owner(decision)
            return
        now = time.monotonic()
        if identity:
            # People overlapping is exactly when ByteTrack and the bridge can both swap: the
            # bridge never carries identity through it; ReID must re-confirm after separation.
            target = next((d for d in self.selection.candidates
                           if (self.selection.epoch, d.track_id) == self.selection.target_id), None)
            overlapped = target is not None and overlaps_anyone(
                target.box, [d.box for d in self.selection.candidates if d is not target])
            self.selection.note_bridge(decision.action in ('agree', 'fill', 'handover') and not overlapped, now)
        if decision.action == 'lost':
            if identity:
                # Withdraw the association; ReID re-confirms the same person automatically,
                # but the bridge may not carry it again until it is trusted anew.
                self.selection.invalidate()
                self.selection.distrust()
                self.sot_detail = 'Identity withdrawn: ' + decision.reason + '; waiting for ReID re-confirmation'
            else:
                self.selection.release()
                self.auto_lock_suppressed = True
                self.sot_detail = 'Released: ' + decision.reason
        elif decision.action == 'handover':
            if identity:
                self.selection.bridge_handover(decision.track_id, now)
            else:
                self.selection.handover(decision.track_id, self.snapshot[2])
        elif decision.action == 'fill' and (not identity or self.selection.carried(now)):
            self.sot_bridged = self.selection.target_id
        elif decision.action == 'conflict':
            self.sot_paused = self.selection.target_id

    def apply_owner(self, decision):
        received = self.snapshot[2]
        if decision.action == 'lost':
            self.selection.lose('Bridge: ' + decision.reason)
        elif decision.action == 'handover':
            self.selection.handover(decision.track_id, received)
        elif decision.action == 'fill':
            self.sot_bridged = self.selection.target_id
            self.selection.note_bridging(received)
        elif decision.action == 'conflict':
            self.sot_paused = self.selection.target_id
        self.selection.check_absence(received, bridging=decision.action == 'fill')

    @serialized
    def tick(self):
        if self.loading is not None and self.loading.done():
            try:
                self.backend = self.loading.result()
                self.error = ''
            except Exception as exc:
                self.error = f'Model initialization failed: {type(exc).__name__}: {exc}'
                self.get_logger().error(self.error)
            self.loading = None
        if self.future is not None and self.future.done():
            try:
                if self.stream_reset_pending:
                    # The worker still owns its filters until it has finished.
                    # Withdraw its old result; the next valid pair resets them.
                    self.future.result()
                    self.future = None
                    self.snapshot = None
                    self.selection.candidates = []
                    self.sot_result = None
                    return
                (self.snapshot, detections, self.positions,
                 self.position_ages, self.position_stamps,
                 self.held_position_ids) = self.future.result()
                self.selection.update(detections, self.snapshot[0].width, self.snapshot[2])
                self.apply_arbiter()
                if self.fresh_snapshot():
                    if (self.cfg['auto_lock_single'] and self.cfg['depth_registered']
                            and not self.auto_lock_suppressed):
                        if self.owner.state == 'UNCLAIMED' and self.selection.auto_lock_single(max_age=self.cfg['max_age_s']):
                            self.owner.claim()
                            self.selection_generation += 1
                    self.detections_pub.publish(detection_array(self.snapshot[0].header,
                        [(d.box, 'person', d.confidence,
                          '' if d.track_id is None else f'{self.selection.epoch}:{d.track_id}')
                         for d in detections]))
                    self.publish_image(self.snapshot[0], self.snapshot[4], detections)
            except Exception as exc:
                self.error = f'Inference failed: {type(exc).__name__}: {exc}'
                self.snapshot = None
                self.position_ages = {}
                self.position_stamps = {}
                self.held_position_ids = set()
                self.selection.reset_stream()
                self.last_key = ('failed',)
                self.get_logger().error(self.error)
            self.future = None
        msg = TargetState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.source = 'yolo'
        msg.is_simulated = False
        msg.target_id = ('' if self.selection.target_id is None else
                         ':'.join(map(str, self.selection.target_id)))
        msg.position.x = msg.position.y = msg.position.z = math.nan
        msg.horizontal_distance_m = msg.bearing_rad = math.nan
        msg.measurement_age_s = msg.confidence = math.nan
        if not self.cfg['depth_registered']:
            msg.status, msg.detail = TargetState.NOT_READY, (
                self.error or 'RGB-D registration unverified; 2D detections only')
        elif self.backend is None:
            msg.status, msg.detail = TargetState.NOT_READY, self.error or 'Loading model'
        elif self.error:
            msg.status, msg.detail = TargetState.NOT_READY, self.error
        elif self.snapshot is None or not self.fresh_snapshot():
            msg.status, msg.detail = TargetState.STALE, 'Waiting for fresh synchronized RGB-D inference'
        else:
            color, depth, _, intrinsics, _, _, _ = self.snapshot
            msg.header.frame_id = color.header.frame_id
            msg.observation_stamp = copy.deepcopy(color.header.stamp)
            chosen = self.selection.selected()
            if self.selection.target_id is None:
                msg.status = TargetState.SEARCHING
                msg.detail = ('Waiting for one stable track (3 frames); multiple people require lock_target'
                              if self.cfg['auto_lock_single'] and not self.auto_lock_suppressed
                              else 'Call lock_target to select central track')
                if self.sot_detail.startswith('Released: '):
                    msg.detail = self.sot_detail + '; call release_target before lock_target'
                if getattr(self.selection, 'detail', ''):
                    msg.detail = 'Owner: ' + self.selection.detail
            elif self.sot_paused is not None and self.sot_paused == self.selection.target_id:
                msg.status = TargetState.LOST
                msg.detail = 'Paused: single-object tracker disagrees with the selected track (possible ID swap)'
            elif (chosen is None and self.sot_bridged == self.selection.target_id
                  and self.selection.target_id[1] in self.positions
                  and self.selection.target_id[1] not in self.held_position_ids):
                xyz = self.positions[self.selection.target_id[1]]
                msg.status = TargetState.TRACKING
                msg.detail = 'Bridged: target undetected; single-object tracker box depth (fallback gate)'
                msg.position_valid = True
                msg.position.x, msg.position.y, msg.position.z = xyz
                msg.horizontal_distance_m = math.hypot(xyz[0], xyz[2])
                msg.bearing_rad = math.atan2(xyz[0], xyz[2])
                msg.measurement_age_s = (self.get_clock().now().nanoseconds*1e-9
                                         - stamp_seconds(color.header.stamp))
            elif chosen is None:
                msg.status, msg.detail = TargetState.LOST, 'Selected track absent; call release_target before lock_target after stream reset'
                if getattr(self, 'reid_lock_enabled', False):
                    msg.detail = ('Waiting for fresh confirmed identity: '
                                  + (self.selection.person_id or 'initial association'))
            else:
                msg.status = TargetState.TRACKING
                msg.confidence = chosen.confidence
                xyz = self.positions.get(chosen.track_id)
                held = chosen.track_id in self.held_position_ids
                measurement_age = (self.get_clock().now().nanoseconds*1e-9
                                   - stamp_seconds(color.header.stamp)
                                   + self.position_ages.get(chosen.track_id, 0.0))
                if held and not 0 <= measurement_age <= self.cfg['position_hold_s']:
                    xyz = None
                if xyz is None:
                    msg.detail = ('Tracked; waiting for current body-part depth'
                                  if chosen not in self.selection.candidates
                                  else 'Tracked; body-part depth rejected')
                else:
                    msg.detail = ('Tracked; short predicted depth hold'
                                  if held else 'Tracked; registered body-part depth')
                if xyz is not None:
                    observation_stamp = self.position_stamps.get(chosen.track_id)
                    # Fresh measurements retain the exact ROS sensor stamp above;
                    # a float round-trip loses nanoseconds at Unix epoch scale.
                    if held and observation_stamp is not None and math.isfinite(observation_stamp):
                        sec = int(observation_stamp)
                        nanosec = int(round((observation_stamp-sec)*1e9))
                        if nanosec >= 1000000000:
                            sec += 1
                            nanosec -= 1000000000
                        msg.observation_stamp.sec = sec
                        msg.observation_stamp.nanosec = nanosec
                    msg.position_valid = True
                    msg.position.x, msg.position.y, msg.position.z = xyz
                    msg.horizontal_distance_m = math.hypot(xyz[0], xyz[2])
                    msg.bearing_rad = math.atan2(xyz[0], xyz[2])
                    msg.measurement_age_s = measurement_age
        now = time.monotonic()
        if self.owner.update(msg.status == TargetState.TRACKING, now):
            self.selection.release()
            self.selection_generation += 1
            self.sot_bridged = self.sot_paused = None
            self.sot_detail = ''
            self.auto_lock_suppressed = False
            msg.target_id, msg.status, msg.position_valid = '', TargetState.SEARCHING, False
            msg.position.x = msg.position.y = msg.position.z = math.nan
            msg.horizontal_distance_m = msg.bearing_rad = math.nan
            msg.measurement_age_s = msg.confidence = math.nan
            msg.detail = f'Owner absent for {self.owner.timeout_s:g} seconds; ready for a new claim'
        self.owner_state_pub.publish(OwnerState(
            header=copy.deepcopy(msg.header), state=self.owner.state, event=self.owner.event,
            sequence=self.owner.sequence, target_id=msg.target_id,
            person_id=getattr(self.selection, 'owner_id', '') or getattr(self.selection, 'person_id', ''),
            lost_for_s=float(self.owner.lost_for(now)), reclaim_after_s=float(self.owner.timeout_s),
            can_claim=self.owner.state == 'UNCLAIMED'))
        self.pub.publish(msg)
        marker = Marker()
        marker.header = copy.deepcopy(msg.header)
        marker.ns, marker.id = 'yolo_target', 0
        marker.action = Marker.DELETE
        if msg.position_valid:
            marker.action, marker.type = Marker.ADD, Marker.SPHERE
            marker.pose.position = copy.deepcopy(msg.position)
            marker.pose.orientation.w = 1.
            marker.scale.x = marker.scale.y = marker.scale.z = .15
            marker.color.g, marker.color.a = 1., 1.
            marker.lifetime.nanosec = 200000000
        self.marker_pub.publish(marker)

    def publish_image(self, color, image, detections, overlay_image=None, extra_labels=None):
        if self.rectified_color_pub is not None:
            # ReID must see exactly the rectified pixels used by this inference,
            # with the same acquisition header, independent of visualization rate.
            rectified = self.bridge.cv2_to_imgmsg(image, encoding='bgr8')
            rectified.header = copy.deepcopy(color.header)
            self.rectified_color_pub.publish(rectified)
        now = time.monotonic()
        fps = self.cfg['visualization_fps']
        if fps == 0 or now-self.last_visualization_at < 1.0/fps:
            return
        self.last_visualization_at = now
        annotated = (image if overlay_image is None else overlay_image).copy()
        for detection in detections:
            x1, y1, x2, y2 = map(int, detection.box)
            cv2.rectangle(annotated, (x1,y1), (x2,y2), (0,255,0), 2)
        scale = self.cfg['visualization_scale']
        raw_preview = image
        if scale != 1:
            raw_preview = cv2.resize(image, None, fx=scale, fy=scale,
                                     interpolation=cv2.INTER_AREA)
            annotated = cv2.resize(annotated, None, fx=scale, fy=scale,
                                   interpolation=cv2.INTER_AREA)
        # Labels are rendered after downscaling, so the two rows stay readable.
        for detection in detections:
            identity = 'person' if detection.track_id is None else f'{self.selection.epoch}:{detection.track_id}'
            label = f'{identity} {detection.confidence:.2f}'
            if self.selection.target_id == (self.selection.epoch, detection.track_id):
                label += ' LOCKED'
            lines = [(label, (0, 255, 0))]
            if extra_labels and detection.track_id in extra_labels:
                lines.append(extra_labels[detection.track_id])
            draw_labels(annotated, [v * scale for v in detection.box], lines)
        output = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
        output.header = copy.deepcopy(color.header)
        self.image_pub.publish(output)
        for frame, publisher in ((raw_preview, self.color_preview_pub),
                                 (annotated, self.detections_preview_pub)):
            ok, jpeg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
            if ok:
                compressed = CompressedImage()
                compressed.header = copy.deepcopy(color.header)
                compressed.format = 'jpeg'
                compressed.data = jpeg.tobytes()
                publisher.publish(compressed)

    def destroy_node(self):
        self.pool.shutdown(wait=True, cancel_futures=True)
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TrackerNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
