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
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker
from person_interfaces.msg import TargetState
from .backend import YoloBackend
from .depth import measure
from .state import Selection
from .input_contract import stamp_seconds, validate_pair


def serialized(method):
    """Keep selection/snapshot/future transitions atomic across callback groups."""
    @wraps(method)
    def call(self, *args, **kwargs):
        with self.state_lock:
            return method(self, *args, **kwargs)
    return call


class TrackerNode(Node):
    def __init__(self, backend=None, **kwargs):
        super().__init__('yolo_person_tracker', **kwargs)
        defaults = {
            'model_path': '', 'device': 'cpu', 'image_size': 640, 'nms_free': True,
            'color_topic': '/camera/color/image_rect',
            'depth_topic': '/camera/aligned_depth_to_color/image_raw',
            'camera_info_topic': '/camera/color/camera_info',
            'depth_registered': False, 'sync_slop_s': .06, 'max_age_s': .5,
            'visualization_fps': 10.0, 'visualization_scale': 0.5,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.cfg = {name: self.get_parameter(name).value for name in defaults}
        if not 0 < self.cfg['sync_slop_s'] < self.cfg['max_age_s']:
            raise ValueError('Require 0 < sync_slop_s < max_age_s')
        if self.cfg['image_size'] < 32:
            raise ValueError('image_size must be at least 32')
        if (not math.isfinite(self.cfg['visualization_fps'])
                or not 0 <= self.cfg['visualization_fps'] <= 30):
            raise ValueError('visualization_fps must be finite and in [0, 30]')
        if (not math.isfinite(self.cfg['visualization_scale'])
                or not 0 < self.cfg['visualization_scale'] <= 1):
            raise ValueError('visualization_scale must be finite and in (0, 1]')
        self.state_lock = RLock()
        self.input_group = MutuallyExclusiveCallbackGroup()
        self.state_group = MutuallyExclusiveCallbackGroup()
        self.positions = {}
        self.bridge = CvBridge()
        self.selection = Selection()
        self.info = None
        self.snapshot = None
        self.future = None
        self.last_key = None
        self.last_stamp = None
        self.last_input_at = -math.inf
        self.error = ''
        self.last_visualization_at = -math.inf
        self.backend = backend
        self.pool = ThreadPoolExecutor(max_workers=1, thread_name_prefix='yolo')
        self.pub = self.create_publisher(TargetState, 'target_state', 10)
        self.marker_pub = self.create_publisher(Marker, 'target_marker', 10)
        self.image_pub = self.create_publisher(Image, 'detections_image', 2)
        self.create_service(Trigger, 'lock_target', self.lock_target, callback_group=self.state_group)
        self.create_service(Trigger, 'release_target', self.release_target, callback_group=self.state_group)
        self.create_subscription(CameraInfo, self.cfg['camera_info_topic'],
                                 self.on_info, qos_profile_sensor_data, callback_group=self.input_group)
        self.color_sub = message_filters.Subscriber(
            self, Image, self.cfg['color_topic'], qos_profile=qos_profile_sensor_data, callback_group=self.input_group)
        self.depth_sub = message_filters.Subscriber(
            self, Image, self.cfg['depth_topic'], qos_profile=qos_profile_sensor_data, callback_group=self.input_group)
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.color_sub, self.depth_sub], queue_size=5, slop=self.cfg['sync_slop_s'])
        self.sync.registerCallback(self.on_pair)
        self.loading = None
        if not self.cfg['depth_registered']:
            self.error = 'Set depth_registered only after verifying rectified color/depth registration'
        elif backend is None:
            if not self.cfg['model_path']:
                self.error = 'Configure an existing local model_path; no automatic weight download'
            else:
                self.loading = self.pool.submit(YoloBackend, self.cfg['model_path'],
                                                self.cfg['device'], self.cfg['image_size'],
                                                self.cfg['nms_free'])
        self.timer = self.create_timer(.05, self.tick, callback_group=self.state_group)

    @serialized
    def on_info(self, msg):
        self.info = msg

    @serialized
    def lock_target(self, request, response):
        fresh = self.snapshot is not None and self.fresh_snapshot()
        if self.error or not fresh:
            response.success, response.message = False, 'No fresh valid inference'
        else:
            response.success, response.message = self.selection.lock(max_age=self.cfg['max_age_s'])
        return response

    @serialized
    def release_target(self, request, response):
        self.selection.release()
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
        if self.backend is None or not self.cfg['depth_registered'] or self.future is not None:
            return
        received_at = time.monotonic()
        try:
            info = self.info
            intrinsics = validate_pair(
                color, depth, info, self.get_clock().now().nanoseconds*1e-9,
                self.cfg['max_age_s'], self.cfg['sync_slop_s'])
            stamp = stamp_seconds(color.header.stamp)
            key = (color.header.frame_id, color.width, color.height, tuple(info.p))
            reset = self.last_key is not None and (
                key != self.last_key or stamp <= self.last_stamp
                or stamp-self.last_stamp > self.cfg['max_age_s']
                or received_at-self.last_input_at > self.cfg['max_age_s'])
            if reset:
                self.selection.reset_stream()
            self.last_key, self.last_stamp, self.last_input_at = key, stamp, received_at
            self.error = ''
            self.future = self.pool.submit(self.process_pair, color, depth, received_at, intrinsics, reset)
        except (ValueError, TypeError, cv2.error, CvBridgeError) as exc:
            self.error = str(exc)
            self.snapshot = None
            self.selection.candidates = []

    def process_pair(self, color, depth, received_at, intrinsics, reset):
        # One worker owns all backend calls; no callback can reset/update ByteTrack concurrently.
        bridge = CvBridge()
        image = bridge.imgmsg_to_cv2(color, desired_encoding='bgr8').copy()
        metres = bridge.imgmsg_to_cv2(depth, desired_encoding='passthrough').astype(np.float32)
        if depth.encoding == '16UC1':
            metres *= .001
        if reset:
            self.backend.reset()
        detections = self.backend.infer(image)
        positions = {d.track_id: measure(metres, d.box, intrinsics) for d in detections}
        return (color, metres, received_at, intrinsics, image, depth), detections, positions

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
                self.snapshot, detections, self.positions = self.future.result()
                self.selection.update(detections, self.snapshot[0].width, self.snapshot[2])
                if self.fresh_snapshot():
                    self.publish_image(self.snapshot[0], self.snapshot[4], detections)
            except Exception as exc:
                self.error = f'Inference failed: {type(exc).__name__}: {exc}'
                self.snapshot = None
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
        if self.backend is None or not self.cfg['depth_registered']:
            msg.status, msg.detail = TargetState.NOT_READY, self.error or 'Loading model'
        elif self.error:
            msg.status, msg.detail = TargetState.NOT_READY, self.error
        elif self.snapshot is None or not self.fresh_snapshot():
            msg.status, msg.detail = TargetState.STALE, 'Waiting for fresh synchronized RGB-D inference'
        else:
            color, depth, _, intrinsics, _, _ = self.snapshot
            msg.header.frame_id = color.header.frame_id
            msg.observation_stamp = copy.deepcopy(color.header.stamp)
            msg.measurement_age_s = (stamp_seconds(msg.header.stamp)-stamp_seconds(color.header.stamp))
            chosen = self.selection.selected()
            if self.selection.target_id is None:
                msg.status, msg.detail = TargetState.SEARCHING, 'Call lock_target to select central track'
            elif chosen is None:
                msg.status, msg.detail = TargetState.LOST, 'Selected track absent; explicit relock required after stream reset'
            else:
                msg.status = TargetState.TRACKING
                msg.confidence = chosen.confidence
                xyz = self.positions.get(chosen.track_id)
                msg.detail = 'Tracked; torso depth rejected' if xyz is None else 'Tracked; registered torso depth'
                if xyz is not None:
                    msg.position_valid = True
                    msg.position.x, msg.position.y, msg.position.z = xyz
                    msg.horizontal_distance_m = math.hypot(xyz[0], xyz[2])
                    msg.bearing_rad = math.atan2(xyz[0], xyz[2])
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

    def publish_image(self, color, image, detections):
        now = time.monotonic()
        fps = self.cfg['visualization_fps']
        if fps == 0 or now-self.last_visualization_at < 1.0/fps:
            return
        self.last_visualization_at = now
        annotated = image.copy()
        for detection in detections:
            x1, y1, x2, y2 = map(int, detection.box)
            cv2.rectangle(annotated, (x1,y1), (x2,y2), (0,255,0), 2)
            label = f'{self.selection.epoch}:{detection.track_id} {detection.confidence:.2f}'
            if self.selection.target_id == (self.selection.epoch, detection.track_id):
                label += ' LOCKED'
            cv2.putText(annotated, label, (x1,max(15,y1)), cv2.FONT_HERSHEY_SIMPLEX,
                        .5, (0,255,0), 1)
        scale = self.cfg['visualization_scale']
        if scale != 1:
            annotated = cv2.resize(annotated, None, fx=scale, fy=scale,
                                   interpolation=cv2.INTER_AREA)
        output = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
        output.header = copy.deepcopy(color.header)
        self.image_pub.publish(output)

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
