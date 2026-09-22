"""Automatic red target selection; no model, skeleton or motor publisher."""
import copy
import math
import time
import cv2
import numpy as np
import message_filters
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo
from visualization_msgs.msg import Marker
from person_interfaces.msg import TargetState
from vision_msgs.msg import Detection2DArray
from astra_body_adapter.detections import detection_array
from astra_body_adapter.performance import Performance
from .vision import Selection, detect, measure


def seconds(stamp):
    return stamp.sec + stamp.nanosec*1e-9


class RedTrackerNode(Node):
    def __init__(self, **kwargs):
        super().__init__('red_object_tracker', **kwargs)
        defaults = dict(color_topic='/camera/color/image_rect',
                        depth_topic='/camera/aligned_depth_to_color/image_raw',
                        camera_info_topic='/camera/color/camera_info',
                        depth_registered=False, sync_slop_s=.06, max_age_s=.5,
                        hue_low_max=10, hue_high_min=170, saturation_min=100,
                        value_min=70, min_area_fraction=.001,
                        confirm_frames=3, lost_timeout_s=1.)
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        self.cfg = {k: self.get_parameter(k).value for k in defaults}
        c = self.cfg
        if not 0 < c['sync_slop_s'] < c['max_age_s'] <= c['lost_timeout_s']:
            raise ValueError('Require 0 < sync_slop_s < max_age_s <= lost_timeout_s')
        if not (0 <= c['hue_low_max'] < c['hue_high_min'] <= 179
                and 0 <= c['saturation_min'] <= 255 and 0 <= c['value_min'] <= 255
                and 0 < c['min_area_fraction'] <= 1 and c['confirm_frames'] >= 1):
            raise ValueError('Invalid red thresholds or confirmation count')
        self.bridge = CvBridge()
        self.selection = Selection(c['confirm_frames'], c['lost_timeout_s'])
        self.info = self.snapshot = self.last_stamp = self.last_key = None
        self.performance = Performance(self, 'performance', 'red_object')
        self.last_pair_at = -math.inf
        self.error = ''
        self.pub = self.create_publisher(TargetState, 'target_state', 10)
        self.marker_pub = self.create_publisher(Marker, 'target_marker', 10)
        self.detections_pub = self.create_publisher(Detection2DArray, 'detections', 10)
        self.image_pub = self.create_publisher(Image, 'detections_image', 2)
        self.raw_pub = self.create_publisher(Image, 'color_image', 2)
        self.mask_pub = self.create_publisher(Image, 'red_mask_image', 2)
        self.create_subscription(CameraInfo, c['camera_info_topic'], self.on_info,
                                 qos_profile_sensor_data)
        self.color_sub = message_filters.Subscriber(self, Image, c['color_topic'],
                                                     qos_profile=qos_profile_sensor_data)
        self.depth_sub = message_filters.Subscriber(self, Image, c['depth_topic'],
                                                     qos_profile=qos_profile_sensor_data)
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.color_sub, self.depth_sub], 5, c['sync_slop_s'])
        self.sync.registerCallback(self.on_pair)
        self.color_sub.registerCallback(self.on_color)
        self.create_timer(.05, self.tick)

    def on_info(self, msg):
        self.info = msg

    def on_color(self, color):
        started = time.perf_counter()
        if self.performance.enabled:
            self.performance.inputs += 1
        # Visualization needs only RGB: never wait for depth or registration.
        self.raw_pub.publish(color)
        try:
            if color.encoding not in ('rgb8', 'bgr8', 'rgba8', 'bgra8'):
                raise ValueError('Expected RGB/BGR image for red visualization')
            image = self.bridge.imgmsg_to_cv2(color, 'bgr8').copy()
            components, mask = detect(image, **{k: self.cfg[k] for k in (
                'hue_low_max', 'hue_high_min', 'saturation_min', 'value_min', 'min_area_fraction')})
            # Green means this exact observation was accepted for RGB-D tracking.
            selected = self.selection.selected
            accepted = (self.snapshot is not None and not self.error
                        and self.snapshot[0] == color.header
                        and self.snapshot[3] is not None)
            for component in components:
                x, y, right, bottom = component.box
                locked = accepted and selected is not None and component.box == selected.box
                tint = (0, 255, 0) if locked else (0, 255, 255)
                cv2.rectangle(image, (x, y), (right, bottom), tint, 2)
                label = self.selection.target_id if locked else 'RED'
                cv2.putText(image, label, (x, max(15, y)),
                            cv2.FONT_HERSHEY_SIMPLEX, .5, tint, 1)
            for pixels, encoding, publisher in ((image, 'bgr8', self.image_pub),
                                                (mask, 'mono8', self.mask_pub)):
                output = self.bridge.cv2_to_imgmsg(pixels, encoding)
                output.header = copy.deepcopy(color.header)
                publisher.publish(output)
            if self.performance.enabled:
                self.performance.outputs += 1
                self.performance.record('observation_age', self.performance.observation_age(color.header.stamp))
        except (ValueError, TypeError, CvBridgeError, cv2.error) as exc:
            self.get_logger().warning(f'Color visualization rejected: {exc}')
        finally:
            self.performance.record('processing', (time.perf_counter()-started)*1000)

    def fresh(self, color, depth, received_at):
        now = self.get_clock().now().nanoseconds*1e-9
        return (time.monotonic()-received_at <= self.cfg['max_age_s']
                and all(seconds(m.header.stamp) > 0
                        and 0 <= now-seconds(m.header.stamp) <= self.cfg['max_age_s']
                        for m in (color, depth)))

    def on_pair(self, color, depth):
        if not self.cfg['depth_registered']:
            return
        at = time.monotonic()
        started = time.perf_counter()
        try:
            info = self.info
            if info is None:
                raise ValueError('Waiting for rectified color CameraInfo')
            if not self.fresh(color, depth, at):
                raise ValueError('Zero, future or expired image timestamps')
            if abs(seconds(color.header.stamp)-seconds(depth.header.stamp)) > self.cfg['sync_slop_s']:
                raise ValueError('Unsynchronized color/depth')
            if (not color.header.frame_id or color.header.frame_id != depth.header.frame_id
                    or color.header.frame_id != info.header.frame_id):
                raise ValueError('Registered images and calibration must share optical frame')
            if (color.width, color.height) != (depth.width, depth.height) or (
                    color.width, color.height) != (info.width, info.height):
                raise ValueError('Image/calibration dimensions differ')
            if info.binning_x > 1 or info.binning_y > 1 or info.roi.x_offset or info.roi.y_offset:
                raise ValueError('Cropped/binned calibration is unsupported')
            intrinsics = (info.p[0], info.p[5], info.p[2], info.p[6])
            if (not all(map(math.isfinite, info.p)) or intrinsics[0] <= 0
                    or intrinsics[1] <= 0 or info.p[3] != 0 or info.p[7] != 0):
                raise ValueError('Invalid rectified projection matrix')
            if color.encoding not in ('rgb8', 'bgr8', 'rgba8', 'bgra8'):
                raise ValueError('Expected RGB/BGR color image')
            if depth.encoding not in ('16UC1', '32FC1'):
                raise ValueError('Expected millimetre 16UC1 or metre 32FC1 depth')
            image = self.bridge.imgmsg_to_cv2(color, 'bgr8').copy()
            metres = self.bridge.imgmsg_to_cv2(depth, 'passthrough').astype(np.float32)
            if depth.encoding == '16UC1':
                metres *= .001
            key = (color.header.frame_id, color.width, color.height, tuple(info.p))
            stamp = seconds(color.header.stamp)
            if self.last_stamp is not None and (stamp <= self.last_stamp or key != self.last_key):
                self.selection.clear()
                self.snapshot = None
                if stamp <= self.last_stamp:
                    raise ValueError('Duplicate or backwards image timestamp')
            self.last_stamp, self.last_key = stamp, key
            components, mask = detect(image, **{k: self.cfg[k] for k in (
                'hue_low_max', 'hue_high_min', 'saturation_min', 'value_min', 'min_area_fraction')})
            self.selection.update(components, at)
            self.snapshot = (copy.deepcopy(color.header), copy.deepcopy(depth.header),
                             at, None)
            selected = self.selection.selected
            if selected is not None:
                self.snapshot = (*self.snapshot[:3], measure(metres, selected.mask, intrinsics))
            if self.fresh(color, depth, at):
                self.detections_pub.publish(detection_array(color.header,
                    [(c.box, 'red_object', math.nan,
                      self.selection.target_id if c is selected else '') for c in components]))
            self.error = ''
            self.last_pair_at = at
        except (ValueError, TypeError, CvBridgeError, cv2.error) as exc:
            self.error = str(exc)
            self.snapshot = None
            self.selection.selected = None
            self.selection.hits = 0
        finally:
            self.performance.record('rgbd', (time.perf_counter()-started)*1000)

    def tick(self):
        msg = TargetState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.source = 'red_object'
        msg.position.x = msg.position.y = msg.position.z = math.nan
        msg.horizontal_distance_m = msg.bearing_rad = math.nan
        msg.measurement_age_s = msg.confidence = math.nan
        msg.target_id = self.selection.target_id
        if not self.cfg['depth_registered']:
            msg.status, msg.detail = TargetState.NOT_READY, 'Confirm RGB/depth registration first'
        elif self.error:
            msg.status, msg.detail = TargetState.NOT_READY, self.error
        elif self.snapshot is None:
            msg.status, msg.detail = TargetState.NOT_READY, 'Waiting for synchronized RGB-D and CameraInfo'
        else:
            color_header, depth_header, at, xyz = self.snapshot
            now = seconds(msg.header.stamp)
            ages = (now-seconds(color_header.stamp), now-seconds(depth_header.stamp))
            age = max(ages)
            if (min(ages) < 0 or not 0 <= age <= self.cfg['max_age_s']
                    or time.monotonic()-at > self.cfg['max_age_s']):
                msg.status, msg.detail = TargetState.STALE, 'RGB-D stream expired'
                # A long stream gap must confirm a new identity, not revive old observations.
                if time.monotonic()-at >= self.cfg['lost_timeout_s']:
                    self.selection.clear()
                    msg.target_id = ''
            else:
                msg.header.frame_id = color_header.frame_id
                msg.observation_stamp = copy.deepcopy(color_header.stamp)
                msg.measurement_age_s = age
                if not self.selection.target_id:
                    msg.status, msg.detail = TargetState.SEARCHING, 'Confirming largest red component'
                elif self.selection.selected is None:
                    msg.status, msg.detail = TargetState.LOST, 'Selected red component absent'
                else:
                    msg.status = TargetState.TRACKING
                    msg.detail = 'Registered mask depth' if xyz else 'Red target; depth rejected'
                    if xyz is not None:
                        msg.position_valid = True
                        msg.position.x, msg.position.y, msg.position.z = xyz
                        msg.horizontal_distance_m = math.hypot(xyz[0], xyz[2])
                        msg.bearing_rad = math.atan2(xyz[0], xyz[2])
        self.pub.publish(msg)
        marker = Marker()
        marker.header = copy.deepcopy(msg.header)
        marker.ns, marker.id = 'red_target', 0
        marker.action = Marker.DELETE
        if msg.position_valid:
            marker.action, marker.type = Marker.ADD, Marker.SPHERE
            marker.pose.position = copy.deepcopy(msg.position)
            marker.pose.orientation.w = 1.
            marker.scale.x = marker.scale.y = marker.scale.z = .15
            marker.color.r = marker.color.a = 1.
            marker.lifetime.nanosec = 200000000
        self.marker_pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = RedTrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
