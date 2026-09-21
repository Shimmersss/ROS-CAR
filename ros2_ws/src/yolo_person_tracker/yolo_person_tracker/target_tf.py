"""Transform fresh optical observations into base coordinates; never publish commands."""
import copy
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import TransformStamped, PointStamped
from visualization_msgs.msg import Marker
from tf2_ros import Buffer, TransformListener, StaticTransformBroadcaster, TransformException
from tf2_geometry_msgs import do_transform_point
from person_interfaces.msg import TargetState
from .input_contract import stamp_seconds


class TargetTransform(Node):
    def __init__(self, **kwargs):
        super().__init__('target_transform', **kwargs)
        defaults = dict(target_frame='base_link', expected_source_frame='camera_color_optical_frame',
                        extrinsics_calibrated=False, publish_mount_tf=False,
                        mount_child_frame='camera_link', mount_translation=[0., 0., 0.],
                        mount_quaternion=[0., 0., 0., 1.], max_age_s=.5)
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.cfg = {name: self.get_parameter(name).value for name in defaults}
        c = self.cfg
        if (not math.isfinite(c['max_age_s']) or c['max_age_s'] <= 0
                or not c['target_frame'] or not c['expected_source_frame']
                or c['target_frame'] == c['expected_source_frame']):
            raise ValueError('Require finite positive max_age_s and distinct nonempty source/target frames')
        t, q = c['mount_translation'], c['mount_quaternion']
        if (len(t) != 3 or len(q) != 4 or not all(math.isfinite(v) for v in [*t, *q])
                or abs(sum(v*v for v in q)-1.) > 1e-6):
            raise ValueError('Mount requires finite xyz and a unit quaternion [x,y,z,w]')
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.broadcaster = None
        if c['publish_mount_tf']:
            if not c['mount_child_frame'] or c['mount_child_frame'] in (c['target_frame'], c['expected_source_frame']):
                raise ValueError('Mount child must be a distinct mechanical camera frame')
            self.broadcaster = StaticTransformBroadcaster(self)
            mount = TransformStamped()
            mount.header.stamp = self.get_clock().now().to_msg()
            mount.header.frame_id, mount.child_frame_id = c['target_frame'], c['mount_child_frame']
            mount.transform.translation.x, mount.transform.translation.y, mount.transform.translation.z = t
            (mount.transform.rotation.x, mount.transform.rotation.y,
             mount.transform.rotation.z, mount.transform.rotation.w) = q
            self.broadcaster.sendTransform(mount)
        if not c['extrinsics_calibrated']:
            self.get_logger().warning('Mount calibration UNCONFIRMED; base positions remain invalid')
        self.latest = None
        self.received_at = -math.inf
        self.pub = self.create_publisher(TargetState, 'target_state_base', 10)
        self.marker_pub = self.create_publisher(Marker, 'target_marker_base', 10)
        self.create_subscription(TargetState, 'target_state', self.on_state, 10)
        self.create_timer(.05, self.tick)

    def on_state(self, msg):
        self.latest = msg
        self.received_at = time.monotonic()

    def tick(self):
        msg = copy.deepcopy(self.latest) if self.latest is not None else TargetState()
        now = self.get_clock().now()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = self.cfg['target_frame']
        msg.source = 'yolo'
        msg.position_valid = False
        msg.position.x = msg.position.y = msg.position.z = math.nan
        msg.horizontal_distance_m = msg.bearing_rad = math.nan
        msg.measurement_age_s = math.nan
        original = self.latest
        age = (now.nanoseconds*1e-9-stamp_seconds(original.observation_stamp)
               if original is not None else math.inf)
        if original is None or time.monotonic()-self.received_at > self.cfg['max_age_s']:
            msg.status, msg.detail = TargetState.STALE, 'No fresh source state'
        elif original.source != 'yolo' or original.is_simulated:
            msg.status, msg.detail = TargetState.NOT_READY, 'Expected real YOLO source'
        elif not original.position_valid or original.status != TargetState.TRACKING:
            msg.detail = 'Base position unavailable: '+original.detail
        elif stamp_seconds(original.observation_stamp) <= 0 or not 0 <= age <= self.cfg['max_age_s']:
            msg.status, msg.detail = TargetState.STALE, 'Expired, future or unknown observation time'
        elif not self.cfg['extrinsics_calibrated']:
            msg.status, msg.detail = TargetState.NOT_READY, 'Mount calibration UNCONFIRMED (identity placeholder)'
        elif original.header.frame_id != self.cfg['expected_source_frame']:
            msg.status, msg.detail = TargetState.NOT_READY, 'Unexpected source optical frame'
        elif not all(math.isfinite(v) for v in (original.position.x, original.position.y, original.position.z)):
            msg.status, msg.detail = TargetState.NOT_READY, 'Non-finite optical position'
        else:
            try:
                # Non-blocking lookup at observation time, never substitute latest TF.
                transform = self.buffer.lookup_transform(
                    self.cfg['target_frame'], original.header.frame_id,
                    Time.from_msg(original.observation_stamp))
                point = PointStamped()
                point.header.frame_id = original.header.frame_id
                point.header.stamp = original.observation_stamp
                point.point = original.position
                result = do_transform_point(point, transform).point
                if not all(math.isfinite(v) for v in (result.x, result.y, result.z)):
                    raise ValueError('Non-finite transformed position')
                msg.position = result
                msg.position_valid = True
                msg.horizontal_distance_m = math.hypot(result.x, result.y)
                msg.bearing_rad = math.atan2(result.y, result.x)  # positive to the left
                msg.measurement_age_s = age
                msg.status, msg.detail = TargetState.TRACKING, 'Calibrated base position at observation time'
            except (TransformException, ValueError) as exc:
                msg.status, msg.detail = TargetState.NOT_READY, 'TF unavailable: '+str(exc)
        self.pub.publish(msg)
        marker = Marker()
        marker.header = copy.deepcopy(msg.header)
        marker.ns, marker.id, marker.action = 'yolo_target_base', 0, Marker.DELETE
        if msg.position_valid:
            marker.header.stamp = copy.deepcopy(msg.observation_stamp)
            marker.type, marker.action = Marker.SPHERE, Marker.ADD
            marker.pose.position = copy.deepcopy(msg.position)
            marker.pose.orientation.w = 1.
            marker.scale.x = marker.scale.y = marker.scale.z = .15
            marker.color.b, marker.color.a = 1., 1.
            marker.lifetime.nanosec = 200000000
        self.marker_pub.publish(marker)
        return msg


def main(args=None):
    rclpy.init(args=args)
    node = TargetTransform()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
