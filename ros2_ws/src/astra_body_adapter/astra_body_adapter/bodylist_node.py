"""Opt-in adapter from WheelTec ``Bodylist`` messages to ``TargetState``.

The vendor message has no header or joint confidence fields.  This adapter
therefore leaves ``observation_stamp`` at zero and reports confidence and
measurement age as NaN instead of inventing sensor metadata.
"""

from dataclasses import dataclass
import math
from typing import Optional, Sequence


LEFT_SHOULDER = 2
LEFT_HAND = 4
RIGHT_SHOULDER = 5
RIGHT_HAND = 7
BASE_SPINE = 9


@dataclass(frozen=True)
class TrackingResult:
    status: str
    detail: str
    target_id: str = ''
    position_valid: bool = False
    x_m: float = math.nan
    y_m: float = math.nan
    z_m: float = math.nan


def is_akimbo(body) -> bool:
    """Match the vendor's lock gesture using its millimetre world positions."""
    joints = body.joints
    if len(joints) <= BASE_SPINE:
        return False
    left_hand = joints[LEFT_HAND].worldposition
    right_hand = joints[RIGHT_HAND].worldposition
    left_shoulder = joints[LEFT_SHOULDER].worldposition
    right_shoulder = joints[RIGHT_SHOULDER].worldposition
    base = joints[BASE_SPINE].worldposition
    values = (
        left_hand.x, left_hand.y, right_hand.x, right_hand.y,
        left_shoulder.x, left_shoulder.y,
        right_shoulder.x, right_shoulder.y, base.y,
    )
    if not all(math.isfinite(value) for value in values):
        return False
    return (
        left_hand.y - base.y > 50.0
        and right_hand.y - base.y > 50.0
        and abs(left_shoulder.x - left_hand.x) < 100.0
        and abs(right_shoulder.x - right_hand.x) < 100.0
        and right_shoulder.y - right_hand.y > 50.0
        and left_shoulder.y - left_hand.y > 50.0
    )


class BodylistTracker:
    """Small state machine that acquires or changes target only by gesture."""

    def __init__(self, invert_y: bool = True):
        self.invert_y = invert_y
        self.locked_id: Optional[int] = None

    def process(self, bodies: Sequence) -> TrackingResult:
        # Preserve the vendor behavior that an explicit akimbo gesture selects a
        # person, including switching away from a previously locked/lost ID.
        gesture_body = next((body for body in bodies if is_akimbo(body)), None)
        if gesture_body is not None:
            self.locked_id = int(gesture_body.bodyid)

        if self.locked_id is None:
            return TrackingResult(
                status='SEARCHING',
                detail='body stream active; waiting for an akimbo lock gesture',
            )

        target_id = str(self.locked_id)
        target = next(
            (body for body in bodies if int(body.bodyid) == self.locked_id),
            None,
        )
        if target is None:
            return TrackingResult(
                status='LOST',
                detail='locked body ID is absent from the current body list',
                target_id=target_id,
            )

        center = target.centerofmass
        raw = (float(center.x), float(center.y), float(center.z))
        if not all(math.isfinite(value) for value in raw) or raw[2] <= 0.0:
            return TrackingResult(
                status='TRACKING',
                detail='locked body is present but its centre of mass is invalid',
                target_id=target_id,
            )

        y_sign = -1.0 if self.invert_y else 1.0
        return TrackingResult(
            status='TRACKING',
            detail='locked body observed in the current body list',
            target_id=target_id,
            position_valid=True,
            x_m=raw[0] / 1000.0,
            y_m=y_sign * raw[1] / 1000.0,
            z_m=raw[2] / 1000.0,
        )


def main(args=None):
    # Delay both ROS and vendor imports so the readiness-only entry point and
    # pure state-machine tests remain usable without the proprietary workspace.
    import rclpy
    from rclpy.node import Node
    from person_interfaces.msg import TargetState
    from visualization_msgs.msg import Marker
    from sensor_msgs.msg import Image

    try:
        from bodyreader_msg.msg import Bodylist
        from bodyreader_msg.msg import Maskdata
    except ImportError as exc:
        raise RuntimeError(
            'bodylist_adapter requires bodyreader_msg from the WheelTec '
            'bodyreader workspace; source that install space before starting it'
        ) from exc

    class BodylistAdapterNode(Node):
        def __init__(self):
            super().__init__('astra_bodylist_adapter')
            self.declare_parameter('bodylist_topic', '/bodylist')
            self.declare_parameter('frame_id', 'astra_depth_optical_frame')
            self.declare_parameter('stale_timeout_s', 0.5)
            self.declare_parameter('invert_sdk_y', True)

            topic = self.get_parameter('bodylist_topic').value
            self.frame_id = self.get_parameter('frame_id').value
            self.stale_timeout_s = float(
                self.get_parameter('stale_timeout_s').value)
            if not topic or not self.frame_id:
                raise ValueError('bodylist_topic and frame_id must not be empty')
            if not math.isfinite(self.stale_timeout_s) or self.stale_timeout_s <= 0.0:
                raise ValueError('stale_timeout_s must be positive')

            self.tracker = BodylistTracker(
                invert_y=bool(self.get_parameter('invert_sdk_y').value))
            self.start_ns = self.get_clock().now().nanoseconds
            self.last_input_ns = None
            self.publisher = self.create_publisher(
                TargetState, 'target_state', 10)
            self.marker_publisher = self.create_publisher(
                Marker, 'target_marker', 10)
            self.box_publisher = self.create_publisher(
                Marker, 'detection_box', 10)
            self.mask_publisher = self.create_publisher(
                Image, 'body_mask_image', 10)
            self.subscription = self.create_subscription(
                Bodylist, topic, self._bodylist_callback, 1)
            self.mask_subscription = self.create_subscription(
                Maskdata, '/body/mask', self._mask_callback, 1)
            self.timer = self.create_timer(
                min(self.stale_timeout_s / 2.0, 0.5), self._watchdog)
            self.get_logger().info(
                f'adapting {topic} only; no vehicle command publisher is created')

        def _mask_callback(self, msg):
            values = list(msg.data)
            # WheelTec output_body_mask downsamples the SDK mask by 2 on
            # both axes; Maskdata carries exactly 76800 pixels (320 x 240).
            if len(values) != 320 * 240:
                return
            image = Image()
            image.header.stamp = self.get_clock().now().to_msg()
            image.header.frame_id = self.frame_id
            image.height = 240
            image.width = 320
            image.encoding = 'mono8'
            image.is_bigendian = False
            image.step = 320
            image.data = bytes(255 if int(value) else 0 for value in values)
            self.mask_publisher.publish(image)

        def _publish(self, result, now):
            msg = TargetState()
            msg.header.stamp = now.to_msg()
            msg.header.frame_id = (
                self.frame_id if result.position_valid else '')
            msg.source = 'astra'
            msg.is_simulated = False
            msg.status = getattr(TargetState, result.status)
            msg.detail = result.detail
            msg.target_id = result.target_id
            msg.position_valid = result.position_valid
            msg.position.x = result.x_m
            msg.position.y = result.y_m
            msg.position.z = result.z_m
            if result.position_valid:
                msg.horizontal_distance_m = math.hypot(result.x_m, result.z_m)
                msg.bearing_rad = math.atan2(result.x_m, result.z_m)
            else:
                msg.horizontal_distance_m = math.nan
                msg.bearing_rad = math.nan
            # Bodylist has no source stamp or confidence.  Receipt/publication
            # time is deliberately not substituted for either value.
            msg.measurement_age_s = math.nan
            msg.confidence = math.nan
            self.publisher.publish(msg)
            self._publish_marker(result, now)
            self._publish_box(result, now)

        def _publish_marker(self, result, now):
            marker = Marker()
            marker.header.stamp = now.to_msg()
            marker.header.frame_id = self.frame_id
            marker.ns = 'person_target'
            marker.id = 0
            marker.type = Marker.SPHERE
            if not result.position_valid:
                marker.action = Marker.DELETE
                self.marker_publisher.publish(marker)
                return
            marker.action = Marker.ADD
            marker.pose.position.x = result.x_m
            marker.pose.position.y = result.y_m
            marker.pose.position.z = result.z_m
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.25
            marker.scale.y = 0.25
            marker.scale.z = 0.25
            marker.color.r = 0.1
            marker.color.g = 0.9
            marker.color.b = 0.2
            marker.color.a = 0.9
            marker.lifetime.sec = 1
            self.marker_publisher.publish(marker)

        def _publish_box(self, result, now):
            box = Marker()
            box.header.stamp = now.to_msg()
            box.header.frame_id = self.frame_id
            box.ns = 'person_detection'
            box.id = 0
            box.type = Marker.CUBE
            box.pose.orientation.w = 1.0
            if not result.position_valid:
                box.action = Marker.DELETE
                self.box_publisher.publish(box)
                return
            box.action = Marker.ADD
            box.pose.position.x = result.x_m
            box.pose.position.y = result.y_m
            box.pose.position.z = result.z_m
            # Approximate person volume for Foxglove 3D visualization; this
            # is not a calibrated 2D image bounding box.
            box.scale.x = 0.6
            box.scale.y = 1.7
            box.scale.z = 0.4
            box.color.r = 0.1
            box.color.g = 0.6
            box.color.b = 1.0
            box.color.a = 0.25
            box.lifetime.sec = 1
            self.box_publisher.publish(box)

        def _bodylist_callback(self, msg):
            now = self.get_clock().now()
            self.last_input_ns = now.nanoseconds
            count = int(msg.count)
            if count < 0 or count > len(msg.bodies):
                target_id = (
                    '' if self.tracker.locked_id is None
                    else str(self.tracker.locked_id))
                result = TrackingResult(
                    status='LOST' if target_id else 'SEARCHING',
                    detail=f'invalid body count {count}',
                    target_id=target_id,
                )
            else:
                result = self.tracker.process(msg.bodies[:count])
            self._publish(result, now)

        def _watchdog(self):
            now = self.get_clock().now()
            reference_ns = (
                self.start_ns if self.last_input_ns is None
                else self.last_input_ns)
            if ((now.nanoseconds - reference_ns) / 1e9
                    <= self.stale_timeout_s):
                return
            target_id = (
                '' if self.tracker.locked_id is None
                else str(self.tracker.locked_id))
            self._publish(
                TrackingResult(
                    status='STALE',
                    detail='body list input has not arrived within the timeout',
                    target_id=target_id,
                ),
                now,
            )

    rclpy.init(args=args)
    node = None
    try:
        node = BodylistAdapterNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
