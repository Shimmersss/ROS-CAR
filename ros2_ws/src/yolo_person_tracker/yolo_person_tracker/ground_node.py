"""Monocular ground-contact localization of tracked people; observation only, never commands."""
from dataclasses import replace
import copy
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo
from tf2_ros import Buffer, TransformListener, TransformException
from visualization_msgs.msg import Marker, MarkerArray
from person_interfaces.msg import PersonGround, PersonGroundArray, PersonStateArray, TargetState
from .ground import GroundConfig, GroundTrackFilter, camera_model_from_info, estimate_ground_point
from .input_contract import stamp_seconds

# Growth of the declared std while only predicting; speed is unknown without a measurement.
PREDICTION_SPEED_STD_MPS = 1.0
PRUNE_AGE_S = 2.0



class GroundLocalizer(Node):
    NODE_NAME = 'ground_localizer'
    PERSON_TOPIC = 'person_ground_states'
    MARKER_TOPIC = 'ground_markers'
    TARGET_TOPIC = 'target_state_ground'
    SOURCE = 'yolo_ground'
    DESCRIPTION = 'Monocular ground contact; unvalidated against physical measurements'
    def __init__(self, **kwargs):
        super().__init__(self.NODE_NAME, **kwargs)
        defaults = dict(
            target_frame='base_link', expected_source_frame='camera_color_optical_frame',
            camera_info_topic='/camera/color/camera_info',
            extrinsics_calibrated=False, ground_plane_confirmed=False,
            max_age_s=.5, position_hold_s=.25,
            kalman_acceleration_std_mps2=2., jump_reset_m=1.,
            height_prior_track_id='', standing_stable_s=.3, reid_lock_enabled=False)
        defaults.update(vars(GroundConfig()))
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.cfg = {name: self.get_parameter(name).value for name in defaults}
        c = self.cfg
        self.config = GroundConfig(**{name: c[name] for name in vars(GroundConfig())})
        for name in ('max_age_s', 'kalman_acceleration_std_mps2', 'jump_reset_m', 'standing_stable_s'):
            if not isinstance(c[name], (int, float)) or not math.isfinite(c[name]) or c[name] <= 0:
                raise ValueError(f'{name} must be finite and positive')
        if (not isinstance(c['position_hold_s'], (int, float))
                or not math.isfinite(c['position_hold_s']) or c['position_hold_s'] < 0):
            raise ValueError('position_hold_s must be finite and non-negative')
        if (not c['target_frame'] or not c['expected_source_frame']
                or c['target_frame'] == c['expected_source_frame']):
            raise ValueError('Require distinct non-empty source/target frames')
        if not isinstance(c['height_prior_track_id'], str):
            raise ValueError('height_prior_track_id must be an explicit track identity string')
        for name in ('extrinsics_calibrated', 'ground_plane_confirmed'):
            if not isinstance(c[name], bool):
                raise ValueError(f'{name} must be bool')
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.model = None
        self.info_error = 'Waiting for color CameraInfo'
        self.standing_since = {}
        self.seen_at = {}
        self.filters = {}
        self.obs_stamps = {}
        self.last_std = {}
        self.latest = None
        self.latest_at = -math.inf
        self.target_id = ''
        self.target_at = -math.inf
        self.identity_target_ok = False
        self.identity_target_stamp = 0.
        self.active_markers = set()
        if not (c['extrinsics_calibrated'] and c['ground_plane_confirmed']):
            self.get_logger().warning(
                'Mount extrinsics or ground plane UNCONFIRMED; no ground position is published')
        self.person_pub = self.create_publisher(PersonGroundArray, self.PERSON_TOPIC, 10)
        self.marker_pub = self.create_publisher(MarkerArray, self.MARKER_TOPIC, 10)
        self.target_pub = self.create_publisher(TargetState, self.TARGET_TOPIC, 10)
        self.create_subscription(CameraInfo, c['camera_info_topic'], self.on_info,
                                 qos_profile_sensor_data)
        self.create_subscription(PersonStateArray, 'person_states', self.on_persons, 10)
        self.create_subscription(TargetState, 'target_state', self.on_target, 10)
        self.create_timer(.05, self.tick)

    def on_info(self, msg):
        try:
            if msg.header.frame_id != self.cfg['expected_source_frame']:
                raise ValueError('CameraInfo frame differs from expected optical frame')
            self.model = camera_model_from_info(msg)
            self.info_error = ''
        except ValueError as exc:
            self.model, self.info_error = None, str(exc)

    def on_target(self, msg):
        self.target_id = msg.target_id
        self.target_at = time.monotonic()
        self.identity_target_ok = (msg.status == TargetState.TRACKING
                                   and msg.source == 'yolo' and not msg.is_simulated)
        self.identity_target_stamp = stamp_seconds(msg.header.stamp)

    def gate(self, msg, now):
        """Return (reason, transform); an empty reason means the estimate may run."""
        c = self.cfg
        if not c['extrinsics_calibrated']:
            return 'Mount calibration UNCONFIRMED (identity placeholder)', None
        if not c['ground_plane_confirmed']:
            return 'Ground plane/height UNCONFIRMED', None
        if msg.is_simulated:
            return 'Expected real person states, got simulated data', None
        if not msg.valid:
            return msg.detail or 'No valid person states', None
        stamp = stamp_seconds(msg.header.stamp)
        if stamp <= 0 or not 0 <= now-stamp <= c['max_age_s']:
            return 'Zero, future or expired observation time', None
        if msg.header.frame_id != c['expected_source_frame']:
            return 'Unexpected source optical frame', None
        if self.model is None:
            return self.info_error, None
        try:
            # Observation-time lookup only; never substitute the latest TF.
            transform = self.buffer.lookup_transform(
                c['target_frame'], msg.header.frame_id, Time.from_msg(msg.header.stamp))
        except TransformException as exc:
            return f'TF unavailable: {exc}', None
        return '', transform.transform

    def on_persons(self, msg):
        now = self.get_clock().now().nanoseconds*1e-9
        stamp = stamp_seconds(msg.header.stamp)
        out = PersonGroundArray()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.cfg['target_frame']
        out.is_simulated = msg.is_simulated
        reason, transform = self.gate(msg, now)
        if reason:
            self.standing_since.clear(); self.seen_at.clear()
            self.filters.clear(); self.obs_stamps.clear(); self.last_std.clear()
            out.detail = reason
            self.finish(out, MarkerArray(), set())
            return
        out.valid = True
        out.detail = self.DESCRIPTION
        for identity in [k for k, f in self.filters.items()
                         if f.stamp is None or f.stamp < stamp-PRUNE_AGE_S]:
            self.filters.pop(identity)
            self.obs_stamps.pop(identity, None)
            self.last_std.pop(identity, None)
        identities = {p.track_id for p in msg.persons}
        for identity in set(self.seen_at)-identities:
            self.seen_at.pop(identity, None); self.standing_since.pop(identity, None)
            self.filters.pop(identity, None); self.obs_stamps.pop(identity, None)
            self.last_std.pop(identity, None)
        for person in msg.persons:
            out.persons.append(self.locate(person, stamp, msg.header.stamp, transform))
        self.finish(out, self.markers(out), {p.track_id for p in out.persons if p.valid})

    def locate(self, person, stamp, stamp_msg, transform):
        identity = person.track_id
        result = PersonGround(track_id=identity, detail='')
        result.position.x = result.position.y = result.position.z = math.nan
        result.horizontal_distance_m = result.bearing_rad = math.nan
        result.std_m = result.confidence = result.measurement_age_s = math.nan
        previous = self.seen_at.get(identity)
        if previous is None or not 0 < stamp-previous <= self.cfg['max_age_s']:
            self.standing_since.pop(identity, None)
            self.filters.pop(identity, None)
        self.seen_at[identity] = stamp
        if person.posture == 1 and person.fall_stage == 0:
            self.standing_since.setdefault(identity, stamp)
        else:
            self.standing_since.pop(identity, None)
        stable = stamp-self.standing_since.get(identity, stamp) >= self.cfg['standing_stable_s']
        config = replace(self.config, height_prior_confirmed=(
            self.config.height_prior_confirmed and stable
            and identity == self.cfg['height_prior_track_id']))
        try:
            estimate, reason = estimate_ground_point(
                person, self.model[0], self.model[1], transform, config)
        except ValueError as exc:
            estimate, reason = None, str(exc)
        track = self.filters.get(identity)
        if estimate is not None:
            track = self.filters.setdefault(identity, GroundTrackFilter(
                self.cfg['kalman_acceleration_std_mps2'], self.cfg['jump_reset_m']))
            smoothed = track.update_ground(estimate.point, stamp, estimate.std_m)
            if smoothed is not None:
                track.ground_method = estimate.method
                self.obs_stamps[identity] = copy.deepcopy(stamp_msg)
                self.last_std[identity] = estimate.std_m
                result.valid, result.method = True, estimate.method
                result.std_m, result.confidence = estimate.std_m, estimate.confidence
                result.measurement_age_s = 0.
                result.detail = estimate.detail
                self.fill_position(result, smoothed)
                return result
            reason = 'Non-finite or non-monotonic measurement'
        # Contradictory evidence or invalid posture invalidates the reference,
        # rather than hiding it with a previously plausible prediction.
        hard_invalid = (person.posture not in (0, 1, 2) or person.fall_stage != 0
                        or 'conflicting' in (reason or '')
                        or (track is not None and track.ground_method == 'height_prior'
                            and person.posture != 1))
        now = self.get_clock().now().nanoseconds*1e-9
        limit = min(self.cfg['position_hold_s'], self.cfg['max_age_s'])
        if hard_invalid:
            self.filters.pop(identity, None)
        held = (track.hold(stamp, limit) if track is not None and not hard_invalid
                and track.stamp is not None and 0 <= now-track.stamp <= limit else None)
        if held is not None and identity in self.last_std:
            point, age = held
            result.valid, result.method = True, 'predicted'
            result.std_m = math.hypot(self.last_std[identity], PREDICTION_SPEED_STD_MPS*age)
            result.measurement_age_s = age
            result.detail = f'Prediction from last monocular measurement: {reason}'
            self.fill_position(result, point)
        else:
            result.detail = reason or 'monocular ground estimation unavailable'
        return result

    def fill_position(self, result, point):
        result.position.x, result.position.y = float(point[0]), float(point[1])
        result.position.z = float(self.config.ground_z_m)
        result.horizontal_distance_m = math.hypot(point[0], point[1])
        result.bearing_rad = math.atan2(point[1], point[0])  # positive to the left

    def markers(self, out):
        markers = MarkerArray()
        for person in out.persons:
            if not person.valid:
                continue
            marker = Marker()
            marker.header = copy.deepcopy(out.header)
            marker.ns, marker.id = f'ground/{person.track_id}', 0
            marker.type, marker.action = Marker.CYLINDER, Marker.ADD
            marker.pose.position = copy.deepcopy(person.position)
            marker.pose.orientation.w = 1.
            marker.scale.x = marker.scale.y = max(.2, 2.*person.std_m)
            marker.scale.z = .02
            locked = person.track_id == self.target_id
            marker.color.r, marker.color.g, marker.color.b = (0., 1., 0.) if locked else (1., .6, 0.)
            marker.color.a = .8
            marker.lifetime.nanosec = 500000000
            markers.markers.append(marker)
        return markers

    def finish(self, out, markers, current):
        for identity in self.active_markers-current:
            marker = Marker()
            marker.header = copy.deepcopy(out.header)
            marker.ns, marker.id, marker.action = f'ground/{identity}', 0, Marker.DELETE
            markers.markers.append(marker)
        self.active_markers = current
        self.latest, self.latest_at = out, time.monotonic()
        self.person_pub.publish(out)
        if markers.markers:
            self.marker_pub.publish(markers)

    def tick(self):
        c = self.cfg
        msg = TargetState()
        now = self.get_clock().now()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = c['target_frame']
        msg.source, msg.is_simulated, msg.status = self.SOURCE, False, TargetState.NOT_READY
        msg.position.x = msg.position.y = msg.position.z = math.nan
        msg.horizontal_distance_m = msg.bearing_rad = math.nan
        msg.measurement_age_s = msg.confidence = math.nan
        out = self.latest
        fresh_target = time.monotonic()-self.target_at <= c['max_age_s']
        msg.target_id = self.target_id if fresh_target else ''
        if not (c['extrinsics_calibrated'] and c['ground_plane_confirmed']):
            msg.detail = 'Mount calibration or ground plane UNCONFIRMED'
        elif out is None or time.monotonic()-self.latest_at > c['max_age_s']:
            msg.status, msg.detail = TargetState.STALE, 'No fresh person states'
        elif out.is_simulated:
            msg.detail = 'Expected real person states'
        elif not out.valid:
            msg.detail = out.detail
        elif not msg.target_id:
            msg.status, msg.detail = TargetState.SEARCHING, 'No locked target id'
        elif c['reid_lock_enabled'] and (not self.identity_target_ok
                or not 0 <= now.nanoseconds*1e-9-self.identity_target_stamp <= c['max_age_s']):
            msg.status, msg.detail = TargetState.LOST, 'Waiting for confirmed identity lock'
        else:
            person = next((p for p in out.persons if p.track_id == msg.target_id), None)
            if person is None or not person.valid:
                msg.status = TargetState.LOST
                msg.detail = 'Locked track absent' if person is None else person.detail
            else:
                observation = self.obs_stamps[person.track_id]
                age = now.nanoseconds*1e-9-stamp_seconds(observation)
                limit = (min(c['position_hold_s'], c['max_age_s'])
                         if person.method == 'predicted' else c['max_age_s'])
                if not 0 <= age <= limit:
                    msg.status, msg.detail = TargetState.STALE, 'Expired or future observation'
                else:
                    msg.observation_stamp = copy.deepcopy(observation)
                    msg.status, msg.position_valid = TargetState.TRACKING, True
                    msg.position = copy.deepcopy(person.position)
                    msg.horizontal_distance_m = person.horizontal_distance_m
                    msg.bearing_rad = person.bearing_rad
                    msg.measurement_age_s = age
                    msg.confidence = person.confidence
                    msg.detail = f'Ground contact point ({person.method}); not the RGB-D target point'
        self.target_pub.publish(msg)
        return msg


def main(args=None):
    rclpy.init(args=args)
    node = GroundLocalizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
