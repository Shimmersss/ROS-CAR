"""Synthetic ROS acceptance for monocular ground localization; not a field accuracy claim."""
import copy
import math
import sys
import time
from pathlib import Path

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from geometry_msgs.msg import Point, TransformStamped
from sensor_msgs.msg import CameraInfo
from visualization_msgs.msg import Marker
from person_interfaces.msg import PersonState, PersonStateArray, TargetState, PersonGroundArray
from visualization_msgs.msg import MarkerArray

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'ros2_ws/src/yolo_person_tracker/test'))
from test_ground import mount, person as synthetic  # noqa: E402
from yolo_person_tracker.ground_node import GroundLocalizer  # noqa: E402

FRAME = 'camera_color_optical_frame'


def to_state(p, track_id='0:7'):
    msg = PersonState(track_id=track_id, posture=p.posture, fall_stage=p.fall_stage)
    msg.box = list(map(float, p.box))
    msg.confidence = p.confidence
    msg.keypoints_2d = [Point(x=float(k.x), y=float(k.y)) for k in p.keypoints_2d]
    msg.keypoint_confidences = [float(v) for v in p.keypoint_confidences]
    return msg


def static_tf(transform, parent='base_link', child=FRAME):
    t = TransformStamped()
    t.header.frame_id, t.child_frame_id = parent, child
    t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = (
        transform.translation.x, transform.translation.y, transform.translation.z)
    r = transform.rotation
    t.transform.rotation.x, t.transform.rotation.y = r.x, r.y
    t.transform.rotation.z, t.transform.rotation.w = r.z, r.w
    return t


def make(namespace, calibrated, confirmed, **params):
    overrides = [Parameter('extrinsics_calibrated', value=calibrated),
                 Parameter('ground_plane_confirmed', value=confirmed),
                 Parameter('max_age_s', value=1.), Parameter('position_hold_s', value=.25)]
    overrides += [Parameter(k, value=v) for k, v in params.items()]
    return GroundLocalizer(namespace=namespace, parameter_overrides=overrides)


def main():
    rclpy.init()
    good = make('good', True, True)
    uncalibrated = make('uncal', False, True)
    unconfirmed = make('unconf', True, False)
    nodes = (good, uncalibrated, unconfirmed)
    ex = SingleThreadedExecutor()
    for n in nodes:
        ex.add_node(n)
    outputs = {n.get_namespace(): ([], [], [], n) for n in nodes}
    for ns, (persons, targets, markers, n) in outputs.items():
        n.create_subscription(PersonGroundArray, 'person_ground_states', persons.append, 10)
        n.create_subscription(TargetState, 'target_state_ground', targets.append, 10)
        n.create_subscription(MarkerArray, 'ground_markers', markers.append, 10)
    persons, targets, markers, _ = outputs['/good']
    pose_pub = good.create_publisher(PersonStateArray, 'person_states', 10)
    target_pub = good.create_publisher(TargetState, 'target_state', 10)
    info_pub = good.create_publisher(CameraInfo, good.cfg['camera_info_topic'], 10)
    others = [(n.create_publisher(PersonStateArray, 'person_states', 10),
               n.create_publisher(CameraInfo, n.cfg['camera_info_topic'], 10)) for n in (uncalibrated, unconfirmed)]
    transform = mount()

    def spin(seconds=.15):
        end = time.monotonic()+seconds
        while time.monotonic() < end:
            ex.spin_once(timeout_sec=.01)

    def info(width=640):
        msg = CameraInfo()
        msg.header.frame_id, msg.width, msg.height = FRAME, width, 480
        msg.p = [500., 0., 320., 0., 0., 500., 240., 0., 0., 0., 1., 0.]
        return msg

    def now_s():
        return good.get_clock().now().nanoseconds*1e-9

    def stamp(offset=0.):
        return rclpy.time.Time(nanoseconds=int((now_s()-offset)*1e9)).to_msg()

    def send(people, offset=0., valid=True, simulated=False, track='0:7', locked='0:7', at=None):
        msg = PersonStateArray(valid=valid, is_simulated=simulated, detail='' if valid else 'stale')
        msg.header.frame_id = FRAME
        msg.header.stamp = stamp(offset) if at is None else rclpy.time.Time(
            nanoseconds=int(at*1e9)).to_msg()
        msg.persons = [to_state(p, track) for p in people]
        if locked is not None:
            target = TargetState(target_id=locked)
            target_pub.publish(target)
            spin(.05)
        pose_pub.publish(msg)
        spin()
        return msg

    try:
        spin(.2)
        info_pub.publish(info())
        for _, camera_pub in others:
            camera_pub.publish(info())
        spin()
        # Missing TF can never become a valid position.
        send([synthetic(transform)])
        assert not persons[-1].valid and 'TF unavailable' in persons[-1].detail, persons[-1].detail
        assert targets[-1].status == TargetState.NOT_READY and not targets[-1].position_valid
        good.buffer.set_transform_static(static_tf(transform), 'test')
        # Unconfirmed mount or plane stays NOT_READY even with TF and perfect input.
        for (pub, _), node in zip(others, (uncalibrated, unconfirmed)):
            node.buffer.set_transform_static(static_tf(transform), 'test')
            msg = PersonStateArray(valid=True)
            msg.header.frame_id, msg.header.stamp = FRAME, stamp()
            msg.persons = [to_state(synthetic(transform))]
            pub.publish(msg)
            spin()
            _, got_targets, _, _ = outputs[node.get_namespace()]
            got_persons = outputs[node.get_namespace()][0]
            assert not got_persons[-1].valid and 'UNCONFIRMED' in got_persons[-1].detail
            assert got_targets[-1].status == TargetState.NOT_READY
            assert math.isnan(got_targets[-1].position.x)

        message = send([synthetic(transform)])
        out = persons[-1]
        assert out.valid and out.header.frame_id == 'base_link' and out.header.stamp == message.header.stamp
        p = out.persons[0]
        assert p.valid and p.method == 'feet_ankles' and p.measurement_age_s == 0.
        assert abs(p.position.x-3.) < .02 and abs(p.position.y-.4) < .02 and p.position.z == 0.
        assert p.bearing_rad > 0 and abs(p.bearing_rad-math.atan2(.4, 3.)) < .01
        assert p.std_m > 0 and not math.isnan(p.std_m)
        assert markers[-1].markers[0].action == Marker.ADD
        goal = targets[-1]
        assert goal.status == TargetState.TRACKING and goal.source == 'yolo_ground', goal
        assert goal.position_valid and goal.header.frame_id == 'base_link' and not goal.is_simulated
        assert goal.observation_stamp == message.header.stamp and 0 <= goal.measurement_age_s < .5
        assert abs(goal.horizontal_distance_m-math.hypot(3., .4)) < .02

        # Other lock, lying posture, simulated data and invalid person arrays produce no position.
        send([synthetic(transform)], locked='0:99')
        assert targets[-1].status == TargetState.LOST and not targets[-1].position_valid
        good.filters.clear()  # A fresh track has no recent measurement to predict from.
        send([synthetic(transform, posture=3)])
        assert not persons[-1].persons[0].valid and targets[-1].status == TargetState.LOST
        send([synthetic(transform)], simulated=True)
        assert not persons[-1].valid and targets[-1].status == TargetState.NOT_READY
        send([synthetic(transform)], valid=False)
        assert not persons[-1].valid and 'stale' in persons[-1].detail
        send([synthetic(transform)], offset=3.)
        assert not persons[-1].valid and 'expired' in persons[-1].detail
        send([synthetic(transform)], offset=-3.)
        assert not persons[-1].valid

        # Real publication age, not just inter-frame time, bounds prediction.
        def direct(age, missing=False, posture=1):
            msg=PersonStateArray(valid=True)
            msg.header.frame_id=FRAME;msg.header.stamp=stamp(age)
            p=synthetic(transform, posture=posture, ankles=not missing)
            msg.persons=[to_state(p)]
            good.on_persons(msg)
            good.on_target(TargetState(target_id='0:7'))
            return good.latest.persons[0], good.tick()
        good.filters.clear();good.seen_at.clear()
        direct(.35)
        p,target=direct(.25, missing=True)
        assert not p.valid and not target.position_valid, 'Delayed prediction exceeded hold'
        good.filters.clear();good.seen_at.clear()
        p,first=direct(.03)
        p,target=direct(.01, missing=True)
        assert p.valid and p.method=='predicted' and target.position_valid
        assert target.observation_stamp==first.observation_stamp
        time.sleep(.26)
        assert not good.tick().position_valid, 'Timer continued expired prediction'
        direct(.01)
        p,target=direct(0., posture=3)
        assert not p.valid and not target.position_valid, 'Lying must clear prediction'

        # Optional identity lock must gate the selected ground output, not all-person geometry.
        direct(0.)
        good.cfg['reid_lock_enabled'] = True
        assert good.latest.persons[0].valid and not good.tick().position_valid
        identity_target=TargetState(target_id='0:7', source='yolo', status=TargetState.TRACKING)
        identity_target.header.stamp=stamp(0.)
        good.on_target(identity_target)
        assert good.tick().position_valid  # Optical depth is not required for ground geometry.
        identity_target.status=TargetState.LOST
        good.on_target(identity_target)
        assert not good.tick().position_valid
        identity_target.status=TargetState.TRACKING
        identity_target.header.stamp=stamp(1.)
        good.on_target(identity_target)
        assert not good.tick().position_valid
        good.cfg['reid_lock_enabled'] = False

        # Without fresh person states the target state expires by itself.
        time.sleep(1.1)
        spin(.2)
        assert targets[-1].status == TargetState.STALE and not targets[-1].position_valid
        # Wrong-size or non-rectified CameraInfo invalidates the model.
        bad = info()
        bad.p[3] = 5.
        info_pub.publish(bad)
        spin()
        send([synthetic(transform)])
        assert not persons[-1].valid and 'projection' in persons[-1].detail
        for n in nodes:
            assert not any(name == '/cmd_vel' for name, _ in n.get_topic_names_and_types())
        print('PASS ground ROS: gating, TF observation-time, bearing/frame, lock state, prediction, expiry')
    finally:
        ex.shutdown()
        for n in nodes:
            n.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
