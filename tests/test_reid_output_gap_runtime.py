"""Real ReID node, timed synthetic worker results; no hardware or model accuracy claim."""
from concurrent.futures import Future
import time
import numpy as np
import rclpy
from rclpy.parameter import Parameter
from rclpy.time import Time
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import Header
from person_interfaces.msg import PersonIdentityArray,PersonIdentity,PersonStateArray
from yolo_person_tracker.reid_node import ReIDNode
from yolo_person_tracker.owner_selection import OwnerSelection
from types import SimpleNamespace

rclpy.init()
node=ReIDNode(backend=object(),parameter_overrides=[Parameter(k,value=v) for k,v in dict(
    enabled=True,use_sim_time=True,max_age_s=.5,max_gap_s=1.,forget_released_owner=True).items()])
for timer in node.timers:timer.cancel()
ex=SingleThreadedExecutor();ex.add_node(node);messages=[]
owner=OwnerSelection(reacquire_only_after_loss=True,reacquire_frames=3,reacquire_min_s=.4)
def receive(message):
    messages.append(message)
    owner.observe(message.header.stamp.sec*10**9+message.header.stamp.nanosec,
                  message.header.frame_id,
                  [(p.track_id,p.person_id,p.verified,p.visible) for p in message.persons],
                  node.get_clock().now().nanoseconds,valid=message.valid)
node.create_subscription(PersonIdentityArray,'person_identities',receive,10)
A=np.array([1.,0.,0.])


def clock(stamp):node.get_clock().set_ros_time_override(Time(nanoseconds=round(stamp*1e9)))
def drain(seconds=.06):
    end=time.monotonic()+seconds
    while time.monotonic()<end:ex.spin_once(timeout_sec=.005)
def result(stamp,generation):
    owner.update([SimpleNamespace(track_id=7,box=(10,10,60,100),confidence=.9)],100,stamp)
    owner.record_frame(round(stamp*1e9),'camera_color_optical_frame')
    if owner.target_id is None and not owner.owner_id:assert owner.lock(now=stamp)[0]
    header=Header(frame_id='camera_color_optical_frame',stamp=Time(nanoseconds=round(stamp*1e9)).to_msg())
    return header,{'0:7':A},{'0:7':'controlled fresh descriptor'},20.,generation
def finish(stamp,now=None):
    node.future=Future();node.future.set_result(result(stamp,node.generation))
    clock(stamp+.25 if now is None else now);node.tick();drain()


try:
    drain(.3)
    finish(10.)
    assert messages[-1].valid and messages[-1].persons[0].state=='PENDING'
    for previous,stamp in ((10.,10.35),(10.35,10.70)):
        future=Future();node.future=future;generation=node.generation
        clock(previous+.51);node.tick();drain()
        assert not messages[-1].valid and not messages[-1].persons, 'expired output must withdraw every identity'
        future.set_result(result(stamp,generation))
        clock(stamp+.25);node.tick();drain()
        assert messages[-1].valid, 'old output expiry must not cancel a different fresh worker result'
    identity=messages[-1].persons[0].person_id
    assert identity and messages[-1].persons[0].state=='REGISTERED', 'three distinct fresh results must accumulate across short output gaps'
    finish(11.05)
    assert messages[-1].persons[0].state=='VERIFIED' and messages[-1].persons[0].person_id==identity
    finish(11.40)
    assert owner.owner_id==identity, 'fresh ReID results must reach the actual owner binding logic'
    # A genuinely late worker still cannot publish or refresh associations.
    finish(11.75,now=12.45)
    assert not messages[-1].valid and not messages[-1].persons and not node.manager.bindings
    # A gap beyond max_gap_s requires fresh multi-frame association again.
    finish(13.)
    assert not messages[-1].persons[0].verified
    finish(13.35)
    assert not messages[-1].persons[0].verified
    finish(13.70)
    assert messages[-1].persons[0].person_id==identity and messages[-1].persons[0].verified
    # Invalid Pose and owner release still invalidate in-flight results immediately.
    for release in (False,True):
        future=Future();node.future=future;generation=node.generation
        if release:
            node.on_owner_focus(PersonIdentity(person_id=identity,track_id='0:7'))
            node.on_owner_focus(PersonIdentity())
            assert identity not in node.manager.galleries
        else:node.on_pose_gate(PersonStateArray(valid=False))
        assert node.generation>generation
        future.set_result(result(14.,generation));clock(14.25);node.tick();drain()
        assert not messages[-1].valid and not messages[-1].persons
    assert node.count_publishers('/cmd_vel')==0
    print('PASS ReID output gaps: stale output withdraws identities, fresh in-flight results survive, 3 fresh descriptors accumulate and owner binds, late work rejected, long gaps reconfirm, invalid pose/release cancel; no motion')
finally:
    ex.shutdown();node.destroy_node();rclpy.shutdown()
