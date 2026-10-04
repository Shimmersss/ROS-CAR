"""ROS prior gating regressions: individual identity, stable standing, and reset."""
import sys
from pathlib import Path
import rclpy
from sensor_msgs.msg import CameraInfo
from person_interfaces.msg import PersonStateArray
sys.path.insert(0,str(Path(__file__).resolve().parent))
from test_ground_runtime import make, mount, synthetic, to_state, static_tf, FRAME

rclpy.init()
n=make('prior',True,True,enable_height_prior=True,height_prior_confirmed=True,
       height_prior_track_id='0:7',standing_stable_s=.1)
try:
    info=CameraInfo();info.header.frame_id=FRAME;info.width=640;info.height=480
    info.p=[500.,0.,320.,0.,0.,500.,240.,0.,0.,0.,1.,0.]
    n.on_info(info);t=mount();n.buffer.set_transform_static(static_tf(t),'test')
    base=n.get_clock().now().nanoseconds-int(.5e9)
    def send(offset,identity='0:7',posture=1):
        msg=PersonStateArray(valid=True);msg.header.frame_id=FRAME
        msg.header.stamp=rclpy.time.Time(nanoseconds=base+int(offset*1e9)).to_msg()
        msg.persons=[to_state(synthetic(t,ankles=False,box_bottom=480.,posture=posture),identity)]
        n.on_persons(msg)
        return n.latest.persons[0]
    assert not send(0.).valid
    p=send(.12);assert p.valid and p.method=='height_prior'
    assert not send(.14,posture=0).valid, 'UNKNOWN reused standing-prior prediction'
    assert not send(.16).valid, 'Standing stability was not reset'
    assert send(.28).valid
    assert not send(.30,'1:7').valid
    assert not send(.42,'1:7').valid, 'Prior leaked across track epoch'
    print('PASS ROS individual prior: confirmation identity, standing dwell, UNKNOWN invalidation, epoch reset')
finally:
    n.destroy_node();rclpy.shutdown()
