"""Real ROS fall -> SayText bridge; synthetic alarms, no physical fall acceptance."""
import time
import rclpy
from rclpy.node import Node
from person_interfaces.msg import FallAlarm
from roscar_interfaces.srv import SayText
from roscar_api.announcements import PhaseAnnouncements
from rclpy.executors import SingleThreadedExecutor


def main():
    rclpy.init();io=Node('fall_speech_test_io');ann=PhaseAnnouncements()
    ex=SingleThreadedExecutor();ex.add_node(io);ex.add_node(ann)
    spoken=[]
    def say(req,res):
        spoken.append(req.text);res.success=True;res.status='completed';return res
    io.create_service(SayText,'/voice/say',say)
    pub=io.create_publisher(FallAlarm,'/perception/fall_alarm',10)
    def spin(seconds=.15):
        end=time.monotonic()+seconds
        while time.monotonic()<end:ex.spin_once(timeout_sec=.01)
    def send(stage,valid=True,identity='0:7',continued=False,age=0.):
        msg=FallAlarm(valid=valid,stage=stage,track_id=identity,continued=continued)
        ns=io.get_clock().now().nanoseconds-int(age*1e9)
        msg.header.stamp.sec,msg.header.stamp.nanosec=divmod(ns,10**9)
        msg.header.frame_id='camera_color_optical_frame'
        pub.publish(msg);spin();return msg
    try:
        spin(.4)
        send(2,age=2.)  # expired/future alarms must never be spoken
        send(2,age=-2.);assert not spoken
        send(1);assert spoken==['疑似跌倒，请查看']
        send(1);send(2);assert spoken==['疑似跌倒，请查看','检测到跌倒，请查看']
        send(2,identity='0:8',continued=True);send(0,valid=False);send(2)
        assert len(spoken)==2, 'handover/continuation/invalid gap re-announced one fall'
        for _ in range(15):send(0)
        send(2);assert spoken.count('检测到跌倒，请查看')==2
        assert io.count_publishers('/cmd_vel')==0
        print('PASS fresh rising fall stages -> unique SayText; stale/future rejection; ID/continued/invalid-gap dedup; recovery allows new fall; no motion')
    finally:
        ex.shutdown();ann.destroy_node();io.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
