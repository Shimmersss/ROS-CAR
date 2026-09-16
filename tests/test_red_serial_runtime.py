"""Synthetic images + real ROS follower + real C++ driver on a PTY. No hardware."""
import copy
import math
import os
import pty
import signal
import struct
import subprocess
import tempfile
import time
import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo, Image, Imu
from std_msgs.msg import Float32
from visualization_msgs.msg import Marker
from person_interfaces.msg import TargetState, RuntimeMetrics
from red_object_tracker.node import RedTrackerNode
from astra_body_adapter.person_follower_node import PersonFollowerNode


def bcc(payload):
    result=0
    for byte in payload: result ^= byte
    return result


def feedback(x=100, voltage=12000):
    payload=bytes([0x7b,0])+struct.pack('>10h', x,-20,-100,0,0,16384,0,0,-375,voltage)
    return payload+bytes([bcc(payload),0x7d])


def main():
    rclpy.init()
    executor=SingleThreadedExecutor()
    probe=Node('red_serial_probe'); executor.add_node(probe)
    master, slave=pty.openpty(); path=os.ttyname(slave)
    os.set_blocking(master,False)
    control_metrics=[]; states=[]; markers=[]; odom=[]; imu=[]; volts=[]; masks=[]; frames=[]
    for kind, topic, sink in (
        (TargetState,'/perception/target_state',states),
        (RuntimeMetrics,'/control/performance',control_metrics),
        (Marker,'/perception/target_marker',markers),
        (Image,'/perception/red_mask_image',masks),
        (Odometry,'/odom',odom),(Imu,'/imu/data_raw',imu),(Float32,'/PowerVoltage',volts)):
        probe.create_subscription(kind,topic,sink.append,10)
    command_pub=probe.create_publisher(Twist,'/cmd_vel',10)
    command=None; send_feedback=True; feedback_data=feedback(); camera_hook=None
    wire=bytearray(); last_tx=0
    output=tempfile.TemporaryFile(mode='w+')
    args=['ros2','run','turn_on_wheeltec_robot','wheeltec_robot_node','--ros-args',
          '-p',f'usart_port_name:={path}','-p','car_mode:=mini_mec']  # PTY fixture, NOT actual car model.
    driver=subprocess.Popen(args,stdout=output,stderr=subprocess.STDOUT,start_new_session=True)
    red=follower=None

    def pump(duration):
        nonlocal last_tx
        end=time.monotonic()+duration
        while time.monotonic()<end:
            assert driver.poll() is None, 'serial driver exited'
            now=time.monotonic()
            if now-last_tx >= .04:
                if send_feedback: os.write(master,feedback_data)
                if command is not None: command_pub.publish(command)
                if camera_hook is not None: camera_hook()
                last_tx=now
            executor.spin_once(timeout_sec=.005)
            try: wire.extend(os.read(master,65536))
            except BlockingIOError: pass
            while len(wire)>=11:
                if wire[0]!=0x7b: del wire[0]; continue
                frame=bytes(wire[:11])
                if frame[10]!=0x7d or bcc(frame[:9])!=frame[9]: del wire[0]; continue
                del wire[:11]
                frames.append((now,struct.unpack('>hhh',frame[3:9])))

    def latest_zero(window=.2):
        recent=[v for t,v in frames if t>time.monotonic()-window]
        assert recent and all(v==(0,0,0) for v in recent), recent

    try:
        pump(1.5)
        command=Twist();command.linear.x=.12;command.linear.y=-.02;command.angular.z=-.3
        pump(.5)
        assert any(v==(120,-20,-300) for _,v in frames[-10:]), frames[-10:]
        assert odom and imu and volts
        assert abs(odom[-1].twist.twist.linear.x-.1)<1e-5
        assert abs(odom[-1].twist.twist.linear.y+.02)<1e-5
        assert abs(imu[-1].angular_velocity.z+375*.00026644)<1e-5
        assert abs(volts[-1].data-12)<1e-5
        command.linear.x=-.1;command.angular.z=.2;pump(.4)
        assert any(v==(-100,-20,200) for _,v in frames[-8:])
        command.linear.x=10.;command.angular.z=-9.;pump(.4)
        assert any(v==(150,-20,-500) for _,v in frames[-8:])
        command.linear.x=math.nan;pump(.4);latest_zero()
        command.linear.x=.1;command.angular.z=0.;pump(.3)
        command=None;pump(.85);latest_zero()  # ROS command timeout despite feedback.
        command=Twist();command.linear.x=.1;send_feedback=False
        pump(.85);latest_zero()  # STM32 timeout despite commands.
        count=len(odom)
        corrupt=bytearray(feedback());corrupt[22]^=1
        os.write(master,b'noise'+bytes(corrupt));pump(.1)
        assert len(odom)==count, 'bad BCC published odometry'
        # Fragmented frame following junk, bad tail and a partial frame.
        broken=bytearray(feedback());broken[-1]=0
        os.write(master,bytes(broken)+feedback()[:8]);pump(.05)
        os.write(master,feedback()[8:]);pump(.1)
        assert len(odom)>count
        send_feedback=True;pump(.4)
        assert any(v==(100,0,0) for _,v in frames[-8:]), 'did not recover after valid feedback'
        extra=probe.create_publisher(Twist,'/cmd_vel',10);pump(.8);latest_zero()
        probe.destroy_publisher(extra);pump(.5)
        # A second process must fail to acquire the same physical-device lock.
        second=subprocess.run(args,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,timeout=6)
        assert second.returncode != 0 and b'already owned' in second.stdout, second.stdout
        print('PASS PTY driver: signed frames/BCC, limits, telemetry, bad/fragmented frames, timeouts, duplicate ownership',flush=True)

        command=None;probe.destroy_publisher(command_pub);pump(.5)
        red=RedTrackerNode(namespace='perception',parameter_overrides=[Parameter('depth_registered',value=True)])
        executor.add_node(red)
        follower=PersonFollowerNode(parameter_overrides=[Parameter('enabled',value=True),
                                                         Parameter('expected_source',value='red_object')])
        executor.add_node(follower)
        color_pub=probe.create_publisher(Image,red.cfg['color_topic'],10)
        depth_pub=probe.create_publisher(Image,red.cfg['depth_topic'],10)
        info_pub=probe.create_publisher(CameraInfo,red.cfg['camera_info_topic'],10)
        has_red=True;depth_value=3000;encoding='16UC1';frame='camera_optical';image_age=0.;depth_offset=0.
        def camera():
            info=CameraInfo();info.width=info.height=100;info.header.frame_id='camera_optical'
            info.p=[100.,0.,50.,0.,0.,100.,50.,0.,0.,0.,1.,0.]
            info_pub.publish(info)
            pixels=np.zeros((100,100,3),np.uint8)
            if has_red: pixels[20:80,30:70]=(0,0,255)
            color=red.bridge.cv2_to_imgmsg(pixels,'bgr8');color.header.frame_id='camera_optical'
            stamp=probe.get_clock().now().nanoseconds-int(image_age*1e9)
            color.header.stamp.sec=stamp//10**9;color.header.stamp.nanosec=stamp%10**9
            depth=red.bridge.cv2_to_imgmsg(np.full((100,100),depth_value,
                np.uint16 if encoding=='16UC1' else np.float32),encoding)
            depth.header=copy.deepcopy(color.header);depth.header.frame_id=frame
            depth_stamp=stamp+int(depth_offset*1e9)
            depth.header.stamp.sec=depth_stamp//10**9;depth.header.stamp.nanosec=depth_stamp%10**9
            color_pub.publish(color);depth_pub.publish(depth)
        camera_hook=camera;pump(1.5)
        assert states[-1].source=='red_object' and states[-1].position_valid,states[-1]
        assert abs(states[-1].position.z-3)<1e-5
        assert masks and any(m.data.count(255)>0 for m in masks)
        assert markers[-1].action==Marker.ADD
        assert any(v[0]>0 for _,v in frames[-8:]), frames[-8:]
        assert any(math.isfinite(m.control_latency_ms) and m.control_latency_ms>=0
                   for m in control_metrics), 'no valid control latency samples'
        first=states[-1].target_id
        depth_value=0.25;encoding='32FC1';pump(.4)
        assert abs(states[-1].position.z-0.25)<1e-5
        assert all(v[0]==0 for t,v in frames if t>time.monotonic()-.15)
        depth_value=0;pump(.4);assert not states[-1].position_valid;latest_zero()
        depth_value=3;has_red=False;pump(.4)
        assert states[-1].status==TargetState.LOST;latest_zero()
        pump(.9);assert states[-1].status==TargetState.SEARCHING
        has_red=True;pump(.5)
        assert states[-1].position_valid and states[-1].target_id!=first
        frame='raw_depth';pump(.4)
        assert states[-1].status==TargetState.NOT_READY;latest_zero()
        frame='camera_optical';image_age=3;pump(.4)
        assert states[-1].status==TargetState.NOT_READY;latest_zero()
        image_age=0;pump(.5)
        camera_hook=None;pump(.85)
        assert states[-1].status==TargetState.STALE and not states[-1].position_valid
        assert markers[-1].action==Marker.DELETE;latest_zero()
        # Offset beyond slop with monotonically advancing streams cannot yield a valid pair.
        depth_offset=2.;camera_hook=camera;pump(.8);latest_zero()
        camera_hook=None
        executor.remove_node(red);red.destroy_node();red=None;pump(.3)
        spoof=probe.create_publisher(TargetState,'/perception/target_state',10)
        def old_target():
            msg=TargetState();msg.header.stamp=probe.get_clock().now().to_msg()
            msg.observation_stamp=copy.deepcopy(msg.header.stamp);msg.observation_stamp.sec-=5
            msg.source='red_object';msg.status=TargetState.TRACKING;msg.position_valid=True
            msg.horizontal_distance_m=3.;msg.bearing_rad=0.;msg.measurement_age_s=0.
            spoof.publish(msg)
        camera_hook=old_target;pump(.7);latest_zero()
        print('PASS synthetic red -> follower -> PTY: mask, units, tracking, loss/relock, invalid depth/frame/time, stale replay stop',flush=True)
    except Exception:
        output.seek(0);print(output.read());raise
    finally:
        camera_hook=None
        if follower is not None: executor.remove_node(follower);follower.destroy_node()
        if red is not None: executor.remove_node(red);red.destroy_node()
        if driver.poll() is None:
            os.killpg(driver.pid,signal.SIGINT)
            try: driver.wait(timeout=4)
            except subprocess.TimeoutExpired: os.killpg(driver.pid,signal.SIGKILL);driver.wait()
        os.close(master);os.close(slave);output.close()
        executor.remove_node(probe);probe.destroy_node();rclpy.shutdown()


if __name__=='__main__': main()
