"""RGB-only Foxglove video outputs; no depth, calibration or motion."""
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from sensor_msgs.msg import Image
from person_interfaces.msg import TargetState, RuntimeMetrics
from red_object_tracker.node import RedTrackerNode

rclpy.init()
tracker=RedTrackerNode(namespace='perception');probe=Node('video_probe')
executor=SingleThreadedExecutor();executor.add_node(tracker);executor.add_node(probe)
raw=[];boxed=[];states=[];metrics=[]
probe.create_subscription(Image,'/perception/color_image',raw.append,10)
probe.create_subscription(Image,'/perception/detections_image',boxed.append,10)
probe.create_subscription(TargetState,'/perception/target_state',states.append,10)
probe.create_subscription(RuntimeMetrics,'/perception/performance',metrics.append,10)
pub=probe.create_publisher(Image,tracker.cfg['color_topic'],10)
try:
    pixels=np.zeros((100,100,3),np.uint8);pixels[30:70,30:70]=(0,0,255)
    last=None
    end=time.monotonic()+2
    while time.monotonic()<end:
        msg=tracker.bridge.cv2_to_imgmsg(pixels,'bgr8');msg.header.frame_id='camera_optical'
        msg.header.stamp=probe.get_clock().now().to_msg();pub.publish(msg);last=msg
        for _ in range(5): executor.spin_once(timeout_sec=.01)
    assert raw and boxed and states
    assert metrics and metrics[-1].input_fps>0 and metrics[-1].output_fps>0
    assert metrics[-1].processing_ms>=0 and metrics[-1].observation_age_ms>=0
    assert bytes(raw[-1].data)==pixels.tobytes()
    matching={ (m.header.stamp.sec,m.header.stamp.nanosec):m for m in raw }
    frame=boxed[-1];key=(frame.header.stamp.sec,frame.header.stamp.nanosec)
    assert key in matching and matching[key].header==frame.header
    annotated=tracker.bridge.imgmsg_to_cv2(frame,'bgr8')
    assert not np.array_equal(annotated,pixels)
    assert np.any(np.all(annotated==(0,255,255),axis=2))
    assert all(m.status==TargetState.NOT_READY and not m.position_valid for m in states)
    assert '/cmd_vel' not in dict(probe.get_topic_names_and_types())
    print('PASS RGB-only original/boxed video: original pixels, matching headers, yellow red-component boxes, no depth or motion')
finally:
    tracker.destroy_node();probe.destroy_node();rclpy.shutdown()
