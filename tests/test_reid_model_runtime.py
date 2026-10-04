"""Actual local OSNet/OpenCV on real-photo crops in ARM64 ROS (scripted repeated frames)."""
import argparse
import copy
import json
from pathlib import Path
import time
import cv2
import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from person_interfaces.msg import PersonState,PersonStateArray,PersonIdentityArray
from yolo_person_tracker.reid_backend import OSNetBackend
from yolo_person_tracker.reid_node import ReIDNode

p=argparse.ArgumentParser(description=__doc__)
p.add_argument('--model',required=True);p.add_argument('--evidence',type=Path,required=True)
a=p.parse_args()
fixtures=json.loads((a.evidence/'fixtures.json').read_text())
reference=np.load(a.evidence/'features.npz')
backend=OSNetBackend(a.model);measurements=[]
for fixture in fixtures:
    image=cv2.imread(str(a.evidence/fixture['file']))
    start=time.perf_counter();feature=backend.extract([image])[0]
    elapsed=(time.perf_counter()-start)*1000
    error=float(np.max(np.abs(feature-reference[fixture['file']])))
    assert error<1e-4,(fixture['file'],error)
    measurements.append(dict(crop=fixture['file'],cpu_ms=elapsed,max_cross_platform_error=error))
rclpy.init()
node=ReIDNode(namespace='real_reid',parameter_overrides=[Parameter('enabled',value=True),
    Parameter('model_path',value=a.model),Parameter('sample_hz',value=50.),
    Parameter('max_age_s',value=1.)])
ex=SingleThreadedExecutor();ex.add_node(node);out=[]
node.create_subscription(PersonIdentityArray,'person_identities',out.append,10)
cp=node.create_publisher(Image,node.cfg['color_topic'],10)
pp=node.create_publisher(PersonStateArray,'person_states',10)
def spin(seconds):
    until=time.monotonic()+seconds
    while time.monotonic()<until:ex.spin_once(timeout_sec=.01)
def send(fixture,track):
    image=cv2.imread(str(a.evidence/fixture['file']))
    c=node.bridge.cv2_to_imgmsg(image,'bgr8');c.header.frame_id='camera_color_optical_frame'
    c.header.stamp=node.get_clock().now().to_msg()
    pose=PersonState(track_id=track,confidence=float(fixture['confidence']),
        box=[0.,0.,float(image.shape[1]),float(image.shape[0])])
    pose.keypoints_2d=[Point(x=x,y=y) for x,y,_ in fixture['keypoints']]
    pose.keypoint_confidences=[v for _,_,v in fixture['keypoints']]
    msg=PersonStateArray(header=copy.deepcopy(c.header),valid=True,persons=[pose])
    cp.publish(c);pp.publish(msg);spin(.17)
    return [v for v in out[-1].persons if v.verified]
try:
    spin(.5);assert node.backend is not None,node.error
    for _ in range(3):current=send(fixtures[0],'0:7')
    assert current,current
    identity=current[0].person_id
    for _ in range(3):current=send(fixtures[0],'0:19')
    assert current[0].person_id==identity
    if len(fixtures)>1:
        assert not send(fixtures[1],'0:19'),'Changed appearance must not inherit identity'
    print(json.dumps(dict(evidence='actual model and ROS; isolated static photo crops, NOT field tracking accuracy',
                          opencv=cv2.__version__,measurements=measurements,
                          same_crop_reassociated=True,changed_crop_rejected=len(fixtures)>1),indent=2))
finally:
    ex.shutdown();node.destroy_node();rclpy.shutdown()
