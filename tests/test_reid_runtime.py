"""Synthetic ROS lifecycle/association checks; actual model tested separately."""
import copy
import time
import threading
import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from person_interfaces.msg import PersonState,PersonStateArray,PersonIdentityArray
from yolo_person_tracker.reid_node import ReIDNode

class Backend:
    def __init__(self):self.feature=np.array([1.,0.,0.]);self.fail=False;self.gate=threading.Event();self.gate.set()
    def extract(self,crops):
        if not self.gate.wait(3.):raise RuntimeError('Test worker not released')
        if self.fail:raise RuntimeError('intentional synthetic ReID failure')
        return [self.feature for _ in crops]


def main():
    rclpy.init();backend=Backend()
    node=ReIDNode(backend=backend,namespace='perception',parameter_overrides=[
        Parameter('enabled',value=True),Parameter('sample_hz',value=50.),
        Parameter('max_age_s',value=.7)])
    missing=ReIDNode(namespace='missing',parameter_overrides=[Parameter('enabled',value=True),
        Parameter('model_path',value='/nonexistent/osnet.onnx')])
    ex=SingleThreadedExecutor();ex.add_node(node);ex.add_node(missing)
    out=[];missing_out=[]
    node.create_subscription(PersonIdentityArray,'person_identities',out.append,10)
    missing.create_subscription(PersonIdentityArray,'person_identities',missing_out.append,10)
    cp=node.create_publisher(Image,node.cfg['color_topic'],10)
    pp=node.create_publisher(PersonStateArray,'person_states',10)
    def spin(seconds=.13):
        end=time.monotonic()+seconds
        while time.monotonic()<end:ex.spin_once(timeout_sec=.01)
    def send(track='0:7',valid=True,blank=False,skew=False,simulated=False):
        image=np.random.default_rng(3).integers(0,256,(160,200,3),dtype=np.uint8)
        if blank:image[:]=0
        color=node.bridge.cv2_to_imgmsg(image,'bgr8')
        color.header.frame_id='camera_color_optical_frame';color.header.stamp=node.get_clock().now().to_msg()
        people=PersonStateArray(header=copy.deepcopy(color.header),valid=valid,is_simulated=simulated)
        if track:
            p=PersonState(track_id=track,confidence=.9,box=[20.,10.,100.,140.])
            p.keypoints_2d=[Point(x=50.,y=50.) for _ in range(17)]
            for i,xy in {5:(35.,35.),6:(85.,35.),11:(40.,85.),12:(80.,85.)}.items():
                p.keypoints_2d[i]=Point(x=xy[0],y=xy[1])
            p.keypoint_confidences=[.9]*17;people.persons=[p]
        if skew:people.header.stamp.nanosec=(people.header.stamp.nanosec+1)%1000000000
        cp.publish(color);pp.publish(people)
        if valid and not simulated and not skew and backend.gate.is_set() and not backend.fail:
            end=time.monotonic()+.6
            while time.monotonic()<end and not any(m.header==people.header for m in out):
                ex.spin_once(timeout_sec=.005)
            assert out and out[-1].header==people.header, ('Expected matching ReID result before age limit', track, len(out), out[-1] if out else None, node.last_stamp, node.last_submit, node.future, node.generation)
            spin(.025)  # Keep test sampling below the configured 50 Hz ceiling.
        else:
            spin()
        return people
    def verified():return [p for p in out[-1].persons if p.verified]
    try:
        spin(.5)
        assert missing_out and not missing_out[-1].valid and 'unavailable' in missing_out[-1].detail
        for _ in range(3):msg=send()
        assert out[-1].header==msg.header and out[-1].valid
        identity=verified()[0].person_id;assert verified()[0].track_id=='0:7'
        send(track='');assert not verified() and any(p.state=='LOST' for p in out[-1].persons)
        for i in range(3):
            send('0:19')
            assert bool(verified())==(i==2)
        assert verified()[0].person_id==identity
        backend.feature=np.array([0.,1.,0.]);send('0:19');assert not verified()
        send('0:19',blank=True);assert not verified()
        send('0:19',valid=False);assert not out[-1].valid
        send('0:19',simulated=True);assert not out[-1].valid
        backend.feature=np.array([1.,0.,0.])
        for _ in range(3):send('1:7')
        assert verified()[0].person_id==identity
        before=len([x for x in out if x.valid]);send(skew=True)
        assert len([x for x in out if x.valid])==before,'Mismatched frames were associated'
        spin(.8);assert not out[-1].valid and not out[-1].persons
        backend.gate.clear();send();assert node.future is not None
        spin(.8);assert not out[-1].valid, 'Heartbeat blocked by inference'
        backend.gate.set();spin(.2);assert not out[-1].valid, 'Stale worker result was published'
        backend.fail=True;send();assert not out[-1].valid
        topics={name for name,_ in node.get_topic_names_and_types()}
        assert '/cmd_vel' not in topics and '/perception/target_state' not in topics
        print('PASS ReID ROS: exact frames, missing model, registration, recovery, contradiction, quality, invalidation, stale, isolation')
    finally:
        backend.gate.set();ex.shutdown();node.destroy_node();missing.destroy_node();rclpy.shutdown()

if __name__=='__main__':main()
