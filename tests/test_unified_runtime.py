"""ROS unified contact output contracts with synthetic geometry; no physical accuracy claim."""
import copy
import math
import time
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import Point
from person_interfaces.msg import PersonStateArray, PersonGroundArray, TargetState
from visualization_msgs.msg import MarkerArray, Marker
from test_ground_runtime import to_state, static_tf, FRAME
from test_unified import measured
from test_ground import mount
from yolo_person_tracker.unified_node import UnifiedLocalizer


def main():
    rclpy.init()
    good=UnifiedLocalizer(namespace='unified_test',parameter_overrides=[
        Parameter('extrinsics_calibrated',value=True),Parameter('ground_plane_confirmed',value=True),
        Parameter('max_age_s',value=.5),Parameter('reid_lock_enabled',value=True)])
    uncal=UnifiedLocalizer(namespace='unified_uncal')
    ex=SingleThreadedExecutor();ex.add_node(good);ex.add_node(uncal)
    output=[];markers=[]
    good.create_subscription(PersonGroundArray,'person_positions',output.append,10)
    good.create_subscription(MarkerArray,'position_markers',markers.append,10)
    t=mount();good.buffer.set_transform_static(static_tf(t),'synthetic_fixture')
    info=CameraInfo();info.width=640;info.height=480;info.header.frame_id=FRAME
    info.p=[500.,0.,320.,0.,0.,500.,240.,0.,0.,0.,1.,0.];good.on_info(info)
    def spin(seconds=.06):
        end=time.monotonic()+seconds
        while time.monotonic()<end:ex.spin_once(timeout_sec=.005)
    def send(indices=(15,16),distance=2.,identity='0:7',valid=True,body=False,ankles=True):
        fixture=measured(t,foot=(distance,.4),indices=indices)
        p=to_state(fixture,identity)
        p.keypoints_3d=[Point(x=float(q.x),y=float(q.y),z=float(q.z)) for q in fixture.keypoints_3d]
        p.keypoints_3d_valid=list(fixture.keypoints_3d_valid)
        p.body_depth_valid=body;p.body_depth_m=float(distance-.1) if body else math.nan
        p.body_depth_source='regions' if body else ''
        if not ankles:
            p.keypoint_confidences[15]=p.keypoint_confidences[16]=0.
            p.keypoints_3d_valid[15]=p.keypoints_3d_valid[16]=False
        msg=PersonStateArray(valid=valid);msg.header.frame_id=FRAME;msg.header.stamp=good.get_clock().now().to_msg();msg.persons=[p]
        good.on_persons(msg)
        target=TargetState(target_id=identity,status=TargetState.TRACKING,source='yolo')
        target.header.stamp=copy.deepcopy(msg.header.stamp);good.on_target(target)
        result=good.tick();spin()
        return msg,result
    try:
        spin(.3)
        for _ in range(2):assert not send()[1].position_valid
        msg,result=send()
        assert result.position_valid and result.source=='yolo_unified',(result,good.latest,vars(good.tracks.get('0:7')) if good.tracks.get('0:7') else None)
        assert result.header.frame_id=='base_link' and result.observation_stamp==msg.header.stamp
        assert output[-1].persons[0].method=='fused_contact'
        assert abs(result.position.x-2.)<1e-5 and result.position.z==0.
        assert all(m.header.frame_id=='base_link' for a in markers for m in a.markers)
        # Short depth dropout does not publish a stale prediction as a new observation.
        assert send(indices=())[1].position_valid
        assert send(indices=())[1].position_valid
        assert send(indices=())[1].position_valid
        assert output[-1].persons[0].method=='mono_ankles'
        # Fresh mono continues while depth returns, then source switches.
        for _ in range(2):assert send()[1].position_valid and output[-1].persons[0].method=='fused_contact'
        assert send()[1].position_valid and output[-1].persons[0].method=='fused_contact'
        # Original body depth is mapped using earlier paired feet, then remains
        # a current measurement when the feet briefly disappear.
        for _ in range(4):send(body=True)
        _,result=send(body=True,ankles=False,distance=2.1)
        assert result.position_valid and output[-1].persons[0].method=='body_contact', result
        assert abs(result.position.x-2.1)<.02 and result.measurement_age_s<.2
        _,result=send(body=True,ankles=False,distance=2.2)
        assert result.position_valid and abs(result.position.x-2.2)<.02
        for _ in range(14):_,result=send(body=True,ankles=False,distance=2.2)
        assert not result.position_valid, 'Body-to-feet mapping must expire even with fresh body frames'
        # New epoch cannot inherit the learned torso-to-feet relation either.
        assert not send(identity='1:7',body=True,ankles=False)[1].position_valid
        # New epoch cannot inherit the previous filter/confirmation.
        assert not send(identity='1:7')[1].position_valid
        send(identity='1:7');assert send(identity='1:7')[1].position_valid
        # Identity lock invalidates only selected output, not all-person geometry.
        lost=TargetState(target_id='1:7',source='yolo',status=TargetState.LOST)
        lost.header.stamp=good.get_clock().now().to_msg();good.on_target(lost)
        assert not good.tick().position_valid and good.latest.persons[0].valid
        spin(.6);assert not good.tick().position_valid
        assert not output[-1].valid and not output[-1].persons
        assert any(m.action==Marker.DELETE for a in markers for m in a.markers)
        # Far depth exceeds configured range but a fresh mono estimate can reacquire.
        for _ in range(3):msg,result=send(distance=6.)
        assert result.position_valid and output[-1].persons[0].method=='mono_ankles'
        uncal.on_persons(msg);assert not uncal.tick().position_valid
        msg.is_simulated=True;good.on_persons(msg);assert not good.tick().position_valid
        assert not any(n=='/cmd_vel' for n,_ in good.get_topic_names_and_types())
        print('PASS unified ROS: depth/mono/reference/header/confirmation/epoch/ReID/stale/marker/calibration/simulated gates')
    finally:
        ex.shutdown();good.destroy_node();uncal.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
