"""Launch defaults and fail-closed process supervision, without physical devices."""
import os
import signal
import subprocess
import tempfile
import time
import rclpy
from rclpy.node import Node
from person_interfaces.msg import TargetState


def main():
    rclpy.init();probe=Node('route_a_launch_probe');states=[]
    probe.create_subscription(TargetState,'/perception/target_state',states.append,10)
    base=['ros2','launch','perception_bringup','route_a.launch.py']
    with tempfile.TemporaryFile(mode='w+') as log:
        child=subprocess.Popen(base,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        try:
            until=time.monotonic()+8
            while time.monotonic()<until and len(states)<3:
                assert child.poll() is None
                rclpy.spin_once(probe,timeout_sec=.1)
            assert len(states)>=3
            assert all(s.source=='red_object' and s.status==TargetState.NOT_READY for s in states)
            topics=dict(probe.get_topic_names_and_types())
            assert '/cmd_vel' not in topics and '/bodylist' not in topics
            assert '/odom' not in topics
        finally:
            if child.poll() is None:
                os.killpg(child.pid,signal.SIGINT)
                try: child.wait(timeout=6)
                except subprocess.TimeoutExpired: os.killpg(child.pid,signal.SIGKILL);child.wait()
    for arguments in (['motion_enabled:=true'],['with_chassis:=true']):
        result=subprocess.run(base+arguments,capture_output=True,timeout=8)
        assert result.returncode != 0
    # A driver that fails to open must bring down perception and follower too.
    result=subprocess.run(base+['with_chassis:=true','car_mode:=mini_mec',
                                'serial_port:=/dev/roscar_nonexistent_test'],
                          capture_output=True,text=True,timeout=8)
    assert 'Serial device missing' in result.stdout+result.stderr
    probe.destroy_node();rclpy.shutdown()
    print('PASS Route A launch: red default/no skeleton/no chassis; invalid enable rejected; driver failure shuts down stack')


if __name__=='__main__': main()
