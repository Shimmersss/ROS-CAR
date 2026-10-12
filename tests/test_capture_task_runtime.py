#!/usr/bin/env python3
"""Native Humble integration; isolated DDS only. Synthetic images/fake speech."""
import hashlib
import json
import os
from pathlib import Path
import tempfile
import threading
import time
import unittest
import yaml
import rclpy
from rclpy.action import ActionClient, ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Bool
from capture_task_interfaces.action import CaptureTask
from execution_mode_interfaces.action import SpeakText
from capture_task.executor_node import CaptureExecutor
from capture_task.bridge_node import CaptureBridge
from capture_task.fake_tts_node import FakeTTS
from capture_task.synthetic_camera_node import SyntheticCamera
from capture_task.mock_cloud import MockCloud


def parameters(**values):
    return [Parameter(k,value=v) for k,v in values.items()]


class CaptureRuntime(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if os.environ.get('ROS_DOMAIN_ID')!='187' or os.environ.get('ROS_LOCALHOST_ONLY')!='1':
            raise RuntimeError('Require isolated ROS_DOMAIN_ID=187 ROS_LOCALHOST_ONLY=1')
        rclpy.init();cls.temp=tempfile.TemporaryDirectory();cls.root=Path(cls.temp.name)
        data=yaml.safe_load((Path(__file__).parents[1]/'ros2_ws/src/capture_task/config/tasks.yaml').read_text())
        for task in data['tasks'].values(): task['prepare_sec']=0.
        data['defaults']['speech_timeout_sec']=.5
        cls.catalog=cls.root/'tasks.yaml';cls.catalog.write_text(yaml.safe_dump(data,allow_unicode=True))
        cls.cloud=MockCloud(('127.0.0.1',0),cls.root/'cloud')
        cls.http_thread=threading.Thread(target=cls.cloud.serve_forever,daemon=True);cls.http_thread.start()
        cls.url='http://127.0.0.1:'+str(cls.cloud.server_port)
        cls.executor=MultiThreadedExecutor(num_threads=12)
        cls.capture=CaptureExecutor(parameter_overrides=parameters(catalog=str(cls.catalog),cloud_url=cls.url,storage_dir=str(cls.root/'records')))
        cls.speaker=FakeTTS(parameter_overrides=parameters(test_only=True,char_seconds=.0001))
        cls.camera=SyntheticCamera(parameter_overrides=parameters(test_only=True))
        cls.client_node=Node('capture_runtime_client');cls.action=ActionClient(cls.client_node,CaptureTask,'/capture_task/run')
        cls.nodes=[cls.capture,cls.speaker,cls.camera,cls.client_node]
        for node in cls.nodes: cls.executor.add_node(node)
        cls.thread=threading.Thread(target=cls.executor.spin,daemon=True);cls.thread.start()
        assert cls.action.wait_for_server(timeout_sec=5.)
        assert cls.capture.speech.wait_for_server(timeout_sec=5.)
        time.sleep(1.)
        cls.counter=0

    @classmethod
    def tearDownClass(cls):
        if hasattr(cls,'bridge'):
            cls.bridge.stop()
        cls.capture.stop();time.sleep(.1)
        cls.executor.shutdown(timeout_sec=5.);cls.thread.join(5.)
        for node in reversed(cls.nodes): node.destroy_node()
        cls.cloud.stop();cls.http_thread.join(3.);rclpy.shutdown();cls.temp.cleanup()

    def setUp(self):
        self.extra=[];self.cloud.fail_upload=0;self.cloud.slow_upload=0.
        self.capture.mode={};self.speaker.char_seconds=.0001
        self.wait(lambda:self.capture.job is None)

    def tearDown(self):
        for item in self.extra:
            self.client_node.destroy_publisher(item)
        self.capture.mode={};self.speaker.char_seconds=.0001;self.cloud.fail_upload=0;self.cloud.slow_upload=0.
        self.wait(lambda:self.capture.job is None)
        time.sleep(.2)

    def wait(self,condition,timeout=8.):
        end=time.monotonic()+timeout
        while time.monotonic()<end:
            if condition(): return
            time.sleep(.02)
        self.fail('condition timeout')

    def future(self,value,timeout=8.):
        self.wait(value.done,timeout)
        return value.result()

    def goal(self,kind='helmet',ttl=30.,**params):
        type(self).counter+=1
        task_id='test-'+str(self.counter)
        goal=CaptureTask.Goal(task_id=task_id,task_type=kind,source='local_test',params_json=json.dumps(params),ttl_sec=ttl)
        return self.future(self.action.send_goal_async(goal)),task_id

    def result(self,handle):
        return self.future(handle.get_result_async(),timeout=10.).result

    def test_01_four_tasks_fresh_and_hash(self):
        for kind in ('face','tongue','helmet','pcb'):
            before=time.monotonic();handle,task_id=self.goal(kind)
            self.assertTrue(handle.accepted)
            result=self.result(handle);self.assertEqual(result.status,'uploaded',result.detail)
            record=json.loads((self.root/'records'/result.capture_id/'result.json').read_text())
            self.assertEqual(record['quality']['width'],1920)
            self.assertGreater(self.capture.frames.latest.received,before)
            image=self.root/'cloud'/task_id/(result.capture_id+'.jpg')
            self.assertEqual(hashlib.sha256(image.read_bytes()).hexdigest(),record['sha256'])
            self.assertFalse((self.root/'records'/result.capture_id/'image.jpg').exists())

    def test_02_busy_cancel_matching(self):
        handle,_=self.goal(prepare_sec=2.)
        other,_=self.goal();self.assertFalse(other.accepted)
        self.future(handle.cancel_goal_async());self.assertEqual(self.result(handle).status,'cancelled')

    def test_03_bench_velocity_gate(self):
        pub=self.client_node.create_publisher(Twist,'/cmd_vel',1);self.extra.append(pub)
        self.wait(lambda:self.capture.count_publishers('/cmd_vel')==1)
        handle,_=self.goal();result=self.result(handle)
        self.assertEqual(result.status,'failed');self.assertIn('bench_velocity',result.detail)

    def test_04_voice_mode_rejected(self):
        self.capture.mode={'active':True}
        handle,_=self.goal();self.assertFalse(handle.accepted)

    def test_05_camera_missing_and_duplicate(self):
        self.executor.remove_node(self.camera)
        self.camera.destroy_node()
        self.wait(lambda:self.capture.count_publishers('/camera/color/image_raw')==0)
        handle,_=self.goal();self.assertEqual(self.result(handle).detail,'camera_publisher_count')
        type(self).camera=SyntheticCamera(parameter_overrides=parameters(test_only=True))
        self.nodes[2]=self.camera;self.executor.add_node(self.camera)
        pub=self.client_node.create_publisher(Image,'/camera/color/image_raw',qos_profile_sensor_data);self.extra.append(pub)
        self.wait(lambda:self.capture.count_publishers('/camera/color/image_raw')==2)
        handle,_=self.goal();self.assertEqual(self.result(handle).detail,'camera_publisher_count')

    def test_06_cancel_speech_and_deadline(self):
        self.speaker.char_seconds=.05
        handle,_=self.goal();self.wait(lambda:self.capture.job and self.capture.job.speech is not None)
        self.future(handle.cancel_goal_async());self.assertEqual(self.result(handle).status,'cancelled')
        self.wait(lambda:len(self.speaker.cancelled)>0)
        self.speaker.char_seconds=.0001
        handle,_=self.goal(ttl=.1,prepare_sec=2.)
        self.assertEqual(self.result(handle).status,'failed')

    def test_07_upload_cancel_and_409(self):
        self.cloud.fail_upload=409
        handle,_=self.goal();self.assertEqual(self.result(handle).status,'uploaded')
        self.cloud.fail_upload=0;self.cloud.slow_upload=3.
        before=len(self.speaker.received)
        handle,task_id=self.goal();self.wait(lambda:self.capture.job and self.capture.job.phase=='UPLOAD')
        started=time.monotonic();self.future(handle.cancel_goal_async())
        self.assertEqual(self.result(handle).status,'cancelled')
        self.assertLess(time.monotonic()-started,1.)
        self.assertEqual(len(self.speaker.received)-before,1)
        self.assertFalse(any(json.loads(path.read_text()).get('task_id')==task_id for path in (self.root/'records').glob('*/result.json')))

    def test_08_late_speech_accept_cancelled(self):
        self.speaker.server.destroy();cancelled=threading.Event()
        def admit(_):
            time.sleep(.5);return GoalResponse.ACCEPT
        def execute(goal):
            end=time.monotonic()+1.
            while time.monotonic()<end and not goal.is_cancel_requested: time.sleep(.01)
            if goal.is_cancel_requested:
                cancelled.set();goal.canceled();return SpeakText.Result(status='cancelled')
            goal.succeed();return SpeakText.Result(status='completed')
        self.speaker.server=ActionServer(self.speaker,SpeakText,'/voice/speak',execute,goal_callback=admit,
            cancel_callback=lambda _:CancelResponse.ACCEPT,callback_group=ReentrantCallbackGroup())
        self.wait(lambda:self.capture.speech.server_is_ready())
        handle,_=self.goal(ttl=.15)
        self.assertEqual(self.result(handle).status,'failed')
        self.assertTrue(cancelled.wait(2.))
        self.speaker.server.destroy()
        self.speaker.server=ActionServer(self.speaker,SpeakText,'/voice/speak',self.speaker.execute,goal_callback=self.speaker.admit,
            cancel_callback=lambda _:CancelResponse.ACCEPT,callback_group=ReentrantCallbackGroup())
        time.sleep(.3)

    def test_09_bridge_e2e_dedup_cancel_auth_recovery_resources(self):
        type(self).bridge=CaptureBridge(parameter_overrides=parameters(catalog=str(self.catalog),cloud_url=self.url))
        self.nodes.append(self.bridge);self.executor.add_node(self.bridge)
        self.wait(lambda:self.bridge.state=='online')
        def send(data):
            with self.cloud.condition:
                self.cloud.commands.append(data);self.cloud.condition.notify_all()
        cmd=dict(command_id='bridge-first',type='capture',task_id='bridge-pcb',task_type='pcb',params={'prepare_sec':0},ttl_sec=60)
        send(cmd)
        self.wait(lambda:any(e.get('result',{}).get('status')=='uploaded' for e in self.cloud.events.get('bridge-pcb',[])),15.)
        self.assertTrue(self.cloud.acks['bridge-first']['accepted'])
        images=len(self.cloud.images);send(cmd);time.sleep(.5)
        self.assertEqual(len(self.cloud.images),images)
        phases=[e['phase'] for e in self.cloud.events['bridge-pcb']]
        self.assertIn('ANNOUNCE',phases);self.assertIn('UPLOAD',phases)
        seq=[e['seq'] for e in self.cloud.events['bridge-pcb']];self.assertEqual(seq,sorted(set(seq)))
        expired={**cmd,'command_id':'expired','task_id':'expired','ttl_sec':.01}
        self.bridge.dispatch(expired,received=time.monotonic()-1)
        self.wait(lambda:'expired' in self.cloud.acks)
        self.assertEqual(self.cloud.acks['expired']['reason'],'expired')
        long={**cmd,'command_id':'long','task_id':'long','params':{'prepare_sec':5}}
        send(long);self.wait(lambda:self.bridge.current and self.bridge.current['task_id']=='long')
        send(dict(command_id='wrong-cancel',type='cancel',task_id='wrong'))
        self.wait(lambda:'wrong-cancel' in self.cloud.acks)
        self.assertFalse(self.cloud.acks['wrong-cancel']['accepted'])
        send(dict(command_id='right-cancel',type='cancel',task_id='long'))
        self.wait(lambda:any(e.get('result',{}).get('status')=='cancelled' for e in self.cloud.events.get('long',[])))
        self.cloud.drop_poll=True;self.wait(lambda:self.bridge.state=='backoff');self.cloud.drop_poll=False
        self.wait(lambda:self.bridge.state=='online')
        self.cloud.token='test-token';self.wait(lambda:self.bridge.state=='auth_failed')
        self.assertGreater(self.bridge.retry_at-time.monotonic(),55.)
        self.cloud.token='';self.bridge.retry_at=time.monotonic() # accelerate only the test's 60s backoff.
        self.wait(lambda:self.bridge.state=='online')
        self.cloud.fail_upload=401
        send({**cmd,'command_id':'upload-auth','task_id':'upload-auth','task_type':'helmet'})
        self.wait(lambda:self.bridge.state=='auth_failed')
        self.assertGreater(self.bridge.retry_at-time.monotonic(),55.)
        self.cloud.fail_upload=0;self.bridge.retry_at=time.monotonic()
        self.wait(lambda:self.bridge.state=='online')
        self.wait(lambda:any(e.get('result',{}).get('detail')=='upload_http_401' for e in self.cloud.events.get('upload-auth',[])))
        self.wait(lambda:self.bridge.current is None and self.capture.job is None)
        time.sleep(.5)
        threads=threading.active_count();fds=len(list(Path('/proc/self/fd').iterdir()))
        for i in range(20):
            task_id='stress-'+str(i)
            send({**cmd,'command_id':task_id,'task_id':task_id,'task_type':'helmet'})
            self.wait(lambda:any(e.get('result',{}).get('status')=='uploaded' for e in self.cloud.events.get(task_id,[])),10.)
        time.sleep(1.)
        delta_threads=threading.active_count()-threads;delta_fds=len(list(Path('/proc/self/fd').iterdir()))-fds
        print('20 task resource deltas: threads=',delta_threads,'fds=',delta_fds,flush=True)
        self.assertLessEqual(delta_threads,2);self.assertLessEqual(delta_fds,3)
        self.bridge.stop()

    def test_10_robot_guard_lease_renew_release(self):
        from motion_guard.node import MotionGuard
        self.executor.remove_node(self.capture);self.capture.destroy_node()
        guard=MotionGuard(parameter_overrides=parameters(motion_enabled=False,estop_latch_path=str(self.root/'estop.json')))
        self.executor.add_node(guard)
        odom_pub=self.client_node.create_publisher(Odometry,'/odom',qos_profile_sensor_data)
        kws_pub=self.client_node.create_publisher(Bool,'/voice/emergency_healthy',10)
        def publish():
            msg=Odometry();msg.header.stamp=self.client_node.get_clock().now().to_msg();odom_pub.publish(msg);kws_pub.publish(Bool(data=True))
        timer=self.client_node.create_timer(.05,publish)
        type(self).capture=CaptureExecutor(parameter_overrides=parameters(catalog=str(self.catalog),cloud_url=self.url,
            storage_dir=str(self.root/'robot-records'),safety_profile='robot',hold_lease_sec=1.))
        self.nodes[0]=self.capture;self.executor.add_node(self.capture)
        time.sleep(1.)
        try:
            handle,_=self.goal(prepare_sec=1.5);self.assertTrue(handle.accepted)
            self.wait(lambda:guard.task_hold.active(time.monotonic()))
            first_until=guard.task_hold.until
            self.wait(lambda:guard.task_hold.until>first_until+.4)
            result=self.result(handle);self.assertEqual(result.status,'uploaded',result.detail)
            self.assertFalse(guard.task_hold.active(time.monotonic()))
            self.assertEqual(guard.authority.state,'DISARMED');self.assertEqual(guard.command_mode,'IDLE')
            original_acquire=guard.task_hold.acquire
            def delayed_acquire(*args,**kwargs):
                time.sleep(.4)
                return original_acquire(*args,**kwargs)
            guard.task_hold.acquire=delayed_acquire
            handle,_=self.goal(ttl=.15)
            self.assertEqual(self.result(handle).status,'failed')
            time.sleep(.5)
            self.wait(lambda:not guard.task_hold.active(time.monotonic()),timeout=.2) # before the 1s lease could expire
            guard.task_hold.acquire=original_acquire
            # Cleanup keeps the executor spinning while its own task releases.
            handle,_=self.goal(prepare_sec=2.)
            self.wait(lambda:guard.task_hold.active(time.monotonic()))
            self.capture.stop()
            self.assertEqual(self.result(handle).status,'cancelled')
            self.assertFalse(guard.task_hold.active(time.monotonic()))
            self.capture.closing=False
            guard.authority.estop()
            handle,_=self.goal();self.assertEqual(self.result(handle).detail,'hold_failed')
            self.assertEqual(guard.authority.state,'ESTOP')
        finally:
            self.client_node.destroy_timer(timer);self.client_node.destroy_publisher(odom_pub);self.client_node.destroy_publisher(kws_pub)
            self.executor.remove_node(guard);guard.destroy_node()


if __name__=='__main__':
    unittest.main(verbosity=2)
