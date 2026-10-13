#!/usr/bin/env python3
"""Native Humble localhost domain 188; synthetic images and fake speech only."""
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import threading
import time
import unittest
import yaml
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import String
from capture_task_interfaces.action import CaptureTask
from capture_task_interfaces.srv import RunCapture, CancelCapture
from capture_task.api_node import CaptureApi
from capture_task.executor_node import CaptureExecutor
from capture_task.bridge_node import CaptureBridge
from capture_task.fake_tts_node import FakeTTS
from capture_task.synthetic_camera_node import SyntheticCamera
from capture_task.mock_cloud import MockCloud


def isolated():
    if os.environ.get('ROS_DOMAIN_ID') != '188' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        raise RuntimeError('Require isolated ROS_DOMAIN_ID=188 ROS_LOCALHOST_ONLY=1')


def parameters(**values):
    return [Parameter(key,value=value) for key,value in values.items()]


class CaptureApiRuntime(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        isolated();rclpy.init()
        cls.temp=tempfile.TemporaryDirectory();cls.root=Path(cls.temp.name)
        data=yaml.safe_load((Path(__file__).parents[1]/'ros2_ws/src/capture_task/config/tasks.yaml').read_text())
        for task in data['tasks'].values():task['prepare_sec']=0.
        data['defaults']['speech_timeout_sec']=1.5
        cls.catalog=cls.root/'tasks.yaml';cls.catalog.write_text(yaml.safe_dump(data,allow_unicode=True))
        cls.cloud=MockCloud(('127.0.0.1',0),cls.root/'cloud')
        cls.http=threading.Thread(target=cls.cloud.serve_forever,daemon=True);cls.http.start()
        cls.url='http://127.0.0.1:'+str(cls.cloud.server_port)
        cls.capture=cls.new_capture();cls.api=CaptureApi(parameter_overrides=parameters(catalog=str(cls.catalog)))
        cls.speaker=FakeTTS(parameter_overrides=parameters(test_only=True,char_seconds=.0001))
        cls.camera=SyntheticCamera(parameter_overrides=parameters(test_only=True))
        cls.client_node=Node('capture_api_test_client')
        cls.nodes=[cls.capture,cls.api,cls.speaker,cls.camera,cls.client_node]
        cls.clients={kind:cls.client_node.create_client(RunCapture,'/capture/'+kind) for kind in cls.api.catalog.tasks}
        cls.cancel=cls.client_node.create_client(CancelCapture,'/capture/cancel')
        cls.resume()
        assert cls.clients['face'].wait_for_service(timeout_sec=5.)
        assert cls.api.client.wait_for_server(timeout_sec=5.)
        assert cls.capture.speech.wait_for_server(timeout_sec=5.)
        time.sleep(.5)

    @classmethod
    def new_capture(cls):
        return CaptureExecutor(parameter_overrides=parameters(catalog=str(cls.catalog),cloud_url=cls.url,storage_dir=str(cls.root/'records')))

    @classmethod
    def pause(cls):
        cls.executor.shutdown(timeout_sec=5.);cls.thread.join(5.)

    @classmethod
    def resume(cls):
        cls.executor=MultiThreadedExecutor(num_threads=12)
        for node in cls.nodes:
            if node is not None:cls.executor.add_node(node)
        cls.thread=threading.Thread(target=cls.executor.spin,daemon=True);cls.thread.start()

    @classmethod
    def tearDownClass(cls):
        cls.api.stop();cls.capture.stop();time.sleep(.1);cls.pause()
        # ActionServer holds its own node handle; Node.destroy_node alone
        # leaves fake speech discovery alive across a later ROS context.
        cls.speaker.server.destroy()
        for node in reversed(cls.nodes):node.destroy_node()
        cls.cloud.stop();cls.http.join(3.);rclpy.shutdown();cls.temp.cleanup()

    def setUp(self):
        self.capture.mode={};self.cloud.reject_unknown_task=False;self.cloud.slow_upload=0.
        self.wait(lambda:self.api.job is None and self.capture.job is None)

    def tearDown(self):
        self.capture.mode={};self.cloud.reject_unknown_task=False;self.cloud.slow_upload=0.
        self.wait(lambda:self.api.job is None and self.capture.job is None)
        time.sleep(.2)

    def wait(self,condition,timeout=8.):
        end=time.monotonic()+timeout
        while time.monotonic()<end:
            if condition():return
            time.sleep(.02)
        self.fail('condition timeout')

    def result(self,future,timeout=10.):
        self.wait(future.done,timeout);return future.result()

    def call(self,kind='helmet',**values):
        req=dict(task_id='',announce='',prepare_sec=-1.,timeout_sec=0.);req.update(values)
        return self.clients[kind].call_async(RunCapture.Request(**req))

    def cancellation(self,task_id):
        return self.result(self.cancel.call_async(CancelCapture.Request(task_id=task_id)))

    def replace_capture(self,node):
        self.pause();self.capture.destroy_node()
        type(self).capture=node;self.nodes[0]=node;self.resume()

    def start_bridge(self):
        bridge=CaptureBridge(parameter_overrides=parameters(catalog=str(self.catalog),cloud_url=self.url))
        self.nodes.append(bridge);self.executor.add_node(bridge)
        self.wait(lambda:bridge.state=='online' and not bridge.executor_stale())
        return bridge

    def stop_bridge(self,bridge):
        bridge.stop();self.pause();self.nodes.remove(bridge);bridge.destroy_node();self.resume()

    def send(self,data):
        with self.cloud.condition:
            self.cloud.commands.append(data);self.cloud.condition.notify_all()

    def test_01_four_services_sha_source_and_generated_ids(self):
        for kind in ('face','tongue','helmet','pcb'):
            result=self.result(self.call(kind))
            self.assertTrue(result.success,result.detail);self.assertEqual(result.status,'uploaded')
            self.assertRegex(result.task_id,r'^ros-[0-9a-f]{32}$')
            receipt=json.loads(result.upload_json)[0]
            image=self.root/'cloud'/result.task_id/(result.capture_id+'.jpg')
            self.assertEqual(hashlib.sha256(image.read_bytes()).hexdigest(),receipt['sha256'])
            self.assertEqual(self.cloud.upload_sources[(result.task_id,result.capture_id)],'ros_api')
            self.assertNotIn(result.task_id,self.cloud.events)
        self.assertEqual(self.client_node.count_publishers('/cmd_vel'),0)

    def test_02_supplied_id_and_announce(self):
        text='这是接口指定的安全播报，请面向镜头。'
        result=self.result(self.call('face',task_id='ros-custom',announce=text,prepare_sec=0.))
        self.assertEqual(result.task_id,'ros-custom');self.assertTrue(result.success,result.detail)
        self.assertIn(text,self.speaker.received)
        # Also verify the public CLI resolves the generated srv from this overlay.
        cli=subprocess.run(['ros2','service','call','/capture/helmet','capture_task_interfaces/srv/RunCapture',
            "{task_id: 'ros-cli', announce: '', prepare_sec: -1.0, timeout_sec: 10.0}"],
            capture_output=True,text=True,timeout=15.)
        self.assertEqual(cli.returncode,0,cli.stdout+cli.stderr)
        self.assertIn("status='uploaded'",cli.stdout)
        self.assertTrue(any(task_id=='ros-cli' for task_id,_ in self.cloud.images))

    def test_03_invalid_announce_does_not_reach_executor(self):
        before=len(self.speaker.received);attempts=self.cloud.upload_attempts
        events=self.root/'records'/'events.jsonl';size=events.stat().st_size
        result=self.result(self.call(announce='请停车'))
        self.assertEqual(result.status,'invalid');self.assertFalse(result.success)
        self.assertIsNone(self.capture.job);self.assertEqual(events.stat().st_size,size)
        self.assertEqual(len(self.speaker.received),before);self.assertEqual(self.cloud.upload_attempts,attempts)

    def test_04_facade_concurrency_no_queue(self):
        first=self.call(prepare_sec=1.5)
        self.wait(lambda:self.capture.job and self.capture.job.phase=='PREPARE')
        started=time.monotonic();other=self.result(self.call('pcb'))
        self.assertEqual(other.status,'busy');self.assertLess(time.monotonic()-started,.5)
        self.assertTrue(self.result(first).success)

    def test_05_cloud_busy_and_cancel_ownership(self):
        bridge=self.start_bridge()
        try:
            cmd=dict(command_id='cloud-owned',type='capture',task_id='cloud-owned',task_type='helmet',params={'prepare_sec':1.5},ttl_sec=30)
            self.send(cmd)
            self.wait(lambda:self.capture.job and self.capture.job.phase=='PREPARE')
            self.wait(lambda:self.api.executor_state.get('task_id')=='cloud-owned')
            result=self.result(self.call('face'));self.assertEqual(result.status,'busy')
            cancelled=self.cancellation('cloud-owned')
            self.assertFalse(cancelled.success);self.assertEqual(cancelled.detail,'not_owner')
            self.wait(lambda:any(e.get('result',{}).get('status')=='uploaded' for e in self.cloud.events.get('cloud-owned',[])))
        finally:self.stop_bridge(bridge)

    def test_06_ros_busy_to_cloud_and_cloud_cannot_cancel_ros(self):
        bridge=self.start_bridge()
        try:
            future=self.call(task_id='ros-owned',prepare_sec=2.)
            self.wait(lambda:self.capture.job and self.capture.job.phase=='PREPARE')
            self.wait(lambda:bridge.executor_state.get('active'))
            self.send(dict(command_id='cloud-later',type='capture',task_id='cloud-later',task_type='face',ttl_sec=30))
            self.send(dict(command_id='cloud-cancel-ros',type='cancel',task_id='ros-owned'))
            self.wait(lambda:'cloud-cancel-ros' in self.cloud.acks)
            self.assertEqual(self.cloud.acks['cloud-later']['reason'],'busy')
            self.assertEqual(self.cloud.acks['cloud-cancel-ros']['reason'],'not_active')
            self.assertTrue(self.result(future).success)
            self.assertNotIn('ros-owned',self.cloud.events)
        finally:self.stop_bridge(bridge)

    def test_07_voice_mode_rejected(self):
        self.capture.on_mode(String(data='{"active":true}'))
        result=self.result(self.call())
        self.assertEqual((result.status,result.detail),('rejected','voice_mode_active'))

    def test_08_matching_cancel_no_failure_speech(self):
        fail=self.capture.catalog.defaults['fail_announce'];before=self.speaker.received.count(fail)
        future=self.call(task_id='ros-cancel',prepare_sec=10.)
        self.wait(lambda:self.capture.job and self.capture.job.phase=='PREPARE' and self.api.job.handle is not None)
        wrong=self.cancellation('other');self.assertEqual(wrong.detail,'not_owner');self.assertFalse(wrong.success)
        cancel=self.cancellation('ros-cancel');self.assertTrue(cancel.success)
        result=self.result(future);self.assertEqual((result.status,result.detail),('cancelled','cancelled'))
        self.wait(lambda:self.capture.job is None)
        self.assertEqual(self.speaker.received.count(fail),before)
        self.wait(lambda:not self.api.executor_state.get('active'))
        idle=self.cancellation('ros-cancel');self.assertEqual(idle.detail,'not_active')

    def test_09_executor_unavailable_two_seconds(self):
        self.pause();self.capture.destroy_node();self.nodes[0]=None;self.resume()
        try:
            self.wait(lambda:not self.api.client.server_is_ready())
            started=time.monotonic();result=self.result(self.call(),3.)
            self.assertEqual(result.status,'unavailable');self.assertLess(time.monotonic()-started,2.6)
        finally:
            self.pause();type(self).capture=self.new_capture();self.nodes[0]=self.capture;self.resume();time.sleep(.3)

    def test_10_strict_cloud_404(self):
        self.cloud.reject_unknown_task=True
        result=self.result(self.call())
        self.assertEqual((result.status,result.detail),('failed','upload_http_404'))
        self.assertFalse(result.success)

    def test_11_slow_upload_respects_executor_ttl(self):
        self.cloud.slow_upload=3.
        started=time.monotonic();result=self.result(self.call(timeout_sec=.6))
        self.assertEqual(result.status,'failed');self.assertIn(result.detail,('task_timeout','deadline_exceeded'))
        self.assertLess(time.monotonic()-started,2.)
        self.wait(lambda:self.capture.job is None)

    def test_12_facade_timeout_and_late_accept_cancel_only_owned_goal(self):
        # Fault injection: a server ignoring TTL/result, unlike the real executor.
        for delay in (0.,7.):
            stub=Node('stalled_capture_executor');cancelled=threading.Event()
            def admit(request):
                time.sleep(delay);return GoalResponse.ACCEPT
            def execute(goal):
                end=time.monotonic()+12.
                while not goal.is_cancel_requested and time.monotonic()<end:time.sleep(.01)
                if goal.is_cancel_requested:
                    cancelled.set();goal.canceled();return CaptureTask.Result(status='cancelled',detail='injected cancellation')
                goal.abort();return CaptureTask.Result(status='failed')
            server=ActionServer(stub,CaptureTask,'/capture_task/run',execute,goal_callback=admit,
                                cancel_callback=lambda _:CancelResponse.ACCEPT,callback_group=ReentrantCallbackGroup())
            self.replace_capture(stub)
            try:
                self.wait(lambda:self.api.client.server_is_ready())
                result=self.result(self.call(timeout_sec=.1),8.)
                self.assertEqual((result.status,result.detail),('timeout','service_timeout'))
                self.assertTrue(cancelled.wait(3.))
            finally:
                self.pause();server.destroy();stub.destroy_node()
                type(self).capture=self.new_capture();self.nodes[0]=self.capture;self.resume();time.sleep(.3)

    def test_13_api_sigterm_cancels_inflight_goal(self):
        self.pause();self.api.destroy_node();self.nodes[1]=None;self.resume()
        log=(self.root/'api-sigterm.log').open('w')
        process=subprocess.Popen([sys.executable,'-c','from capture_task.api_node import main; main()',
            '--ros-args','-p','catalog:='+str(self.catalog)],stdout=log,stderr=subprocess.STDOUT)
        try:
            time.sleep(.8)
            self.assertTrue(self.clients['helmet'].wait_for_service(timeout_sec=5.))
            future=self.call(task_id='ros-sigterm',prepare_sec=10.)
            self.wait(lambda:self.capture.job and self.capture.job.phase=='PREPARE')
            before=self.speaker.received.count(self.capture.catalog.defaults['fail_announce'])
            process.send_signal(signal.SIGTERM)
            self.assertEqual(self.result(future).status,'cancelled')
            self.assertEqual(process.wait(timeout=6.),0)
            self.wait(lambda:self.capture.job is None)
            self.assertEqual(self.speaker.received.count(self.capture.catalog.defaults['fail_announce']),before)
        finally:
            if process.poll() is None:process.kill();process.wait()
            log.close();self.pause()
            type(self).api=CaptureApi(parameter_overrides=parameters(catalog=str(self.catalog)))
            self.nodes[1]=self.api;self.resume();time.sleep(.3)


class CaptureLaunchDefaults(unittest.TestCase):
    def test_default_closed_and_explicit_launch(self):
        isolated();rclpy.init();node=Node('capture_launch_probe')
        try:
            for enabled in (False,True):
                expected={'capture_launch_probe','capture_task_executor','capture_cloud_bridge'}
                expected_services={'/capture/'+kind for kind in ('face','tongue','helmet','pcb','cancel')} if enabled else set()
                if enabled:expected.add('capture_api')
                with tempfile.TemporaryDirectory() as temp:
                    log=(Path(temp)/'launch.log').open('w')
                    command=['ros2','launch','capture_task','capture_task.launch.py','storage_dir:='+temp]
                    if enabled:command.append('with_ros_api:=true')
                    process=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
                    try:
                        end=time.monotonic()+8.
                        while time.monotonic()<end:
                            names={name for name,_ in node.get_node_names_and_namespaces()}
                            services={name for name,_ in node.get_service_names_and_types() if name.startswith('/capture/')}
                            # Wait for DDS to remove names from the preceding fixture too.
                            if names==expected and services==expected_services:break
                            rclpy.spin_once(node,timeout_sec=.05)
                        self.assertTrue({'capture_task_executor','capture_cloud_bridge'}<=names,(Path(temp)/'launch.log').read_text())
                        self.assertEqual(names,expected)
                        if enabled:
                            self.assertEqual(services,{'/capture/'+kind for kind in ('face','tongue','helmet','pcb','cancel')})
                        else:
                            self.assertEqual(services,set());self.assertNotIn('capture_api',names)
                    finally:
                        os.killpg(process.pid,signal.SIGINT);process.wait(timeout=10.);log.close()
                    end=time.monotonic()+3.
                    while time.monotonic()<end:
                        rclpy.spin_once(node,timeout_sec=.05)
                        if not any(name=='capture_task_executor' for name,_ in node.get_node_names_and_namespaces()):break
        finally:node.destroy_node();rclpy.shutdown()


if __name__=='__main__':
    isolated();unittest.main(verbosity=2)
