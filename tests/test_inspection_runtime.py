#!/usr/bin/env python3
"""Humble x86_64, isolated domain 189, fake inference/TTS and synthetic RGB only."""
import hashlib
import json
import os
from pathlib import Path
import tempfile
import threading
import time
import unittest

import cv2
import numpy as np
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import String
import yaml

from capture_task_interfaces.srv import RunInspection, CancelCapture
from capture_task.api_node import CaptureApi
from capture_task.executor_node import CaptureExecutor
from capture_task.bridge_node import CaptureBridge
from capture_task.fake_tts_node import FakeTTS
from capture_task.synthetic_camera_node import SyntheticCamera
from capture_task.mock_cloud import MockCloud
from capture_task.inference_client import InferenceClient, LABELS, PIPELINE_ID, PIPELINE_DIGEST
from services.inference_gateway.gateway import Gateway, Server


class Backend:
    def __init__(self, kind):
        self.kind = kind
        self.status = 'ok'
        self.delay = 0.
        self.entered = threading.Event()

    def ready(self):
        return True

    def info(self):
        return dict(name='fake-' + self.kind, version='test', weights_sha256='0' * 64, device='fake')

    def infer(self, image, metadata, deadline):
        self.entered.set()
        time.sleep(self.delay)
        if self.kind == 'pcb':
            result = dict(defects=[], reject=False, imgsz=640, conf_threshold=.2)
        elif self.kind == 'helmet':
            result = dict(detections=[], helmet_count=0, no_helmet_count=0, violation=False,
                          imgsz=640, conf_threshold=.45, iou_threshold=.45)
        else:
            performed = self.status != 'retake'
            result = dict(pipeline_id=PIPELINE_ID, pipeline_digest=PIPELINE_DIGEST,
                          classification_performed=performed, labels={k:dict(score=.1,threshold=.5,decision='negative')
                          for k in LABELS} if performed else {}, positives=[], quality={}, roi={})
        return dict(status=self.status, result=result, summary=dict(positive=False,count=0,text_key='test'))


def parameters(**values):
    return [Parameter(k,value=v) for k,v in values.items()]


class InspectionRuntime(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if os.environ.get('ROS_DOMAIN_ID')!='189' or os.environ.get('ROS_LOCALHOST_ONLY')!='1':
            raise RuntimeError('Require ROS_DOMAIN_ID=189 ROS_LOCALHOST_ONLY=1')
        rclpy.init()
        cls.temp=tempfile.TemporaryDirectory();cls.root=Path(cls.temp.name)
        cls.backends={k:Backend(k) for k in ('tongue','helmet','pcb')}
        cls.gateway=Gateway(cls.backends)
        cls.http=Server(('127.0.0.1',0),cls.gateway)
        cls.http_thread=threading.Thread(target=cls.http.serve_forever,daemon=True);cls.http_thread.start()
        cls.url='http://127.0.0.1:'+str(cls.http.server_port)
        cls.cloud=MockCloud(('127.0.0.1',0),cls.root/'cloud',reject_unknown_task=True)
        cls.cloud_thread=threading.Thread(target=cls.cloud.serve_forever,daemon=True);cls.cloud_thread.start()
        cls.cloud_url='http://127.0.0.1:'+str(cls.cloud.server_port)
        data=yaml.safe_load((Path(__file__).parents[1]/'ros2_ws/src/capture_task/config/tasks.yaml').read_text())
        for task in data['tasks'].values():
            task['prepare_sec']=0.;task['inference']['timeout_sec']=2.
        data['defaults']['speech_timeout_sec']=1.5
        cls.catalog=cls.root/'tasks.yaml';cls.catalog.write_text(yaml.safe_dump(data,allow_unicode=True))
        cls.capture=CaptureExecutor(parameter_overrides=parameters(catalog=str(cls.catalog),cloud_url=cls.cloud_url,
            pipeline_mode='infer',inference_url=cls.url,storage_dir=str(cls.root/'records'),return_original_image=True))
        cls.api=CaptureApi(parameter_overrides=parameters(catalog=str(cls.catalog),pipeline_mode='infer'))
        cls.bridge=CaptureBridge(parameter_overrides=parameters(catalog=str(cls.catalog),cloud_url=cls.cloud_url,
            pipeline_mode='infer',poll_wait_sec=0,return_original_image=True))
        cls.tts=FakeTTS(parameter_overrides=parameters(test_only=True,char_seconds=.0001))
        cls.camera=SyntheticCamera(parameter_overrides=parameters(test_only=True))
        cls.client=Node('inspection_runtime_client')
        cls.clients={k:cls.client.create_client(RunInspection,'/inspect/'+k) for k in ('face','tongue','helmet','pcb')}
        cls.cancel=cls.client.create_client(CancelCapture,'/inspect/cancel')
        cls.nodes=[cls.capture,cls.api,cls.bridge,cls.tts,cls.camera,cls.client]
        cls.executor=MultiThreadedExecutor(num_threads=16)
        for node in cls.nodes:cls.executor.add_node(node)
        cls.spin_thread=threading.Thread(target=cls.executor.spin,daemon=True);cls.spin_thread.start()
        assert cls.clients['pcb'].wait_for_service(timeout_sec=5)
        assert cls.api.client.wait_for_server(timeout_sec=5)
        assert cls.capture.speech.wait_for_server(timeout_sec=5)
        time.sleep(.7)

    @classmethod
    def tearDownClass(cls):
        cls.bridge.stop();cls.api.stop();cls.capture.stop();time.sleep(.1)
        cls.executor.shutdown(timeout_sec=5);cls.spin_thread.join(5)
        cls.tts.server.destroy()
        for node in reversed(cls.nodes):node.destroy_node()
        cls.http.shutdown();cls.http.server_close();cls.gateway.close();cls.http_thread.join(3)
        cls.cloud.stop();cls.cloud_thread.join(3);rclpy.shutdown();cls.temp.cleanup()

    def wait(self,condition,timeout=8.):
        until=time.monotonic()+timeout
        while time.monotonic()<until:
            if condition():return
            time.sleep(.01)
        self.fail('condition timeout')

    def result(self,future,timeout=8.):
        self.wait(future.done,timeout)
        return future.result()

    def call(self,kind='pcb',**extra):
        values=dict(task_id='',announce='',prepare_sec=0.,timeout_sec=5.);values.update(extra)
        return self.clients[kind].call_async(RunInspection.Request(**values))

    def setUp(self):
        self.wait(lambda:self.capture.job is None and self.api.job is None and self.bridge.current is None)
        self.wait(lambda:not self.bridge.executor_stale() and not self.bridge.executor_state.get('active'))
        self.capture.mode={}
        for backend in self.backends.values():
            backend.status='ok';backend.delay=0.;backend.entered.clear()

    def tearDown(self):
        self.capture.mode={}
        self.wait(lambda:self.capture.job is None and self.api.job is None and self.bridge.current is None)

    def test_01_all_services_and_face_unavailable(self):
        names=dict(self.client.get_service_names_and_types())
        assert '/inspect/pcb' in names and '/capture/pcb' not in names
        for kind in ('tongue','helmet','pcb'):
            response=self.result(self.call(kind))
            self.assertTrue(response.success,(kind,response.status,response.detail))
            self.assertEqual(response.status,'analyzed')
            value=json.loads(response.result_json);raw=bytes(response.annotated_image.data)
            self.assertEqual(value['model']['device'],'fake')
            self.assertEqual(value['task_type'],kind)
            self.assertNotIn('data_b64',value['annotated_image'])
            self.assertEqual(hashlib.sha256(raw).hexdigest(),value['annotated_image']['sha256'])
            self.assertEqual(cv2.imdecode(np.frombuffer(raw,np.uint8),1).shape[:2],(720,1280))
        response=self.result(self.call('face'))
        self.assertFalse(response.success);self.assertEqual(response.status,'failed')
        self.assertEqual(response.detail,'inference_model_unavailable')
        self.assertFalse(response.annotated_image.data)
        self.assertEqual(self.cloud.upload_attempts,0)
        self.assertFalse(self.cloud.events)  # ROS callers do not create cloud events.

    def test_02_cloud_event_then_annotated_and_opt_in_original(self):
        task='cloud-infer-1'
        with self.cloud.condition:
            self.cloud.commands.append(dict(command_id='infer-command-1',type='capture',task_id=task,task_type='pcb',params={'prepare_sec':0.},ttl_sec=5.))
            self.cloud.condition.notify_all()
        self.wait(lambda:len([k for k in self.cloud.result_images if k[0]==task])==2)
        events=self.cloud.events[task]
        final=next(e for e in events if e.get('result',{}).get('status')=='analyzed')
        self.assertEqual(final['result']['analysis']['task_type'],'pcb')
        self.assertNotIn('data_b64',final['result']['analysis']['annotated_image'])
        for key,receipt in self.cloud.result_images.items():
            if key[0]==task:
                self.assertEqual(hashlib.sha256(Path(receipt['path']).read_bytes()).hexdigest(),receipt['sha256'])
        self.assertEqual(self.cloud.upload_attempts,0)

    def test_03_cancel_own_busy_and_no_late_summary(self):
        backend=self.backends['pcb'];backend.delay=.5
        future=self.call(task_id='cancel-own')
        self.wait(backend.entered.is_set)
        busy=self.result(self.call())
        self.assertEqual(busy.status,'busy')
        cancelled=self.result(self.cancel.call_async(CancelCapture.Request(task_id='cancel-own')))
        self.assertTrue(cancelled.success)
        response=self.result(future)
        self.assertEqual(response.status,'cancelled');self.assertFalse(response.annotated_image.data)
        self.wait(lambda:self.capture.job is None)
        received=len(self.tts.received);time.sleep(.6)
        self.assertEqual(len(self.tts.received),received)

    def test_04_retake_uncertain_and_busy(self):
        backend=self.backends['tongue']
        backend.status='retake'
        response=self.result(self.call('tongue'))
        self.assertEqual(response.status,'retake');self.assertFalse(response.success)
        self.assertFalse(response.annotated_image.data)
        self.assertIn('这次没有拍清楚，请重新发起检测。',self.tts.received)
        backend.status='uncertain'
        self.assertEqual(self.result(self.call('tongue')).status,'analyzed')
        backend.status='busy'
        self.assertEqual(self.result(self.call('tongue')).detail,'inference_busy')

    def test_05_disconnect_timeout_and_voice_exclusion(self):
        original=self.capture.inference
        try:
            self.capture.inference=InferenceClient('http://127.0.0.1:1')
            response=self.result(self.call('helmet'))
            self.assertEqual(response.status,'failed');self.assertFalse(response.annotated_image.data)
        finally:self.capture.inference=original
        backend=self.backends['helmet'];backend.delay=.5
        response=self.result(self.call('helmet',timeout_sec=.15))
        self.assertEqual(response.status,'failed');self.assertFalse(response.annotated_image.data)
        time.sleep(.6)
        self.capture.mode={'active':True};self.capture.mode_at=time.monotonic()
        response=self.result(self.call('helmet'))
        self.assertEqual(response.status,'rejected');self.assertEqual(response.detail,'voice_mode_active')

    def test_06_cloud_owned_cancel_cannot_be_cancelled_by_ros(self):
        task='cloud-owner'
        with self.cloud.condition:
            self.cloud.commands.append(dict(command_id='cloud-owner-command',type='capture',task_id=task,
                task_type='pcb',params={'prepare_sec':1.},ttl_sec=5.))
            self.cloud.condition.notify_all()
        self.wait(lambda:self.bridge.current is not None and self.capture.job is not None)
        self.wait(lambda:self.api.executor_state.get('active') and self.api.executor_state.get('task_id')==task)
        response=self.result(self.cancel.call_async(CancelCapture.Request(task_id=task)))
        self.assertFalse(response.success);self.assertEqual(response.detail,'not_owner')
        with self.cloud.condition:
            self.cloud.commands.append(dict(command_id='cloud-owner-cancel',type='cancel',task_id=task))
            self.cloud.condition.notify_all()
        self.wait(lambda:any(e.get('result',{}).get('status')=='cancelled' for e in self.cloud.events.get(task,[])))
        self.assertFalse(any(k[0]==task for k in self.cloud.result_images))


if __name__=='__main__':
    unittest.main(verbosity=2)
