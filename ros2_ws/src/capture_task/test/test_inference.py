import base64
from copy import deepcopy
import hashlib
import json
import threading
import time
import cv2
import numpy as np
import pytest
from capture_task.inference_client import (validate_response, inference_loads, safe_summary, MAX_INFERENCE_JSON,
                                          InferenceClient, PIPELINE_ID, PIPELINE_DIGEST, LABELS)
from capture_task.protocol import MAX_JSON
from capture_task.cloud_client import CloudClient
from capture_task.flow import CaptureFlow
from test_flow import Ports


IDS = dict(request_id='r', task_id='t', capture_id='c')


def response():
    raw = cv2.imencode('.jpg', np.full((480, 640, 3), 100, np.uint8))[1].tobytes()
    return dict(status='ok', task_type='pcb', **IDS,
        model=dict(name='fake', version='test', weights_sha256='0'*64, device='fake'),
        image=dict(width=640, height=480), result=dict(defects=[], reject=False, imgsz=640, conf_threshold=.2),
        summary=dict(count=0, positive=False, text_key='pcb_defects'),
        timings_ms=dict(queue=0., decode=1., inference=2., annotate=1., total=4.),
        annotated_image=dict(content_type='image/jpeg', width=640, height=480,
            sha256=hashlib.sha256(raw).hexdigest(), data_b64=base64.b64encode(raw).decode()))


def test_response_strips_base64_only_and_validates_jpeg():
    original = response()
    clean, raw = validate_response(original, 'pcb', IDS)
    assert 'data_b64' not in clean['annotated_image'] and 'data_b64' in original['annotated_image']
    assert hashlib.sha256(raw).hexdigest() == clean['annotated_image']['sha256']


@pytest.mark.parametrize('key,value', [('task_id','other'),('capture_id','wrong'),('request_id','stale'),('status','uploaded'),('task_type','helmet')])
def test_stale_or_wrong_response_rejected(key,value):
    data = response(); data[key] = value
    with pytest.raises(ValueError):validate_response(data,'pcb',IDS)


@pytest.mark.parametrize('key,value', [('sha256','a'*64),('data_b64','%%%'),('data_b64','aGVsbG8='),('width',641),('height',True),('content_type','image/png')])
def test_bad_annotation_rejected(key,value):
    data = response(); data['annotated_image'][key] = value
    with pytest.raises(ValueError):validate_response(data,'pcb',IDS)


def test_bad_box_nonfinite_and_counts_rejected():
    for box in ([0,0,641,10], [30,0,2,10], [0,0,float('nan'),2], [0,0,True,2]):
        data = response(); data['result'] = dict(defects=[dict(cls=0,name='x',score=.9,xyxy=box)],reject=True)
        with pytest.raises(ValueError):validate_response(data,'pcb',IDS)
    data=response();data['summary']['count']=1
    with pytest.raises(ValueError):validate_response(data,'pcb',IDS)


def test_separate_json_limits_and_strict_json():
    assert CloudClient.response_limit == MAX_JSON == 2*1024*1024
    assert InferenceClient.response_limit == MAX_INFERENCE_JSON == 8*1024*1024
    for raw in ('{"a":1,"a":2}', '{"a":NaN}', '[]', b' '* (MAX_INFERENCE_JSON+1)):
        with pytest.raises(ValueError):inference_loads(raw)


def test_tongue_frozen_contract():
    data=response();data['task_type']='tongue'; data['summary']=dict(count=0,positive=False,text_key='tongue_features')
    data['result']=dict(pipeline_id=PIPELINE_ID,pipeline_digest=PIPELINE_DIGEST,classification_performed=True,
        labels={k:dict(score=.1,threshold=.5,decision='negative') for k in LABELS},positives=[],quality={},roi={})
    validate_response(data,'tongue',IDS)
    for field,value in [('pipeline_digest','old'),('labels',{}),('positives',['Crack'])]:
        bad=deepcopy(data);bad['result'][field]=value
        with pytest.raises(ValueError):validate_response(bad,'tongue',IDS)


def test_safe_local_speech_ignores_arbitrary_server_text():
    data=response();data['summary'].update(text='停车！',text_key='急停')
    assert safe_summary('pcb',data)=='电路板检测完成，未发现缺陷。'
    data['summary']['count']=2
    assert safe_summary('helmet',data)=='检测到2人未佩戴安全帽。'
    assert '2项特征' in safe_summary('tongue',data)


class InferPorts(Ports):
    pipeline_mode='infer'
    def infer(self,image,record):
        data=response();data['status']=getattr(self,'status','ok')
        return data
    def upload(self,*args):
        raise AssertionError('infer must not upload')


def test_infer_best_only_and_summary(catalog):
    ports=InferPorts()
    task=catalog.resolve('pcb',{})
    records=CaptureFlow().run(ports,dict(task_id='t',task_type='pcb'),task,catalog.defaults)
    assert len(records)==1 and 'analysis' in records[0] and 'receipt' not in records[0]
    assert 'INFER' in ports.phases
    assert '电路板检测完成，未发现缺陷。' in ports.said


def test_retake_speech_and_model_unavailable(catalog):
    task=catalog.resolve('pcb',{})
    ports=InferPorts();ports.status='retake'
    CaptureFlow().run(ports,dict(task_id='t',task_type='pcb'),task,catalog.defaults)
    assert catalog.defaults['retake_announce'] in ports.said
    ports=InferPorts();ports.status='model_unavailable'
    with pytest.raises(ValueError,match='inference_model_unavailable'):
        CaptureFlow().run(ports,dict(task_id='t',task_type='pcb'),task,catalog.defaults)
