import base64
from concurrent.futures import ThreadPoolExecutor
import hashlib
import http.client
import json
import threading
import time

import cv2
import numpy as np
import pytest

from services.inference_gateway.gateway import Gateway, Server, Work, MAX_INPUT


class Fake:
    def ready(self):
        return True

    def info(self):
        return dict(name='fake-pcb', version='test', weights_sha256='0' * 64, device='fake', ready=True)

    def infer(self, image, metadata, deadline):
        return dict(status='ok', result=dict(defects=[dict(cls=0, name='missing_hole', score=.9, xyxy=[1., 2., 30., 40.])], reject=True, conf_threshold=.2, imgsz=640),
                    summary=dict(count=1, positive=True, text_key='pcb_defects'))


@pytest.fixture
def jpeg():
    return cv2.imencode('.jpg', np.full((480, 640, 3), 100, np.uint8))[1].tobytes()


@pytest.fixture
def http_server():
    gateway = Gateway({'pcb': Fake()}, token='test-token')
    server = Server(('127.0.0.1', 0), gateway)
    thread = threading.Thread(target=server.serve_forever, daemon=True); thread.start()
    yield server
    server.shutdown(); server.server_close(); gateway.close(); thread.join(2)


def request(server, method, path, body=None, headers=None):
    conn = http.client.HTTPConnection('127.0.0.1', server.server_port, timeout=3)
    try:
        conn.request(method, path, body, headers or {})
        response = conn.getresponse()
        return response.status, json.loads(response.read())
    finally:
        conn.close()


def fields(**extra):
    return {'Authorization': 'Bearer test-token', 'Content-Type': 'image/jpeg',
            'X-Request-Id': 'r', 'X-Task-Id': 't', 'X-Capture-Id': 'c', 'X-Robot-Id': 'robot',
            'X-Capture-Time': '2026-10-15T14:00:00+00:00', **extra}


def test_http_inference_and_unavailable(http_server, jpeg):
    code, value = request(http_server, 'POST', '/v1/infer/pcb', jpeg, fields())
    assert code == 200 and value['status'] == 'ok' and value['summary']['count'] == 1
    assert (value['request_id'], value['task_id'], value['capture_id']) == ('r', 't', 'c')
    raw = base64.b64decode(value['annotated_image']['data_b64'], validate=True)
    assert hashlib.sha256(raw).hexdigest() == value['annotated_image']['sha256']
    assert cv2.imdecode(np.frombuffer(raw, np.uint8), 1).shape[:2] == (480, 640)
    code, value = request(http_server, 'POST', '/v1/infer/face', jpeg, fields())
    assert code == 200 and value['status'] == 'model_unavailable' and value['annotated_image'] is None


def test_protocol_limits_auth_paths_and_decode(http_server):
    assert request(http_server, 'GET', '/health/live')[0] == 401
    assert request(http_server, 'GET', '/health/live', headers=fields(Authorization='Bearer bad'))[0] == 401
    assert request(http_server, 'GET', '/health/live', headers=fields())[1]['status'] == 'live'
    assert request(http_server, 'GET', '/v1/models', headers=fields())[1]['models']['pcb']['ready']
    assert request(http_server, 'POST', '/v1/infer/nope', b'a', fields())[0] == 404
    assert request(http_server, 'POST', '/v1/infer/pcb', b'a', fields(**{'Content-Length': str(MAX_INPUT + 1)}))[0] == 413
    assert request(http_server, 'POST', '/v1/infer/pcb', b'a', fields(**{'X-Task-Id': '../bad'}))[0] == 400
    assert request(http_server, 'POST', '/v1/infer/pcb', b'a', fields(**{'X-Capture-Time': ''}))[0] == 400
    assert request(http_server, 'POST', '/v1/infer/pcb', b'not an image', fields())[1]['status'] == 'invalid_input'


def test_single_worker_bounded_queue_and_abandoned_work(jpeg):
    entered, release = threading.Event(), threading.Event()
    class Slow(Fake):
        calls = 0
        def infer(self, *args):
            self.calls += 1
            entered.set(); release.wait(2)
            return super().infer(*args)
    fake = Slow(); gateway = Gateway({'pcb': fake}, max_queue=1)
    def work(timeout=2):
        now = time.monotonic()
        return Work('pcb', jpeg, dict(request_id='r', task_id='t', capture_id='c'), now, now + timeout)
    with ThreadPoolExecutor(3) as pool:
        first = pool.submit(gateway.submit, work())
        assert entered.wait(1)
        expired = work(.05)
        second = pool.submit(gateway.submit, expired)
        end = time.monotonic() + 1
        while gateway.queue.empty() and time.monotonic() < end:
            time.sleep(.001)
        assert gateway.submit(work())['status'] == 'busy'
        assert second.result(1)['status'] == 'timeout'
        release.set()
        assert first.result(2)['status'] == 'ok'
    gateway.close()
    assert fake.calls == 1


def test_no_torch_import_on_gateway_import():
    import subprocess, sys
    result = subprocess.run([sys.executable, '-c',
        'import sys; import services.inference_gateway.gateway; assert "torch" not in sys.modules; assert "ultralytics" not in sys.modules'], capture_output=True)
    assert result.returncode == 0, result.stderr


def test_tongue_annotation_draws_frozen_roi_key():
    from services.inference_gateway.annotate import annotate
    blank=np.zeros((480,640,3),np.uint8)
    raw,width,height=annotate(blank,'tongue',{'roi':{'box_original':[100,100,300,200]}},'ok')
    decoded=cv2.imdecode(np.frombuffer(raw,np.uint8),1)
    assert (width,height)==(640,480)
    assert decoded[100,150,1]>100  # Green top edge, far from the status text.
