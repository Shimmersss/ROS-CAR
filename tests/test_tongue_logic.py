"""Hardware-free regression for contracts, intent safety and bounded cancellation."""
import copy
from io import BytesIO
import json
from pathlib import Path
import socket
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import numpy as np
from PIL import Image
import pytest

ROOT = Path(__file__).resolve().parents[1]
for name in ('tongue_mode', 'execution_mode_manager', 'motion_guard', 'xfyun_speech'):
    sys.path.insert(0, str(ROOT/'ros2_ws/src'/name))
from tongue_mode.capture import FrameBuffer, Frame, encode_jpeg
from tongue_mode.cloud import CloudClient, Cancelled
from tongue_mode.flow import DetectionFlow
from tongue_mode.results import LABELS, AnalysisError, spoken_result, validate_response
from execution_mode_manager.catalog import Catalog
from motion_guard.task_hold import StationaryLease
from xfyun_speech.audio import AplaySink

PIPELINE, DIGEST = 'test_pipeline', 'a'*64


def body_for(ids=None, status='ok'):
    ids = ids or dict(request_id='r', capture_id='c', session_id='s')
    return dict(**ids, pipeline_id=PIPELINE, pipeline_digest=DIGEST, status=status,
                classification_performed=status in ('ok', 'uncertain'),
                labels={k:dict(score=.8 if k in ('FurYellow', 'FurThick') else .1, threshold=.5,
                               decision='positive' if k in ('FurYellow', 'FurThick') else 'negative')
                        for k in LABELS} if status in ('ok', 'uncertain') else {})


def validate(body):
    return validate_response(body, 'r', 'c', PIPELINE, DIGEST, 's')


def test_known_and_negated_intents():
    c = Catalog.load(ROOT/'ros2_ws/src/execution_mode_manager/config/modes.yaml')
    assert c.parse('请帮我舌苔检测。') == ('tongue', 'ENTER')
    assert c.parse('不要检查舌苔') == ('tongue', 'EXIT')
    assert c.parse('向前一点再拍照') is None
    assert c.parse('我想了解舌苔检测的原理') is None
    assert c.parse('停止') is None
    assert c.parse('重新拍摄') == ('tongue', 'RETAKE')


@pytest.mark.parametrize('text', ['舌苔检测？', '“舌苔检测”', '他说舌苔检测', '能不能检查舌苔', '先检查舌苔再前进'])
def test_mode_catalog_does_not_strip_question_or_context(text):
    c = Catalog.load(ROOT/'ros2_ws/src/execution_mode_manager/config/modes.yaml')
    assert c.parse(text) is None


def test_delayed_camera_frame_before_capture_window_is_rejected():
    frames = FrameBuffer()
    window = time.monotonic()
    frames.put(np.zeros((8, 8, 3), np.uint8), ros_stamp=10.)
    with pytest.raises(AnalysisError):
        frames.capture(window, threading.Event(), timeout=.06, ros_after=11.)


def test_future_mode_registers_custom_commands_without_tongue_dependency():
    catalog = Catalog(dict(modes=[dict(id='patrol', action='/modes/patrol/run', enter=['巡逻模式'],
                                      commands={'NEXT_POINT':['下一个巡逻点']})]))
    assert catalog.parse('下一个巡逻点') == ('patrol', 'NEXT_POINT')
    assert catalog.operations('patrol') == {'NEXT_POINT'}


@pytest.mark.parametrize('phrases', [['停止'], ['急停'], ['x', 'x']])
def test_catalog_rejects_unsafe_aliases(phrases):
    with pytest.raises(ValueError):
        Catalog(dict(modes=[dict(id='other', action='/other/run', enter=phrases)]))


def test_result_success_and_multilabel_meaning():
    body = validate(body_for())
    text = spoken_result(body)
    assert '黄苔、厚苔' in text
    for forbidden in ('白苔', '薄苔', '身体健康', '患病', '正常'):
        assert forbidden not in text


@pytest.mark.parametrize('field,value', [('request_id', 'wrong'), ('capture_id', 'wrong'),
                                        ('session_id', 'wrong'), ('pipeline_digest', 'wrong'),
                                        ('classification_performed', False), ('labels', {})])
def test_invalid_identity_or_contract(field, value):
    body = body_for(); body[field] = value
    with pytest.raises(AnalysisError): validate(body)


@pytest.mark.parametrize('value', [float('nan'), float('inf'), -.1, 1.1, True, '0.9'])
def test_invalid_scores(value):
    body = body_for(); body['labels']['FurYellow']['score'] = value
    with pytest.raises(AnalysisError): validate(body)


def test_uncertain_is_preserved():
    body = body_for(status='uncertain'); body['labels']['FurYellow']['decision'] = 'uncertain'
    assert validate(body)['labels']['FurYellow']['decision'] == 'uncertain'
    assert '部分特征暂不确定' in spoken_result(body)


def test_retake_does_not_create_fake_negative():
    body = validate(body_for(status='retake'))
    assert not body['labels'] and '重新拍摄' in spoken_result(body)
    body['labels'] = body_for()['labels']
    with pytest.raises(AnalysisError): validate(body)


def test_stationary_lease_exclusivity_expiry_and_token():
    lease = StationaryLease(); token = lease.acquire('tongue', 'session1', 3., 10.)
    with pytest.raises(ValueError): lease.acquire('other', 'session2', 3., 11.)
    with pytest.raises(ValueError): lease.renew('wrong', 3., 11.)
    lease.renew(token, 3., 12.)
    assert lease.active(14.9) and lease.expire(15.)
    assert not lease.active(15.)
    with pytest.raises(ValueError): lease.renew(token, 3., 15.)
    token2 = lease.acquire('other', 'session2', 2., 16.)
    lease.release(token2)
    assert not lease.active(17.)


def test_capture_requires_new_frame_and_preserves_rgb():
    frames = FrameBuffer(); red = np.zeros((16, 16, 3), np.uint8); red[:,:,0] = 255
    frames.put(red, 12.)
    window = time.monotonic()
    producer = threading.Thread(target=lambda:(time.sleep(.04), frames.put(red, 13.)))
    producer.start()
    frame = frames.capture(window, threading.Event(), .5)
    producer.join()
    assert frame.sequence == 2 and frame.ros_stamp == 13.
    image = np.array(Image.open(BytesIO(encode_jpeg(frame))))
    assert image[:,:,0].mean() > 240 and image[:,:,2].mean() < 10
    with pytest.raises(AnalysisError): frames.capture(time.monotonic(), threading.Event(), .05)


def test_capture_cancel():
    event = threading.Event(); event.set()
    with pytest.raises(Cancelled): FrameBuffer().capture(0., event)


class MockCloudHandler(BaseHTTPRequestHandler):
    def log_message(self, *args): pass
    def send(self, value):
        raw = json.dumps(value).encode()
        try:
            self.send_response(200); self.send_header('Content-Length', str(len(raw))); self.end_headers(); self.wfile.write(raw)
        except (BrokenPipeError, ConnectionResetError): pass
    def do_GET(self):
        if self.path == '/health/ready': self.send({'status':'ready'})
        else: self.send(dict(pipeline_id=PIPELINE, pipeline_digest=DIGEST, device='cuda', labels=list(LABELS)))
    def do_POST(self):
        data = self.rfile.read(int(self.headers['Content-Length']))
        self.server.images.append(data)
        self.server.began.set()
        time.sleep(self.server.delay)
        self.send(body_for({key:self.headers[header] for key, header in (
            ('request_id','X-Request-Id'), ('capture_id','X-Capture-Id'), ('session_id','X-Session-Id'))}))


@pytest.fixture
def cloud_server():
    server = ThreadingHTTPServer(('127.0.0.1', 0), MockCloudHandler)
    server.daemon_threads = True; server.delay = 0.; server.images = []; server.began = threading.Event()
    thread = threading.Thread(target=server.serve_forever, daemon=True); thread.start()
    yield server
    server.shutdown(); server.server_close(); thread.join()


def test_http_real_bytes_and_contract(cloud_server):
    client = CloudClient('http://127.0.0.1:'+str(cloud_server.server_port))
    event = threading.Event()
    assert client.ready(event, time.monotonic()+3., PIPELINE, DIGEST)['device'] == 'cuda'
    jpeg = encode_jpeg(Frame(np.full((12, 14, 3), 90, np.uint8), 1, time.monotonic(), 1.))
    value = client.analyze(jpeg, dict(request_id='r', capture_id='c', session_id='s'), 'test', 'now', event, time.monotonic()+3.)
    assert validate(value)['status'] == 'ok'
    assert cloud_server.images == [jpeg]


def test_http_cancel_unblocks_and_drops_late_result(cloud_server):
    cloud_server.delay = 1.
    client = CloudClient('http://127.0.0.1:'+str(cloud_server.server_port))
    event = threading.Event(); outcomes = []
    def call():
        try: outcomes.append(client.analyze(b'jpeg', dict(request_id='r', capture_id='c', session_id='s'), 'test', 'now', event, time.monotonic()+3.))
        except Cancelled: outcomes.append('cancelled')
    thread = threading.Thread(target=call); thread.start()
    assert cloud_server.began.wait(1.)
    event.set(); began = time.monotonic(); client.cancel(); thread.join(.5)
    assert not thread.is_alive() and outcomes == ['cancelled'] and time.monotonic()-began < .5


def test_flow_waits_for_speech_before_new_capture():
    events = []
    class Ports:
        def stage(self, phase, body=None): events.append(phase)
        def ready(self): events.append('cloud_ready')
        def speak(self, text): events.append(('speech_complete', text))
        def wait(self, seconds): events.append('prepared')
        def capture(self, after, cancel):
            assert events[-2:] == ['CAPTURE', ('speech_complete', '正在拍摄')]
            assert events.index(('speech_complete', '准备拍摄')) < events.index('prepared')
            return Frame(np.zeros((12, 12, 3), np.uint8), 1, after+.01, 1.)
        def analyze(self, jpeg, ids, capture_time): return body_for(ids)
        def save(self, record, jpeg): events.append('saved')
    cfg = dict(prepare_sec=0., jpeg_quality=95, expected_pipeline_id=PIPELINE, expected_pipeline_digest=DIGEST)
    body = DetectionFlow(Ports(), cfg).run('s', threading.Event(), True)
    assert body['status'] == 'ok' and events[-1][0] == 'speech_complete'
    assert events.index('saved') < events.index('SPEAK_RESULT')
    for phase, text in [('CHECK_CLOUD', '检查检测服务'), ('PREPARE', '准备拍摄'),
                        ('CAPTURE', '正在拍摄'), ('ANALYZING', '正在分析')]:
        assert events[events.index(phase)+1] == ('speech_complete', text)


def test_audio_abort_prevents_late_start():
    sink = AplaySink('unused', 16000)
    sink.abort()
    with pytest.raises(RuntimeError): sink.start()


def test_audio_cancel_race_reaps_spawned_process(monkeypatch):
    sink = AplaySink('unused', 16000)
    class Process:
        def __init__(self):
            self.stdin=BytesIO(); self.stderr=BytesIO(); self.terminated=False; self.waited=False
        def poll(self): return None if not self.terminated else -15
        def terminate(self): self.terminated=True
        def wait(self, timeout): self.waited=True; return -15
    process=Process()
    def spawn(*args, **kwargs):
        sink.abort()
        return process
    monkeypatch.setattr('xfyun_speech.audio.subprocess.Popen', spawn)
    with pytest.raises(RuntimeError): sink.start()
    assert process.terminated and process.waited and sink._process is None
