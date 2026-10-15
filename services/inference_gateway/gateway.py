"""Bounded HTTP inference gateway with one model worker. No ROS dependency."""
import argparse
import base64
from datetime import datetime
from dataclasses import dataclass, field
import hashlib
import hmac
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import queue
import re
import socket
import threading
import time
from pathlib import Path

from .annotate import annotate
from .backends.base import Unavailable

MAX_INPUT = 12 * 1024 * 1024
MAX_RESPONSE = 8 * 1024 * 1024
KINDS = ('face', 'tongue', 'helmet', 'pcb')


def identifier(value):
    if not isinstance(value, str) or not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_.-]{0,63}', value):
        raise ValueError('invalid_identifier')
    return value


@dataclass
class Work:
    kind: str
    raw: bytes
    metadata: dict
    received: float
    deadline: float
    done: threading.Event = field(default_factory=threading.Event)
    abandoned: threading.Event = field(default_factory=threading.Event)
    value: dict = field(default_factory=dict)
    context: dict = field(default_factory=dict)


class Gateway:
    def __init__(self, backends, token='', max_queue=4, timeout_sec=120.):
        if not 1 <= max_queue <= 16 or not 0 < timeout_sec <= 600:
            raise ValueError('invalid_gateway_limits')
        self.backends = {kind: backends.get(kind, Unavailable()) for kind in KINDS}
        self.token, self.timeout = token, timeout_sec
        self.queue = queue.Queue(maxsize=max_queue)
        self.closing = threading.Event()
        self.worker = threading.Thread(target=self.run, name='single-model-worker', daemon=True)
        self.worker.start()

    def response(self, work, status):
        return dict(status=status, task_type=work.kind, **work.metadata,
                    model=self.backends[work.kind].info(), image={}, result={},
                    summary=dict(positive=False, count=0, text_key=status), annotated_image=None,
                    timings_ms=dict(queue=0., decode=0., inference=0., annotate=0., total=0.))

    def submit(self, work):
        try:
            self.queue.put_nowait(work)
        except queue.Full:
            return self.response(work, 'busy')
        while not work.done.wait(min(.05, max(.001, work.deadline - time.monotonic()))):
            if self.closing.is_set() or time.monotonic() >= work.deadline:
                work.abandoned.set()
                return self.response(work, 'timeout')
        return work.value

    def run(self):
        while not self.closing.is_set():
            try:
                work = self.queue.get(timeout=.1)
            except queue.Empty:
                continue
            try:
                work.value = self.process(work)
            except Exception:
                # Model exceptions and their messages may contain local paths/input data.
                work.value = self.response(work, 'error')
            finally:
                work.raw = b''
                work.done.set()
                self.queue.task_done()

    def process(self, work):
        import cv2
        import numpy as np
        value = self.response(work, 'timeout')
        if work.abandoned.is_set() or time.monotonic() >= work.deadline:
            return value
        start = time.monotonic()
        timing = value['timings_ms']
        timing['queue'] = (start - work.received) * 1000
        # Inspect dimensions before allocating a potentially huge decoded bitmap.
        from PIL import Image
        import io
        try:
            with Image.open(io.BytesIO(work.raw)) as probe:
                if probe.format not in ('JPEG', 'PNG') or not 0 < probe.width * probe.height <= 40000000:
                    raise ValueError()
            image = cv2.imdecode(np.frombuffer(work.raw, np.uint8), cv2.IMREAD_COLOR)
            if image is None:
                raise ValueError()
        except Exception:
            value['status'] = 'invalid_input'
            return value
        value['image'] = dict(width=image.shape[1], height=image.shape[0])
        at = time.monotonic(); timing['decode'] = (at - start) * 1000
        backend = self.backends[work.kind]
        if not backend.ready():
            value['status'] = 'model_unavailable'
            return value
        try:
            output = backend.infer(image, {**work.metadata, **work.context, '_raw': work.raw}, work.deadline)
        except (TimeoutError, socket.timeout):
            return value
        finished = time.monotonic(); timing['inference'] = (finished - at) * 1000
        if work.abandoned.is_set() or finished >= work.deadline:
            return value
        if output.get('status') not in ('ok', 'uncertain', 'retake', 'model_unavailable', 'invalid_input', 'busy', 'timeout', 'error'):
            raise ValueError('backend_status')
        value.update(output)
        if value['status'] in ('ok', 'uncertain'):
            encoded, width, height = annotate(image, work.kind, value['result'], value['status'])
            value['annotated_image'] = dict(content_type='image/jpeg', width=width, height=height,
                                           sha256=hashlib.sha256(encoded).hexdigest(),
                                           data_b64=base64.b64encode(encoded).decode('ascii'))
        timing['annotate'] = (time.monotonic() - finished) * 1000
        timing['total'] = (time.monotonic() - work.received) * 1000
        if time.monotonic() >= work.deadline:
            return self.response(work, 'timeout')
        return value

    def close(self):
        self.closing.set()
        self.worker.join(timeout=2.)


class Server(ThreadingHTTPServer):
    daemon_threads = True
    request_queue_size = 16

    def __init__(self, address, gateway):
        self.gateway = gateway
        # Bound request handlers as well as GPU queue; reject excess connections.
        self.slots = threading.BoundedSemaphore(20)
        super().__init__(address, Handler)

    def process_request(self, request, client_address):
        if not self.slots.acquire(False):
            request.close()
            return
        try:
            super().process_request(request, client_address)
        except Exception:
            self.slots.release()
            raise

    def process_request_thread(self, request, address):
        try:
            super().process_request_thread(request, address)
        finally:
            self.slots.release()


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *args):
        pass

    def setup(self):
        super().setup()
        self.connection.settimeout(15.)

    def reply(self, code, value):
        raw = json.dumps(value, ensure_ascii=False, allow_nan=False).encode()
        if len(raw) > MAX_RESPONSE:
            if isinstance(value, dict) and 'task_type' in value:
                value = dict(value, status='error', result={}, annotated_image=None)
                value['summary'] = dict(positive=False, count=0, text_key='response_too_large')
                raw = json.dumps(value, allow_nan=False).encode()
            else:
                raw = b'{"status":"error"}'
        self.send_response(code)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Content-Length', str(len(raw)))
        self.send_header('Connection', 'close')
        self.end_headers()
        try:
            self.wfile.write(raw)
        except (OSError, TimeoutError):
            pass
        self.close_connection = True

    def authorized(self):
        token = self.server.gateway.token
        if token and not hmac.compare_digest(self.headers.get('Authorization', ''), 'Bearer ' + token):
            self.reply(401, {'status': 'unauthorized'})
            return False
        return True

    def do_GET(self):
        if not self.authorized():
            return
        gateway = self.server.gateway
        if self.path == '/health/live':
            return self.reply(200, {'status': 'live'})
        if self.path in ('/health/ready', '/v1/models'):
            models = {kind: backend.info() for kind, backend in gateway.backends.items()}
            gpu = None
            # Report allocator data only if torch is already loaded by a model.
            import sys
            torch = sys.modules.get('torch')
            if torch and torch.cuda.is_available():
                free, total = torch.cuda.mem_get_info()
                gpu = dict(free_bytes=free, total_bytes=total)
            return self.reply(200, dict(status='ready' if any(b.ready() for b in gateway.backends.values()) else 'unavailable',
                                        models=models, gpu=gpu, queue_size=gateway.queue.qsize()))
        self.reply(404, {'status': 'not_found'})

    def do_POST(self):
        if not self.authorized():
            return
        if self.path not in ['/v1/infer/' + kind for kind in KINDS]:
            return self.reply(404, {'status': 'not_found'})
        try:
            if self.headers.get('Transfer-Encoding'):
                raise ValueError()
            length = int(self.headers.get('Content-Length', '-1'))
            if length > MAX_INPUT:
                return self.reply(413, {'status': 'too_large'})
            if length <= 0 or self.headers.get('Content-Type') not in ('image/jpeg', 'image/png'):
                raise ValueError()
            metadata = {key: identifier(self.headers.get(header)) for key, header in
                        [('request_id', 'X-Request-Id'), ('task_id', 'X-Task-Id'), ('capture_id', 'X-Capture-Id')]}
            identifier(self.headers.get('X-Robot-Id'))
            capture_time = self.headers.get('X-Capture-Time', '')
            if not 1 <= len(capture_time) <= 64 or datetime.fromisoformat(capture_time.replace('Z', '+00:00')).tzinfo is None:
                raise ValueError()
            raw = self.rfile.read(length)
            if len(raw) != length:
                raise ValueError()
        except (ValueError, OSError):
            return self.reply(400, {'status': 'bad_request'})
        now = time.monotonic()
        work = Work(self.path.rsplit('/', 1)[1], raw, metadata, now, now + self.server.gateway.timeout)
        work.context = dict(_content_type=self.headers['Content-Type'],
                            capture_time=self.headers.get('X-Capture-Time', ''),
                            robot_id=self.headers['X-Robot-Id'])
        self.reply(200, self.server.gateway.submit(work))


def load_backends(config):
    backends = {}
    for kind in KINDS:
        entry = config.get('models', {}).get(kind)
        if not entry:
            backends[kind] = Unavailable()
            continue
        try:
            if kind in ('helmet', 'pcb'):
                from .backends.yolo import YoloBackend
                backends[kind] = YoloBackend(kind, entry)
            elif kind == 'tongue':
                from .backends.tongue import TongueBackend
                backends[kind] = TongueBackend(entry)
            else:
                backends[kind] = Unavailable()
        except Exception as exc:
            # Each failure is independent; never silently substitute another model.
            backends[kind] = Unavailable(type(exc).__name__)
    return backends


def read_token(path):
    import os
    import stat
    if not path:
        return ''
    fd = os.open(Path(path).expanduser(), os.O_RDONLY | os.O_NOFOLLOW)
    try:
        st = os.fstat(fd)
        if not stat.S_ISREG(st.st_mode) or stat.S_IMODE(st.st_mode) != 0o600 or st.st_uid != os.getuid():
            raise ValueError('token_requires_owner_0600')
        token = os.read(fd, 4097).decode().strip()
        if not token or len(token) > 4096 or any(ord(c) < 33 or ord(c) > 126 for c in token):
            raise ValueError('invalid_token')
        return token
    finally:
        os.close(fd)


def main():
    import yaml
    parser = argparse.ArgumentParser()
    parser.add_argument('--config', type=Path, required=True)
    args = parser.parse_args()
    config = yaml.safe_load(args.config.read_text())
    token = read_token(config.get('token_file', ''))
    gateway = Gateway(load_backends(config), token=token, max_queue=config.get('max_queue', 4),
                      timeout_sec=config.get('timeout_sec', 120.))
    server = Server((config.get('host', '127.0.0.1'), config.get('port', 8200)), gateway)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close(); gateway.close()


if __name__ == '__main__':
    main()
