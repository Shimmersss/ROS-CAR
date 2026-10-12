"""Loopback mock HTTP v0 reference server. Stores synthetic/test data only."""
from collections import deque
import hashlib
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import re
import secrets
import threading
import time
from urllib.parse import parse_qs, urlsplit
from .cloud_client import path_prefix
from .protocol import MAX_IMAGE, MAX_JSON, identifier, loads


class MockCloud(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = True

    def __init__(self, address, store, token='', fail_upload=0, slow_upload=0., drop_poll=False, prefix=''):
        self.prefix = path_prefix(prefix)
        super().__init__(address, Handler)
        self.store = Path(store)
        self.store.mkdir(parents=True, exist_ok=True)
        self.token, self.fail_upload, self.slow_upload, self.drop_poll = token, fail_upload, slow_upload, drop_poll
        self.condition = threading.Condition()
        self.commands, self.acks, self.events, self.heartbeats, self.images = deque(), {}, {}, [], {}
        self.upload_attempts = 0
        self.stopping = threading.Event()

    def stop(self):
        self.stopping.set()
        with self.condition:
            self.condition.notify_all()
        self.shutdown()
        self.server_close()

    def snapshot(self):
        with self.condition:
            return dict(acks=dict(self.acks), events={k:list(v) for k,v in self.events.items()},
                        images=list(self.images.values()), heartbeats=self.heartbeats[-20:],
                        queued=len(self.commands), upload_attempts=self.upload_attempts)


class Handler(BaseHTTPRequestHandler):
    protocol_version = 'HTTP/1.0'

    def log_message(self, *_):
        pass  # Never log URLs, headers, tokens or image payloads.

    def setup(self):
        super().setup()
        self.connection.settimeout(5.)

    def reply(self, code, value=None):
        body = b'' if value is None else json.dumps(value, ensure_ascii=False, allow_nan=False).encode('utf-8')
        try:
            self.send_response(code)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            self.wfile.write(body)
        except (BrokenPipeError, ConnectionResetError):
            pass

    def authorized(self):
        expected = 'Bearer ' + self.server.token
        if self.server.token and not secrets.compare_digest(self.headers.get('Authorization', ''), expected):
            self.reply(401, {'error':'unauthorized'})
            return False
        return True

    def read_body(self, limit):
        size = int(self.headers.get('Content-Length', '-1'))
        if not 0 <= size <= limit:
            raise ValueError('body_size')
        body = self.rfile.read(size)
        if len(body) != size:
            raise ValueError('short_body')
        return body

    def routed_path(self):
        path = urlsplit(self.path)
        if self.server.prefix and not path.path.startswith(self.server.prefix+'/'):
            self.reply(404, {'error':'not_found'})
            return None
        return path._replace(path=path.path[len(self.server.prefix):])

    def do_GET(self):
        if not self.authorized():
            return
        path = self.routed_path()
        if path is None:
            return
        if path.path == '/admin/status':
            self.reply(200, self.server.snapshot())
            return
        if not re.fullmatch(r'/v1/robots/[A-Za-z0-9_-]{1,64}/commands/next', path.path):
            self.reply(404, {'error':'not_found'}); return
        if self.server.drop_poll:
            self.reply(503, {'error':'injected_poll_failure'}); return
        try:
            wait = min(20., max(0., float(parse_qs(path.query).get('wait', ['20'])[0])))
            if not 0 <= wait <= 20:
                raise ValueError()
        except ValueError:
            self.reply(400, {'error':'invalid_wait'}); return
        end = time.monotonic()+wait
        with self.server.condition:
            while not self.server.commands and not self.server.stopping.is_set() and time.monotonic() < end:
                self.server.condition.wait(min(.2, end-time.monotonic()))
            data = self.server.commands.popleft() if self.server.commands else None
        self.reply(200, data) if data else self.reply(204)

    def do_POST(self):
        if not self.authorized():
            return
        try:
            self.post()
        except (ValueError, TypeError, KeyError, UnicodeError, OSError):
            self.reply(400, {'error':'invalid_request'})

    def post(self):
        routed = self.routed_path()
        if routed is None:
            return
        path = routed.path
        if path == '/admin/commands':
            data = loads(self.read_body(MAX_JSON))
            # Deliberately allow malformed commands for admission/fault tests.
            with self.server.condition:
                if len(self.server.commands) >= 2048:
                    self.reply(429, {'error':'queue_full'}); return
                self.server.commands.append(data)
                self.server.condition.notify_all()
            self.reply(200, {'queued':True, 'command_id':data.get('command_id')}); return
        match = re.fullmatch(r'/v1/robots/([A-Za-z0-9_-]{1,64})/(.+)', path)
        if match:
            data = loads(self.read_body(MAX_JSON))
            tail = match.group(2)
            with self.server.condition:
                ack = re.fullmatch(r'commands/([A-Za-z0-9_-]{1,64})/ack', tail)
                event = re.fullmatch(r'tasks/([A-Za-z0-9_-]{1,64})/events', tail)
                if ack:
                    if type(data.get('accepted')) is not bool or not isinstance(data.get('reason'), str):
                        raise ValueError()
                    self.server.acks[ack.group(1)] = data
                elif event:
                    task_id = event.group(1)
                    seq = data.get('seq')
                    if type(seq) is not int or seq < 1:
                        raise ValueError()
                    events = self.server.events.setdefault(task_id, [])
                    if events and seq <= events[-1]['seq']:
                        if any(e == data for e in events):
                            self.reply(200, {'received':True}); return
                        self.reply(409, {'error':'event_sequence'}); return
                    events.append(data)
                    folder = self.server.store / task_id
                    folder.mkdir(exist_ok=True)
                    with (folder/'events.jsonl').open('a', encoding='utf-8') as handle:
                        handle.write(json.dumps(data, ensure_ascii=False)+'\n')
                elif tail == 'heartbeat':
                    self.server.heartbeats.append(data)
                    self.server.heartbeats = self.server.heartbeats[-100:]
                else:
                    self.reply(404, {'error':'not_found'}); return
            self.reply(200, {'received':True}); return
        match = re.fullmatch(r'/v1/tasks/([A-Za-z0-9_-]{1,64})/images', path)
        if not match:
            self.reply(404, {'error':'not_found'}); return
        task_id = match.group(1)
        capture_id = identifier(self.headers.get('X-Capture-Id'))
        identifier(self.headers.get('X-Robot-Id')); identifier(self.headers.get('X-Task-Type'))
        quality = loads(self.headers.get('X-Quality', '{}'))
        if not {'brightness','sharpness','rb_ratio','width','height'} <= set(quality):
            raise ValueError()
        image = self.read_body(MAX_IMAGE)
        digest = hashlib.sha256(image).hexdigest()
        if self.headers.get('Content-Type') != 'image/jpeg' or not image.startswith(b'\xff\xd8') or digest != self.headers.get('X-Image-Sha256'):
            raise ValueError()
        with self.server.condition:
            self.server.upload_attempts += 1
        self.server.stopping.wait(self.server.slow_upload)
        with self.server.condition:
            if self.server.fail_upload in (500, 401):
                self.reply(self.server.fail_upload, {'error':'injected_upload_failure'}); return
            key = (task_id, capture_id)
            existing = self.server.images.get(key)
            if existing:
                self.reply(409, existing); return
            value = dict(status='received', task_id=task_id, capture_id=capture_id,
                         upload_id='u-'+secrets.token_hex(12), sha256=digest, quality=quality)
            folder = self.server.store / task_id
            folder.mkdir(exist_ok=True)
            (folder/(capture_id+'.jpg')).write_bytes(image)
            (folder/(capture_id+'.json')).write_text(json.dumps(value), encoding='utf-8')
            self.server.images[key] = value
        self.reply(409 if self.server.fail_upload == 409 else 200, value)
