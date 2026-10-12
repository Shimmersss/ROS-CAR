"""Bounded, cancellable raw-image HTTP transport; no model dependencies."""
import http.client
import json
import socket
import threading
import time
from urllib.parse import urlsplit
from .results import AnalysisError
from .results import LABELS


class Cancelled(RuntimeError):
    pass


class CloudClient:
    def __init__(self, base_url, connect_timeout=3., response_timeout=130., bearer=''):
        self.url = urlsplit(base_url)
        if self.url.scheme not in ('http', 'https') or not self.url.hostname or self.url.username or self.url.password or self.url.query or self.url.fragment:
            raise ValueError('Expected a plain HTTP(S) base URL without credentials')
        self.connect_timeout, self.response_timeout = connect_timeout, response_timeout
        self.bearer = bearer
        self._lock = threading.Lock()
        self._connection = None
        self._serial = threading.Lock()

    def cancel(self):
        with self._lock:
            conn = self._connection
            if conn and conn.sock:
                try:
                    conn.sock.shutdown(socket.SHUT_RDWR)
                except OSError:
                    pass
            if conn:
                conn.close()

    def request(self, path, cancel, deadline, body=None, headers=None):
        if not self._serial.acquire(blocking=False):
            raise AnalysisError('A cloud request is still being cleaned up')
        conn = None
        try:
            if cancel.is_set():
                raise Cancelled()
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise AnalysisError('Task deadline exceeded')
            cls = http.client.HTTPSConnection if self.url.scheme == 'https' else http.client.HTTPConnection
            conn = cls(self.url.hostname, self.url.port, timeout=min(self.connect_timeout, remaining))
            with self._lock:
                self._connection = conn
            conn.connect()
            if cancel.is_set():
                raise Cancelled()
            conn.sock.settimeout(min(self.response_timeout, max(.1, deadline-time.monotonic())))
            send_headers = dict(headers or {})
            if self.bearer:
                send_headers['Authorization'] = 'Bearer ' + self.bearer
            conn.request('POST' if body is not None else 'GET', self.url.path.rstrip('/') + path,
                         body=body, headers=send_headers)
            response = conn.getresponse()
            raw = response.read(2 * 1024 * 1024 + 1)
            if cancel.is_set():
                raise Cancelled()
            if time.monotonic() > deadline:
                raise AnalysisError('Task deadline exceeded')
            if len(raw) > 2 * 1024 * 1024:
                raise AnalysisError('Response exceeded size limit')
            try:
                data = json.loads(raw.decode('utf-8'))
            except (ValueError, UnicodeError) as exc:
                raise AnalysisError('Invalid JSON response') from exc
            if not isinstance(data, dict):
                raise AnalysisError('Invalid response object')
            if not 200 <= response.status < 300 and data.get('status') not in ('busy', 'timeout', 'invalid_input', 'error'):
                raise AnalysisError('Unexpected HTTP status ' + str(response.status))
            return data
        except (OSError, http.client.HTTPException) as exc:
            if cancel.is_set():
                raise Cancelled() from exc
            raise AnalysisError('Cloud transport failed: ' + type(exc).__name__) from exc
        finally:
            with self._lock:
                if self._connection is conn:
                    self._connection = None
            if conn:
                conn.close()
            self._serial.release()

    def ready(self, cancel, deadline, pipeline_id, digest):
        ready = self.request('/health/ready', cancel, deadline)
        if ready.get('status') not in ('ready', 'ok'):
            raise AnalysisError('Cloud service is not ready')
        info = self.request('/v1/model-info', cancel, deadline)
        if info.get('pipeline_id') != pipeline_id or info.get('pipeline_digest') != digest:
            raise AnalysisError('Cloud pipeline version mismatch')
        if info.get('device') != 'cuda' or set(info.get('labels') or []) != set(LABELS):
            raise AnalysisError('Cloud device or label contract mismatch')
        return info

    def analyze(self, jpeg, ids, camera_profile, capture_time, cancel, deadline):
        if not jpeg or len(jpeg) > 12 * 1024 * 1024:
            raise AnalysisError('Invalid image size')
        headers = {'Content-Type': 'image/jpeg', 'X-Request-Id': ids['request_id'],
                   'X-Capture-Id': ids['capture_id'], 'X-Session-Id': ids['session_id'],
                   'X-Camera-Profile': camera_profile, 'X-Capture-Time': capture_time}
        return self.request('/v1/tongue/analyze', cancel, deadline, jpeg, headers)
