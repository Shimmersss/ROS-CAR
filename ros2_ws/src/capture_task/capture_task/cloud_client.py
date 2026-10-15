"""Cancellable stdlib HTTP, adapted from tongue_mode/cloud.py (no dependency).

No proxy environment, redirects, URL credentials or unbounded responses.
"""
import hashlib
import http.client
import json
import os
import re
from pathlib import Path
import socket
import stat
import threading
import time
from urllib.parse import urlsplit
from .protocol import MAX_IMAGE, MAX_JSON, loads, receipt


class Cancelled(Exception):
    pass


class CloudError(Exception):
    def __init__(self, status=0, detail='network_error'):
        self.status = status
        super().__init__(detail)


def token_from_env(path='', env_name='CAPTURE_CLOUD_TOKEN'):
    value = os.environ.get(env_name, '')
    if path and not value:
        source = Path(path).expanduser()
        fd = os.open(source, os.O_RDONLY | os.O_NOFOLLOW)
        try:
            meta = os.fstat(fd)
            if not stat.S_ISREG(meta.st_mode) or stat.S_IMODE(meta.st_mode) != 0o600 or meta.st_uid != os.getuid():
                raise ValueError('token_file_requires_owner_0600')
            value = os.read(fd, 4097).decode('utf-8').strip()
        finally:
            os.close(fd)
    if len(value) > 4096 or any(ord(c) < 33 or ord(c) > 126 for c in value):
        raise ValueError('invalid_token')
    return value


def path_prefix(path):
    if path in ('', '/'):
        return ''
    if not re.fullmatch(r'(?:/[A-Za-z0-9._~-]+)+/?', path):
        raise ValueError('invalid_cloud_prefix')
    normalized = path.removesuffix('/')
    if any(segment in ('.', '..') for segment in normalized.split('/')[1:]):
        raise ValueError('invalid_cloud_prefix')
    return normalized


class CloudClient:
    response_limit = MAX_JSON
    parse_response = staticmethod(loads)
    def __init__(self, url, token='', timeout=3.):
        parsed = urlsplit(url)
        if parsed.scheme not in ('http', 'https') or not parsed.hostname or parsed.username is not None or parsed.password is not None or parsed.query or parsed.fragment or any(c.isspace() for c in url) or '?' in url or '#' in url:
            raise ValueError('invalid_cloud_url')
        self.prefix = path_prefix(parsed.path)
        parsed.port  # Validate the authority before the first request.
        self.url, self.token, self.timeout = parsed, token, timeout
        self.lock = threading.Lock()
        self.serial = threading.Lock()
        self.connection = None

    def cancel(self):
        with self.lock:
            conn = self.connection
        if conn:
            try:
                if conn.sock:
                    conn.sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            conn.close()

    def request(self, method, path, body=None, headers=None, cancel=None, deadline=None, timeout=None):
        assert path.startswith('/v1/'), 'internal_cloud_path_requires_v1'
        return self._request(method,path,body,headers,cancel,deadline,timeout)

    def admin_request(self, method, path, body=None, **kwargs):
        """Loopback mock tooling only; production clients use request (/v1/)."""
        assert path.startswith('/admin/'), 'internal_mock_admin_path'
        return self._request(method,path,body,**kwargs)

    def _request(self, method, path, body=None, headers=None, cancel=None, deadline=None, timeout=None):
        timeout = self.timeout if timeout is None else timeout
        cancel = cancel or threading.Event()
        deadline = deadline or time.monotonic() + timeout
        def check():
            if cancel.is_set():
                raise Cancelled()
            if time.monotonic() >= deadline:
                raise CloudError(0, 'deadline_exceeded')
        check()
        data = body
        fields = dict(headers or {})
        if isinstance(body, dict):
            data = json.dumps(body, ensure_ascii=False, allow_nan=False).encode('utf-8')
            fields['Content-Type'] = 'application/json'
        if data is not None and len(data) > (MAX_IMAGE if fields.get('Content-Type') == 'image/jpeg' else MAX_JSON):
            raise ValueError('request_too_large')
        if self.token:
            fields['Authorization'] = 'Bearer ' + self.token
        fields['Connection'] = 'close'
        with self.serial:
            check()
            cls = http.client.HTTPSConnection if self.url.scheme == 'https' else http.client.HTTPConnection
            conn = cls(self.url.hostname, self.url.port, timeout=max(.01, min(timeout, deadline-time.monotonic())))
            with self.lock:
                self.connection = conn
            try:
                check()
                conn.request(method, self.prefix + path, data, fields)
                check()
                response = conn.getresponse()
                raw = response.read(self.response_limit + 1)
                check()
                if len(raw) > self.response_limit:
                    raise CloudError(response.status, 'response_too_large')
                if response.status == 204:
                    return 204, {}
                try:
                    value = self.parse_response(raw)
                except ValueError as exc:
                    raise CloudError(response.status, 'invalid_response') from exc
                return response.status, value
            except (OSError, http.client.HTTPException) as exc:
                check()
                raise CloudError(0, 'network_' + type(exc).__name__) from exc
            finally:
                conn.close()
                with self.lock:
                    if self.connection is conn:
                        self.connection = None

    def upload(self, task_id, capture_id, image, metadata, cancel, deadline):
        digest = hashlib.sha256(image).hexdigest()
        headers = {'Content-Type': 'image/jpeg', 'X-Image-Sha256': digest,
                   'X-Capture-Id': capture_id, **metadata}
        for attempt in range(3):
            try:
                status, value = self.request('POST', '/v1/tasks/'+task_id+'/images', image,
                                             headers, cancel, deadline)
                if status in (200, 409):
                    return receipt(value, task_id, capture_id, digest, status == 409)
                raise CloudError(status, 'upload_http_' + str(status))
            except CloudError as exc:
                if exc.status in (401,403):
                    raise CloudError(exc.status,'upload_http_'+str(exc.status)) from exc
                if exc.status and exc.status < 500 or attempt == 2:
                    raise
                until = min(deadline, time.monotonic()+2**attempt)
                if cancel.wait(max(0, until-time.monotonic())):
                    raise Cancelled()
                if time.monotonic() >= deadline:
                    raise CloudError(0, 'deadline_exceeded')
        raise CloudError(0, 'upload_failed')
