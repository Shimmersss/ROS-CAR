"""Signing helpers shared by the iFLYTEK WebSocket APIs."""

import base64
from datetime import datetime, timezone
from email.utils import format_datetime
import hashlib
import hmac
from urllib.parse import urlencode, urlsplit, urlunsplit


def build_signed_websocket_url(endpoint, api_key, api_secret, now=None):
    """Return an iFLYTEK HMAC-authenticated WebSocket URL."""
    if not api_key or not api_secret:
        raise ValueError('api_key and api_secret are required')

    parsed = urlsplit(endpoint)
    if parsed.scheme not in {'ws', 'wss'} or not parsed.hostname:
        raise ValueError('endpoint must be a ws:// or wss:// URL')

    current = now or datetime.now(timezone.utc)
    if current.tzinfo is None:
        current = current.replace(tzinfo=timezone.utc)
    date = format_datetime(current.astimezone(timezone.utc), usegmt=True)
    request_path = parsed.path or '/'
    if parsed.query:
        request_path += '?' + parsed.query

    signature_origin = (
        f'host: {parsed.hostname}\n'
        f'date: {date}\n'
        f'GET {request_path} HTTP/1.1'
    )
    signature_sha = hmac.new(
        api_secret.encode('utf-8'),
        signature_origin.encode('utf-8'),
        digestmod=hashlib.sha256,
    ).digest()
    signature = base64.b64encode(signature_sha).decode('ascii')
    authorization_origin = (
        f'api_key="{api_key}", algorithm="hmac-sha256", '
        f'headers="host date request-line", signature="{signature}"'
    )
    authorization = base64.b64encode(
        authorization_origin.encode('utf-8')
    ).decode('ascii')
    query = urlencode({
        'authorization': authorization,
        'date': date,
        'host': parsed.hostname,
    })
    return urlunsplit((parsed.scheme, parsed.netloc, parsed.path, query, ''))
