import base64
from datetime import datetime, timezone
import json
from urllib.parse import parse_qs, urlsplit

from xfyun_speech.auth import build_signed_websocket_url
from xfyun_speech.protocol import (
    IatResultAssembler,
    decode_tts_response,
    make_iat_frame,
    make_tts_request,
)


def test_signed_url_contains_expected_public_fields():
    now = datetime(2026, 9, 14, 12, 0, tzinfo=timezone.utc)
    url = build_signed_websocket_url(
        'wss://iat-api.xfyun.cn/v2/iat', 'test-key', 'test-secret', now=now)
    parsed = urlsplit(url)
    query = parse_qs(parsed.query)
    assert parsed.path == '/v2/iat'
    assert query['host'] == ['iat-api.xfyun.cn']
    assert query['date'] == ['Mon, 14 Sep 2026 12:00:00 GMT']
    authorization = base64.b64decode(query['authorization'][0]).decode('utf-8')
    assert 'api_key="test-key"' in authorization
    assert 'algorithm="hmac-sha256"' in authorization


def test_iat_assembler_applies_dynamic_replacement():
    assembler = IatResultAssembler()
    first = {
        'code': 0,
        'data': {'status': 1, 'result': {
            'sn': 0, 'pgs': 'apd', 'ws': [{'cw': [{'w': '你好'}]}],
        }},
    }
    replacement = {
        'code': 0,
        'data': {'status': 2, 'result': {
            'sn': 1, 'pgs': 'rpl', 'rg': [0, 0],
            'ws': [{'cw': [{'w': '你好呀'}]}],
        }},
    }
    assert assembler.consume(first) == ('你好', False)
    assert assembler.consume(replacement) == ('你好呀', True)


def test_iat_first_frame_and_tts_round_trip():
    frame = make_iat_frame('appid', b'\x01\x02', 0)
    assert frame['common']['app_id'] == 'appid'
    assert base64.b64decode(frame['data']['audio']) == b'\x01\x02'

    request = make_tts_request('appid', '测试')
    assert base64.b64decode(request['data']['text']).decode('utf-8') == '测试'
    payload = json.loads(json.dumps({
        'code': 0,
        'data': {'audio': base64.b64encode(b'pcm').decode('ascii'), 'status': 2},
    }))
    assert decode_tts_response(payload) == (b'pcm', True)
