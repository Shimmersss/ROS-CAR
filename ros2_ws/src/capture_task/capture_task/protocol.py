"""HTTP v0 schemas, bounded JSON and receipt validation."""
import json
import re
from .catalog import number, speech_text

MAX_JSON = 2 * 1024 * 1024
MAX_IMAGE = 12 * 1024 * 1024


def identifier(value):
    if not isinstance(value, str) or not re.fullmatch(r'[A-Za-z0-9_-]{1,64}', value):
        raise ValueError('invalid_id')
    return value


def loads(body):
    if not isinstance(body, (bytes, str)) or len(body if isinstance(body, bytes) else body.encode('utf-8')) > MAX_JSON:
        raise ValueError('json_too_large')
    def pairs(items):
        result = {}
        for key, value in items:
            if key in result:
                raise ValueError('duplicate_key')
            result[key] = value
        return result
    def invalid(_):
        raise ValueError('nonfinite_json')
    try:
        value = json.loads(body, object_pairs_hook=pairs, parse_constant=invalid)
    except (RecursionError, UnicodeError, json.JSONDecodeError) as exc:
        raise ValueError('invalid_json') from exc
    if not isinstance(value, dict):
        raise ValueError('expected_object')
    return value


def command(body, catalog):
    data = loads(body) if isinstance(body, (bytes, str)) else body
    if not isinstance(data, dict):
        raise ValueError('invalid_command')
    identifier(data.get('command_id'))
    kind = data.get('type')
    required = {'command_id', 'type'}
    if kind == 'capture':
        required |= {'task_id', 'task_type'}
        if not required <= set(data) or set(data) - required - {'params', 'ttl_sec'}:
            raise ValueError('invalid_fields')
        identifier(data['task_id'])
        catalog.resolve(data['task_type'], data.get('params', {}))
        number(data.get('ttl_sec', 0), 0, 600)
    elif kind == 'cancel':
        if set(data) != required | {'task_id'}:
            raise ValueError('invalid_fields')
        identifier(data['task_id'])
    elif kind == 'speak':
        if set(data) != required | {'text'}:
            raise ValueError('invalid_fields')
        speech_text(data['text'])
    else:
        raise ValueError('unknown_command')
    return data


def receipt(value, task_id, capture_id, digest=None, conflict=False):
    if not isinstance(value, dict) or value.get('status') != 'received' or value.get('task_id') != task_id or value.get('capture_id') != capture_id:
        raise ValueError('invalid_receipt')
    identifier(value.get('upload_id'))
    if conflict and value.get('sha256') != digest:
        raise ValueError('conflicting_image')
    # Return only validated fields, never arbitrary server text.
    clean = {k: value[k] for k in ('status', 'task_id', 'capture_id', 'upload_id')}
    if digest:
        clean['sha256'] = digest
    return clean
