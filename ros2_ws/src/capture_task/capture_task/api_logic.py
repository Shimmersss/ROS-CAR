"""Pure mappings for the optional capture service facade; no ROS side effects."""
import json
import math
import uuid
from .catalog import number, speech_text
from .protocol import identifier


def request_goal(kind, request, catalog):
    task_id = 'ros-'+uuid.uuid4().hex if request.task_id == '' else identifier(request.task_id)
    params = {}
    if request.announce != '':
        params['announce'] = speech_text(request.announce)
    number(request.prepare_sec, -math.inf, 30)
    if request.prepare_sec >= 0:
        params['prepare_sec'] = float(request.prepare_sec)
    number(request.timeout_sec, 0, 600)
    catalog.resolve(kind, params)
    return dict(task_id=task_id, task_type=kind, source='ros_api',
                params_json=json.dumps(params, ensure_ascii=False, allow_nan=False),
                ttl_sec=float(request.timeout_sec))


def service_timeout(timeout_sec, defaults):
    number(timeout_sec, 0, 600)
    return (timeout_sec or defaults['task_timeout_sec']) + defaults['speech_timeout_sec'] + 5.


def response_fields(task_id, status, detail='', capture_id='', upload_json=''):
    return dict(success=status == 'uploaded', status=status, detail=detail,
                task_id=task_id, capture_id=capture_id, upload_json=upload_json)


def action_result(task_id, result):
    return response_fields(task_id, result.status, result.detail, result.capture_id, result.upload_json)


def rejection_result(task_id, reason='rejected'):
    return response_fields(task_id, 'busy' if reason == 'busy' else 'rejected', reason)
