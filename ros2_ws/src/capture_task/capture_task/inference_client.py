"""Strict inference contract and safe local summaries; no ROS or model imports."""
import base64
import binascii
from copy import deepcopy
import hashlib
import json
import math
import time
import uuid
from .catalog import speech_text
from .cloud_client import CloudClient, CloudError

MAX_INFERENCE_JSON = 8 * 1024 * 1024  # Separate from the unchanged HTTP-v0 MAX_JSON.
LABELS = {'TonguePale', 'TipSideRed', 'Spot', 'Ecchymosis', 'Crack', 'Toothmark', 'FurThick', 'FurYellow'}
PIPELINE_ID = 'cloud_r50_384_cvAC10_passthrough_debug'
PIPELINE_DIGEST = '1a897fa5e7967eef62d60de50c98a1909224008e27054dec8e31a10e8f76aef1'
STATUSES = {'ok', 'uncertain', 'retake', 'model_unavailable', 'invalid_input', 'busy', 'timeout', 'error'}


def inference_loads(raw):
    if not isinstance(raw, (str, bytes)) or len(raw if isinstance(raw, bytes) else raw.encode()) > MAX_INFERENCE_JSON:
        raise ValueError('inference_response_too_large')
    def pairs(items):
        out = {}
        for key, value in items:
            if key in out:
                raise ValueError('duplicate_key')
            out[key] = value
        return out
    def invalid(_):
        raise ValueError('nonfinite_json')
    try:
        value = json.loads(raw, object_pairs_hook=pairs, parse_constant=invalid)
    except (UnicodeError, RecursionError, json.JSONDecodeError) as exc:
        raise ValueError('invalid_inference_json') from exc
    if not isinstance(value, dict):
        raise ValueError('expected_inference_object')
    return value


def numeric(value, lo, hi):
    if type(value) not in (int, float) or not math.isfinite(value) or not lo <= value <= hi:
        raise ValueError('invalid_inference_number')
    return value


def image_shape(value):
    if not isinstance(value, dict):
        raise ValueError('invalid_image_dimensions')
    width, height = value.get('width'), value.get('height')
    if type(width) is not int or type(height) is not int or not 0 < width * height <= 40000000 or width <= 0 or height <= 0:
        raise ValueError('invalid_image_dimensions')
    return width, height


def validate_tongue_result(result, summary):
    if not isinstance(result, dict):
        raise ValueError('invalid_tongue_result')
    performed = result.get('classification_performed')
    if result.get('pipeline_id') != PIPELINE_ID or result.get('pipeline_digest') != PIPELINE_DIGEST:
        raise ValueError('tongue_manifest_mismatch')
    if type(performed) is not bool or not isinstance(result.get('labels'), dict) or set(result['labels']) != (LABELS if performed else set()):
        raise ValueError('tongue_labels_mismatch')
    positives = []
    for label, item in result['labels'].items():
        if not isinstance(item, dict) or item.get('decision') not in ('positive', 'negative', 'uncertain'):
            raise ValueError('invalid_tongue_label')
        numeric(item.get('score'), 0, 1); numeric(item.get('threshold'), 0, 1)
        if item['decision'] == 'positive':
            positives.append(label)
    if not isinstance(result.get('positives'), list) or set(result['positives']) != set(positives) or len(result['positives']) != len(positives) or summary['count'] != len(positives) or summary['positive'] != bool(positives):
        raise ValueError('inconsistent_tongue_summary')


def validate_response(value, kind, ids):
    """Return metadata without base64 plus a hash- and decoder-verified JPEG."""
    import cv2
    import numpy as np
    if not isinstance(value, dict) or value.get('status') not in STATUSES or value.get('task_type') != kind:
        raise ValueError('invalid_inference_status')
    if any(value.get(key) != expected for key, expected in ids.items()):
        raise ValueError('inference_id_mismatch')
    model, summary, timing = value.get('model'), value.get('summary'), value.get('timings_ms')
    if not isinstance(model, dict) or any(not isinstance(model.get(k), str) for k in ('name', 'version', 'weights_sha256', 'device')):
        raise ValueError('invalid_model_info')
    if not isinstance(summary, dict) or type(summary.get('positive')) is not bool or type(summary.get('count')) is not int or not 0 <= summary['count'] <= 10000 or not isinstance(summary.get('text_key'), str):
        raise ValueError('invalid_summary')
    if not isinstance(timing, dict) or not {'queue', 'decode', 'inference', 'annotate', 'total'} <= set(timing):
        raise ValueError('invalid_timings')
    for key in ('queue', 'decode', 'inference', 'annotate', 'total'):
        numeric(timing[key], 0, 1e9)
    clean = deepcopy(value)
    data = value.get('annotated_image')
    if kind == 'tongue' and value['status'] in ('ok', 'uncertain', 'retake'):
        validate_tongue_result(value.get('result'), summary)
        result=value['result']
        if result['classification_performed'] != (value['status'] != 'retake') or not isinstance(result.get('quality'),dict):
            raise ValueError('invalid_tongue_status_contract')
        roi=result.get('roi')
        if roi is not None and not isinstance(roi,dict):
            raise ValueError('invalid_tongue_roi')
        if roi and 'box_original' in roi:
            width,height=image_shape(value.get('image'))
            box=roi['box_original']
            if not isinstance(box,list) or len(box)!=4:
                raise ValueError('invalid_tongue_roi')
            for i,coordinate in enumerate(box):
                numeric(coordinate,0,width if i%2==0 else height)
            if box[0]>=box[2] or box[1]>=box[3]:
                raise ValueError('invalid_tongue_roi')
    if value['status'] not in ('ok', 'uncertain'):
        if data is not None:
            raise ValueError('image_on_failed_inference')
        return clean, b''
    width, height = image_shape(value.get('image'))
    result = value.get('result')
    if not isinstance(result, dict):
        raise ValueError('invalid_result')
    if kind in ('pcb', 'helmet'):
        rows = result.get('defects' if kind == 'pcb' else 'detections')
        if not isinstance(rows, list) or len(rows) > 10000:
            raise ValueError('invalid_detections')
        for row in rows:
            if not isinstance(row, dict) or type(row.get('cls')) is not int or row['cls'] < 0 or not isinstance(row.get('name'), str):
                raise ValueError('invalid_detection_class')
            numeric(row.get('score'), 0, 1)
            box = row.get('xyxy')
            if not isinstance(box, list) or len(box) != 4:
                raise ValueError('invalid_box')
            for i, coordinate in enumerate(box):
                numeric(coordinate, 0, width if i % 2 == 0 else height)
            if box[2] < box[0] or box[3] < box[1]:
                raise ValueError('invalid_box')
        if kind == 'pcb':
            expected_names = ('missing_hole','mouse_bite','open_circuit','short','spur','spurious_copper')
            if any(row['cls'] >= len(expected_names) or expected_names[row['cls']] != row['name'] for row in rows):
                raise ValueError('unknown_pcb_class')
            if result.get('imgsz') != 640 or result.get('conf_threshold') != .2:
                raise ValueError('pcb_threshold_mismatch')
            count = len(rows)
            if type(result.get('reject')) is not bool or result['reject'] != bool(count):
                raise ValueError('invalid_pcb_result')
        else:
            if result.get('imgsz') != 640 or result.get('conf_threshold') != .45 or result.get('iou_threshold') != .45:
                raise ValueError('helmet_threshold_mismatch')
            if any(type(result.get(k)) is not int for k in ('helmet_count','no_helmet_count')):
                raise ValueError('invalid_helmet_count')
            if any(row['name'] not in ('helmet', 'no_helmet') for row in rows):
                raise ValueError('unknown_helmet_class')
            count = sum(row['name'] == 'no_helmet' for row in rows)
            if type(result.get('violation')) is not bool or result['violation'] != bool(count) or result.get('no_helmet_count') != count or result.get('helmet_count') != len(rows) - count:
                raise ValueError('invalid_helmet_result')
        if summary['count'] != count or summary['positive'] != bool(count):
            raise ValueError('inconsistent_summary')
    if not isinstance(data, dict) or data.get('content_type') != 'image/jpeg' or not isinstance(data.get('data_b64'), str):
        raise ValueError('missing_annotation')
    aw, ah = image_shape(data)
    ratio = min(1., 1280 / max(width, height))
    if (aw, ah) != (round(width * ratio), round(height * ratio)):
        raise ValueError('annotation_dimensions_mismatch')
    try:
        image = base64.b64decode(data['data_b64'], validate=True)
    except (ValueError, binascii.Error) as exc:
        raise ValueError('invalid_annotation_base64') from exc
    if not image.startswith(b'\xff\xd8') or hashlib.sha256(image).hexdigest() != data.get('sha256'):
        raise ValueError('annotation_hash_mismatch')
    # JPEG dimensions must match the small advertised bitmap before full decode.
    import io
    from PIL import Image
    try:
        with Image.open(io.BytesIO(image)) as probe:
            if probe.format != 'JPEG' or probe.size != (aw, ah):
                raise ValueError('annotation_jpeg_dimensions')
        decoded = cv2.imdecode(np.frombuffer(image, np.uint8), cv2.IMREAD_COLOR)
    except Exception as exc:
        raise ValueError('invalid_annotation_jpeg') from exc
    if decoded is None or decoded.shape[:2] != (ah, aw):
        raise ValueError('invalid_annotation_jpeg')
    del clean['annotated_image']['data_b64']
    if len(json.dumps(clean, ensure_ascii=False, allow_nan=False).encode()) > 1024 * 1024:
        raise ValueError('inference_metadata_too_large')
    return clean, image


def safe_summary(kind, value):
    """Never speak arbitrary model text, exception text or diagnostic labels."""
    count = value['summary']['count']
    if type(count) is not int or not 0 <= count <= 10000:
        raise ValueError('invalid_summary_count')
    if kind == 'pcb':
        text = '电路板检测完成，发现%d处疑似缺陷。' % count if count else '电路板检测完成，未发现缺陷。'
    elif kind == 'helmet':
        text = '检测到%d人未佩戴安全帽。' % count if count else '未发现未佩戴安全帽的人员。'
    elif kind == 'tongue':
        text = '舌象分析完成，识别到%d项特征，详细结果已返回。' % count
    else:
        text = '图像分析完成，详细结果已返回。'
    return speech_text(text)


class InferenceClient(CloudClient):
    response_limit = MAX_INFERENCE_JSON
    parse_response = staticmethod(inference_loads)

    def infer(self, kind, image, record, robot_id, cancel, deadline, timeout):
        ids = dict(request_id='infer-' + uuid.uuid4().hex, task_id=record['task_id'], capture_id=record['capture_id'])
        headers = {'Content-Type': 'image/jpeg', 'X-Request-Id': ids['request_id'], 'X-Task-Id': ids['task_id'],
                   'X-Capture-Id': ids['capture_id'], 'X-Robot-Id': robot_id, 'X-Capture-Time': record['capture_time']}
        deadline = min(deadline, time.monotonic() + timeout)
        status, value = self.request('POST', '/v1/infer/' + kind, image, headers, cancel, deadline, timeout)
        if status != 200:
            raise CloudError(status, 'inference_http_' + str(status))
        return validate_response(value, kind, ids)
