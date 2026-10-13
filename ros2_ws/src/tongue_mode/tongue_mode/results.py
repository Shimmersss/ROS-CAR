"""Strict response contract and deterministic spoken summaries."""
import math

LABELS = {'TonguePale': '舌色偏淡', 'TipSideRed': '舌尖或舌边偏红',
          'Spot': '红点', 'Ecchymosis': '瘀斑', 'Crack': '裂纹',
          'Toothmark': '齿痕', 'FurThick': '厚苔', 'FurYellow': '黄苔'}
ORDER = ('FurYellow', 'FurThick', 'TonguePale', 'TipSideRed', 'Spot',
         'Ecchymosis', 'Crack', 'Toothmark')


class AnalysisError(RuntimeError):
    pass


def validate_response(body, request_id, capture_id, pipeline_id, digest, session_id=None):
    if not isinstance(body, dict):
        raise AnalysisError('Response is not an object')
    if body.get('request_id') != request_id:
        raise AnalysisError('Request id mismatch')
    status = body.get('status')
    if status in ('busy', 'timeout', 'invalid_input', 'error'):
        return body
    if status not in ('ok', 'uncertain', 'retake'):
        raise AnalysisError('Unknown response status')
    if body.get('capture_id') != capture_id:
        raise AnalysisError('Capture id mismatch')
    if session_id is not None and body.get('session_id') != session_id:
        raise AnalysisError('Session id mismatch')
    if body.get('pipeline_id') != pipeline_id or body.get('pipeline_digest') != digest:
        raise AnalysisError('Pipeline version mismatch')
    labels = body.get('labels')
    if status == 'retake':
        if body.get('classification_performed') or labels:
            raise AnalysisError('Retake response contains classification')
        return body
    if body.get('classification_performed') is not True or not isinstance(labels, dict):
        raise AnalysisError('Classification missing')
    if set(labels) != set(LABELS):
        raise AnalysisError('Incomplete label set')
    for item in labels.values():
        if not isinstance(item, dict):
            raise AnalysisError('Invalid label')
        for field in ('score', 'threshold'):
            value = item.get(field)
            if isinstance(value, bool) or not isinstance(value, (float, int)) or not math.isfinite(value) or not 0 <= value <= 1:
                raise AnalysisError('Invalid probability or threshold')
        decision = item.get('decision')
        if decision not in ('positive', 'negative', 'uncertain'):
            raise AnalysisError('Invalid decision')
        if decision != 'uncertain' and (decision == 'positive') != (item['score'] >= item['threshold']):
            raise AnalysisError('Score/threshold decision inconsistent')
        if status == 'ok' and decision == 'uncertain':
            raise AnalysisError('OK response contains uncertain decision')
    return body


def spoken_result(body):
    status = body.get('status')
    if status == 'retake':
        return '舌象不清晰，请重新拍摄。'
    errors = {'busy': '服务繁忙，请稍后说重新拍摄。',
              'timeout': '这次连接超时，请稍后说重新拍摄。',
              'invalid_input': '这次图片提交失败，请重新拍摄。',
              'error': '这次没有获得完整结果，请稍后重新拍摄。'}
    if status in errors:
        return errors[status]
    positives = [LABELS[key] for key in ORDER if body['labels'][key]['decision'] == 'positive']
    text = ('本次识别到' + '、'.join(positives) + '。') if positives else '本次没有明确检出这八项舌象特征。'
    if status == 'uncertain':
        text += '部分特征暂不确定，可以重新拍摄。'
    return text
