"""Proxy for the frozen tongue package; its files and thresholds stay untouched."""
import http.client
import json
import time

LABELS = ('TonguePale', 'TipSideRed', 'Spot', 'Ecchymosis', 'Crack', 'Toothmark', 'FurThick', 'FurYellow')
PIPELINE_ID = 'cloud_r50_384_cvAC10_passthrough_debug'
DIGEST = '1a897fa5e7967eef62d60de50c98a1909224008e27054dec8e31a10e8f76aef1'


class TongueBackend:
    def __init__(self, config):
        self.host, self.port = '127.0.0.1', int(config.get('port', 8096))
        expected_device = config.get('device', 'cuda')
        if expected_device not in ('cpu', 'cuda', 'cuda:0'):
            raise ValueError('invalid_tongue_device')
        self.manifest = self.request('GET', '/v1/model-info', timeout=5.)
        if self.manifest.get('pipeline_id') != PIPELINE_ID or self.manifest.get('pipeline_digest') != DIGEST:
            raise ValueError('tongue_manifest_mismatch')
        if set(self.manifest.get('labels', [])) != set(LABELS):
            raise ValueError('tongue_model_labels_mismatch')
        self.device = self.manifest.get('device')
        if self.device not in (('cuda', 'cuda:0') if expected_device.startswith('cuda') else ('cpu',)):
            raise ValueError('tongue_device_mismatch')
        health = self.request('GET', '/health/ready', timeout=5.)
        if health.get('status') != 'ready':
            raise ValueError('tongue_not_ready')

    def request(self, method, path, body=None, headers=None, timeout=120.):
        conn = http.client.HTTPConnection(self.host, self.port, timeout=max(.01, timeout))
        try:
            conn.request(method, path, body, headers or {})
            response = conn.getresponse()
            raw = response.read(2 * 1024 * 1024 + 1)
            if len(raw) > 2 * 1024 * 1024:
                raise ValueError('tongue_bad_response')
            value = json.loads(raw)
            if response.status != 200 and value.get('status') not in ('busy','timeout','error','invalid_input'):
                raise ValueError('tongue_bad_response')
            return value
        finally:
            conn.close()

    def ready(self):
        return True

    def info(self):
        return dict(name='teacher_cvAC10', version=PIPELINE_ID, weights_sha256=DIGEST,
                    device=self.device, ready=True, pipeline_digest=DIGEST, labels=list(LABELS))

    def infer(self, image, metadata, deadline):
        import cv2
        raw = metadata.get('_raw')
        content_type = metadata.get('_content_type', 'image/jpeg')
        if raw is None:  # Explicit direct-backend probes, not the HTTP pipeline.
            ok, encoded = cv2.imencode('.jpg', image, [cv2.IMWRITE_JPEG_QUALITY, 95])
            if not ok:
                raise ValueError('tongue_encode_failed')
            raw = encoded.tobytes()
        value = self.request('POST', '/v1/tongue/analyze', raw,
                             {'Content-Type': content_type, 'X-Request-Id': metadata['request_id'],
                              'X-Capture-Id': metadata['capture_id'], 'X-Session-Id': metadata['task_id'],
                              'X-Capture-Time': metadata.get('capture_time','')},
                             timeout=deadline - time.monotonic())
        if value.get('status') in ('busy','timeout','error','invalid_input'):
            return dict(status=value['status'],result={},summary=dict(positive=False,count=0,text_key=value['status']))
        if value.get('request_id') != metadata['request_id'] or value.get('capture_id') != metadata['capture_id'] or value.get('session_id') != metadata['task_id']:
            raise ValueError('tongue_id_mismatch')
        if value.get('pipeline_id') != PIPELINE_ID or value.get('pipeline_digest') != DIGEST:
            raise ValueError('tongue_manifest_mismatch')
        performed = value.get('classification_performed')
        if type(performed) is not bool or not isinstance(value.get('labels'), dict) or set(value['labels']) != (set(LABELS) if performed else set()):
            raise ValueError('tongue_labels_mismatch')
        if value.get('status') not in ('ok', 'uncertain', 'retake'):
            raise ValueError('tongue_status_invalid')
        result = {k: value.get(k) for k in ('pipeline_id', 'pipeline_digest', 'classification_performed',
                                          'labels', 'positives', 'quality', 'roi')}
        count = len(result['positives'] or [])
        return dict(status=value['status'], result=result,
                    summary=dict(positive=bool(count), count=count, text_key='tongue_features'))
