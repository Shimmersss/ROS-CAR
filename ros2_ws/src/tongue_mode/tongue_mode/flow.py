"""One capture's orchestration, testable through explicit hardware/transport ports."""
import time
import uuid
from datetime import datetime, timezone
from .capture import encode_jpeg
from .results import validate_response, spoken_result


class DetectionFlow:
    def __init__(self, ports, config):
        self.ports, self.config = ports, config

    def run(self, session_id, cancel, first=False):
        p, c = self.ports, self.config
        ids = dict(session_id=session_id, capture_id=uuid.uuid4().hex, request_id=uuid.uuid4().hex)
        p.stage('CHECK_CLOUD')
        p.speak('检查检测服务')
        p.ready()
        p.stage('PROMPT')
        p.speak('请正对相机，伸舌保持不动。')
        p.stage('PREPARE')
        p.speak('准备拍摄')
        p.wait(c['prepare_sec'])
        p.stage('CAPTURE')
        p.speak('正在拍摄')
        window = time.monotonic()
        frame = p.capture(window, cancel)
        capture_time = datetime.now(timezone.utc).isoformat()
        jpeg = encode_jpeg(frame, c['jpeg_quality'])
        p.stage('ANALYZING')
        p.speak('正在分析')
        body = p.analyze(jpeg, ids, capture_time)
        body = validate_response(body, ids['request_id'], ids['capture_id'],
                                 c['expected_pipeline_id'], c['expected_pipeline_digest'], session_id)
        record = dict(ids=ids, frame_sequence=frame.sequence, ros_stamp=frame.ros_stamp,
                      capture_time=capture_time, width=int(frame.rgb.shape[1]),
                      height=int(frame.rgb.shape[0]), image_bytes=len(jpeg), response=body)
        # Save only validated responses. Never re-use a prior successful capture on error.
        p.save(record, jpeg)
        p.stage('SPEAK_RESULT', body)
        p.speak(spoken_result(body))
        return body
