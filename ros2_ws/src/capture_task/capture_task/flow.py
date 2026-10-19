"""Port-injected single capture flow; adapted from tongue_mode/flow.py.

The ROS adapter owns cancellation, deadline, resources and final persistence.
"""
import hashlib
from datetime import datetime, timezone
import uuid
from .encode import encode_jpeg
from .quality import measure, warnings
from .inference_client import safe_summary


def failure_result(ports, detail):
    """A notification failure cannot replace the original task outcome."""
    try:
        ports.speak_failure(detail)
    except Exception as exc:
        try:
            ports.failure_notice_skipped(type(exc).__name__)
        except Exception:
            pass  # Even failure-event storage is best effort.
    return dict(status='failed', detail=detail)


class CaptureFlow:
    def run(self, ports, job, task, defaults):
        ports.stage('CHECK'); ports.ready()
        if ports.robot:
            ports.stage('HOLD'); ports.acquire()
        ports.stage('ANNOUNCE'); ports.speak(task['announce'])
        ports.stage('PREPARE'); ports.wait(task['prepare_sec'])
        ports.stage('CAPTURE')
        frames = []
        after, ros_after = ports.window()
        for index in range(task['burst']):
            if index:
                ports.wait(.1)
                after, ros_after = ports.window()
            frame = ports.capture(after, ros_after)
            frames.append((frame, measure(frame.rgb)))
        ports.stage('QUALITY')
        infer = getattr(ports, 'pipeline_mode', 'upload') == 'infer'
        collect = getattr(ports, 'pipeline_mode', 'upload') == 'collect'
        selected = [max(frames,key=lambda item:item[1]['sharpness'])] if infer or collect or task['upload'] == 'best' else frames
        records = []
        # Check all selected frames before uploading any in fail policy.
        for frame, metrics in selected:
            issues = warnings(metrics,task)
            if issues and defaults['quality_policy'] == 'fail':
                raise ValueError('quality_failed:'+','.join(issues))
        ports.stage('COLLECT' if collect else 'INFER' if infer else 'UPLOAD')
        for index, (frame, metrics) in enumerate(selected):
            ports.check()
            issues = warnings(metrics,task)
            image = encode_jpeg(frame.rgb,defaults['jpeg_quality'])
            capture_id = 'cap-'+uuid.uuid4().hex
            record = dict(task_id=job['task_id'],task_type=job['task_type'],capture_id=capture_id,
                          frame_index=index, frame_sequence=frame.sequence, frame_stamp=frame.ros_stamp,
                          capture_time=datetime.fromtimestamp(frame.ros_stamp,timezone.utc).isoformat(), quality=metrics,
                          warnings=issues, bytes=len(image),sha256=hashlib.sha256(image).hexdigest())
            if collect:
                ports.collect(image, record)
            elif infer:
                analysis = ports.infer(image, record)
                record['analysis'] = analysis
            else:
                record['receipt'] = ports.upload(image,record)
            ports.check()
            ports.record(record,image)
            records.append(record)
        if collect:
            ports.stage('DONE_ANNOUNCE'); ports.speak('图像采集完成，面诊模型尚未接入。')
            ports.stage('CAPTURED', 'model_not_integrated')
        elif infer:
            status = records[0]['analysis']['status']
            ports.stage('DONE_ANNOUNCE')
            if status in ('ok', 'uncertain'):
                text = safe_summary(job['task_type'], records[0]['analysis']) if task.get('inference', {}).get('speak_summary', True) else defaults.get('analysis_done_announce', '图像分析完成，详细结果已返回。')
                ports.speak(text)
                ports.stage('ANALYZED', status)
            elif status == 'retake':
                ports.speak(defaults.get('retake_announce', '这次没有拍清楚，请重新发起检测。'))
                ports.stage('RETAKE', 'retake')
            else:
                raise ValueError('inference_' + status)
        else:
            ports.stage('DONE_ANNOUNCE'); ports.speak(defaults['done_announce'])
            ports.stage('UPLOADED', ','.join(sorted({warning for record in records for warning in record['warnings']})))
        return records
