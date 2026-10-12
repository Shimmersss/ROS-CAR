"""Port-injected single capture flow; adapted from tongue_mode/flow.py.

The ROS adapter owns cancellation, deadline, resources and final persistence.
"""
import hashlib
from datetime import datetime, timezone
import uuid
from .encode import encode_jpeg
from .quality import measure, warnings


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
        selected = [max(frames,key=lambda item:item[1]['sharpness'])] if task['upload'] == 'best' else frames
        records = []
        # Check all selected frames before uploading any in fail policy.
        for frame, metrics in selected:
            issues = warnings(metrics,task)
            if issues and defaults['quality_policy'] == 'fail':
                raise ValueError('quality_failed:'+','.join(issues))
        ports.stage('UPLOAD')
        for index, (frame, metrics) in enumerate(selected):
            ports.check()
            issues = warnings(metrics,task)
            image = encode_jpeg(frame.rgb,defaults['jpeg_quality'])
            capture_id = 'cap-'+uuid.uuid4().hex
            record = dict(task_id=job['task_id'],task_type=job['task_type'],capture_id=capture_id,
                          frame_index=index, frame_sequence=frame.sequence, frame_stamp=frame.ros_stamp,
                          capture_time=datetime.fromtimestamp(frame.ros_stamp,timezone.utc).isoformat(), quality=metrics,
                          warnings=issues, bytes=len(image),sha256=hashlib.sha256(image).hexdigest())
            record['receipt'] = ports.upload(image,record)
            ports.check()
            ports.record(record,image)
            records.append(record)
        ports.stage('DONE_ANNOUNCE'); ports.speak(defaults['done_announce'])
        ports.stage('UPLOADED', ','.join(sorted({warning for record in records for warning in record['warnings']})))
        return records
