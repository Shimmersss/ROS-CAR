#!/usr/bin/env python3
"""Call /inspect/<kind>, saving validated result metadata and JPEG locally."""
import argparse
import hashlib
import json
from pathlib import Path
import time
import uuid


def main():
    import rclpy
    from capture_task_interfaces.srv import RunInspection
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('task_type', choices=('face', 'tongue', 'helmet', 'pcb'))
    parser.add_argument('--task-id', default='')
    parser.add_argument('--timeout', type=float, default=180.)
    parser.add_argument('--prepare', type=float, default=-1.)
    parser.add_argument('--output', type=Path, default=Path('artifacts/inspection'))
    args = parser.parse_args()
    task_id = args.task_id or 'inspect-' + uuid.uuid4().hex
    from capture_task.protocol import identifier
    identifier(task_id)
    rclpy.init()
    node = rclpy.create_node('inspection_client')
    client = node.create_client(RunInspection, '/inspect/' + args.task_type)
    try:
        if not client.wait_for_service(timeout_sec=5.):
            raise RuntimeError('inspection_service_unavailable')
        future = client.call_async(RunInspection.Request(task_id=task_id, announce='',
            prepare_sec=args.prepare, timeout_sec=args.timeout))
        rclpy.spin_until_future_complete(node, future, timeout_sec=args.timeout + 40.)
        if not future.done():
            # A ROS service future cancel does not cancel the server task: use owned cancel.
            from capture_task_interfaces.srv import CancelCapture
            cancel = node.create_client(CancelCapture, '/inspect/cancel')
            if cancel.wait_for_service(timeout_sec=1.):
                sent = cancel.call_async(CancelCapture.Request(task_id=task_id))
                rclpy.spin_until_future_complete(node, sent, timeout_sec=2.)
            raise TimeoutError('inspection_client_timeout')
        response = future.result()
        record = dict(success=response.success, status=response.status, detail=response.detail,
                      task_id=response.task_id, capture_id=response.capture_id,
                      analysis=json.loads(response.result_json) if response.result_json else None)
        root = args.output/task_id
        root.mkdir(parents=True, exist_ok=True)
        if response.annotated_image.data:
            raw = bytes(response.annotated_image.data)
            expected = record['analysis']['annotated_image']['sha256']
            if response.annotated_image.format != 'jpeg' or hashlib.sha256(raw).hexdigest() != expected:
                raise ValueError('inspection_image_hash_mismatch')
            (root/'annotated.jpg').write_bytes(raw)
        (root/'result.json').write_text(json.dumps(record, ensure_ascii=False, indent=2)+'\n')
        print(json.dumps(dict(status=response.status, detail=response.detail, output=str(root.resolve())), ensure_ascii=False))
        return 0 if response.success else 1
    finally:
        node.destroy_node();rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
