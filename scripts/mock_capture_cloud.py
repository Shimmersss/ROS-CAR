#!/usr/bin/env python3
"""Loopback reference cloud CLI; does not require ROS or installed packages."""
import argparse
import json
from pathlib import Path
import sys
import uuid
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'ros2_ws/src/capture_task'))
from capture_task.cloud_client import CloudClient, token_from_env
from capture_task.mock_cloud import MockCloud


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--url', default='http://127.0.0.1:8090')
    parser.add_argument('--robot-id', default='roscar-01')
    parser.add_argument('--token-file', default='')
    sub = parser.add_subparsers(dest='operation', required=True)
    serve = sub.add_parser('serve')
    serve.add_argument('--prefix', default='')
    serve.add_argument('--port', type=int, default=8090)
    serve.add_argument('--store', default='artifacts/capture-mock')
    serve.add_argument('--token', default='', help='Prefer CAPTURE_CLOUD_TOKEN over shell history')
    serve.add_argument('--fail-upload', type=int, choices=[500,409,401], default=0)
    serve.add_argument('--slow-upload', type=float, default=0.)
    serve.add_argument('--reject-unknown-task', action='store_true')
    serve.add_argument('--drop-poll', action='store_true')
    send = sub.add_parser('send')
    send.add_argument('task_type', choices=['face','tongue','helmet','pcb'])
    send.add_argument('--announce'); send.add_argument('--prepare', type=float)
    send.add_argument('--ttl', type=float, default=60.)
    send.add_argument('--task-id'); send.add_argument('--command-id')
    sub.add_parser('cancel').add_argument('task_id')
    sub.add_parser('speak').add_argument('text')
    sub.add_parser('status')
    sub.add_parser('result').add_argument('task_id')
    args = parser.parse_args()
    token = token_from_env(args.token_file)
    if args.operation == 'serve':
        server = MockCloud(('127.0.0.1', args.port), args.store, args.token or token,
                           args.fail_upload, args.slow_upload, args.drop_poll, args.prefix, args.reject_unknown_task)
        print('Mock cloud: loopback port', server.server_port, flush=True)
        try:
            server.serve_forever()
        except KeyboardInterrupt:
            server.stopping.set(); server.server_close()
        return
    client = CloudClient(args.url, token)
    if args.operation == 'status':
        code, value = client.admin_request('GET', '/admin/status')
    elif args.operation == 'result':
        from capture_task.protocol import identifier
        code, value = client.admin_request('GET', '/admin/result/' + identifier(args.task_id))
    else:
        cmd = dict(command_id='c-'+uuid.uuid4().hex, type=args.operation)
        if args.operation == 'send':
            cmd.update(type='capture', task_id=args.task_id or 't-'+uuid.uuid4().hex,
                       task_type=args.task_type, ttl_sec=args.ttl, params={})
            if args.command_id:
                cmd['command_id'] = args.command_id
            if args.announce is not None:
                cmd['params']['announce'] = args.announce
            if args.prepare is not None:
                cmd['params']['prepare_sec'] = args.prepare
        elif args.operation == 'cancel':
            cmd['task_id'] = args.task_id
        else:
            cmd['text'] = args.text
        code, value = client.admin_request('POST', '/admin/commands', cmd)
        value['command'] = cmd
    print(json.dumps(value, ensure_ascii=False, indent=2))
    if code != 200:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
