#!/usr/bin/env python3
"""Bounded, read-only ROS bag capture with optional shared-queue speech."""
import argparse
import json
import os
from pathlib import Path
import shutil
import signal
import sqlite3
import subprocess
import threading
import time

TOPICS = ['/camera/color/image_raw', '/camera/color/image_rect',
          '/camera/color/camera_info', '/camera/color/camera_info_rect',
          '/camera/depth/image_raw', '/camera/depth/camera_info',
          '/perception/person_states', '/perception/person_states_continued',
          '/perception/fall_alarm', '/perception/owner_state',
          '/perception/person_identities', '/perception/target_state',
          '/control/state', '/control/cmd_vel_request', '/cmd_vel', '/odom',
          '/tf', '/tf_static', '/rosout']
REQUIRED = ['/camera/color/image_raw', '/camera/depth/image_raw',
            '/perception/person_states', '/perception/fall_alarm', '/odom']


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('name', help='e.g. follow-straight-01 or fall-toward-camera-01')
    parser.add_argument('--seconds', type=int, default=60, choices=range(10, 181), metavar='10..180')
    parser.add_argument('--root', type=Path, default=Path(__file__).resolve().parents[1]/'data/demo-trials')
    parser.add_argument('--speech', action='store_true')
    args = parser.parse_args()
    if not args.name or any(c not in 'abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789-_' for c in args.name):
        parser.error('name must contain only letters, digits, hyphens or underscores')
    args.root.mkdir(parents=True, exist_ok=True)
    if shutil.disk_usage(args.root).free < 10*1024**3:
        parser.error('at least 10 GiB free space required')
    topics = subprocess.check_output(['ros2', 'topic', 'list', '--no-daemon', '--spin-time', '3'], text=True, timeout=20).splitlines()
    missing = set(REQUIRED)-set(topics)
    if missing:
        parser.error('missing required topics: '+', '.join(sorted(missing)))
    out = args.root/(time.strftime('%Y%m%d-%H%M%S')+'-'+args.name)
    out.mkdir(exist_ok=False)
    events = []
    speech_lock = threading.Lock()
    speech_thread = None

    def say(message, final=False):
        nonlocal speech_thread
        if final and speech_thread is not None:
            speech_thread.join(timeout=26)
        print(message, flush=True)
        events.append(dict(elapsed=round(time.monotonic()-started, 2), message=message))
        if args.speech:
            def speak():
                if not speech_lock.acquire(blocking=False):
                    return
                try:
                    with (out/'speech.log').open('a') as log:
                        subprocess.run(['ros2', 'service', 'call', '/voice/say',
                                        'roscar_interfaces/srv/SayText', json.dumps({'text': message}, ensure_ascii=False)],
                                       stdout=log, stderr=log, timeout=25, check=False)
                except subprocess.TimeoutExpired:
                    print('Speech timed out; capture continues', flush=True)
                finally:
                    speech_lock.release()
            speech_thread = threading.Thread(target=speak, daemon=True)
            speech_thread.start()
            if final:
                speech_thread.join(timeout=26)

    stopped = False
    def stop(*_):
        nonlocal stopped
        stopped = True
    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    started = time.monotonic()
    metadata = dict(name=args.name, seconds=args.seconds, domain=os.environ.get('ROS_DOMAIN_ID'),
                    topics=TOPICS, note='ROS capture only; no motion authorization; manually label actions after capture')
    (out/'session.json').write_text(json.dumps(metadata, ensure_ascii=False, indent=2))
    with (out/'recorder.log').open('w') as log:
        proc = subprocess.Popen(['ros2', 'bag', 'record', '-o', str(out/'bag'),
                                 '--max-bag-size', '1073741824', *TOPICS], stdout=log, stderr=log,
                                start_new_session=True)
        try:
            # Announce recording only after storage was created, not merely process launch.
            deadline = time.monotonic()+15
            while not list((out/'bag').glob('*.db3')):
                if stopped or proc.poll() is not None or time.monotonic() > deadline:
                    raise RuntimeError('recorder did not become ready; inspect recorder.log')
                time.sleep(.2)
            started = time.monotonic()
            say('录制已开始，计划录制%d秒。' % args.seconds)
            next_progress = 30
            while not stopped and time.monotonic()-started < args.seconds:
                if proc.poll() is not None:
                    raise RuntimeError('recorder exited unexpectedly')
                if shutil.disk_usage(out).free < 3*1024**3:
                    raise RuntimeError('recording ended: disk reserve reached')
                elapsed = time.monotonic()-started
                if elapsed >= next_progress:
                    say('已录制%d秒。' % next_progress)
                    next_progress += 30
                time.sleep(.2)
        finally:
            if proc.poll() is None:
                os.killpg(proc.pid, signal.SIGINT)
                try:
                    proc.wait(timeout=30)
                except subprocess.TimeoutExpired:
                    os.killpg(proc.pid, signal.SIGTERM)
                    proc.wait(timeout=10)
            metadata.update(elapsed=round(time.monotonic()-started, 2), interrupted=stopped,
                            recorder_returncode=proc.returncode, events=events)
            (out/'session.json').write_text(json.dumps(metadata, ensure_ascii=False, indent=2))
    if proc.returncode not in (0, -signal.SIGINT) or not (out/'bag/metadata.yaml').exists():
        raise RuntimeError('bag did not finalize cleanly; preserve and inspect output')
    subprocess.run(['ros2', 'bag', 'info', str(out/'bag')], check=True,
                   stdout=(out/'bag-info.txt').open('w'), timeout=20)
    counts = {}
    for database in (out/'bag').glob('*.db3'):
        with sqlite3.connect('file:'+str(database)+'?mode=ro', uri=True) as db:
            for topic, count in db.execute('SELECT topics.name, count(messages.id) FROM topics LEFT JOIN messages ON topics.id=messages.topic_id GROUP BY topics.id'):
                counts[topic] = counts.get(topic, 0)+count
    metadata['message_counts'] = counts
    (out/'session.json').write_text(json.dumps(metadata, ensure_ascii=False, indent=2))
    empty = [topic for topic in REQUIRED if not counts.get(topic)]
    if empty:
        say('录制已结束，但关键数据缺失，请检查录制结果。', final=True)
        raise RuntimeError('no recorded messages: '+', '.join(empty))
    say('本段录制已结束，文件已保存。', final=True)
    print(out, flush=True)


if __name__ == '__main__':
    main()
