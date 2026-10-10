#!/usr/bin/env python3
"""Local RGB Pose preview, using the C backend/rules; no ROS or vehicle output."""
import argparse
from dataclasses import fields
import json
import os
from pathlib import Path
import sys
import threading
import time

os.environ['YOLO_AUTOINSTALL'] = 'false'
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'ros2_ws/src/yolo_person_tracker'))
import cv2
import yaml
from yolo_person_tracker.backend import YoloBackend
from yolo_person_tracker.pose import PoseConfig, PostureTracker, EDGES, LABELS, points2d


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--camera', type=int, default=0)
    parser.add_argument('--model', default=str(ROOT/'models/weights/yolo26s-pose.pt'))
    parser.add_argument('--config', default=str(ROOT/'ros2_ws/src/perception_bringup/config/pose.yaml'))
    args = parser.parse_args()
    params = yaml.safe_load(Path(args.config).read_text())['/**']['ros__parameters']
    config = PoseConfig(**{f.name: params['pose_'+f.name] for f in fields(PoseConfig)
                          if 'pose_'+f.name in params})
    backend = YoloBackend(args.model, 'cpu', 640, task='pose')
    postures = PostureTracker(config)
    capture = cv2.VideoCapture(args.camera, cv2.CAP_AVFOUNDATION)
    if not capture.isOpened():
        raise RuntimeError('Cannot open selected camera')
    capture.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    capture.set(cv2.CAP_PROP_FPS, 30)
    lock = threading.Lock()
    stopped = threading.Event()
    latest = [None]

    def acquire():
        try:
            while not stopped.is_set():
                ok, frame = capture.read()
                if not ok:
                    break
                with lock:
                    latest[0] = (time.monotonic(), frame)
        finally:
            stopped.set()

    worker = threading.Thread(target=acquire, daemon=True)
    worker.start()
    title = 'Gemini Pose + State (2D) - Q to close'
    cv2.namedWindow(title, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(title, 960, 720)
    previous = None
    epoch = 0
    reported = 0.
    try:
        while not stopped.is_set():
            with lock:
                item = latest[0]
            if item is None or item[0] == previous:
                if cv2.waitKey(5) & 255 in (27, ord('q')):
                    break
                continue
            stamp, raw = item
            if previous is not None and stamp-previous > config.max_gap_s:
                backend.reset()
                postures.reset()
                epoch += 1
            previous = stamp
            started = time.monotonic()
            detections = backend.infer(raw)
            elapsed = time.monotonic()-started
            frame = raw.copy()
            rows = []
            fresh = time.monotonic()-stamp <= config.max_gap_s
            if not fresh:
                postures.reset()
            for person in detections:
                identity = f'{epoch}:{person.track_id}' if person.track_id is not None else 'pending'
                state, phase, reason = (postures.update(identity, stamp, person.box, person.keypoints)
                    if fresh and person.track_id is not None else (0, 0, 'Untracked or stale'))
                color = (0, 80, 255) if phase else (40, 220, 80)
                x1,y1,x2,y2 = map(int, person.box)
                cv2.rectangle(frame,(x1,y1),(x2,y2),color,2)
                points, valid = points2d(person.keypoints,config.confidence)
                for a,b in EDGES:
                    if valid[a] and valid[b]:
                        cv2.line(frame,tuple(map(int,points[a,:2])),tuple(map(int,points[b,:2])),color,2)
                for point, usable in zip(points,valid):
                    if usable:
                        cv2.circle(frame,tuple(map(int,point[:2])),3,(0,220,255),-1)
                label = f'{identity} {LABELS[state]}' + (' FALL SUSPECTED' if phase == 1 else '')
                cv2.putText(frame,label,(max(0,x1),max(62,y1-8)),cv2.FONT_HERSHEY_SIMPLEX,.48,color,1,cv2.LINE_AA)
                rows.append(dict(id=identity,state=LABELS[state],fall_phase=phase,reason=reason))
            cv2.rectangle(frame,(0,0),(frame.shape[1],48),(25,25,25),-1)
            cv2.putText(frame,f'2D / CPU | {1/max(elapsed,.001):.1f} inference FPS | {len(rows)} people',
                        (8,19),cv2.FONT_HERSHEY_SIMPLEX,.5,(255,255,255),1,cv2.LINE_AA)
            cv2.putText(frame,'No synced depth | '+('Upright confirmed' if config.upright_confirmed else 'Upright unconfirmed: fall confirmation disabled'),
                        (8,39),cv2.FONT_HERSHEY_SIMPLEX,.4,(0,210,255),1,cv2.LINE_AA)
            cv2.imshow(title,frame)
            if time.monotonic()-reported >= 1:
                print(json.dumps(dict(people=rows,inference_ms=elapsed*1000,fresh=fresh)),flush=True)
                reported = time.monotonic()
            if cv2.waitKey(1) & 255 in (27,ord('q')) or cv2.getWindowProperty(title,cv2.WND_PROP_VISIBLE)<1:
                break
    finally:
        stopped.set()
        worker.join(timeout=2)
        capture.release()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
