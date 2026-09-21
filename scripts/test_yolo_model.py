#!/usr/bin/env python3
"""Real checkpoint/ByteTrack smoke test; bundled still images are not live camera validation."""
import argparse
import hashlib
import json
from pathlib import Path
import sys
import time
import statistics
import cv2
import numpy as np

root = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(root / 'ros2_ws/src/yolo_person_tracker'))
from yolo_person_tracker.backend import YoloBackend


def main():
    manifest = json.loads((root/'models/manifest.json').read_text())
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model', default=manifest['default_model'])
    parser.add_argument('--legacy-nms', action='store_true')
    args = parser.parse_args()
    entry = next(item for item in manifest['models'] if item['name'] == args.model)
    path = root/'models'/entry['local_path']
    assert hashlib.sha256(path.read_bytes()).hexdigest() == entry['sha256']
    backend = YoloBackend(str(path), 'cpu', 640, nms_free=not args.legacy_nms)
    for _ in range(2):
        assert backend.infer(np.zeros((480,640,3), dtype=np.uint8)) == []
    from ultralytics.utils import ASSETS
    image = cv2.imread(str(ASSETS/'bus.jpg'))
    assert image is not None
    backend.reset()
    tracks, elapsed = [], []
    for _ in range(4):
        start = time.perf_counter()
        detections = backend.infer(image)
        elapsed.append((time.perf_counter()-start)*1000)
        tracks.append({d.track_id for d in detections})
        assert detections and all(d.confidence > 0 for d in detections)
    assert set.intersection(*tracks), 'No persistent ID on repeated identical image'
    backend.reset()
    print(json.dumps(dict(model=args.model, device='cpu', image_size=640,
                          nms_free=bool(backend.model.predictor.model.end2end),
                          tracks=[sorted(t) for t in tracks],
                          mean_warm_wall_ms=statistics.mean(elapsed[1:]),
                          scope='Mac CPU, repeated official bus.jpg, not live tracking or Jetson performance'), indent=2))
    print('PASS verified checkpoint, blank frames, real person detections, ByteTrack IDs and reset')


if __name__ == '__main__':
    main()
