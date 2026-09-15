#!/usr/bin/env python3
"""Local CPU weight-loading smoke test; blank input is not person validation."""
import hashlib
import json
from pathlib import Path
import sys
import numpy as np

root = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(root / 'ros2_ws/src/yolo_person_tracker'))
from yolo_person_tracker.backend import YoloBackend

entry = json.loads((root / 'models/manifest.json').read_text())['models'][0]
path = root / 'models' / entry['local_path']
assert hashlib.sha256(path.read_bytes()).hexdigest() == entry['sha256']
backend = YoloBackend(str(path), 'cpu', 320)
for _ in range(2):
    assert backend.infer(np.zeros((240,320,3), dtype=np.uint8)) == []
backend.reset()
print('PASS local CPU: verified weights, 2 blank frames, ByteTrack path and reset. No person/camera validation.')
