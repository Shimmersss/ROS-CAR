#!/usr/bin/env python3
"""Compare .pt/.engine on the same RGB camera frames saved as an NPZ 'bgr' array.

Runs no ROS nodes and publishes no commands. Results measure warmed backend
latency (preprocessing + inference + postprocessing + ByteTrack), not camera-to-control.
"""
import argparse
import gc
import json
from pathlib import Path
import sys
import time

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'ros2_ws/src/yolo_person_tracker'))
from yolo_person_tracker.backend import YoloBackend


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--frames', type=Path, required=True)
    parser.add_argument('--pt', type=Path, required=True)
    parser.add_argument('--engine', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    import torch
    with np.load(args.frames, allow_pickle=False) as archive:
        frames = archive['bgr']
    if frames.dtype != np.uint8 or frames.ndim != 4 or frames.shape[-1] != 3 or len(frames) < 10:
        raise ValueError('Require >=10 uint8 BGR frames, shape (N,H,W,3)')
    report = {'frames': len(frames), 'timing': 'warmed backend including ByteTrack; CUDA synchronized',
              'scope': 'recorded frames only; no ROS end-to-end or motion acceptance', 'backends': {}}
    outputs = []
    for label, path in (('pytorch', args.pt), ('tensorrt', args.engine)):
        backend = YoloBackend(str(path), '0', 640)
        for _ in range(10):
            backend.infer(frames[0])
        backend.reset()
        times, boxes = [], []
        for frame in frames:
            torch.cuda.synchronize()
            start = time.perf_counter()
            detections = backend.infer(frame)
            torch.cuda.synchronize()
            times.append((time.perf_counter()-start)*1000)
            boxes.append([list(d.box) for d in detections])
        report['backends'][label] = dict(mean_ms=float(np.mean(times)),
                                         p95_ms=float(np.percentile(times, 95)),
                                         counts=[len(b) for b in boxes])
        outputs.append(boxes)
        del backend
        gc.collect()
        torch.cuda.empty_cache()
    report['count_agreement_fraction'] = float(np.mean([
        len(a) == len(b) for a, b in zip(*outputs)]))
    report['mean_speedup'] = report['backends']['pytorch']['mean_ms']/report['backends']['tensorrt']['mean_ms']
    report['boxes_xyxy'] = dict(zip(('pytorch', 'tensorrt'), outputs))
    args.output.write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps({key: value for key, value in report.items() if key != 'boxes_xyxy'}, indent=2))


if __name__ == '__main__':
    main()
