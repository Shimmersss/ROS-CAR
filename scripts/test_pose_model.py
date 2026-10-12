#!/usr/bin/env python3
"""Real CPU pose/ByteTrack smoke test; explicit local checkpoint, no automatic download."""
import argparse
import hashlib
import json
from pathlib import Path
import sys
import time
import os
os.environ['YOLO_AUTOINSTALL']='false'
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'ros2_ws/src/yolo_person_tracker'))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--image',help='Optional real camera image; default is official bundled bus.jpg')
    parser.add_argument('--output',default=str(ROOT/'artifacts/c-model-smoke.json'))
    args=parser.parse_args()
    from yolo_person_tracker.backend import YoloBackend
    from ultralytics.utils import ASSETS
    import cv2
    import numpy as np
    source=ROOT/'models/weights/yolo26s-pose.pt'
    entry=next(m for m in json.loads((ROOT/'models/manifest.json').read_text())['models'] if m['name']==source.name)
    if hashlib.sha256(source.read_bytes()).hexdigest()!=entry['sha256']:
        raise ValueError('Pose weight hash mismatch; run prepare_model.py explicitly')
    backend=YoloBackend(str(source),'cpu',640,task='pose')
    backend.infer(np.zeros((640,640,3),np.uint8));backend.reset()
    image=cv2.imread(args.image or str(ASSETS/'bus.jpg'))
    if image is None:raise ValueError('Image cannot be decoded')
    rows=[]
    for _ in range(4):
        started=time.perf_counter();detections=backend.infer(image)
        assert all(len(d.keypoints)==17 for d in detections)
        rows.append(dict(ms=(time.perf_counter()-started)*1000,people=len(detections),
                         ids=[d.track_id for d in detections],confident_joints=[sum(p[2]>=.5 for p in d.keypoints) for d in detections]))
    if args.image is None:
        assert len(detections)>=2 and rows[0]['ids']==rows[-1]['ids']
    report=dict(task='pose',source_sha256=entry['sha256'],device='Mac CPU',image=args.image or 'official bundled bus.jpg',frames=rows,
                validation='Repeated still image only; not live fall/occlusion or Jetson performance acceptance')
    output=Path(args.output);output.parent.mkdir(parents=True,exist_ok=True);output.write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))


if __name__=='__main__':main()
