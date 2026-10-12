#!/usr/bin/env python3
"""Recall of fallen / non-fallen people on E-FPDS images for local YOLO person models.

E-FPDS (GRAM, Universidad de Alcalá; cite Maldonado-Bascón et al., Electronics 2019): 640x480
PNG from a robot camera about 76 cm high; each .txt line is `label left right top bottom` with
label 1 = fallen, -1 = not fallen. Every ground-truth box is matched to the best-IoU person
detection; recall is reported per confidence threshold. Detection only: no tracking or rules.
"""
import argparse
from collections import defaultdict
import json
from pathlib import Path

import cv2
import numpy as np

THRESHOLDS = (.1, .25, .5)


def read_boxes(path):
    boxes = []
    for line in Path(path).read_text().split('\n'):
        parts = line.split()
        if len(parts) != 5:
            continue
        label, left, right, top, bottom = map(int, parts)
        boxes.append((label == 1, (left, top, right, bottom)))
    return boxes


def iou(a, b):
    x1, y1, x2, y2 = max(a[0], b[0]), max(a[1], b[1]), min(a[2], b[2]), min(a[3], b[3])
    inter = max(0., x2-x1)*max(0., y2-y1)
    union = (a[2]-a[0])*(a[3]-a[1])+(b[2]-b[0])*(b[3]-b[1])-inter
    return inter/union if union > 0 else 0.


def best_match(gt, detections, min_iou):
    """Highest confidence among detections overlapping the ground truth by at least min_iou."""
    scores = [conf for box, conf in detections if iou(gt, box) >= min_iou]
    return max(scores, default=0.)


PREPROCESS = {
    'none': lambda image: image,
    'gray': lambda image: cv2.cvtColor(cv2.cvtColor(image, cv2.COLOR_BGR2GRAY), cv2.COLOR_GRAY2BGR),
}


def evaluate(model, images, min_iou, image_size, preprocess='none'):
    rows = []
    for image_path in images:
        gts = read_boxes(image_path.with_suffix('.txt'))
        if not gts:
            continue
        image = PREPROCESS[preprocess](cv2.imread(str(image_path)))
        result = model.predict(image, classes=[0], conf=.03, imgsz=image_size, verbose=False)[0]
        detections = list(zip(result.boxes.xyxy.cpu().tolist(), result.boxes.conf.cpu().tolist()))
        matched = set()
        for fallen, gt in gts:
            score = best_match(gt, detections, min_iou)
            rows.append(dict(group=image_path.parent.name, fallen=fallen, score=score,
                             aspect=(gt[2]-gt[0])/max(1, gt[3]-gt[1])))
            matched |= {i for i, (box, _) in enumerate(detections) if iou(gt, box) >= min_iou}
        # Unmatched confident detections: false positives or unannotated people.
        rows += [dict(group=image_path.parent.name, extra=True, score=conf)
                 for i, (_, conf) in enumerate(detections) if i not in matched]
    return rows


def summarize(rows):
    out = {}
    for fallen, name in ((True, 'fallen'), (False, 'not_fallen')):
        scores = np.array([r['score'] for r in rows if r.get('fallen') is fallen])
        if not len(scores):
            continue
        out[name] = dict(n=int(len(scores)), median_conf=round(float(np.median(scores)), 3),
                         **{f'recall@{t}': round(float((scores >= t).mean()), 3) for t in THRESHOLDS})
    extras = np.array([r['score'] for r in rows if r.get('extra')])
    out['unmatched_detections'] = {f'conf>={t}': int((extras >= t).sum()) for t in THRESHOLDS}
    return out


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('roots', nargs='+', type=Path, help='directories searched recursively for PNG + TXT')
    parser.add_argument('--model', action='append', required=True, help='local .pt weights (repeatable)')
    parser.add_argument('--min-iou', type=float, default=.5)
    parser.add_argument('--image-size', type=int, default=640)
    parser.add_argument('--preprocess', choices=sorted(PREPROCESS), default='none')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    from ultralytics import YOLO
    images = sorted(p for root in args.roots for p in root.rglob('*.png') if p.with_suffix('.txt').exists())
    if not images:
        parser.error('no annotated PNG images found')
    args.output.mkdir(parents=True, exist_ok=True)
    report = {}
    for weights in args.model:
        rows = evaluate(YOLO(weights), images, args.min_iou, args.image_size, args.preprocess)
        groups = defaultdict(list)
        for row in rows:
            groups[row['group']].append(row)
        name = Path(weights).stem
        report[name] = dict(all=summarize(rows), **{g: summarize(r) for g, r in sorted(groups.items())})
        (args.output/f'{name}-rows.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in rows))
        print(name, json.dumps(report[name]['all']))
    (args.output/'summary.json').write_text(json.dumps(dict(min_iou=args.min_iou, preprocess=args.preprocess, images=len(images), models=report),
                                                       indent=2))


if __name__ == '__main__':
    main()
