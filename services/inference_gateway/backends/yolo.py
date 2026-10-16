"""Pinned Ultralytics models; never import or download a model implicitly."""
import hashlib
import time
from pathlib import Path


class YoloBackend:
    def __init__(self, kind, config):
        import numpy as np
        import torch
        import ultralytics
        from ultralytics import YOLO
        self.kind, self.config = kind, config
        torch.set_num_threads(2)
        path = Path(config['weights']).expanduser().resolve(strict=True)
        self.digest = hashlib.sha256(path.read_bytes()).hexdigest()
        if self.digest != config['sha256']:
            raise ValueError('weights_hash_mismatch')
        self.model = YOLO(str(path))
        self.names = {int(k): str(v) for k, v in self.model.names.items()}
        expected = config.get('classes')
        if expected is not None and list(self.names.values()) != expected:
            raise ValueError('classes_mismatch')
        if kind == 'helmet' and not {'helmet', 'no_helmet'} <= set(self.names.values()):
            raise ValueError('helmet_classes_require_explicit_adapter')
        self.version = ultralytics.__version__
        self.options = dict(device=config.get('device', 'cuda:0'), imgsz=640,
                            conf=.2 if kind == 'pcb' else .45, iou=.45,
                            half=config.get('half', False), verbose=False)
        self.model.predict(np.zeros((480, 640, 3), dtype=np.uint8), **self.options)

    def ready(self):
        return True

    def info(self):
        return dict(name=self.kind, version=self.version, weights_sha256=self.digest,
                    device=self.options['device'], ready=True, classes=self.names,
                    thresholds={k: self.options[k] for k in ('conf', 'iou', 'imgsz')})

    def infer(self, image, metadata, deadline):
        if time.monotonic() >= deadline:
            raise TimeoutError()
        prediction = self.model.predict(image, **self.options)[0]
        rows = []
        height, width = image.shape[:2]
        for cls, score, box in zip(prediction.boxes.cls.tolist(), prediction.boxes.conf.tolist(),
                                   prediction.boxes.xyxy.tolist()):
            box = [max(0., min(float(v), float(width if i % 2 == 0 else height))) for i, v in enumerate(box)]
            rows.append(dict(cls=int(cls), name=self.names[int(cls)], score=float(score), xyxy=box))
        if self.kind == 'pcb':
            result = dict(defects=rows, reject=bool(rows), conf_threshold=.2, imgsz=640)
            count = len(rows)
        else:
            count = sum(row['name'] == 'no_helmet' for row in rows)
            result = dict(detections=rows, helmet_count=sum(row['name'] == 'helmet' for row in rows),
                          no_helmet_count=count, violation=bool(count), conf_threshold=.45,
                          iou_threshold=.45, imgsz=640)
        return dict(status='ok', result=result,
                    summary=dict(positive=bool(count), count=count,
                                 text_key='pcb_defects' if self.kind == 'pcb' else 'helmet_violations'))
