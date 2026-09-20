"""Ultralytics is imported only when an explicit local model is configured."""
from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True)
class Detection:
    track_id: int
    box: tuple
    confidence: float


class YoloBackend:
    def __init__(self, model_path, device, image_size, nms_free=True):
        if not Path(model_path).is_file():
            raise ValueError('model_path must point to an existing local weight file')
        if Path(model_path).suffix not in ('.pt', '.engine'):
            raise ValueError('Use a local .pt checkpoint or target-built .engine')
        if Path(model_path).suffix == '.engine' and str(device) not in ('0', 'cuda:0'):
            raise ValueError('TensorRT engine requires device=0 on its build Jetson')
        from ultralytics import YOLO
        self.model = YOLO(model_path, task='detect')
        self.device = device
        self.image_size = image_size
        self.nms_free = nms_free

    def reset(self):
        predictor = self.model.predictor
        if predictor is not None:
            for tracker in getattr(predictor, 'trackers', []):
                tracker.reset()

    def infer(self, image):
        result = self.model.track(
            image, persist=True, tracker='bytetrack.yaml', classes=[0],
            conf=0.1, iou=0.7, imgsz=self.image_size, device=self.device,
            nms=False if self.nms_free else None, rect=False, verbose=False)[0]
        if self.nms_free and not self.model.predictor.model.end2end:
            raise RuntimeError('Requested NMS-free inference but model has no end-to-end output; '
                               'use YOLO26 or export the engine with nms=False')
        boxes = result.boxes
        if boxes is None or boxes.id is None:
            return []
        return [Detection(int(tid), tuple(map(float, box)), float(conf))
                for tid, box, conf in zip(boxes.id.cpu().tolist(),
                                         boxes.xyxy.cpu().tolist(),
                                         boxes.conf.cpu().tolist())]
