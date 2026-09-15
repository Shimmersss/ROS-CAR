"""Ultralytics is imported only when an explicit local model is configured."""
from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True)
class Detection:
    track_id: int
    box: tuple
    confidence: float


class YoloBackend:
    def __init__(self, model_path, device, image_size):
        if not Path(model_path).is_file():
            raise ValueError('model_path must point to an existing local weight file')
        from ultralytics import YOLO
        self.model = YOLO(model_path)
        self.device = device
        self.image_size = image_size

    def reset(self):
        predictor = self.model.predictor
        if predictor is not None:
            for tracker in getattr(predictor, 'trackers', []):
                tracker.reset()

    def infer(self, image):
        result = self.model.track(
            image, persist=True, tracker='bytetrack.yaml', classes=[0],
            conf=0.1, iou=0.7, imgsz=self.image_size, device=self.device,
            verbose=False)[0]
        boxes = result.boxes
        if boxes is None or boxes.id is None:
            return []
        return [Detection(int(tid), tuple(map(float, box)), float(conf))
                for tid, box, conf in zip(boxes.id.cpu().tolist(),
                                         boxes.xyxy.cpu().tolist(),
                                         boxes.conf.cpu().tolist())]
