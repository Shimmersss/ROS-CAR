"""Explicit local OSNet ONNX inference and pose-assisted crop quality checks."""
import hashlib
import json
from pathlib import Path
import cv2
import numpy as np
from .reid import unit


def appearance_crop(image, person, people, min_height=80, min_width=32, blur_min=20.):
    h, w = image.shape[:2]
    box = np.asarray(person.box, float)
    if box.shape != (4,) or not np.isfinite(box).all():
        return None, 'invalid box'
    x1, y1, x2, y2 = box
    if x1 < 0 or y1 < 0 or x2 > w or y2 > h or x2-x1 < min_width or y2-y1 < min_height:
        return None, 'small or truncated crop'
    if not np.isfinite(person.confidence) or person.confidence < .5:
        return None, 'low detection confidence'
    area = (x2-x1)*(y2-y1)
    for other in people:
        if other is person:
            continue
        a, b, c, d = map(float, other.box)
        intersection = max(0., min(x2,c)-max(x1,a))*max(0., min(y2,d)-max(y1,b))
        if intersection > .1*min(area, max(1., (c-a)*(d-b))):
            return None, 'overlapping people'
    if len(person.keypoints_2d) != 17 or len(person.keypoint_confidences) != 17:
        return None, 'COCO17 unavailable'
    visible = []
    for i, (p, confidence) in enumerate(zip(person.keypoints_2d, person.keypoint_confidences)):
        if np.isfinite((p.x,p.y,confidence)).all() and confidence >= .5 and x1 <= p.x < x2 and y1 <= p.y < y2:
            visible.append(i)
    if len(visible) < 6 or not {5,6,11,12}.issubset(visible):
        return None, 'insufficient visible torso keypoints'
    torso = np.asarray([(person.keypoints_2d[i].x, person.keypoints_2d[i].y)
                        for i in (5,6,12,11)], dtype=np.float32)
    if cv2.contourArea(cv2.convexHull(torso)) < .02*area:
        return None, 'degenerate torso geometry'
    crop = image[int(y1):int(y2),int(x1):int(x2)]
    if crop.size == 0 or cv2.Laplacian(cv2.cvtColor(crop,cv2.COLOR_BGR2GRAY),cv2.CV_64F).var() < blur_min:
        return None, 'blurred or textureless crop'
    # Use the full detector crop expected by OSNet. Pose gates visibility; it
    # does not turn this full-body model into a part-trained ReID network.
    return crop, 'pose-qualified full-body crop'


class OSNetBackend:
    def __init__(self, model_path):
        path = Path(model_path)
        if not model_path or path.suffix != '.onnx' or not path.is_file():
            raise ValueError('Configure a local OSNet ONNX; no automatic downloads/fallback')
        metadata = json.loads(Path(str(path)+'.json').read_text())
        if (metadata.get('architecture') != 'osnet_x0_25_msmt17'
                or metadata.get('preprocessing') != 'rgb_256x128_imagenet'
                or metadata.get('sha256') != hashlib.sha256(path.read_bytes()).hexdigest()):
            raise ValueError('OSNet metadata/hash mismatch')
        self.net = cv2.dnn.readNetFromONNX(str(path))
        self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
        self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)

    @staticmethod
    def tensor(crop):
        rgb = cv2.cvtColor(cv2.resize(crop,(128,256)), cv2.COLOR_BGR2RGB).astype(np.float32)/255.
        rgb = (rgb-np.array([.485,.456,.406],np.float32))/np.array([.229,.224,.225],np.float32)
        return np.ascontiguousarray(rgb.transpose(2,0,1)[None])

    def extract(self, crops):
        outputs=[]
        for crop in crops:
            self.net.setInput(self.tensor(crop))
            feature=self.net.forward().reshape(-1)
            if feature.shape != (512,):
                raise ValueError('OSNet must return 512-D descriptors')
            outputs.append(unit(feature))
        return outputs
