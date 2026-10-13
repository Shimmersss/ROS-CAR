"""Explicit local OSNet ONNX inference and pose-assisted crop quality checks."""
import hashlib
import json
from pathlib import Path
import cv2
import numpy as np
from .reid import unit


LIMBS = ((5, 7), (7, 9), (6, 8), (8, 10), (11, 13), (13, 15), (12, 14), (14, 16), (5, 11), (6, 12))
MASK_BGR = (124, 116, 104)  # ImageNet mean, the network's neutral input


def appearance_crop(image, person, people, min_height=80, min_width=32, blur_min=20., mask_overlap=False):
    """Pose-gated full-body crop. With mask_overlap, an overlapping crop is kept but pixels of
    a nearer overlapping person (pose silhouette) are greyed out; a mostly hidden torso is still rejected."""
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
    overlapping = []
    for other in people:
        if other is person:
            continue
        a, b, c, d = map(float, other.box)
        intersection = max(0., min(x2,c)-max(x1,a))*max(0., min(y2,d)-max(y1,b))
        if intersection > .1*min(area, max(1., (c-a)*(d-b))):
            if not mask_overlap:
                return None, 'overlapping people'
            overlapping.append(other)
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
    reason = 'pose-qualified full-body crop'
    if overlapping:
        crop = mask_nearer_people(crop, (int(x1), int(y1)), person, overlapping)
        if crop is None:
            return None, 'overlapping people'
        reason = 'pose-masked overlapping crop'
    if crop.size == 0 or cv2.Laplacian(cv2.cvtColor(crop,cv2.COLOR_BGR2GRAY),cv2.CV_64F).var() < blur_min:
        return None, 'blurred or textureless crop'
    # Use the full detector crop expected by OSNet. Pose gates visibility; it
    # does not turn this full-body model into a part-trained ReID network.
    return crop, reason


def silhouette(shape, origin, person):
    """Approximate body region from COCO17 keypoints, in crop coordinates."""
    k = np.array([(p.x-origin[0], p.y-origin[1], c) for p, c in
                  zip(person.keypoints_2d, person.keypoint_confidences)], float)
    ok = np.isfinite(k).all(axis=1) & (k[:, 2] >= .3)
    m = np.zeros(shape, np.uint8)
    width = max(8., float(person.box[2]-person.box[0]))
    if ok[[5, 6, 11, 12]].all():
        cv2.fillConvexPoly(m, np.int32(k[[5, 6, 12, 11], :2]), 1)
    for a, b in LIMBS:
        if ok[a] and ok[b]:
            cv2.line(m, tuple(map(int, k[a, :2])), tuple(map(int, k[b, :2])), 1, max(3, int(.22*width)))
    head = [i for i in range(5) if ok[i]]
    if head and ok[5] and ok[6]:
        radius = max(4, int(.45*np.linalg.norm(k[5, :2]-k[6, :2])))
        cv2.circle(m, tuple(map(int, k[head, :2].mean(0))), radius, 1, -1)
    return cv2.dilate(m, np.ones((5, 5), np.uint8))


def torso_depth(person):
    points = getattr(person, 'keypoints_3d', None)
    valid = getattr(person, 'keypoints_3d_valid', None)
    if points is None or valid is None or len(points) != 17:
        return None
    z = [points[i].z for i in (5, 6, 11, 12) if valid[i] and np.isfinite(points[i].z)]
    return float(np.median(z)) if z else None


def body_depth(person):
    z = getattr(person, 'body_depth_m', np.nan)
    return float(z) if getattr(person, 'body_depth_valid', False) and np.isfinite(z) else None


def nearer(a, b):
    """Whether a is nearer the camera than b, comparing like with like: torso joint depth,
    then body depth, else the lower box bottom (people standing on the floor)."""
    for depth in (torso_depth, body_depth):
        za, zb = depth(a), depth(b)
        if za is not None and zb is not None:
            return za < zb
    return float(a.box[3]) > float(b.box[3])


def mask_nearer_people(crop, origin, person, overlapping, max_hidden_torso=.5):
    own = silhouette(crop.shape[:2], origin, person).astype(bool)
    hidden = np.zeros(crop.shape[:2], bool)
    for other in overlapping:
        body = silhouette(crop.shape[:2], origin, other).astype(bool)
        # A nearer person hides us; a farther one only claims pixels outside our own body.
        hidden |= body if nearer(other, person) else body & ~own
    torso = np.zeros(crop.shape[:2], np.uint8)
    k = [(person.keypoints_2d[i].x-origin[0], person.keypoints_2d[i].y-origin[1]) for i in (5, 6, 12, 11)]
    cv2.fillConvexPoly(torso, np.int32(k), 1)
    if (hidden & torso.astype(bool)).sum() > max_hidden_torso*max(1, torso.sum()):
        return None
    masked = crop.copy()
    masked[hidden] = MASK_BGR
    return masked


ARCHITECTURES = ('osnet_x0_25_msmt17', 'osnet_x1_0_msmt17')


class OSNetBackend:
    def __init__(self, model_path):
        path = Path(model_path)
        if not model_path or path.suffix not in ('.onnx', '.engine') or not path.is_file():
            raise ValueError('Configure a local OSNet ONNX/engine; no automatic downloads/fallback')
        metadata = json.loads(Path(str(path)+'.json').read_text())
        if (metadata.get('architecture') not in ARCHITECTURES
                or metadata.get('preprocessing') != 'rgb_256x128_imagenet'
                or metadata.get('sha256') != hashlib.sha256(path.read_bytes()).hexdigest()):
            raise ValueError('OSNet metadata/hash mismatch')
        self.engine_backend = None
        if path.suffix == '.engine':
            source_name = metadata.get('source_onnx', '')
            if not source_name or Path(source_name).name != source_name or Path(source_name).suffix != '.onnx':
                raise ValueError('OSNet engine requires a local source ONNX filename')
            source = path.parent / source_name
            original = json.loads(Path(str(source)+'.json').read_text())
            digest = hashlib.sha256(source.read_bytes()).hexdigest()
            if (metadata.get('source_onnx_sha256') != digest or original.get('sha256') != digest
                    or original.get('architecture') != metadata['architecture']
                    or original.get('preprocessing') != metadata['preprocessing']):
                raise ValueError('OSNet engine source metadata/hash mismatch')
            from .reid_tensorrt import OSNetTensorRT
            self.engine_backend = OSNetTensorRT(path)
            return
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
            if self.engine_backend is not None:
                feature = self.engine_backend.run(self.tensor(crop))
            else:
                self.net.setInput(self.tensor(crop))
                feature=self.net.forward().reshape(-1)
            if feature.shape != (512,):
                raise ValueError('OSNet must return 512-D descriptors')
            outputs.append(unit(feature))
        return outputs
