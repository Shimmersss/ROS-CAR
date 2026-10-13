"""Short-term single-object bridge for the selected person (opt-in, perception only).

VitTrack (OpenCV Zoo, GSoC 2023) follows the image patch of the target between
clean detections. It never measures position or decides identity by itself: a
bridge starts from a clean, already selected target box, is checked every frame
against that target's own depth-gated Kalman filter, and stops on low score,
depth contradiction or age. The numpy pre/post-processing is a line-by-line port
of OpenCV 4.10 modules/video/src/tracking/tracker_vit.cpp so that runtimes
without cv2.TrackerVit (ROS Humble's OpenCV 4.5.4 cannot load the HardSwish
layer) can run the same model through onnxruntime.
"""
from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path
import cv2
import numpy as np
from types import SimpleNamespace
from .depth import DepthGate, DepthTrackFilter, measure
from .reid_backend import mask_nearer_people

MEAN = np.array([.485, .456, .406], np.float32)  # applied in BGR order, as OpenCV does
STD = np.array([.229, .224, .225], np.float32)
ARCHITECTURE = 'vittrack_2023sep'


def _trunc_div2(v):
    return int(v/2)  # C++ integer division truncates toward zero


def crop_image(src, box, factor):
    x, y, w, h = box
    crop = int(math.ceil(math.sqrt(w*h)*factor))
    x1 = x+_trunc_div2(w-crop); x2 = x1+crop
    y1 = y+_trunc_div2(h-crop); y2 = y1+crop
    x1p, y1p = max(0, -x1), max(0, -y1)
    x2p, y2p = max(x2-src.shape[1]+1, 0), max(y2-src.shape[0]+1, 0)
    roi = src[y1+y1p:y2-y2p, x1+x1p:x2-x2p]
    return cv2.copyMakeBorder(roi, y1p, y2p, x1p, x2p, cv2.BORDER_CONSTANT, value=0)


def blob(src, size):
    img = cv2.resize(src, (size, size)).astype(np.float32)/255.
    return np.ascontiguousarray(((img-MEAN)/STD).transpose(2, 0, 1)[None])


def hann2d(n=16):
    i = np.arange(1, n+1, dtype=np.float32)
    h = (.5*(1-np.cos(np.float32(2*math.pi/(n+1))*i))).astype(np.float32)
    return np.outer(h, h)


class _OnnxRuntime:
    def __init__(self, path):
        import onnxruntime
        # This small tracker shares the Jetson with pose, ASR and always-on KWS.
        # ORT's default pool otherwise allocates/spins workers across all cores.
        options = onnxruntime.SessionOptions()
        options.intra_op_num_threads = 1
        options.inter_op_num_threads = 1
        options.add_session_config_entry('session.intra_op.allow_spinning', '0')
        self.session = onnxruntime.InferenceSession(
            path, sess_options=options, providers=['CPUExecutionProvider'])

    def __call__(self, template, search):
        return self.session.run(['output1', 'output2', 'output3'], {'template': template, 'search': search})


class _OpenCV:
    def __init__(self, path):
        self.net = cv2.dnn.readNetFromONNX(path)

    def __call__(self, template, search):
        self.net.setInput(template, 'template'); self.net.setInput(search, 'search')
        return self.net.forward(['output1', 'output2', 'output3'])


class VitTrack:
    def __init__(self, model_path, backend='auto'):
        path = Path(model_path)
        if not model_path or path.suffix != '.onnx' or not path.is_file():
            raise ValueError('Configure a local VitTrack ONNX; no automatic downloads/fallback')
        metadata = json.loads(Path(str(path)+'.json').read_text())
        if (metadata.get('architecture') != ARCHITECTURE
                or metadata.get('sha256') != hashlib.sha256(path.read_bytes()).hexdigest()):
            raise ValueError('VitTrack metadata/hash mismatch')
        if backend not in ('auto', 'onnxruntime', 'opencv'):
            raise ValueError('backend must be auto, onnxruntime or opencv')
        errors = []
        for name, cls in (('onnxruntime', _OnnxRuntime), ('opencv', _OpenCV)):
            if backend in ('auto', name):
                try:
                    self.run = cls(str(path)); self.backend = name; break
                except Exception as exc:  # e.g. onnxruntime missing, OpenCV < 4.8 lacks HardSwish
                    errors.append(f'{name}: {type(exc).__name__}: {exc}')
        else:
            raise ValueError('No VitTrack inference backend: '+'; '.join(errors))
        self.window = hann2d()
        self.template = None
        self.rect = None
        self.score = math.nan

    def init(self, image, box):
        """box: integer (x, y, w, h)."""
        self.template = blob(crop_image(image, box, 2), 128)
        self.rect = tuple(int(v) for v in box)

    def update(self, image):
        x, y, w, h = self.rect
        conf, size, offset = self.run(self.template, blob(crop_image(image, self.rect, 4), 256))
        conf = conf.reshape(16, 16)*self.window
        r, c = np.unravel_index(int(np.argmax(conf)), conf.shape)
        self.score = float(conf[r, c])
        size = size.reshape(2, 16, 16); offset = offset.reshape(2, 16, 16)
        cx = (c+offset[0, r, c])/16; cy = (r+offset[1, r, c])/16
        bw, bh = size[0, r, c], size[1, r, c]
        window = 4*int(math.floor(math.sqrt(w*h)))
        x0 = x+_trunc_div2(w-window); y0 = y+_trunc_div2(h-window)
        self.rect = (int(math.floor((cx-bw/2)*window+x0)), int(math.floor((cy-bh/2)*window+y0)),
                     int(math.floor(bw*window)), int(math.floor(bh*window)))
        return self.rect, self.score


@dataclass(frozen=True)
class BridgeConfig:
    min_score: float = .35
    max_bridge_s: float = 3.
    depth_rejects: int = 2
    min_box_px: int = 16

    def __post_init__(self):
        if not (0 <= self.min_score < 1 and math.isfinite(self.max_bridge_s) and self.max_bridge_s > 0
                and type(self.depth_rejects) is int and self.depth_rejects >= 1
                and type(self.min_box_px) is int and self.min_box_px >= 4):
            raise ValueError('Invalid bridge configuration')


@dataclass(frozen=True)
class BridgeResult:
    valid: bool
    box: tuple = ()
    score: float = math.nan
    reason: str = ''


class TargetBridge:
    """One target. refresh() on every clean confirmed target box; step() on other frames."""

    def __init__(self, tracker, config=None, gate=None):
        self.tracker, self.cfg = tracker, config or BridgeConfig()
        self.gate = gate or DepthGate()
        self.active = False; self.filter = None; self.refreshed = None; self.rejects = 0; self.last = None

    def stop(self, reason):
        self.active = False; self.filter = None
        return BridgeResult(False, reason=reason)

    def refresh(self, image, depth, intrinsics, box, stamp):
        """box: (x1, y1, x2, y2) of the clean target detection."""
        x1, y1, x2, y2 = (int(round(v)) for v in box)
        if x2-x1 < self.cfg.min_box_px or y2-y1 < self.cfg.min_box_px:
            return self.stop('target box too small')
        if self.last is not None and stamp < self.last:  # same frame as a step() is allowed
            return self.stop('time reversal')
        self.tracker.init(image, (x1, y1, x2-x1, y2-y1))
        if self.filter is None or not self.active:
            self.filter = DepthTrackFilter(gate=self.gate)
        self.filter.update(measure(depth, box, intrinsics) if depth is not None else None, stamp)
        self.active, self.refreshed, self.rejects, self.last = True, stamp, 0, stamp
        return BridgeResult(True, tuple(map(float, box)), 1., 'refreshed from target detection')

    def step(self, image, depth, intrinsics, stamp):
        if not self.active:
            return BridgeResult(False, reason='inactive')
        if stamp <= self.last:
            return self.stop('time reversal')
        self.last = stamp
        if stamp-self.refreshed > self.cfg.max_bridge_s:
            return self.stop('bridge too long without a clean target detection')
        (x, y, w, h), score = self.tracker.update(image)
        if score < self.cfg.min_score:
            return self.stop(f'tracker score {score:.2f} below limit')
        if w < self.cfg.min_box_px or h < self.cfg.min_box_px:
            return self.stop('tracker box collapsed')
        box = (float(x), float(y), float(x+w), float(y+h))
        if depth is not None:
            self.filter.update(measure(depth, box, intrinsics), stamp)
            self.rejects = self.rejects+1 if self.filter.last_rejected else 0
            if self.rejects >= self.cfg.depth_rejects:
                return self.stop('tracker box depth contradicts the target track')
        return BridgeResult(True, box, score, 'bridged')


def box_iou(a, b):
    ix = max(0., min(a[2], b[2])-max(a[0], b[0])); iy = max(0., min(a[3], b[3])-max(a[1], b[1]))
    inter = ix*iy; union = (a[2]-a[0])*(a[3]-a[1])+(b[2]-b[0])*(b[3]-b[1])-inter
    return inter/union if union > 0 else 0.


def _pose_person(detection):
    points = getattr(detection, 'keypoints', ()) or ()
    if len(points) != 17:
        return None
    return SimpleNamespace(box=detection.box, keypoints_2d=[SimpleNamespace(x=p[0], y=p[1]) for p in points],
                           keypoint_confidences=[p[2] for p in points])


def masked_template_image(image, target, others):
    """Grey out nearer overlapping people (pose silhouettes); None without usable poses or
    when the target torso is mostly hidden."""
    me = _pose_person(target)
    if me is None or min(me.keypoint_confidences[i] for i in (5, 6, 11, 12)) < .5:
        return None
    people = [_pose_person(o) for o in others]
    if any(p is None for p in people):
        return None
    return mask_nearer_people(image, (0, 0), me, people)


def overlaps_anyone(box, others, fraction=.1):
    """Same rule as the ReID crop gate: intersection above 10 % of the smaller box."""
    area = (box[2]-box[0])*(box[3]-box[1])
    for o in others:
        ix = max(0., min(box[2], o[2])-max(box[0], o[0])); iy = max(0., min(box[3], o[3])-max(box[1], o[1]))
        if ix*iy > fraction*min(area, max(1., (o[2]-o[0])*(o[3]-o[1]))):
            return True
    return False


@dataclass(frozen=True)
class ArbiterConfig:
    agree_iou: float = .5
    apart_iou: float = .3
    conflict_frames: int = 3
    handover_frames: int = 2
    mask_refresh: bool = False  # refresh an overlapped target from a pose-masked image

    def __post_init__(self):
        if not (0 < self.apart_iou < self.agree_iou <= 1 and type(self.conflict_frames) is int
                and self.conflict_frames >= 1 and type(self.handover_frames) is int and self.handover_frames >= 1
                and type(self.mask_refresh) is bool):
            raise ValueError('Invalid arbiter configuration')


@dataclass(frozen=True)
class Decision:
    action: str  # none | agree | conflict | lost | handover | fill
    track_id: object = None
    box: tuple = ()
    reason: str = ''


class BridgeArbiter:
    """Combines the bridge with the selected ByteTrack target; it can only keep, hand over
    (bridge-confirmed) or drop the target, never pick a person the bridge did not follow."""

    def __init__(self, bridge, config=None):
        self.bridge, self.cfg = bridge, config or ArbiterConfig()
        self.conflicts = 0; self.handover = (None, 0)
        self.tainted = False  # target overlapped someone while no bridge watched it
        self.covisible = set()  # tracks seen together with the target: other people

    def reset(self):
        self.bridge.stop('reset'); self.conflicts = 0; self.handover = (None, 0); self.tainted = False
        self.covisible = set()

    def update(self, image, depth, intrinsics, stamp, target, candidates):
        """target: selected detection or None; candidates: tracked detections (track_id, box)."""
        others = [c.box for c in candidates if target is None or c.track_id != target.track_id]
        if target is not None:
            self.covisible.update(c.track_id for c in candidates if c.track_id != target.track_id)
        clean = target is not None and not overlaps_anyone(target.box, others)
        if not self.bridge.active:
            self.conflicts = 0; self.handover = (None, 0)
            if target is not None and not clean:
                self.tainted = True
            if clean:
                if self.tainted:
                    # ByteTrack may have swapped people during the unwatched overlap.
                    self.reset()
                    return Decision('lost', reason='target overlapped others while unbridged; reconfirm identity')
                self.bridge.refresh(image, depth, intrinsics, target.box, stamp)
                return Decision('agree', target.track_id, tuple(target.box), 'bridge started')
            return Decision('none', reason='bridge inactive')
        r = self.bridge.step(image, depth, intrinsics, stamp)
        if not r.valid:
            self.conflicts = 0; self.handover = (None, 0)
            return Decision('none', reason=r.reason)
        best = max(candidates, key=lambda c: box_iou(r.box, c.box), default=None)
        best_iou = box_iou(r.box, best.box) if best is not None else 0.
        if target is not None:
            self.handover = (None, 0)
            if box_iou(r.box, target.box) >= self.cfg.agree_iou:
                self.conflicts = 0
                if clean:  # template only ever refreshed where both cues agree
                    self.bridge.refresh(image, depth, intrinsics, target.box, stamp)
                elif self.cfg.mask_refresh:
                    near = [c for c in candidates if c.track_id != target.track_id
                            and overlaps_anyone(target.box, [c.box])]
                    masked = masked_template_image(image, target, near)
                    if masked is not None:
                        self.bridge.refresh(masked, depth, intrinsics, target.box, stamp)
                return Decision('agree', target.track_id, r.box, 'bridge agrees with track')
            if (best is not None and best.track_id != target.track_id and best_iou >= self.cfg.agree_iou
                    and box_iou(r.box, target.box) < self.cfg.apart_iou):
                self.conflicts += 1
                if self.conflicts >= self.cfg.conflict_frames:
                    self.reset()
                    return Decision('lost', reason='bridge follows another track; possible ID swap')
                return Decision('conflict', target.track_id, r.box, 'bridge disagrees with track')
            self.conflicts = 0
            return Decision('agree', target.track_id, r.box, 'bridge inconclusive')
        if best is not None and best_iou >= self.cfg.agree_iou and best.track_id in self.covisible:
            # The bridge sits on someone seen beside the target: not the target, no position.
            self.handover = (None, 0); self.conflicts += 1
            if self.conflicts >= self.cfg.conflict_frames:
                self.reset()
                return Decision('lost', reason='bridge drifted onto another person')
            return Decision('conflict', None, r.box, 'bridge on a track seen beside the target')
        self.conflicts = 0
        if best is not None and best_iou >= self.cfg.agree_iou:
            count = self.handover[1]+1 if self.handover[0] == best.track_id else 1
            self.handover = (best.track_id, count)
            if count >= self.cfg.handover_frames:
                self.handover = (None, 0)
                return Decision('handover', best.track_id, r.box, 'bridge-confirmed reacquisition')
            return Decision('fill', None, r.box, 'awaiting bridge-confirmed reacquisition')
        self.handover = (None, 0)
        return Decision('fill', None, r.box, 'target undetected; bridged box only')
