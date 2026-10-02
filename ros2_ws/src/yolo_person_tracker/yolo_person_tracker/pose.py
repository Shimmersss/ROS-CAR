"""COCO17 geometry and bounded, camera-relative posture rules (not a trained fall classifier)."""
from collections import deque
from dataclasses import dataclass
import math
import numpy as np

EDGES = ((0,1),(0,2),(1,3),(2,4),(5,6),(5,7),(7,9),(6,8),(8,10),
         (5,11),(6,12),(11,12),(11,13),(13,15),(12,14),(14,16))
UNKNOWN, STANDING, SITTING_CROUCHING, LYING, FALLEN = range(5)
LABELS = ('UNKNOWN', 'STANDING', 'SITTING_CROUCHING', 'LYING', 'FALLEN')


@dataclass(frozen=True)
class PoseConfig:
    confidence: float = .5
    upright_confirmed: bool = False
    horizontal_deg: float = 60.
    standing_deg: float = 35.
    drop_fraction: float = .25
    transition_s: float = 1.
    lying_confirm_s: float = 1.
    recovery_s: float = 2.
    stable_s: float = .3
    max_gap_s: float = .5
    lying_aspect: float = 1.2
    knee_height_fraction: float = .2
    joint_jump_m: float = .5

    def __post_init__(self):
        for name, value in vars(self).items():
            if name == 'upright_confirmed':
                continue
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f'pose {name} must be finite and positive')
        if self.confidence > 1 or not 0 < self.standing_deg < self.horizontal_deg < 90:
            raise ValueError('Invalid pose confidence/angle thresholds')


def points2d(keypoints, confidence=.5):
    points = np.asarray(keypoints, dtype=float)
    if points.shape != (17,3):
        return np.full((17,3), np.nan), np.zeros(17, dtype=bool)
    valid = np.isfinite(points).all(axis=1) & (points[:,2] >= confidence) & (points[:,0] > 0) & (points[:,1] > 0)
    return points, valid


def joints3d(depth, keypoints, intrinsics, confidence=.5, reference=None):
    """Use only real samples; no hole filling or stale depth replacement."""
    result = np.full((17,3), np.nan)
    points, valid = points2d(keypoints, confidence)
    fx, fy, cx, cy = intrinsics
    if not all(map(math.isfinite, intrinsics)) or fx <= 0 or fy <= 0:
        return result
    h,w = depth.shape
    for i in np.flatnonzero(valid):
        u,v = points[i,:2]
        x,y = int(round(u)), int(round(v))
        if not (0 <= x < w and 0 <= y < h):
            continue
        roi = depth[max(0,y-3):min(h,y+4), max(0,x-3):min(w,x+4)]
        values = roi[np.isfinite(roi) & (roi >= .2) & (roi <= 8)]
        if len(values) < max(8, roi.size*.5):
            continue
        z = float(np.median(values))
        if np.ptp(np.percentile(values,[10,90])) > max(.10,.08*z):
            continue
        # A keypoint patch on a distant wall is not a body joint.
        if reference is None or abs(z-reference[2]) > max(.5,.3*reference[2]):
            continue
        result[i] = ((u-cx)*z/fx, (v-cy)*z/fy, z)
    return result


class PostureTracker:
    def __init__(self, config=None):
        self.cfg = config or PoseConfig()
        self.tracks = {}

    def reset(self):
        self.tracks.clear()

    def update(self, identity, stamp, box, keypoints):
        cfg = self.cfg
        if not math.isfinite(stamp) or stamp <= 0:
            self.tracks.pop(identity, None)
            return UNKNOWN, 0, 'Invalid observation timestamp'
        self.tracks = {k:v for k,v in self.tracks.items() if 0 <= stamp-v['stamp'] <= cfg.max_gap_s}
        state = self.tracks.get(identity)
        if state is None or stamp <= state['stamp']:
            state = dict(stamp=stamp, history=deque(maxlen=120), pending=None, fallen=False, recovery=None)
            self.tracks[identity] = state
        state['stamp'] = stamp
        if len(self.tracks) > 128:
            oldest = min(self.tracks, key=lambda k:self.tracks[k]['stamp'])
            if oldest != identity:
                self.tracks.pop(oldest)
        points, valid = points2d(keypoints, cfg.confidence)
        x1,y1,x2,y2 = box
        height = y2-y1
        if not np.isfinite(box).all() or height <= 0 or x2 <= x1 or not valid[[5,6,11,12]].all():
            state['history'].clear(); state['pending']=None; state['recovery']=None
            return UNKNOWN, 0, 'Insufficient shoulders/hips or invalid box'
        shoulders = points[[5,6],:2].mean(axis=0)
        hips = points[[11,12],:2].mean(axis=0)
        delta = hips-shoulders
        if np.linalg.norm(delta) < 5 or (delta[1] < 0 and abs(delta[1]) > abs(delta[0])):
            state['history'].clear(); state['pending']=None; state['recovery']=None
            return UNKNOWN, 0, 'Degenerate/inverted torso'
        angle = math.degrees(math.atan2(abs(delta[0]), abs(delta[1])))
        center = float((shoulders[1]+hips[1])*.5)
        lying = angle >= cfg.horizontal_deg and (x2-x1)/height >= cfg.lying_aspect
        posture = LYING if lying else UNKNOWN
        if angle < cfg.standing_deg and valid[[13,14]].all():
            knees = points[[13,14],:2].mean(axis=0)
            posture = SITTING_CROUCHING if knees[1]-hips[1] < height*cfg.knee_height_fraction else STANDING
        history = state['history']
        while history and stamp-history[0][0] > cfg.transition_s:
            history.popleft()
        stable = [item for item in history if item[3] in (STANDING,SITTING_CROUCHING)]
        # Require a continuous stable upright run before a fall transition.
        baseline = stable[0] if (len(stable)>=2 and stable[-1][0]-stable[0][0]>=cfg.stable_s
                                and len(stable)==len(history)) else None
        if state['fallen']:
            if posture == STANDING:
                if state['recovery'] is None: state['recovery']=stamp
                if stamp-state['recovery'] >= cfg.recovery_s:
                    state['fallen']=False; state['pending']=None; history.clear()
            else:
                state['recovery']=None
            if state['fallen']:
                return FALLEN, 2, 'Confirmed fall; awaiting continuous upright recovery'
        if lying and state['pending'] is None and baseline and center-baseline[1] > cfg.drop_fraction*baseline[2]:
            state['pending']=(stamp,baseline[1],baseline[2])
        pending=state['pending']
        if pending:
            low = center-pending[1] > cfg.drop_fraction*pending[2]
            if not lying or not low:
                state['pending']=None; history.clear()
            elif stamp-pending[0] >= cfg.lying_confirm_s:
                if cfg.upright_confirmed:
                    state['fallen']=True
                    return FALLEN, 2, 'Rapid descent and sustained horizontal low posture'
                return LYING, 1, 'Fall suspected; camera upright installation unconfirmed'
            else:
                return LYING, 1, 'Rapid descent; waiting for sustained horizontal low posture'
        history.append((stamp,center,height,posture))
        return posture, 0, ('Static lying; no observed fall transition' if lying else LABELS[posture])
