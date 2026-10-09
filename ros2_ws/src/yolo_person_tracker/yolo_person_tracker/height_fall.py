"""Fall cue from the height of a person's depth points above the floor (no skeleton needed).

From a low camera, lying people often hide their shoulders/hips, so keypoint rules fail; the
depth points inside the person box still show the body top dropping to near the floor. The top
is a high percentile of the nearest surface cluster, measured along gravity. Gravity and
camera height come from calibration, the floor fit or TF; this module never confirms them.
"""
from collections import deque
from dataclasses import dataclass
import math

import numpy as np


@dataclass(frozen=True)
class HeightFallConfig:
    upright_min_m: float = .8       # body top while upright (the depth FOV may cut the head)
    low_max_m: float = .55          # body top while lying on the floor
    drop_min_m: float = .4          # baseline top minus current top
    transition_s: float = 2.        # last upright baseline to first low frame
    stable_s: float = .3            # upright evidence needed for a baseline
    confirm_s: float = 1.           # sustained low before confirmation
    recovery_s: float = 2.          # sustained upright to clear a confirmed fall
    max_gap_s: float = .5           # unobserved time that pauses (not clears) the timers
    min_points: int = 150
    cluster_m: float = .6           # depth band behind the nearest surface kept as the person
    box_shrink: float = .1          # horizontal margin removed from each box side
    top_percentile: float = 95.

    def __post_init__(self):
        for name, value in vars(self).items():
            if isinstance(value, bool) or not math.isfinite(value) or value <= 0:
                raise ValueError(f'height fall {name} must be finite and positive')
        if not self.low_max_m < self.upright_min_m or not self.top_percentile <= 100:
            raise ValueError('height fall requires low_max_m < upright_min_m and percentile <= 100')


def body_heights(depth_m, box, intrinsics, up, camera_height_m, config=None, extent=False):
    """(top, median) height above the floor of the nearest surface inside the box, or None.

    extent=True appends the body's horizontal length (5-95 % along its main floor-plane axis),
    which does not depend on the camera height and separates lying from crouching."""
    cfg = config or HeightFallConfig()
    fx, fy, cx, cy = map(float, intrinsics)
    h, w = depth_m.shape
    x1, y1, x2, y2 = map(float, box)
    margin = (x2-x1)*cfg.box_shrink
    x1, x2 = int(max(0, round(x1+margin))), int(min(w, round(x2-margin)))
    y1, y2 = int(max(0, round(y1))), int(min(h, round(y2)))
    if x2-x1 < 2 or y2-y1 < 2:
        return None
    patch = depth_m[y1:y2, x1:x2]
    valid = np.isfinite(patch) & (patch > .3) & (patch < 6.)
    if valid.sum() < cfg.min_points:
        return None
    z = patch[valid]
    keep = z < np.percentile(z, 20)+cfg.cluster_m
    if keep.sum() < cfg.min_points:
        return None
    v, u = np.nonzero(valid)
    u, v, z = u[keep]+x1, v[keep]+y1, z[keep]
    points = np.column_stack(((u-cx)*z/fx, (v-cy)*z/fy, z))
    up = np.asarray(up, float)
    height = points@up+float(camera_height_m)
    result = (float(np.percentile(height, cfg.top_percentile)), float(np.median(height)))
    if not extent:
        return result
    flat = points-np.outer(points@up, up)
    flat -= flat.mean(axis=0)
    axis = np.linalg.svd(flat, full_matrices=False)[2][0]
    along = flat@axis
    return result+(float(np.percentile(along, 95)-np.percentile(along, 5)),)


class HeightFallTracker:
    """Per-identity upright -> low transition on the body top height; gaps pause the timers."""

    def __init__(self, config=None):
        self.cfg = config or HeightFallConfig()
        self.tracks = {}

    def reset(self):
        self.tracks.clear()

    def _state(self, identity, stamp):
        state = self.tracks.get(identity)
        if state is None or stamp <= state['stamp'] or stamp-state['stamp'] > self.cfg.max_gap_s*4:
            state = dict(stamp=stamp, upright=deque(), baseline=None, pending=None, fallen=False, recovery=None)
            self.tracks[identity] = state
        return state

    def move(self, old, new):
        """Carry a track's history to a new identity (see EnhancedPostureTracker.handover)."""
        if old in self.tracks and new not in self.tracks:
            self.tracks[new] = self.tracks.pop(old)

    def handover(self, stamp, present, boxes, limit_s=1.5):
        """Same rule as EnhancedPostureTracker.handover: one lost track near and below one new one.

        boxes: last known box per identity (caller-maintained). Returns {new: old}.
        """
        from .pose3d import below_near
        lost = {k: boxes[k] for k, v in self.tracks.items()
                if k not in present and k in boxes and 0 < stamp-v['stamp'] <= limit_s}
        pairs = {}
        for new in (k for k in present if k not in self.tracks):
            matches = [old for old, box in lost.items() if below_near(box, present[new])]
            if len(matches) == 1:
                pairs.setdefault(matches[0], []).append(new)
        moved = {}
        for old, news in pairs.items():
            if len(news) == 1:
                self.move(old, news[0])
                moved[news[0]] = old
        return moved

    def update(self, identity, stamp, top, upright_hint=False):
        """top: body top height in metres, or None when unmeasured. Returns (phase, reason).

        upright_hint: the skeleton clearly shows standing or sitting/crouching this frame; a
        low top is then a crouch, so it neither starts nor keeps a suspected fall."""
        cfg, state = self.cfg, self._state(identity, stamp)
        gap = stamp-state['stamp']
        state['stamp'] = stamp
        if top is None or not math.isfinite(top):
            if gap <= cfg.max_gap_s:
                # Unmeasured frame: shift every timer, never count it as evidence.
                for key in ('pending', 'recovery'):
                    if state[key] is not None:
                        state[key] += gap
                if state['baseline'] is not None:
                    state['baseline'] = (state['baseline'][0]+gap, state['baseline'][1])
                state['upright'] = deque((t+gap, h) for t, h in state['upright'])
            return (2 if state['fallen'] else 1 if state['pending'] is not None else 0), 'height unmeasured; timers paused'
        if state['fallen']:
            if top >= cfg.upright_min_m:
                state['recovery'] = stamp if state['recovery'] is None else state['recovery']
                if stamp-state['recovery'] >= cfg.recovery_s:
                    state.update(fallen=False, recovery=None, pending=None, baseline=None, upright=deque())
                    return 0, 'upright recovery complete'
            else:
                state['recovery'] = None
            return 2, f'confirmed fall; top={top:.2f}m'
        upright = state['upright']
        if top >= cfg.upright_min_m:
            upright.append((stamp, top))
            while upright and stamp-upright[0][0] > cfg.transition_s:
                upright.popleft()
            if upright[-1][0]-upright[0][0] >= cfg.stable_s:
                state['baseline'] = (stamp, float(np.median([h for _, h in upright])))
        else:
            upright.clear()
        baseline = state['baseline']
        if baseline is not None and stamp-baseline[0] > cfg.transition_s and state['pending'] is None:
            state['baseline'] = baseline = None
        if upright_hint and top <= cfg.low_max_m:
            state['pending'] = None
            return 0, f'low top but skeleton upright (crouch); top={top:.2f}m'
        if state['pending'] is not None:
            if top > cfg.low_max_m:
                state['pending'] = None
                return 0, f'rose before confirmation; top={top:.2f}m'
            if stamp-state['pending'] >= cfg.confirm_s:
                state.update(fallen=True, pending=None, recovery=None)
                return 2, f'body top stayed low; top={top:.2f}m'
            return 1, f'rapid drop to the floor; top={top:.2f}m'
        if (baseline is not None and top <= cfg.low_max_m and baseline[1]-top >= cfg.drop_min_m):
            state['pending'] = stamp
            return 1, f'rapid drop to the floor; top {baseline[1]:.2f}->{top:.2f}m'
        return 0, f'top={top:.2f}m' + ('; static low, no observed drop' if top <= cfg.low_max_m else '')
