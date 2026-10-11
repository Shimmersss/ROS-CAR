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
    recovery_s: float = 1.          # sustained upright to clear a confirmed fall (2 s missed repeat falls)
    max_gap_s: float = .5           # unobserved time that pauses (not clears) the timers
    min_points: int = 150
    cluster_m: float = .6           # depth band behind the nearest surface kept as the person
    box_shrink: float = .1          # horizontal margin removed from each box side
    top_percentile: float = 95.     # decision value: robust, but usually lands near the shoulders
    head_percentile: float = 99.    # display only: closer to the real head top (2026-10-10: 1.83 m for 1.78 m)
    top_edge_px: float = 15.        # a box this close to the image top is cut off (see clipped_at_top)
    veto_min_aspect: float = 1.5    # box height/width needed before an upright skeleton may veto
    # With a standing skeleton label (any box shape) a lower top still counts as upright (a far
    # person whose head leaves the depth view) for the baseline, and above low_max_m for recovery.
    # A lying person misread as standing has a top below both. Sitting is not used: sitting on
    # the floor then lying back would look like a fall (2026-10-09 A5).
    hint_upright_min_m: float = .65
    # Optional, 0 = off (2026-10-10 seated falls):
    # lost_hold_s: a suspected or confirmed track may hand over after this long unseen (a person
    # lying feet-first towards a low camera is often not detected at all); the gap never counts.
    lost_hold_s: float = 0.
    # sit_hold_s: after a rapid drop, sitting up (top below upright_min_m) or a crouch veto keeps
    # the fall suspected this long; lying low again for confirm_s confirms, standing up cancels.
    sit_hold_s: float = 0.
    # side_edge_px: a box this close to the left/right image edge cannot start a suspected fall
    # (someone leaving the view past the camera shows only a leg, 2026-10-10 A5); its top still
    # counts as measured, since lying people often reach the side edge too. 0 = off.
    side_edge_px: float = 0.

    def __post_init__(self):
        for name, value in vars(self).items():
            if name in ('lost_hold_s', 'sit_hold_s', 'side_edge_px') and math.isfinite(value) and value >= 0:
                continue
            if isinstance(value, bool) or not math.isfinite(value) or value <= 0:
                raise ValueError(f'height fall {name} must be finite and positive')
        if (not self.low_max_m < self.hint_upright_min_m <= self.upright_min_m or not self.top_percentile <= 100
                or not self.head_percentile <= 100):
            raise ValueError('height fall requires low_max_m < hint_upright_min_m <= upright_min_m and percentile <= 100')


def body_heights(depth_m, box, intrinsics, up, camera_height_m, config=None, extent=False, head=False):
    """(top, median) height above the floor of the nearest surface inside the box, or None.

    extent=True appends the body's horizontal length (5-95 % along its main floor-plane axis),
    which does not depend on the camera height and separates lying from crouching.
    head=True appends the head_percentile height, an estimate of the head top for display; the
    head has few points, so top (top_percentile) sits 0.15-0.25 m lower and drives the rules."""
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
    tail = (float(np.percentile(height, cfg.head_percentile)),) if head else ()
    if not extent:
        return result+tail
    flat = points-np.outer(points@up, up)
    flat -= flat.mean(axis=0)
    axis = np.linalg.svd(flat, full_matrices=False)[2][0]
    along = flat@axis
    return result+(float(np.percentile(along, 95)-np.percentile(along, 5)),)+tail


def clipped_at_top(box, config=None):
    """True when the box reaches the image top: the measured top is then only a lower bound.

    It still proves an upright body when high enough, but a low value may just be a person
    passing close to the camera with the upper body out of view (2026-10-09 A1a/A1c)."""
    cfg = config or HeightFallConfig()
    return float(box[1]) <= cfg.top_edge_px


def at_side_edge(box, width, config=None):
    """True when side_edge_px > 0 and the box reaches the left or right image edge."""
    cfg = config or HeightFallConfig()
    return cfg.side_edge_px > 0 and (float(box[0]) <= cfg.side_edge_px or float(box[2]) >= width-cfg.side_edge_px)


def skeleton_veto(upright, box, config=None):
    """True when an upright/crouching skeleton label may veto a low top (see update()).

    From a low camera the 2D rule often calls a lying person standing; their box is wide, while
    a crouching person's box stays tall (2026-10-09 recordings: falls <1.5, crouches ~1.8)."""
    cfg = config or HeightFallConfig()
    x1, y1, x2, y2 = map(float, box)
    return bool(upright) and (y2-y1) >= cfg.veto_min_aspect*max(x2-x1, 1e-6)


class HeightFallTracker:
    """Per-identity upright -> low transition on the body top height; gaps pause the timers."""

    def __init__(self, config=None):
        self.cfg = config or HeightFallConfig()
        self.tracks = {}

    def reset(self):
        self.tracks.clear()

    def _state(self, identity, stamp):
        state = self.tracks.get(identity)
        if (state is not None and self.cfg.max_gap_s*4 < stamp-state['stamp'] <= self.cfg.lost_hold_s
                and (state['pending'] is not None or state['fallen'] or state['drop'] is not None)):
            # Same identity back after a stretch that would reset it (> 4 max_gap_s) while down:
            # pause instead. Shorter gaps keep their existing handling.
            self._shift(state, stamp-state['stamp'])
            state['stamp'] = stamp-1e-6
        if state is None or stamp <= state['stamp'] or stamp-state['stamp'] > self.cfg.max_gap_s*4:
            state = dict(stamp=stamp, first=stamp, upright=deque(), baseline=None, pending=None, fallen=False,
                         recovery=None, drop=None)
            self.tracks[identity] = state
        return state

    def move(self, old, new, stamp=None):
        """Carry a track's history to a new identity (see EnhancedPostureTracker.handover).

        With stamp, the unobserved gap since the old track's last frame pauses every evidence
        timer, so time when neither track was seen never counts towards confirmation."""
        if old not in self.tracks or (new in self.tracks and not self._fresh(self.tracks[new], stamp)):
            return
        state = self.tracks.pop(old)
        if stamp is not None and stamp > state['stamp']:
            self._shift(state, stamp-state['stamp'])
            state['stamp'] = stamp-1e-6
        self.tracks[new] = state

    def _fresh(self, state, stamp, limit_s=None):
        """A track too young to hold its own evidence: the tracker re-identified a person while
        the old identity was still listed (2026-10-10 A1a: two IDs overlapped for 0.1 s)."""
        limit = self.cfg.max_gap_s*3 if limit_s is None else limit_s
        return (stamp is not None and stamp-state.get('first', state['stamp']) <= limit and state['baseline'] is None
                and state['pending'] is None and not state['fallen'] and not state['upright'])

    @staticmethod
    def _shift(state, gap):
        for key in ('pending', 'recovery', 'drop'):
            if state[key] is not None:
                state[key] += gap
        if state['baseline'] is not None:
            state['baseline'] = (state['baseline'][0]+gap, state['baseline'][1])
        state['upright'] = deque((t+gap, h) for t, h in state['upright'])

    def handover(self, stamp, present, boxes, limit_s=1.5):
        """Same rule as EnhancedPostureTracker.handover: one lost track near and below one new one.

        boxes: last known box per identity (caller-maintained). Returns {new: old}.
        """
        from .pose3d import below_near
        def hold(v):
            down = v['pending'] is not None or v['fallen'] or v.get('drop') is not None
            return max(limit_s, self.cfg.lost_hold_s) if down else limit_s
        lost = {k: boxes[k] for k, v in self.tracks.items()
                if k not in present and k in boxes and 0 < stamp-v['stamp'] <= hold(v)}
        pairs = {}
        for new in (k for k in present if k not in self.tracks or self._fresh(self.tracks[k], stamp, limit_s)):
            matches = [old for old, box in lost.items() if below_near(box, present[new])]
            if len(matches) == 1:
                pairs.setdefault(matches[0], []).append(new)
        moved = {}
        for old, news in pairs.items():
            if len(news) == 1:
                self.tracks.pop(news[0], None)   # fresh by the rule above: its own state is discarded
                self.move(old, news[0], stamp)
                moved[news[0]] = old
        return moved

    def update(self, identity, stamp, top, upright_hint=False, posture_upright=False, clipped=False, side=False):
        """top: body top height in metres, or None when unmeasured. Returns (phase, reason).

        upright_hint: the skeleton shows standing or sitting/crouching in a tall box this frame
        (see skeleton_veto); a low top is then a crouch, so it neither starts nor keeps a
        suspected fall. posture_upright: the skeleton says standing (any box shape); a top above
        hint_upright_min_m then counts as upright, and above low_max_m as recovery. clipped: the
        body reaches the image top (clipped_at_top); a top below upright_min_m is then unmeasured.
        side: the box reaches a side edge (at_side_edge); it may not start a suspected fall."""
        cfg, state = self.cfg, self._state(identity, stamp)
        if clipped and top is not None and top < cfg.upright_min_m:
            top = None
        gap = stamp-state['stamp']
        state['stamp'] = stamp
        if top is None or not math.isfinite(top):
            if state['fallen'] and upright_hint and posture_upright:
                # Unmeasured but a standing skeleton in a tall box: recovery evidence on its own
                # (2026-10-09: getting up while the geometry was unavailable kept the alarm on).
                state['recovery'] = stamp if state['recovery'] is None else state['recovery']
                if stamp-state['recovery'] >= cfg.recovery_s:
                    state.update(fallen=False, recovery=None, pending=None, baseline=None, upright=deque())
                    return 0, 'upright recovery complete (skeleton; height unmeasured)'
                return 2, 'confirmed fall; height unmeasured, skeleton standing'
            if gap <= cfg.max_gap_s:
                # Unmeasured frame: shift every timer, never count it as evidence.
                self._shift(state, gap)
            return (2 if state['fallen'] else 1 if state['pending'] is not None else 0), 'height unmeasured; timers paused'
        upright_now = top >= cfg.upright_min_m or (posture_upright and top >= cfg.hint_upright_min_m)
        if state['fallen']:
            if top >= cfg.upright_min_m or (posture_upright and top > cfg.low_max_m):
                state['recovery'] = stamp if state['recovery'] is None else state['recovery']
                if stamp-state['recovery'] >= cfg.recovery_s:
                    state.update(fallen=False, recovery=None, pending=None, baseline=None, upright=deque())
                    return 0, 'upright recovery complete'
            else:
                state['recovery'] = None
            return 2, f'confirmed fall; top={top:.2f}m'
        upright = state['upright']
        if upright_now:
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
        held = cfg.sit_hold_s > 0 and state['drop'] is not None and stamp-state['drop'] <= cfg.sit_hold_s
        if state['drop'] is not None and (not held or top >= cfg.upright_min_m):
            state['drop'] = None
            held = False
        if upright_hint and top <= cfg.low_max_m:
            state['pending'] = None
            if held:
                return 1, f'crouch after a drop; held, top={top:.2f}m'
            return 0, f'low top but skeleton upright (crouch); top={top:.2f}m'
        if state['pending'] is not None:
            if top > cfg.low_max_m:
                state['pending'] = None
                if held:
                    return 1, f'sat up after a drop; held, top={top:.2f}m'
                state['drop'] = None
                return 0, f'rose before confirmation; top={top:.2f}m'
            if stamp-state['pending'] >= cfg.confirm_s:
                state.update(fallen=True, pending=None, recovery=None, drop=None)
                return 2, f'body top stayed low; top={top:.2f}m'
            return 1, f'rapid drop to the floor; top={top:.2f}m'
        if held:
            if top <= cfg.low_max_m:
                state['pending'] = stamp
                return 1, f'low again after a drop; top={top:.2f}m'
            return 1, f'sat up after a drop; held, top={top:.2f}m'
        if (baseline is not None and top <= cfg.low_max_m and baseline[1]-top >= cfg.drop_min_m and not side):
            state['pending'] = state['drop'] = stamp
            return 1, f'rapid drop to the floor; top {baseline[1]:.2f}->{top:.2f}m'
        return 0, f'top={top:.2f}m' + ('; static low, no observed drop' if top <= cfg.low_max_m else '')
