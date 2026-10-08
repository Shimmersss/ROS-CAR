"""RANSAC floor plane from registered depth, expressed in the optical frame.

Evidence only: a fitted plane never confirms the ground plane or gravity by itself.
Callers keep their explicit confirmation flags and use this as a per-frame source.
"""
from dataclasses import dataclass
import math

import numpy as np


@dataclass(frozen=True)
class FloorConfig:
    stride: int = 4
    roi_top_fraction: float = .35
    min_depth_m: float = .3
    max_depth_m: float = 4.
    box_margin_px: float = 10.
    iterations: int = 200
    # Half of the hypotheses take their 2nd/3rd point within this many grid cells of the
    # 1st, so a floor covering a small share of the ROI is still sampled reliably.
    local_window: int = 12
    inlier_m: float = .02
    min_inliers: int = 300
    # Walls dominate the ROI when the gimbal looks up; the up-facing prefilter, absolute
    # inlier count and residual carry the rejection, so the fraction floor stays low.
    min_inlier_fraction: float = .10
    max_rms_m: float = .015
    # A narrow floor strip cannot pin the normal down: require this in-plane spread (std)
    # along BOTH principal axes of the inliers.
    min_extent_m: float = .2
    max_tilt_deg: float = 35.
    min_height_m: float = .05
    max_height_m: float = 2.5
    seed: int = 0
    confirm_frames: int = 3
    max_gap_s: float = .5
    stable_angle_deg: float = 2.
    stable_height_m: float = .03

    def __post_init__(self):
        for name in ('stride', 'iterations', 'local_window', 'min_inliers', 'seed', 'confirm_frames'):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, int) or value < (0 if name == 'seed' else 1):
                raise ValueError(f'floor {name} must be a positive integer')
        for name, value in vars(self).items():
            if isinstance(value, float) and (not math.isfinite(value) or value <= 0):
                raise ValueError(f'floor {name} must be finite and positive')
        if not self.roi_top_fraction < 1 or not self.min_inlier_fraction <= 1:
            raise ValueError('floor fractions must be below 1')
        if not self.min_depth_m < self.max_depth_m or not self.min_height_m < self.max_height_m:
            raise ValueError('floor depth/height ranges must be increasing')
        if not self.max_tilt_deg < 90:
            raise ValueError('floor max_tilt_deg must be below 90')


@dataclass(frozen=True)
class FloorFit:
    valid: bool
    up: tuple = (math.nan, math.nan, math.nan)
    height_m: float = math.nan
    inliers: int = 0
    inlier_fraction: float = 0.
    rms_m: float = math.nan
    reason: str = ''


def camera_angles(up):
    """(pitch_up_deg, roll_deg) of the optical frame relative to the plane normal `up`.

    Optical axes: x right, y down, z forward. A level camera sees up=(0,-1,0).
    Pitch is positive when the optical axis looks above the horizon; roll is
    positive when the image x-axis is raised (up leans towards +x).
    """
    x, y, z = map(float, up)
    return math.degrees(math.atan2(z, -y)), math.degrees(math.atan2(x, -y))


def angle_deg(a, b):
    a, b = np.asarray(a, float), np.asarray(b, float)
    return math.degrees(math.acos(float(np.clip(a@b/(np.linalg.norm(a)*np.linalg.norm(b)), -1., 1.))))


def _orient(normal, offset):
    # Put the camera (origin) on the positive side, so `normal` points from floor to camera.
    return (-normal, -offset) if offset < 0 else (normal, offset)


def fit_floor(depth_m, intrinsics, boxes=(), expected_up=(0., -1., 0.), config=None):
    """Fit the dominant floor plane below the image ROI; people boxes are excluded."""
    cfg = config or FloorConfig()
    depth = np.asarray(depth_m)
    fx, fy, cx, cy = map(float, intrinsics)
    if depth.ndim != 2 or not all(map(math.isfinite, (fx, fy, cx, cy))) or fx <= 0 or fy <= 0:
        return FloorFit(False, reason='invalid depth image or intrinsics')
    expected = np.asarray(expected_up, float)
    if expected.shape != (3,) or not np.isfinite(expected).all() or np.linalg.norm(expected) < 1e-6:
        return FloorFit(False, reason='invalid expected up vector')
    expected = expected/np.linalg.norm(expected)
    h, w = depth.shape
    vs, us = np.mgrid[int(h*cfg.roi_top_fraction):h:cfg.stride, 0:w:cfg.stride]
    z = depth[vs, us].astype(float)
    keep = np.isfinite(z) & (z >= cfg.min_depth_m) & (z <= cfg.max_depth_m)
    m = cfg.box_margin_px
    for box in boxes:
        x1, y1, x2, y2 = map(float, box)
        keep &= ~((us >= x1-m) & (us <= x2+m) & (vs >= y1-m) & (vs <= y2+m))
    index = np.full(keep.shape, -1)
    index[keep] = np.arange(int(keep.sum()))
    z = z[keep]
    points = np.column_stack(((us[keep]-cx)*z/fx, (vs[keep]-cy)*z/fy, z))
    total = len(points)
    if total < cfg.min_inliers:
        return FloorFit(False, reason=f'too few floor samples: {total}')
    rng = np.random.default_rng(cfg.seed)
    half = cfg.iterations//2
    picks = rng.integers(0, total, size=(cfg.iterations, 3))
    rows, cols = np.nonzero(keep)
    seeds = rng.integers(0, total, size=half)
    for k in (1, 2):
        r = np.clip(rows[seeds]+rng.integers(-cfg.local_window, cfg.local_window+1, half), 0, keep.shape[0]-1)
        c = np.clip(cols[seeds]+rng.integers(-cfg.local_window, cfg.local_window+1, half), 0, keep.shape[1]-1)
        neighbour = index[r, c]
        picks[:half, k] = np.where(neighbour >= 0, neighbour, seeds)  # invalid -> degenerate, dropped
    picks[:half, 0] = seeds
    a, b, c = points[picks[:, 0]], points[picks[:, 1]], points[picks[:, 2]]
    normals = np.cross(b-a, c-a)
    lengths = np.linalg.norm(normals, axis=1)
    usable = lengths > 1e-9
    normals[usable] /= lengths[usable, None]
    offsets = -np.einsum('ij,ij->i', normals, a)
    flip = offsets < 0
    normals[flip] *= -1
    offsets[flip] *= -1
    # Reject walls/furniture before counting: the floor must face roughly up.
    usable &= normals@expected >= math.cos(math.radians(cfg.max_tilt_deg))
    if not usable.any():
        return FloorFit(False, reason='no candidate plane faces the expected up direction')
    distances = np.abs(points@normals[usable].T+offsets[usable])
    best = int(np.argmax((distances < cfg.inlier_m).sum(axis=0)))
    normal, offset = normals[usable][best], offsets[usable][best]
    threshold = cfg.inlier_m
    inliers = np.abs(points@normal+offset) < threshold
    for _ in range(3):
        if inliers.sum() < 3:
            break
        selected = points[inliers]
        centroid = selected.mean(axis=0)
        normal = np.linalg.svd(selected-centroid, full_matrices=False)[2][-1]
        normal, offset = _orient(normal, -float(normal@centroid))
        residual = points@normal+offset
        # Tighten to the measured noise: the base strip of a wall lies on one side of
        # the floor within the coarse band and would otherwise tilt the plane.
        sigma = 1.4826*float(np.median(np.abs(residual[inliers])))
        threshold = min(cfg.inlier_m, max(.005, 3*sigma))
        inliers = np.abs(residual) < threshold
    count = int(inliers.sum())
    residual = points[inliers]@normal+offset if count else np.array([math.nan])
    rms = float(np.sqrt(np.mean(residual**2)))
    extent = 0.
    if count >= 3:
        spread = np.linalg.svd(points[inliers]-points[inliers].mean(axis=0), compute_uv=False)
        extent = float(spread[1]/math.sqrt(count))
    fraction = count/total
    tilt = angle_deg(normal, expected)
    fit = dict(up=tuple(map(float, normal)), height_m=float(offset), inliers=count,
               inlier_fraction=float(fraction), rms_m=rms)
    for failed, reason in (
            (count < cfg.min_inliers, f'too few inliers: {count}'),
            (fraction < cfg.min_inlier_fraction, f'inlier fraction {fraction:.2f} too low'),
            (not rms <= cfg.max_rms_m, f'plane residual {rms:.3f} m too high'),
            (extent < cfg.min_extent_m, f'floor patch too narrow (in-plane std {extent:.2f} m)'),
            (tilt > cfg.max_tilt_deg, f'plane tilt {tilt:.1f} deg from expected up'),
            (not cfg.min_height_m <= offset <= cfg.max_height_m, f'camera height {offset:.2f} m out of range')):
        if failed:
            return FloorFit(False, reason=reason, **fit)
    return FloorFit(True, reason=f'inliers={count}; rms={rms:.4f}m; tilt={tilt:.1f}deg', **fit)


class FloorTracker:
    """Consecutive-fit consistency; any gap, rewind or invalid fit restarts confirmation."""

    def __init__(self, config=None):
        self.cfg = config or FloorConfig()
        self.reset()

    def reset(self):
        self.reference, self.stamp, self.count = None, None, 0

    def update(self, stamp, fit):
        cfg = self.cfg
        if (not math.isfinite(stamp) or self.stamp is not None
                and not 0 < stamp-self.stamp <= cfg.max_gap_s):
            self.reset()
        self.stamp = stamp if math.isfinite(stamp) else None
        if not fit.valid:
            self.reference, self.count = None, 0
            return False, fit.reason
        ref = self.reference
        consistent = (ref is not None and angle_deg(ref.up, fit.up) <= cfg.stable_angle_deg
                      and abs(ref.height_m-fit.height_m) <= cfg.stable_height_m)
        self.count = self.count+1 if consistent else 1
        self.reference = fit
        stable = self.count >= cfg.confirm_frames
        return stable, f'{fit.reason}; consistent={min(self.count, cfg.confirm_frames)}/{cfg.confirm_frames}'


def ankle_contact(u, v, intrinsics, up, height_m, ankle_height_m):
    """Optical-frame point where the pixel ray meets the plane raised by ankle_height_m."""
    fx, fy, cx, cy = map(float, intrinsics)
    ray = np.array([(u-cx)/fx, (v-cy)/fy, 1.])
    normal = np.asarray(up, float)
    denominator = float(normal@ray)
    # The ray must head down towards the floor; grazing rays are ill-conditioned.
    if not denominator < -.02*np.linalg.norm(ray):
        return None
    scale = (ankle_height_m-height_m)/denominator
    if not math.isfinite(scale) or scale <= 0:
        return None
    return ray*scale
