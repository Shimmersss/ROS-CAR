"""C-only fusion of current RGB-D observations; no learned or held depth.

The target is a virtual point on the detection-box centre ray at the associated
body depth, not a physical joint, centre of mass or ground contact point. Joints
retain their own measured depth. B's original representative point is unchanged.
"""
from dataclasses import dataclass
from itertools import combinations
import math

import cv2
import numpy as np

from .depth import _measure_region, region_candidates, select_region_candidates
from .pose import plausible_torso3d, points2d, sample_joints3d


@dataclass(frozen=True)
class FusionConfig:
    enabled: bool = True
    min_anchor_joints: int = 2
    cluster_abs_m: float = .15
    cluster_rel: float = .12
    torso_scale: float = .65

    def __post_init__(self):
        if not isinstance(self.enabled, bool):
            raise ValueError('fusion_enabled must be boolean')
        if (type(self.min_anchor_joints) is not int
                or not 2 <= self.min_anchor_joints <= 4):
            raise ValueError('fusion_min_anchor_joints must be an integer in [2, 4]')
        for name in ('cluster_abs_m', 'cluster_rel', 'torso_scale'):
            value = getattr(self, name)
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f'fusion_{name} must be finite and positive')
        if self.cluster_rel > 1 or self.torso_scale > 1:
            raise ValueError('fusion_cluster_rel and fusion_torso_scale must be <= 1')


@dataclass(frozen=True)
class DepthObservation:
    target: tuple | None
    joints: np.ndarray
    source: str
    reason: str


def _empty(reason):
    return DepthObservation(None, np.full((17, 3), np.nan), 'invalid', reason)


def _tolerance(z, cfg):
    return max(cfg.cluster_abs_m, cfg.cluster_rel*z)


def _anchor_cluster(raw, points, indices, cfg):
    """Require distinct non-overlapping 7x7 patches, not repeated pixel votes."""
    indices = [i for i in indices if np.isfinite(raw[i]).all()]
    groups = []
    for n in range(len(indices), cfg.min_anchor_joints-1, -1):
        for group in combinations(indices, n):
            if any(np.max(np.abs(np.rint(points[a, :2])-np.rint(points[b, :2]))) < 7
                   for a, b in combinations(group, 2)):
                continue
            zs = raw[list(group), 2]
            if np.ptp(zs) <= _tolerance(float(np.median(zs)), cfg):
                groups.append(group)
        if groups:
            break
    if not groups:
        zs = raw[indices, 2]
        conflicting = (len(zs) >= cfg.min_anchor_joints
                       and np.ptp(zs) > _tolerance(float(np.median(zs)), cfg))
        return None, bool(conflicting)
    centres = [float(np.median(raw[list(g), 2])) for g in groups]
    if max(centres)-min(centres) > _tolerance(float(np.median(centres)), cfg):
        return None, True  # Equally supported, distinct surfaces: do not pick nearest.
    best = min(groups, key=lambda g: float(np.ptp(raw[list(g), 2])))
    return raw[list(best), 2], False


def fuse_depth(depth, box, keypoints, intrinsics, confidence=.5,
               min_fraction=.08, other_boxes=(), config=None):
    """Collect -> associate -> estimate once for target and all current joints.

    Body anchors outrank broad regions, which can contain a wall. Knees/ankles
    can recover truncated upper bodies; hands never independently anchor the
    target. Correlated samples do not reduce the downstream Kalman covariance.
    """
    cfg = config or FusionConfig()
    b = np.asarray(box, dtype=float)
    k = np.asarray(intrinsics, dtype=float)
    if (depth.ndim != 2 or b.shape != (4,) or not np.isfinite(b).all()
            or b[2] <= b[0] or b[3] <= b[1]
            or k.shape != (4,) or not np.isfinite(k).all() or min(k[:2]) <= 0):
        return _empty('invalid box/depth/intrinsics')
    h, w = depth.shape
    left, top = max(0, math.ceil(b[0])), max(0, math.ceil(b[1]))
    right, bottom = min(w, math.ceil(b[2])), min(h, math.ceil(b[3]))
    if left >= right or top >= bottom:
        return _empty('box outside image')

    # Explicitly exclude pixels outside this person and inside any other
    # detection, including tentative tracks. No cross-person depth borrowing.
    allowed = np.zeros((h, w), dtype=bool)
    allowed[top:bottom, left:right] = True
    overlap = False
    for other in other_boxes:
        o = np.asarray(other, dtype=float)
        if o.shape != (4,) or not np.isfinite(o).all():
            continue
        x1, y1 = max(left, math.ceil(o[0])), max(top, math.ceil(o[1]))
        x2, y2 = min(right, math.ceil(o[2])), min(bottom, math.ceil(o[3]))
        if x1 < x2 and y1 < y2:
            overlap = True
            allowed[y1:y2, x1:x2] = False
    masked = np.where(allowed, depth, 0.)
    points, valid = points2d(keypoints, confidence)
    raw = sample_joints3d(masked, keypoints, k, confidence, box=b)
    for i in np.flatnonzero(valid):
        x, y = map(int, np.rint(points[i, :2]))
        if not (0 <= x < w and 0 <= y < h and allowed[y, x]):
            valid[i] = False
            raw[i] = np.nan

    regions = region_candidates(masked, b, k, min_fraction, repair_small_holes=False)
    torso = None
    if valid[[5, 6, 11, 12]].all():
        vertices = points[[5, 6, 12, 11], :2]
        centre = vertices.mean(axis=0)
        vertices = centre + cfg.torso_scale*(vertices-centre)
        polygon = cv2.convexHull(np.rint(vertices).astype(np.int32))
        if cv2.contourArea(polygon) >= 49:
            mask = np.zeros((h, w), dtype=np.uint8)
            cv2.fillConvexPoly(mask, polygon, 1)
            xs, ys, bw, bh = cv2.boundingRect(polygon)
            bounds = (max(left, xs), max(top, ys), min(right, xs+bw), min(bottom, ys+bh))
            torso = _measure_region(np.where(mask, masked, 0.), bounds, k,
                                    min_fraction, repair_small_holes=False)

    zs, conflict = _anchor_cluster(raw, points, (5, 6, 11, 12), cfg)
    tilted = conflict and plausible_torso3d(raw)
    if conflict and not tilted:
        return _empty('conflicting torso anchor depths')
    if tilted:
        zs = raw[[5,6,11,12],2]

    if zs is not None:
        source = 'pose_3d_anchors' if tilted else 'pose_anchors'
    elif torso is not None:
        z = torso[0][2]
        anchors = raw[[5, 6, 11, 12], 2]
        finite = anchors[np.isfinite(anchors)]
        if len(finite) and np.any(np.abs(finite-z) > _tolerance(z, cfg)):
            return _empty('torso region disagrees with joint depth')
        zs = np.array([z])
        source = 'pose_torso'
    else:
        zs, conflict = _anchor_cluster(raw, points, (13, 14, 15, 16), cfg)
        if conflict:
            return _empty('conflicting lower-body anchor depths')
        if zs is not None:
            z = float(np.median(zs))
            upper = raw[[5, 6, 11, 12], 2]
            if np.any(np.abs(upper[np.isfinite(upper)]-z) > _tolerance(z, cfg)):
                return _empty('upper/lower depth evidence disagrees')
            source = 'pose_lower_body'
        else:
            if overlap:
                return _empty('overlapping people without unambiguous pose depth')
            legacy = select_region_candidates(regions)
            if legacy is None:
                return _empty('insufficient real depth support')
            z = legacy[2]
            anchors = raw[[5, 6, 11, 12], 2]
            if np.any(np.abs(anchors[np.isfinite(anchors)]-z) > _tolerance(z, cfg)):
                return _empty('region fallback disagrees with torso joint')
            zs = np.array([z])
            source = 'regions'

    # One candidate per body region, never thousands of wall pixels outweighing
    # the pose anchors. Keep the anchor median when broad regions are inconsistent.
    anchor_z = float(np.median(zs))
    supporting = [r[0][2] for r in regions
                  if abs(r[0][2]-anchor_z) <= _tolerance(anchor_z, cfg)]
    if torso is not None and abs(torso[0][2]-anchor_z) <= _tolerance(anchor_z, cfg):
        supporting.append(torso[0][2])
    z = float(np.median([anchor_z, float(np.median(supporting))])) if supporting else anchor_z
    fx, fy, cx, cy = k
    u, v = (b[:2]+b[2:])*.5
    target = (float((u-cx)*z/fx), float((v-cy)*z/fy), z)
    joints = raw.copy()
    # A joint may be closer/farther than the representative point, but a distant
    # wall must not become a joint. Never fill a hole with target/predicted depth.
    joints[np.abs(joints[:, 2]-anchor_z) > max(.5, .3*anchor_z)] = np.nan
    return DepthObservation(target, joints, source, 'current real depth; bbox-centre ray')
