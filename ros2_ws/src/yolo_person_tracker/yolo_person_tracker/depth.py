"""Robust measurement from depth already registered to the color optical frame."""
import math
import numpy as np


class DepthTrackFilter:
    """Short-memory XYZ smoother for one ByteTrack identity."""

    def __init__(self, alpha=.35, jump_reset_m=.8):
        if not 0 < alpha <= 1:
            raise ValueError('alpha must be in (0, 1]')
        if jump_reset_m <= 0:
            raise ValueError('jump_reset_m must be positive')
        self.alpha = float(alpha)
        self.jump_reset_m = float(jump_reset_m)
        self.value = None
        self.stamp = None

    def reset(self):
        self.value = None
        self.stamp = None

    def update(self, xyz, stamp):
        if xyz is None:
            return None
        point = np.asarray(xyz, dtype=np.float32)
        if point.shape != (3,) or not np.isfinite(point).all():
            return None
        if (self.value is None or self.stamp is None
                or stamp < self.stamp
                or float(np.linalg.norm(point-self.value)) > self.jump_reset_m):
            self.value = point
        else:
            self.value = self.alpha*point + (1-self.alpha)*self.value
        self.stamp = float(stamp)
        return tuple(float(v) for v in self.value)


def _measure_region(depth, bounds, intrinsics, min_fraction):
    left, top, right, bottom = bounds
    roi = depth[top:bottom, left:right]
    if roi.size == 0:
        return None
    valid = np.isfinite(roi) & (roi >= .2) & (roi <= 8.)
    if valid.sum() < max(12, roi.size * min_fraction):
        return None
    values = roi[valid]
    z = float(np.median(values))
    # Reject a region that mixes foreground and background, while tolerating
    # the depth holes and edge quantisation common in Astra frames.
    if float(np.percentile(values, 75) - np.percentile(values, 25)) > max(.35, .3*z):
        return None
    v, u = np.nonzero(valid & (np.abs(roi-z) <= max(.1, .1*z)))
    if not len(u):
        return None
    u, v = float(np.median(u))+left, float(np.median(v))+top
    fx, fy, cx, cy = intrinsics
    return ((u-cx)*z/fx, (v-cy)*z/fy, z), float(valid.sum()) / roi.size


def measure(depth, box, intrinsics, min_fraction=0.15):
    """Return XYZ from the best stable depth patch inside one YOLO person box.

    The patch candidates cover torso, upper/lower body, and left/right body
    regions. This keeps the measurement associated with the selected YOLO
    detection while allowing a missing torso depth patch to recover from an
    arm, shoulder, head, or leg patch.
    """
    x1, y1, x2, y2 = map(float, box)
    if not all(map(math.isfinite, (x1, y1, x2, y2))) or x2 <= x1 or y2 <= y1:
        return None
    h, w = depth.shape
    bw, bh = x2-x1, y2-y1
    fx, fy, cx, cy = intrinsics
    if not all(map(math.isfinite, intrinsics)) or fx <= 0 or fy <= 0:
        return None
    # Keep every candidate strictly inside the same YOLO detection. The nearest
    # stable patch is preferred: a background wall or floor can have more
    # valid pixels than a person's back, but should never win the association.
    regions = (
        (.30, .25, .70, .60),  # torso
        (.20, .10, .80, .40),  # head/shoulders
        (.20, .55, .80, .95),  # legs/lower body
        (.05, .25, .45, .70),  # left side/arm
        (.55, .25, .95, .70),  # right side/arm
    )
    best = None
    for ax1, ay1, ax2, ay2 in regions:
        left = max(0, int(x1 + ax1*bw)); right = min(w, int(x1 + ax2*bw))
        top = max(0, int(y1 + ay1*bh)); bottom = min(h, int(y1 + ay2*bh))
        result = _measure_region(depth, (left, top, right, bottom), intrinsics, min_fraction)
        if result is not None and (best is None or
                                   (result[0][2], -result[1]) <
                                   (best[0][2], -best[1])):
            best = result
    return None if best is None else best[0]
