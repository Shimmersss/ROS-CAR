"""Robust measurement from depth already registered to the color optical frame."""
import math
import numpy as np


def measure(depth, box, intrinsics, min_fraction=0.3):
    """Return optical XYZ in metres; reject sparse/invalid central torso samples."""
    x1, y1, x2, y2 = map(float, box)
    if not all(map(math.isfinite, (x1, y1, x2, y2))) or x2 <= x1 or y2 <= y1:
        return None
    h, w = depth.shape
    left, right = max(0, int(x1 + .3*(x2-x1))), min(w, int(x1 + .7*(x2-x1)))
    top, bottom = max(0, int(y1 + .25*(y2-y1))), min(h, int(y1 + .6*(y2-y1)))
    if right <= left or bottom <= top:
        return None
    roi = depth[top:bottom, left:right]
    valid = np.isfinite(roi) & (roi >= .2) & (roi <= 8.)
    if valid.sum() < max(12, roi.size * min_fraction):
        return None
    values = roi[valid]
    z = float(np.median(values))
    # Broad depth spread usually means background contamination or an edge.
    if float(np.percentile(values, 75) - np.percentile(values, 25)) > max(.25, .2*z):
        return None
    fx, fy, cx, cy = intrinsics
    if not all(map(math.isfinite, intrinsics)) or fx <= 0 or fy <= 0:
        return None
    v, u = np.nonzero(valid & (np.abs(roi-z) <= max(.1, .1*z)))
    if not len(u):
        return None
    u, v = float(np.median(u))+left, float(np.median(v))+top
    return ((u-cx)*z/fx, (v-cy)*z/fy, z)
