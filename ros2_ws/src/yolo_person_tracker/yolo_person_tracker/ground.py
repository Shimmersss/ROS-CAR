"""Monocular ground-contact estimation from COCO17 keypoints and a confirmed camera mount.

Rays are intersected with horizontal planes in the target (base) frame. Nothing here
proves the mount, ground plane or person height; callers must gate on confirmation.
"""
from dataclasses import dataclass
import math

import numpy as np

from .depth import DepthTrackFilter

LEFT_SHOULDER, RIGHT_SHOULDER, LEFT_HIP, RIGHT_HIP, LEFT_ANKLE, RIGHT_ANKLE = 5, 6, 11, 12, 15, 16
# PersonState posture codes: 0 unknown, 1 standing, 2 sitting/crouching.
UPRIGHT_POSTURES = (1,)
GROUND_POSTURES = (0, 1, 2)
_BOOL_FIELDS = ('enable_height_prior', 'allow_box_bottom_fallback', 'height_prior_confirmed')


@dataclass(frozen=True)
class GroundConfig:
    min_keypoint_confidence: float = 0.5
    person_height_m: float = 1.70
    person_height_std_m: float = 0.08
    ankle_height_m: float = 0.08
    pixel_std: float = 3.0
    max_range_m: float = 12.0
    max_std_m: float = 1.0
    min_ray_slope: float = 0.02
    edge_margin_px: float = 4.0
    # Plane height in the target frame; the caller must confirm it for the installation.
    ground_z_m: float = 0.0
    enable_height_prior: bool = False
    height_prior_confirmed: bool = False
    allow_box_bottom_fallback: bool = False
    consistency_max_m: float = 0.8
    consistency_sigma: float = 2.0
    # Approximate stature ratios for shoulder / hip joint heights (initial values).
    shoulder_height_ratio: float = 0.82
    hip_height_ratio: float = 0.53

    def __post_init__(self):
        for name in _BOOL_FIELDS:
            if not isinstance(getattr(self, name), bool):
                raise ValueError(f'{name} must be bool')
        for name, value in vars(self).items():
            if name not in _BOOL_FIELDS and (isinstance(value, bool) or not math.isfinite(value)):
                raise ValueError(f'{name} must be a finite number')
        if self.allow_box_bottom_fallback:
            raise ValueError('Box-bottom fallback has no contact evidence and is unsupported')
        for name in ('consistency_max_m', 'consistency_sigma', 'min_keypoint_confidence', 'person_height_m', 'pixel_std', 'max_range_m',
                     'max_std_m', 'min_ray_slope', 'shoulder_height_ratio',
                     'hip_height_ratio'):
            if getattr(self, name) <= 0:
                raise ValueError(f'{name} must be positive')
        for name in ('person_height_std_m', 'ankle_height_m', 'edge_margin_px'):
            if getattr(self, name) < 0:
                raise ValueError(f'{name} must be non-negative')
        if self.min_keypoint_confidence > 1.0:
            raise ValueError('min_keypoint_confidence must be <= 1')
        if not self.hip_height_ratio < self.shoulder_height_ratio < 1.0:
            raise ValueError('require hip_height_ratio < shoulder_height_ratio < 1')


@dataclass(frozen=True)
class GroundEstimate:
    point: tuple
    confidence: float
    std_m: float
    method: str
    detail: str


def camera_model_from_info(info):
    """Return ((fx, fy, cx, cy), (width, height)) for an undistorted, uncropped CameraInfo."""
    p = tuple(float(v) for v in info.p)
    if (len(p) != 12 or not all(math.isfinite(v) for v in p)
            or p[0] <= 0 or p[5] <= 0 or p[1] != 0 or p[4] != 0
            or p[3] != 0 or p[7] != 0 or tuple(p[8:12]) != (0.0, 0.0, 1.0, 0.0)):
        raise ValueError('Invalid or unsupported rectified color projection matrix')
    if info.width <= 0 or info.height <= 0:
        raise ValueError('CameraInfo has empty image size')
    if info.binning_x > 1 or info.binning_y > 1 or info.roi.x_offset or info.roi.y_offset:
        raise ValueError('Binned/cropped calibration requires explicit normalization')
    return (p[0], p[5], p[2], p[6]), (int(info.width), int(info.height))


def rotation_matrix(quaternion):
    x, y, z, w = quaternion
    norm = math.sqrt(x*x + y*y + z*z + w*w)
    if not math.isfinite(norm) or norm <= 0:
        raise ValueError('Invalid quaternion norm')
    x, y, z, w = x/norm, y/norm, z/norm, w/norm
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-w*z), 2*(x*z+w*y)],
        [2*(x*y+w*z), 1-2*(x*x+z*z), 2*(y*z-w*x)],
        [2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x*x+y*y)],
    ])


class _Camera:
    """Base-frame pose of the optical frame plus pinhole intrinsics."""

    def __init__(self, intrinsics, size, transform):
        self.fx, self.fy, self.cx, self.cy = map(float, intrinsics)
        self.width, self.height = size
        r, t = transform.rotation, transform.translation
        self.rotation = rotation_matrix((r.x, r.y, r.z, r.w))
        self.origin = np.array([t.x, t.y, t.z], dtype=np.float64)
        if (not all(math.isfinite(v) for v in (self.fx, self.fy, self.cx, self.cy))
                or self.fx <= 0 or self.fy <= 0 or not np.isfinite(self.origin).all()):
            raise ValueError('Non-finite camera model')

    def ray(self, u, v):
        direction = self.rotation @ np.array([(u-self.cx)/self.fx, (v-self.cy)/self.fy, 1.0])
        return direction / np.linalg.norm(direction)

    def inside(self, u, v, margin):
        return margin <= u <= self.width-margin and margin <= v <= self.height-margin

    def hit(self, u, v, plane_z, config):
        """Intersect the pixel ray with z=plane_z; None unless well-conditioned and in range."""
        d = self.ray(u, v)
        horizontal = math.hypot(d[0], d[1])
        if horizontal <= 0 or abs(d[2]) / horizontal < config.min_ray_slope:
            return None
        scale = (plane_z - self.origin[2]) / d[2]
        if not math.isfinite(scale) or scale <= 0:
            return None
        p = self.origin + scale*d
        if not np.isfinite(p).all() or math.hypot(p[0], p[1]) > config.max_range_m:
            return None
        return p[:2]

    def hit_std(self, u, v, plane_z, config, plane_std=0.0):
        """Propagate pixel and plane-height uncertainty by finite differences."""
        base = self.hit(u, v, plane_z, config)
        if base is None:
            return None, math.inf
        total = 0.0
        for du, dv, dz in ((config.pixel_std, 0., 0.), (0., config.pixel_std, 0.),
                           (0., 0., plane_std)):
            worst = 0.0
            for sign in (-1., 1.):
                other = self.hit(u+sign*du, v+sign*dv, plane_z+sign*dz, config)
                if other is None:
                    return base, math.inf
                worst = max(worst, float(np.linalg.norm(other-base)))
            total += worst*worst
        return base, math.sqrt(total)


def _extract(person):
    if len(person.keypoints_2d) != 17 or len(person.keypoint_confidences) != 17:
        return None, None
    points = np.full((17, 2), np.nan)
    confidences = np.zeros(17)
    for i, point in enumerate(person.keypoints_2d):
        confidence = float(person.keypoint_confidences[i])
        if math.isfinite(point.x) and math.isfinite(point.y) and math.isfinite(confidence):
            points[i] = (point.x, point.y)
            confidences[i] = confidence
    return points, confidences


def _usable(points, confidences, index, config, camera, margin=0.0):
    return (confidences[index] >= config.min_keypoint_confidence
            and np.isfinite(points[index]).all()
            and camera.inside(points[index, 0], points[index, 1], margin))


def _feet(person, points, confidences, camera, config):
    # Ankles on the bottom image edge are truncated, not real ground contact.
    results = []
    for index in (LEFT_ANKLE, RIGHT_ANKLE):
        if _usable(points, confidences, index, config, camera, config.edge_margin_px):
            point, std = camera.hit_std(points[index, 0], points[index, 1],
                                        config.ground_z_m+config.ankle_height_m, config,
                                        plane_std=config.ankle_height_m*0.5)
            if point is not None and std <= config.max_std_m:
                results.append((point, std, confidences[index]))
    if len(results) != 2:
        return None, 'two visible usable ankles required; box bottom is not contact evidence'
    return _combine(results, config, 'feet_ankles')


def _combine(samples, config, method):
    # Anatomical lateral separation is allowed, but divergent ranges must not
    # become a fictitious midpoint. A hard limit prevents large noise hiding it.
    for i, (a, sa, _) in enumerate(samples):
        for b, sb, _ in samples[i+1:]:
            limit = min(config.consistency_max_m,
                        .4 + config.consistency_sigma*math.hypot(sa, sb))
            if np.linalg.norm(a-b) > limit:
                return None, 'conflicting ground anchors'
    xy = np.mean([a[0] for a in samples], axis=0)
    spread = max(float(np.linalg.norm(a[0]-xy)) for a in samples)
    std = math.hypot(max(a[1] for a in samples), spread)
    if std > config.max_std_m:
        return None, 'anchor spread exceeds uncertainty limit'
    return GroundEstimate(
        (float(xy[0]), float(xy[1]), config.ground_z_m),
        float(np.mean([a[2] for a in samples])) * (.5 if method == 'height_prior' else 1.),
        std, method, f'anchors={len(samples)}; conditional on ground/contact assumptions'), None


def _height_prior(person, points, confidences, camera, config):
    if not config.enable_height_prior or not config.height_prior_confirmed:
        return None, 'height prior disabled or unconfirmed'
    if int(person.posture) not in UPRIGHT_POSTURES:
        return None, 'height prior requires confirmed standing posture'
    anchors = []
    for index, ratio in ((LEFT_SHOULDER, config.shoulder_height_ratio),
                         (RIGHT_SHOULDER, config.shoulder_height_ratio),
                         (LEFT_HIP, config.hip_height_ratio),
                         (RIGHT_HIP, config.hip_height_ratio)):
        if not _usable(points, confidences, index, config, camera):
            continue
        point, std = camera.hit_std(
            points[index, 0], points[index, 1],
            config.ground_z_m+config.person_height_m*ratio, config,
            plane_std=config.person_height_std_m*ratio)
        if point is not None and std <= config.max_std_m:
            anchors.append((point, std, confidences[index]))
    if len(anchors) < 2:
        return None, 'height prior needs at least two shoulder/hip anchors'
    return _combine(anchors, config, 'height_prior')


def estimate_ground_point(person, intrinsics, size, transform, config=None):
    """Return (GroundEstimate, None) or (None, reason) for one PersonState.

    `transform` is base<-optical (geometry_msgs/Transform). The reference is the
    person's ground-contact point, not the RGB-D body-surface target point.
    """
    config = config or GroundConfig()
    if int(person.posture) not in GROUND_POSTURES or int(person.fall_stage) != 0:
        return None, 'lying/fall posture: ground-contact reference undefined'
    points, confidences = _extract(person)
    if points is None:
        return None, 'invalid COCO17 keypoints'
    camera = _Camera(intrinsics, size, transform)
    estimate, reason_feet = _feet(person, points, confidences, camera, config)
    if estimate is not None:
        return estimate, None
    if reason_feet == 'conflicting ground anchors':
        return None, reason_feet
    estimate, reason_prior = _height_prior(person, points, confidences, camera, config)
    if estimate is not None:
        return estimate, None
    return None, '; '.join(r for r in (reason_feet, reason_prior) if r)


class GroundTrackFilter(DepthTrackFilter):
    """Per-person constant-velocity filter on base-frame ground points with per-sample noise."""

    def __init__(self, acceleration_std_mps2=2.0, jump_reset_m=1.0, reset_gap_s=0.5):
        super().__init__(0.1, acceleration_std_mps2, jump_reset_m, reset_gap_s)
        self.ground_method = ''

    def update_ground(self, point, stamp, std_m):
        self.measurement_variance = max(float(std_m), 0.05)**2
        return self.update(point, stamp)
