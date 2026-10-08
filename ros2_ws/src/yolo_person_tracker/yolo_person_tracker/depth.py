"""Robust measurement from depth already registered to the color optical frame."""
from dataclasses import dataclass
import math
import numpy as np


@dataclass(frozen=True)
class DepthGate:
    """Physical innovation gate: base noise allowance plus plausible motion per second.

    Fallback measurements (no torso corroboration) get a tighter gate and lower
    weight. A rejected point never refreshes the measurement stamp; relocation is
    accepted only after mutually consistent rejected evidence spans confirm_s.
    """
    base_m: float = .35
    speed_mps: float = 2.
    confirm_s: float = .3
    fallback_scale: float = .6
    fallback_noise_scale: float = 2.

    def __post_init__(self):
        for name, value in vars(self).items():
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f'depth gate {name} must be finite and positive')
        if self.fallback_scale > 1 or self.fallback_noise_scale < 1:
            raise ValueError('depth gate fallback_scale must be <=1 and noise scale >=1')

    def limit(self, dt, fallback=False):
        return (self.base_m+self.speed_mps*dt)*(self.fallback_scale if fallback else 1.)


BODY_REGIONS = (
    (.30, .25, .70, .60),  # torso
    (.20, .10, .80, .40),  # head/shoulders
    (.20, .55, .80, .95),  # legs/lower body
    (.05, .25, .45, .70),  # left side/arm
    (.55, .25, .95, .70),  # right side/arm
)


class DepthTrackFilter:
    """Constant-velocity Kalman filter in the camera optical frame.

    Velocity is relative to the moving camera, not world velocity. Missing
    measurements permit only bounded prediction from the last real sample.
    """

    def __init__(self, measurement_std_m=.08, acceleration_std_mps2=2.,
                 jump_reset_m=.8, reset_gap_s=.5, gate=None):
        for name, value in locals().copy().items():
            if name not in ('self', 'gate') and (not math.isfinite(value) or value <= 0):
                raise ValueError(f'{name} must be finite and positive')
        if gate is not None and not isinstance(gate, DepthGate):
            raise ValueError('gate must be a DepthGate or None')
        self.measurement_variance = measurement_std_m**2
        self.acceleration_variance = acceleration_std_mps2**2
        # Without a gate the legacy behaviour is kept: re-initialize on large jumps.
        self.jump_reset_m = jump_reset_m
        self.reset_gap_s = reset_gap_s
        self.gate = gate
        self.reset()

    def reset(self):
        self.state = None
        self.covariance = None
        self.stamp = None  # Last real measurement, never refreshed by prediction.
        self.last_rejected = False
        self.pending = None  # (first stamp, latest stamp, latest point) of rejected evidence.
        self.confirm_required = False  # Stale state dropped while input continued.

    def _initialize(self, point, stamp):
        self.state = np.concatenate((point, np.zeros(3)))
        self.covariance = np.diag([self.measurement_variance]*3 + [1.]*3)
        self.stamp = float(stamp)
        self.pending = None
        self.confirm_required = False

    @property
    def evidence_stamp(self):
        """Latest real evidence, accepted or pending; None when there is none."""
        stamps = [s for s in (self.stamp, self.pending and self.pending[1]) if s is not None]
        return max(stamps) if stamps else None

    def _continues_pending(self, stamp, beyond_noise):
        # The gate widens with time since the last accepted sample. During an active
        # rejection streak a point beyond the noise allowance must be confirmed
        # (clean re-init) rather than blended with a large gain that invents velocity.
        return beyond_noise and self.pending is not None and stamp > self.pending[1]

    def _reject(self, point, stamp):
        """Hold the last accepted state; relocate only on consistent sustained evidence."""
        g = self.gate
        if (self.pending is not None and stamp > self.pending[1]
                and np.linalg.norm(point-self.pending[2]) <= g.limit(stamp-self.pending[1])):
            self.pending = (self.pending[0], float(stamp), point)
        else:
            self.pending = (float(stamp), float(stamp), point)
        if stamp-self.pending[0] >= g.confirm_s:
            self._initialize(point, stamp)
            return tuple(float(v) for v in self.state[:3])
        self.last_rejected = True
        return None

    def _predict(self, dt):
        transition = np.eye(6)
        transition[:3, 3:] = np.eye(3)*dt
        acceleration = np.vstack((np.eye(3)*(.5*dt*dt), np.eye(3)*dt))
        noise = acceleration @ acceleration.T * self.acceleration_variance
        return (transition @ self.state,
                transition @ self.covariance @ transition.T + noise)

    def update(self, xyz, stamp, fallback=False):
        """Return the filtered point, or None for invalid input or a gated rejection.

        `last_rejected` distinguishes a rejected real measurement (callers may hold
        the prediction with its true age) from absent/invalid input.
        """
        self.last_rejected = False
        if xyz is None or not math.isfinite(stamp):
            return None
        point = np.asarray(xyz, dtype=np.float64)
        if point.shape != (3,) or not np.isfinite(point).all():
            return None
        last_input = self.evidence_stamp
        if last_input is not None and (stamp <= last_input or stamp-last_input > self.reset_gap_s):
            # A real input gap or time reversal: forget state and pending evidence.
            # Measured from the latest input, accepted OR rejected, so a sustained
            # run of rejected samples is never mistaken for a stream gap.
            self.reset()
        elif self.gate is not None and self.stamp is not None and stamp-self.stamp > self.reset_gap_s:
            # Input continued but every sample was rejected: the accepted state is
            # too old to predict forward, and any new position must be confirmed.
            self.state = self.covariance = self.stamp = None
            self.confirm_required = True
        if self.stamp is None:
            if self.gate is not None and (fallback or self.confirm_required):
                # Uncorroborated fallback or a stale rejected run: confirm like a relocation.
                return self._reject(point, stamp)
            self._initialize(point, stamp)
        else:
            dt = stamp-self.stamp
            predicted, covariance = self._predict(dt)
            innovation = point-predicted[:3]
            distance = np.linalg.norm(innovation)
            if self.gate is not None and (
                    distance > self.gate.limit(dt, fallback) or self._continues_pending(
                        stamp, distance > self.gate.limit(0., fallback))):
                return self._reject(point, stamp)
            if self.gate is None and distance > self.jump_reset_m:
                self._initialize(point, stamp)
            else:
                self.pending = None
                scale = self.gate.fallback_noise_scale if self.gate is not None and fallback else 1.
                noise = np.eye(3)*self.measurement_variance*scale**2
                gain = np.linalg.solve(covariance[:3, :3]+noise,
                                       covariance[:, :3].T).T
                self.state = predicted + gain @ innovation
                # Joseph form preserves positive semidefiniteness under rounding.
                residual = np.eye(6)
                residual[:, :3] -= gain
                self.covariance = residual @ covariance @ residual.T + gain @ noise @ gain.T
                self.covariance = (self.covariance+self.covariance.T)*.5
                self.stamp = float(stamp)
        return tuple(float(v) for v in self.state[:3])

    def hold(self, stamp, max_age):
        """Predict without changing the last measurement or its age."""
        if (self.stamp is None or not math.isfinite(stamp)
                or not math.isfinite(max_age) or max_age < 0):
            return None
        age = float(stamp)-self.stamp
        if age < 0 or age > min(max_age, self.reset_gap_s):
            return None
        state, _ = self._predict(age)
        return tuple(float(v) for v in state[:3]), age


def _measure_region(depth, bounds, intrinsics, min_fraction, repair_small_holes=True):
    left, top, right, bottom = bounds
    roi = depth[top:bottom, left:right]
    if roi.size == 0:
        return None
    valid = np.isfinite(roi) & (roi >= .2) & (roi <= 8.)
    # Repair only small holes surrounded by real depth samples. Large missing
    # regions remain invalid instead of being filled with a stale/background value.
    if repair_small_holes and valid.sum() >= max(12, roi.size * .03) and not valid.all():
        padded = np.pad(np.where(valid, roi, np.nan), 1, constant_values=np.nan)
        neighbours = np.stack([
            padded[:-2, :-2], padded[:-2, 1:-1], padded[:-2, 2:],
            padded[1:-1, :-2], padded[1:-1, 1:-1], padded[1:-1, 2:],
            padded[2:, :-2], padded[2:, 1:-1], padded[2:, 2:]], axis=0)
        count = np.isfinite(neighbours).sum(axis=0)
        fill = (~valid) & (count >= 5)
        if fill.any():
            filled = np.full(roi.shape, np.nan, dtype=np.float32)
            has_neighbour = count > 0
            filled[has_neighbour] = np.nanmedian(neighbours[:, has_neighbour], axis=0)
            roi = roi.copy()
            roi[fill] = filled[fill]
            valid[fill] = np.isfinite(filled[fill])
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


def region_candidates(depth, box, intrinsics, min_fraction=.15, repair_small_holes=True):
    """Collect B's five candidates; C can explicitly require unfilled samples."""
    x1, y1, x2, y2 = map(float, box)
    if not all(map(math.isfinite, (x1, y1, x2, y2))) or x2 <= x1 or y2 <= y1:
        return []
    h, w = depth.shape
    bw, bh = x2-x1, y2-y1
    fx, fy, cx, cy = intrinsics
    if not all(map(math.isfinite, intrinsics)) or fx <= 0 or fy <= 0:
        return []
    # Keep every candidate strictly inside the same YOLO detection. The nearest
    # stable patch is preferred: a background wall or floor can have more
    # valid pixels than a person's back, but should never win the association.
    results = []
    for ax1, ay1, ax2, ay2 in BODY_REGIONS:
        left = max(0, int(x1 + ax1*bw)); right = min(w, int(x1 + ax2*bw))
        top = max(0, int(y1 + ay1*bh)); bottom = min(h, int(y1 + ay2*bh))
        result = _measure_region(depth, (left, top, right, bottom), intrinsics,
                                 min_fraction, repair_small_holes)
        if result is not None:
            results.append(result)
    return results


def measure(depth, box, intrinsics, min_fraction=0.15):
    """B's unchanged region selection and representative-point calculation."""
    results = region_candidates(depth, box, intrinsics, min_fraction)
    return select_region_candidates(results)


def select_region_candidates(results):
    """B's nearest stable depth cluster; shared without resampling the image."""
    if not results:
        return None
    # Prefer a depth cluster supported by several body parts. This rejects a
    # single floor/wall patch even when it has more valid pixels than the person.
    clusters = []
    for result in results:
        z = result[0][2]
        cluster = [item for item in results
                   if abs(item[0][2] - z) <= max(.15, .12*z)]
        clusters.append(cluster)
    cluster = min(clusters, key=lambda group: (np.median([item[0][2] for item in group]),
                                                -len(group),
                                                -sum(item[1] for item in group)))
    points = np.asarray([item[0] for item in cluster], dtype=np.float32)
    return tuple(float(v) for v in np.median(points, axis=0))
