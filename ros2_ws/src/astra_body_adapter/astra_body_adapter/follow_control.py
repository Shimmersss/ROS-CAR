from dataclasses import dataclass
import math


@dataclass(frozen=True)
class FollowConfig:
    target_distance_m: float = 0.3
    distance_deadband_m: float = 0.0
    bearing_deadband_rad: float = 0.08
    turn_in_place_rad: float = 0.6
    max_linear_mps: float = 0.15
    max_angular_rps: float = 0.5
    distance_gain: float = 0.6
    bearing_gain: float = 1.2

    def __post_init__(self):
        values = (
            self.target_distance_m,
            self.distance_deadband_m,
            self.bearing_deadband_rad,
            self.turn_in_place_rad,
            self.max_linear_mps,
            self.max_angular_rps,
            self.distance_gain,
            self.bearing_gain,
        )
        if not all(map(math.isfinite, values)):
            raise ValueError('follow parameters must be finite')
        if self.target_distance_m < 0.0:
            raise ValueError('target distance must not be negative')
        if self.distance_deadband_m < 0.0 or self.bearing_deadband_rad < 0.0:
            raise ValueError('deadbands must not be negative')
        if self.turn_in_place_rad < self.bearing_deadband_rad:
            raise ValueError('turn threshold must cover the bearing deadband')
        if self.max_linear_mps <= 0.0 or self.max_angular_rps <= 0.0:
            raise ValueError('speed limits must be positive')
        if self.distance_gain <= 0.0 or self.bearing_gain <= 0.0:
            raise ValueError('control gains must be positive')


def _clamp(value, limit):
    return max(-limit, min(limit, value))


def compute_command(distance_m, bearing_rad, tracking, config=FollowConfig()):
    if (not tracking
            or not all(map(math.isfinite, (distance_m, bearing_rad)))
            or distance_m <= 0.0):
        return 0.0, 0.0

    angular = 0.0
    if abs(bearing_rad) > config.bearing_deadband_rad:
        # Optical X points right; positive ROS yaw turns left.
        angular = _clamp(
            -config.bearing_gain * bearing_rad,
            config.max_angular_rps,
        )

    distance_error = distance_m - config.target_distance_m
    linear = 0.0
    if (distance_error > config.distance_deadband_m
            and abs(bearing_rad) <= config.turn_in_place_rad):
        linear = min(
            config.distance_gain * distance_error,
            config.max_linear_mps,
        )

    return linear, angular


def target_is_usable(
        enabled, status, tracking_status, position_valid, age_s, timeout_s):
    return (
        enabled
        and status == tracking_status
        and position_valid
        and math.isfinite(age_s)
        and 0.0 <= age_s <= timeout_s
    )


def observation_is_fresh(now_s, published_s, observed_s, measurement_age_s,
                         timeout_s, source, expected_source):
    """Fresh publication cannot hide an old RGB-D observation.

    Legacy Astra has no sensor stamp; only its explicitly selected source may
    use publication age. Timestamped sources require both observation and age.
    """
    if source != expected_source or not math.isfinite(now_s):
        return False
    publication_age = now_s-published_s
    if published_s <= 0 or not 0 <= publication_age <= timeout_s:
        return False
    if source == 'astra' and observed_s == 0 and math.isnan(measurement_age_s):
        return True
    return (observed_s > 0 and 0 <= now_s-observed_s <= timeout_s
            and math.isfinite(measurement_age_s)
            and 0 <= measurement_age_s+publication_age <= timeout_s)
