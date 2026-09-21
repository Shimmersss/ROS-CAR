"""Pure conservative swept-envelope check. No motor commands or ROS imports."""
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class SafetyConfig:
    length_m: float = .5
    width_m: float = .4
    margin_m: float = .15
    max_linear_mps: float = .15
    max_angular_rps: float = .5
    reaction_s: float = 1.0
    deceleration_mps2: float = .3
    scan_timeout_s: float = .3
    request_timeout_s: float = .2
    target_timeout_s: float = .5
    sampling_margin_m: float = .02
    allow_infinite_clear: bool = False

    def __post_init__(self):
        for key, value in vars(self).items():
            if key == 'allow_infinite_clear':
                continue
            if not math.isfinite(value) or value <= 0:
                raise ValueError(key+' must be finite and positive')
        if self.reaction_s < self.scan_timeout_s + .55:
            raise ValueError('reaction_s must cover scan age + 0.5s driver timeout + 0.05s gate period')
        if self.max_linear_mps > 1 or self.max_angular_rps > 2:
            raise ValueError('Configured velocity exceeds project gate limits')
        if self.sampling_margin_m > .05:
            raise ValueError('sampling_margin_m must be <= .05')


def clearance(scan, transform, v, w, c):
    """Cover the chassis and any stopping path by a conservative expanded disk.

    Reject incomplete coverage: unknown/no-return bins cannot establish clearance
    unless the operator explicitly opts into the sensor's +inf free-space contract.
    transform = planar laser->base (x,y,yaw), from timestamped TF.
    """
    if not all(map(math.isfinite,(v,w))) or not 0 <= v <= c.max_linear_mps or abs(w)>c.max_angular_rps:
        return False, 'invalid_velocity_request'
    fields = (scan.angle_min, scan.angle_max, scan.angle_increment, scan.range_min, scan.range_max)
    if (not all(math.isfinite(x) for x in fields) or scan.angle_increment <= 0
            or not 0 <= scan.range_min < scan.range_max or len(scan.ranges) < 16):
        return False, 'malformed_scan'
    span = (len(scan.ranges)-1)*scan.angle_increment
    if abs(scan.angle_max-scan.angle_min-span) > 2*scan.angle_increment or span < 2*math.pi-.1:
        return False, 'incomplete_scan_coverage'
    if scan.angle_increment > math.radians(2):
        return False, 'scan_resolution_too_low'
    x, y, yaw = transform
    if not all(math.isfinite(a) for a in transform):
        return False, 'invalid_mount'
    # A full angular scan still cannot see inside range_min. That blind disk
    # must lie inside the confirmed rectangular body, not in traversable space.
    if abs(x)+scan.range_min>c.length_m/2 or abs(y)+scan.range_min>c.width_m/2:
        return False, 'sensor_blind_zone_outside_body'
    radius = math.hypot(c.length_m, c.width_m)/2 + c.margin_m
    # Always cover stopping distance for the configured maximum speed. This also
    # protects a zero request while the vehicle is still decelerating.
    travel = c.max_linear_mps*c.reaction_s + c.max_linear_mps**2/(2*c.deceleration_mps2)
    # A disk enlarged by the maximum stopping path covers *any* steering
    # direction, including a command change while the old motion decelerates.
    radius += travel + c.sampling_margin_m + travel*scan.angle_increment
    required_range = math.hypot(x,y) + radius
    if scan.range_max < required_range:
        return False, 'insufficient_sensor_range'
    for i, value in enumerate(scan.ranges):
        if math.isinf(value) and value > 0 and c.allow_infinite_clear:
            continue
        if not math.isfinite(value) or not scan.range_min <= value <= scan.range_max:
            return False, 'unknown_scan_return'
        angle = scan.angle_min+i*scan.angle_increment+yaw
        px, py = x+value*math.cos(angle), y+value*math.sin(angle)
        if math.hypot(px,py) <= radius:
            return False, 'obstacle_in_stopping_envelope'
    return True, 'clear'
