"""Odometry evidence for fixed-image continuation; no pose extrapolation."""
import math

class StationaryGate:
    def __init__(self, hold_s=.5, timeout_s=.3):
        self.hold_s, self.timeout_s = hold_s, timeout_s
        self.since = self.since_stamp = None
        self.stamp = self.received = -math.inf

    def observe(self, stamp, linear, angular, now, ros_now):
        was_fresh = self.fresh(now, ros_now)
        valid = (all(math.isfinite(v) for v in (stamp, linear, angular))
                 and stamp > 0 and stamp > self.stamp and 0 <= ros_now-stamp <= self.timeout_s)
        if not valid or abs(linear) >= .02 or abs(angular) >= .05:
            self.since = self.since_stamp = None
            self.received = -math.inf
            if valid: self.stamp = stamp
            return False
        if not was_fresh or self.since is None:
            self.since, self.since_stamp = now, stamp
        self.stamp, self.received = stamp, now
        return True

    def fresh(self, now, ros_now):
        return 0 <= now-self.received <= self.timeout_s and 0 <= ros_now-self.stamp <= self.timeout_s

    def ready(self, now, ros_now, frame_stamp):
        return (self.since is not None and self.fresh(now, ros_now)
                and now-self.since >= self.hold_s and frame_stamp >= self.since_stamp
                and 0 <= ros_now-frame_stamp <= self.timeout_s)
