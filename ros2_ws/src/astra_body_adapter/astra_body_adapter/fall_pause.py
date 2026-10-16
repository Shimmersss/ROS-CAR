"""Fresh fall evidence latch; no ROS dependencies or motion authority."""
import math

class FallPause:
    def __init__(self, resume_s=3., timeout_s=.5):
        if not all(math.isfinite(v) and v > 0 for v in (resume_s, timeout_s)):
            raise ValueError('fall timing must be finite and positive')
        self.resume_s, self.timeout_s = resume_s, timeout_s
        self.paused = False
        self.clear_since = None
        self.stamp = self.received = -math.inf
        self.valid = False
        self.stage = None

    def fresh(self, now, ros_now):
        return self.valid and 0 <= now-self.received <= self.timeout_s and 0 <= ros_now-self.stamp <= self.timeout_s

    def observe(self, valid, stage, stamp, now, ros_now):
        previous_fresh = self.fresh(now, ros_now)
        if (not valid or stage not in (0, 1, 2) or not math.isfinite(stamp)
                or stamp <= 0 or stamp <= self.stamp or not 0 <= ros_now-stamp <= self.timeout_s):
            self.valid = False
            self.clear_since = None
            return
        self.stamp, self.received, self.valid, self.stage = stamp, now, True, stage
        if stage:
            self.paused = True
            self.clear_since = None
        elif not previous_fresh or self.clear_since is None:
            self.clear_since = now

    def update(self, now, ros_now, can_resume=True):
        fresh = self.fresh(now, ros_now)
        if not fresh:
            self.clear_since = None
        elif self.paused and self.stage == 0 and can_resume and self.clear_since is not None and now-self.clear_since >= self.resume_s:
            self.paused = False
        return self.paused

    def resume(self, now, ros_now):
        if not self.fresh(now, ros_now) or self.stage != 0:
            return False
        self.paused = False
        return True
