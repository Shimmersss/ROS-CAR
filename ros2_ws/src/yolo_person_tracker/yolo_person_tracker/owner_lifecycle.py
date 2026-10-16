"""Owner lifecycle driven by monotonic time; no motion authority or ROS dependencies."""
import math


class OwnerLifecycle:
    def __init__(self, timeout_s=10., continuity_reset_s=0.):
        if not math.isfinite(timeout_s) or timeout_s <= 0:
            raise ValueError('owner_lost_timeout_s must be finite and positive')
        if not math.isfinite(continuity_reset_s) or not 0 <= continuity_reset_s < timeout_s:
            raise ValueError('continuity reset must be nonnegative and below owner timeout')
        self.timeout_s = timeout_s
        self.continuity_reset_s = continuity_reset_s
        self.continuity_reset_done = False
        self.state, self.event = 'UNCLAIMED', 'ready'
        self.sequence = 0
        self.lost_at = None
        self.held_at = None

    def change(self, state, event):
        self.state, self.event = state, event
        self.sequence += 1

    def claim(self):
        self.lost_at = None
        self.held_at = None
        self.continuity_reset_done = False
        self.change('TRACKING', 'claimed')

    def release(self, expired=False):
        self.lost_at = None
        self.held_at = None
        self.continuity_reset_done = False
        self.change('UNCLAIMED', 'expired' if expired else 'released')

    def hold(self, active, now):
        """Freeze loss/continuity clocks without changing selection or granting movement."""
        if self.held_at is not None and self.lost_at is not None:
            self.lost_at += max(0., now-self.held_at)
        self.held_at = now if active and self.state != 'UNCLAIMED' else None

    def update(self, visible, now):
        self.hold(self.held_at is not None, now)
        if self.state == 'UNCLAIMED':
            return False
        # Expiry wins at exactly the deadline, even if a frame arrives then.
        if self.lost_at is not None and now-self.lost_at >= self.timeout_s:
            self.release(expired=True)
            return True
        if visible:
            if self.state == 'LOST':
                self.change('TRACKING', 'returned')
            self.lost_at = None
            self.continuity_reset_done = False
        elif self.lost_at is None:
            self.lost_at = now
            self.change('LOST', 'lost')
        return False

    def reset_continuity_due(self, now):
        self.hold(self.held_at is not None, now)
        if (self.continuity_reset_s > 0 and self.lost_at is not None
                and not self.continuity_reset_done
                and now-self.lost_at >= self.continuity_reset_s):
            self.continuity_reset_done = True
            return True
        return False

    def lost_for(self, now):
        return 0. if self.lost_at is None else max(0., (self.held_at if self.held_at is not None else now)-self.lost_at)
