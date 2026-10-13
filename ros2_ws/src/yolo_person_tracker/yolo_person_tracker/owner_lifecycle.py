"""Owner lifecycle driven by monotonic time; no motion authority or ROS dependencies."""
import math


class OwnerLifecycle:
    def __init__(self, timeout_s=10.):
        if not math.isfinite(timeout_s) or timeout_s <= 0:
            raise ValueError('owner_lost_timeout_s must be finite and positive')
        self.timeout_s = timeout_s
        self.state, self.event = 'UNCLAIMED', 'ready'
        self.sequence = 0
        self.lost_at = None

    def change(self, state, event):
        self.state, self.event = state, event
        self.sequence += 1

    def claim(self):
        self.lost_at = None
        self.change('TRACKING', 'claimed')

    def release(self, expired=False):
        self.lost_at = None
        self.change('UNCLAIMED', 'expired' if expired else 'released')

    def update(self, visible, now):
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
        elif self.lost_at is None:
            self.lost_at = now
            self.change('LOST', 'lost')
        return False

    def lost_for(self, now):
        return 0. if self.lost_at is None else max(0., now-self.lost_at)
