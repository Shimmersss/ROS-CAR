"""A bounded follow-up window; wake always refreshes, never toggles motion."""
import math


class VoiceSession:
    def __init__(self, window_s=8.0):
        if not isinstance(window_s, (int, float)) or not math.isfinite(window_s) or not 5 <= window_s <= 10:
            raise ValueError('follow-up window must be 5..10 seconds')
        self.window_s = window_s
        self.deadline = -math.inf

    def wake(self, now):
        self.deadline = now + self.window_s

    def active(self, now):
        return now < self.deadline

    def success(self, now):
        self.wake(now)

    def close(self):
        self.deadline = -math.inf
