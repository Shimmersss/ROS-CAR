"""Explicit selection; a lost target is never replaced by another detection."""
import time


class Selection:
    def __init__(self):
        self.target_id = None
        self.epoch = 0
        self.candidates = []
        self.received_at = -float('inf')
        self.width = 0

    def update(self, candidates, width, received_at):
        self.candidates = candidates
        self.width = width
        self.received_at = received_at

    def lock(self, now=None, max_age=.5):
        now = time.monotonic() if now is None else now
        if now-self.received_at > max_age or not self.candidates:
            return False, 'No fresh tracked candidates'
        candidate = min(self.candidates,
                        key=lambda d: (abs((d.box[0]+d.box[2])/2-self.width/2), d.track_id))
        self.target_id = (self.epoch, candidate.track_id)
        return True, f'Locked {self.epoch}:{candidate.track_id}'

    def release(self):
        self.target_id = None

    def reset_stream(self):
        self.epoch += 1
        self.candidates = []
        self.received_at = -float('inf')
        # Retain the old epoch in the selected identity: ID reuse cannot reacquire it.

    def selected(self):
        if self.target_id is None or self.target_id[0] != self.epoch:
            return None
        return next((d for d in self.candidates if d.track_id == self.target_id[1]), None)
