"""Explicit selection with optional stable single-person acquisition."""
import time


class Selection:
    def __init__(self):
        self.target_id = None
        self.epoch = 0
        self.candidates = []
        self.received_at = -float('inf')
        self.width = 0
        self.detection_count = 0
        self.auto_candidate = None
        self.auto_count = 0

    def update(self, candidates, width, received_at):
        self.detection_count = len(candidates)
        self.candidates = [d for d in candidates if d.track_id is not None]
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
        self.auto_candidate, self.auto_count = None, 0

    def auto_lock_single(self, now=None, max_age=.5, confirm_frames=3):
        """Opt-in display mode: acquire one stable track, never choose among people."""
        now = time.monotonic() if now is None else now
        if self.target_id is not None and self.target_id[0] != self.epoch:
            self.target_id = None
        if (self.selected() is not None or now-self.received_at > max_age
                or self.detection_count != 1 or len(self.candidates) != 1):
            self.auto_candidate, self.auto_count = None, 0
            return False
        candidate = (self.epoch, self.candidates[0].track_id)
        self.auto_count = self.auto_count+1 if candidate == self.auto_candidate else 1
        self.auto_candidate = candidate
        if self.auto_count < confirm_frames:
            return False
        self.target_id = candidate
        self.auto_candidate, self.auto_count = None, 0
        return True

    def reset_stream(self):
        self.epoch += 1
        self.candidates = []
        self.received_at = -float('inf')
        self.auto_candidate, self.auto_count = None, 0
        # Retain the old epoch in the selected identity: ID reuse cannot reacquire it.

    def selected(self):
        if self.target_id is None or self.target_id[0] != self.epoch:
            return None
        return next((d for d in self.candidates if d.track_id == self.target_id[1]), None)
