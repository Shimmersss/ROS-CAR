"""Explicit selection with optional stable single-person acquisition."""
import time


class Selection:
    def __init__(self, hold_s=.35, reacquire_s=.8, reacquire_center_fraction=.25):
        if hold_s < 0:
            raise ValueError('hold_s must be non-negative')
        if reacquire_s < 0 or reacquire_center_fraction <= 0:
            raise ValueError('invalid reacquire settings')
        self.hold_s = float(hold_s)
        self.reacquire_s = float(reacquire_s)
        self.reacquire_center_fraction = float(reacquire_center_fraction)
        self.target_id = None
        self.epoch = 0
        self.candidates = []
        self.received_at = -float('inf')
        self.width = 0
        self.detection_count = 0
        self.auto_candidate = None
        self.auto_count = 0
        self.last_selected = None
        self.last_selected_at = -float('inf')
        self.last_box = None

    def update(self, candidates, width, received_at):
        self.detection_count = len(candidates)
        self.candidates = [d for d in candidates if d.track_id is not None]
        self.width = width
        self.received_at = received_at
        if self.target_id is not None:
            current = next((d for d in self.candidates
                            if (self.epoch, d.track_id) == self.target_id), None)
            if current is not None:
                self.last_selected = current
                self.last_selected_at = received_at
                self.last_box = tuple(current.box)
            elif (self.last_box is not None
                  and received_at - self.last_selected_at <= self.reacquire_s):
                replacement = self._nearest_replacement()
                if replacement is not None:
                    self.target_id = (self.epoch, replacement.track_id)
                    self.last_selected = replacement
                    self.last_selected_at = received_at
                    self.last_box = tuple(replacement.box)

    def _nearest_replacement(self):
        if not self.candidates or self.last_box is None:
            return None
        x1, y1, x2, y2 = self.last_box
        old_cx, old_cy = (x1+x2)/2, (y1+y2)/2
        old_area = max(1., (x2-x1)*(y2-y1))
        max_dx = max(30., self.width*self.reacquire_center_fraction)
        matches = []
        for candidate in self.candidates:
            bx1, by1, bx2, by2 = candidate.box
            cx, cy = (bx1+bx2)/2, (by1+by2)/2
            area = max(1., (bx2-bx1)*(by2-by1))
            ratio = area/old_area
            distance = ((cx-old_cx)**2 + (cy-old_cy)**2)**.5
            if distance <= max_dx and .35 <= ratio <= 2.8:
                matches.append((distance, abs(ratio-1.), candidate.track_id, candidate))
        return min(matches)[3] if matches else None

    def lock(self, now=None, max_age=.5):
        now = time.monotonic() if now is None else now
        if now-self.received_at > max_age or not self.candidates:
            return False, 'No fresh tracked candidates'
        candidate = min(self.candidates,
                        key=lambda d: (abs((d.box[0]+d.box[2])/2-self.width/2), d.track_id))
        self.target_id = (self.epoch, candidate.track_id)
        self.last_selected = candidate
        self.last_selected_at = self.received_at
        self.last_box = tuple(candidate.box)
        return True, f'Locked {self.epoch}:{candidate.track_id}'

    def release(self):
        self.target_id = None
        self.auto_candidate, self.auto_count = None, 0
        self.last_selected = None
        self.last_selected_at = -float('inf')
        self.last_box = None

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
        self.last_selected = self.candidates[0]
        self.last_selected_at = now
        self.last_box = tuple(self.candidates[0].box)
        self.auto_candidate, self.auto_count = None, 0
        return True

    def reset_stream(self):
        self.epoch += 1
        self.candidates = []
        self.received_at = -float('inf')
        self.auto_candidate, self.auto_count = None, 0
        self.last_selected = None
        self.last_selected_at = -float('inf')
        self.last_box = None
        # Retain the old epoch in the selected identity: ID reuse cannot reacquire it.

    def selected(self, now=None):
        if self.target_id is None or self.target_id[0] != self.epoch:
            return None
        current = next((d for d in self.candidates if d.track_id == self.target_id[1]), None)
        if current is not None:
            self.last_selected = current
            if now is not None:
                self.last_selected_at = now
            self.last_box = tuple(current.box)
            return current
        now = time.monotonic() if now is None else now
        if now - self.last_selected_at <= self.hold_s:
            return self.last_selected
        return None
