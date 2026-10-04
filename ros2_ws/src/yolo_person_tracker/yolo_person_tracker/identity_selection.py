"""Opt-in C identity lock. Appearance evidence never supplies a position."""
from collections import OrderedDict
import math
import time

from .state import Selection


class IdentitySelection(Selection):
    def __init__(self, confirm_frames=3, max_age_s=.5):
        super().__init__(hold_s=0., reacquire_s=0.)
        if (not isinstance(confirm_frames, int) or confirm_frames < 2
                or not math.isfinite(max_age_s) or max_age_s <= 0):
            raise ValueError('Invalid identity lock confirmation/age settings')
        self.confirm_frames = confirm_frames
        self.max_age_s = max_age_s
        self.frames = OrderedDict()
        self.latest_stamp = 0
        self.last_evidence = 0
        self.floor_stamp = 0
        self.person_id = ''
        self.pending = None
        self.count = 0
        self.authorized_at = -math.inf

    def invalidate(self):
        self.pending, self.count = None, 0
        self.authorized_at = -math.inf

    def update(self, candidates, width, received_at):
        # No nearest-box reassignment and no retained detection on disappearance.
        self.detection_count = len(candidates)
        self.candidates = [d for d in candidates if d.track_id is not None]
        self.width, self.received_at = width, received_at
        expected = self.pending[0] if self.pending else ':'.join(map(str, self.target_id or ()))
        if not any(f'{self.epoch}:{d.track_id}' == expected for d in self.candidates):
            self.invalidate()

    def record_frame(self, stamp_ns, frame):
        if stamp_ns <= self.latest_stamp:
            self.frames.clear()
            self.last_evidence = 0
            self.invalidate()
        self.latest_stamp = stamp_ns
        tracks = {f'{self.epoch}:{d.track_id}' for d in self.candidates}
        self.frames[stamp_ns] = (frame, tracks)
        while len(self.frames) > 64:
            self.frames.popitem(last=False)

    def lock(self, now=None, max_age=.5):
        ok, detail = super().lock(now, max_age)
        if ok:
            self.person_id = ''
            self.floor_stamp = self.latest_stamp
            self.invalidate()
            detail += '; waiting for confirmed appearance identity'
        return ok, detail

    def release(self):
        super().release()
        self.person_id = ''
        self.floor_stamp = self.latest_stamp
        self.invalidate()

    def auto_lock_single(self, now=None, max_age=.5, confirm_frames=3):
        # Once acquired, never silently choose a different person or clear an old epoch.
        if self.target_id is not None:
            return False
        ok = super().auto_lock_single(now, max_age, confirm_frames)
        if ok:
            self.floor_stamp = self.latest_stamp
            self.invalidate()
        return ok

    def reset_stream(self):
        super().reset_stream()
        self.frames.clear()
        self.latest_stamp = self.last_evidence = self.floor_stamp = 0
        self.invalidate()

    def observe(self, stamp_ns, frame, rows, now_ns, valid=True, now=None):
        """rows: (track_id, person_id, verified, visible). Exact known RGB header required."""
        now = time.monotonic() if now is None else now
        age = (now_ns-stamp_ns)*1e-9
        if not valid or not 0 <= age <= self.max_age_s:
            self.invalidate()
            return
        # Ignore replayed/pre-lock results without granting or refreshing authority.
        if stamp_ns <= max(self.floor_stamp, self.last_evidence):
            return
        self.last_evidence = stamp_ns
        recorded = self.frames.get(stamp_ns)
        if recorded is None or recorded[0] != frame or self.target_id is None:
            self.invalidate()
            return
        visible = [r for r in rows if r[3]]
        verified = [r for r in visible if r[2] and r[1]]
        if (len({r[0] for r in visible}) != len(visible)
                or any(r[0] not in recorded[1] for r in visible)
                or len({r[1] for r in verified}) != len(verified)):
            self.invalidate()
            return
        target = ':'.join(map(str, self.target_id))
        matches = [r for r in verified if
                   (r[1] == self.person_id if self.person_id else r[0] == target)]
        current = {f'{self.epoch}:{d.track_id}': d for d in self.candidates}
        if len(matches) != 1 or matches[0][0] not in current:
            self.invalidate()
            return
        track, person = matches[0][:2]
        candidate = (track, person)
        if now-self.authorized_at > self.max_age_s:
            self.invalidate()
        self.count = min(self.count+1, self.confirm_frames) if candidate == self.pending else 1
        self.pending = candidate
        # Retain source age: delayed inference must not receive a fresh full TTL.
        self.authorized_at = now-age
        if self.count >= self.confirm_frames:
            self.person_id = person
            self.target_id = (self.epoch, current[track].track_id)

    def selected(self, now=None):
        now = time.monotonic() if now is None else now
        if (self.count < self.confirm_frames or not self.person_id
                or not 0 <= now-self.authorized_at <= self.max_age_s
                or self.pending != (':'.join(map(str, self.target_id)), self.person_id)):
            return None
        return next((d for d in self.candidates
                     if (self.epoch, d.track_id) == self.target_id), None)
