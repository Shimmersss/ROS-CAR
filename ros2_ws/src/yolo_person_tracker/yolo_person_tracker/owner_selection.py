"""Bridge-primary owner following (opt-in, C route).

Continuity comes from ByteTrack plus the single-object bridge arbiter, so following does not
pause whenever ReID merely lacks evidence. ReID has two narrow jobs only:
  * veto: repeated verified evidence that the followed track is someone else (or that the owner
    is on another track) drops the target;
  * re-acquire: after a loss, repeated verified evidence of the owner's persistent identity on a
    current track resumes following there, without a manual relock.
The owner identity is bound from ReID results on the user-locked track. Appearance evidence never
supplies a position.
"""
from collections import OrderedDict
import math
import time

from .state import Selection


class OwnerSelection(Selection):
    def __init__(self, bind_frames=3, veto_frames=2, reacquire_frames=2, max_age_s=.5, lose_after_s=1.):
        super().__init__(hold_s=0., reacquire_s=0., nearest_reacquire=False)
        for name, value in (('bind_frames', bind_frames), ('veto_frames', veto_frames),
                            ('reacquire_frames', reacquire_frames)):
            if not isinstance(value, int) or value < 1:
                raise ValueError(f'{name} must be a positive integer')
        if not all(math.isfinite(v) and v > 0 for v in (max_age_s, lose_after_s)):
            raise ValueError('max_age_s and lose_after_s must be finite and positive')
        self.bind_frames, self.veto_frames, self.reacquire_frames = bind_frames, veto_frames, reacquire_frames
        self.max_age_s, self.lose_after_s = max_age_s, lose_after_s
        self.frames = OrderedDict()
        self.latest_stamp = 0
        self.last_evidence = 0
        self.owner_id = ''
        self.searching = False
        self.detail = ''
        self._clear_counts()
        self.target_seen_at = -math.inf

    def _clear_counts(self):
        self.binding = ('', 0)
        self.veto = 0
        self.found = ('', 0)

    # --- user actions -------------------------------------------------------------------------
    def lock(self, now=None, max_age=.5):
        ok, detail = super().lock(now, max_age)
        if ok:
            self.owner_id, self.searching, self.detail = '', False, ''
            self._clear_counts()
            self.target_seen_at = self.received_at
            detail += '; following; owner identity binds from ReID'
        return ok, detail

    def auto_lock_single(self, now=None, max_age=.5, confirm_frames=3):
        if self.target_id is not None or self.searching:
            return False  # never auto-pick someone while looking for the owner
        ok = super().auto_lock_single(now, max_age, confirm_frames)
        if ok:
            self.owner_id, self.detail = '', ''
            self._clear_counts()
            self.target_seen_at = self.received_at
        return ok

    def release(self):
        super().release()
        self.owner_id, self.searching, self.detail = '', False, ''
        self._clear_counts()

    def reset_stream(self):
        super().reset_stream()
        self.frames.clear()
        self.latest_stamp = self.last_evidence = 0
        self._clear_counts()
        if self.target_id is not None and self.owner_id:
            self.lose('stream reset')

    def lose(self, reason):
        """Drop the followed track; with a bound owner, keep searching for it via ReID."""
        self.target_id = None
        self.last_selected, self.last_box = None, None
        self.last_selected_at = -math.inf
        self.searching = bool(self.owner_id)
        self.detail = reason + ('; searching for owner via ReID' if self.searching
                                else '; owner not yet identified, relock required')
        self._clear_counts()

    # --- per frame ----------------------------------------------------------------------------
    def update(self, candidates, width, received_at):
        super().update(candidates, width, received_at)
        if self.target_id is not None and any((self.epoch, d.track_id) == self.target_id for d in self.candidates):
            self.target_seen_at = received_at

    def check_absence(self, now, bridging):
        """Called each processed frame: an undetected, unbridged target is lost after lose_after_s."""
        if self.target_id is not None and not bridging and now-self.target_seen_at > self.lose_after_s:
            self.lose('target not seen')

    def note_bridging(self, now):
        self.target_seen_at = now

    def record_frame(self, stamp_ns, frame):
        if stamp_ns <= self.latest_stamp:
            self.frames.clear()
            self.last_evidence = 0
        self.latest_stamp = stamp_ns
        self.frames[stamp_ns] = (frame, {f'{self.epoch}:{d.track_id}' for d in self.candidates})
        while len(self.frames) > 64:
            self.frames.popitem(last=False)

    def observe(self, stamp_ns, frame, rows, now_ns, valid=True, now=None):
        """rows: (track_id, person_id, verified, visible) for an exactly recorded RGB frame."""
        age = (now_ns-stamp_ns)*1e-9
        if not valid or not 0 <= age <= self.max_age_s or stamp_ns <= self.last_evidence:
            return
        recorded = self.frames.get(stamp_ns)
        if recorded is None or recorded[0] != frame:
            return
        self.last_evidence = stamp_ns
        verified = [r for r in rows if r[3] and r[2] and r[1] and r[0] in recorded[1]]
        current = {f'{self.epoch}:{d.track_id}': d for d in self.candidates}
        if self.target_id is not None:
            target = ':'.join(map(str, self.target_id))
            on_target = next((r[1] for r in verified if r[0] == target), '')
            if not self.owner_id:
                if on_target:
                    person, count = self.binding
                    self.binding = (on_target, count+1 if person == on_target else 1)
                    if self.binding[1] >= self.bind_frames:
                        self.owner_id = on_target
                return
            elsewhere = any(r[1] == self.owner_id and r[0] != target for r in verified)
            if (on_target and on_target != self.owner_id) or elsewhere:
                self.veto += 1
                if self.veto >= self.veto_frames:
                    self.lose('ReID identifies the followed track as someone else')
                    self.observe_search(verified, current)
            elif on_target == self.owner_id:
                self.veto = 0
            return
        if self.searching:
            self.observe_search(verified, current)

    def observe_search(self, verified, current):
        hits = [r[0] for r in verified if r[1] == self.owner_id and r[0] in current]
        if len(hits) != 1:
            self.found = ('', 0)
            return
        track, count = self.found
        self.found = (hits[0], count+1 if track == hits[0] else 1)
        if self.found[1] >= self.reacquire_frames:
            d = current[hits[0]]
            self.target_id = (self.epoch, d.track_id)
            self.last_selected, self.last_box = d, tuple(d.box)
            self.last_selected_at = self.target_seen_at = self.received_at
            self.searching, self.detail = False, 'owner re-acquired by ReID'
            self._clear_counts()
