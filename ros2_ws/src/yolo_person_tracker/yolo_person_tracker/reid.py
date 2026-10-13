"""Conservative session-local identity association, independent of ByteTrack/selection.

No coordinates or biometric certainty are inferred here. Inputs are current,
quality-approved appearance descriptors; ambiguous frames never update galleries.
"""
from collections import deque
from dataclasses import dataclass
import math
import uuid
import numpy as np


@dataclass(frozen=True)
class IdentityConfig:
    confirm_frames: int = 3
    match_distance: float = .25
    new_identity_distance: float = .45
    margin: float = .08
    update_distance: float = .15
    retention_s: float = 30.
    max_gap_s: float = 1.
    max_identities: int = 64
    gallery_size: int = 8
    # Optional refinements (0/False keeps the original behaviour); see
    # docs/方案C身份ReID接入与验收.md for the recording evaluation.
    verify_distance: float = 0.  # looser keep-alive for an already bound track; 0 = match_distance
    confirm_min_interval_s: float = 0.  # samples closer than this do not count towards confirmation
    co_visible_exclusion: bool = False  # tracks seen together are different people
    reconfirm_on_overlap_only: bool = False  # other bad crops pause, but need not re-confirm, a bound track
    owner_views: int = 0  # >0: a pinned owner collects up to this many extra views while followed
    owner_view_novelty: float = .1  # a view is added only if this far from every stored view

    def __post_init__(self):
        for name in ('confirm_frames', 'max_identities', 'gallery_size'):
            if type(getattr(self, name)) is not int or getattr(self, name) < 2:
                raise ValueError(f'{name} must be integer >= 2')
        for name in ('match_distance', 'new_identity_distance', 'margin', 'update_distance',
                     'retention_s', 'max_gap_s'):
            if not math.isfinite(getattr(self, name)) or getattr(self, name) <= 0:
                raise ValueError(f'{name} must be finite and positive')
        if not 0 < self.update_distance <= self.match_distance < self.new_identity_distance <= 2:
            raise ValueError('Require update <= match < new identity cosine distance <= 2')
        if not (self.verify_distance == 0 or
                self.match_distance <= self.verify_distance < self.new_identity_distance):
            raise ValueError('verify_distance must be 0 or within [match, new identity) distance')
        if not math.isfinite(self.confirm_min_interval_s) or self.confirm_min_interval_s < 0:
            raise ValueError('confirm_min_interval_s must be finite and non-negative')
        if type(self.owner_views) is not int or self.owner_views < 0 or not 0 < self.owner_view_novelty < 1:
            raise ValueError('owner_views must be a non-negative integer, novelty in (0, 1)')
        for name in ('co_visible_exclusion', 'reconfirm_on_overlap_only'):
            if type(getattr(self, name)) is not bool:
                raise ValueError(f'{name} must be boolean')


def unit(vector):
    a = np.asarray(vector, dtype=np.float64)
    if a.ndim != 1 or a.size < 2 or not np.isfinite(a).all() or np.linalg.norm(a) < 1e-8:
        raise ValueError('Invalid appearance descriptor')
    return a/np.linalg.norm(a)


def distance(a, b):
    return float(np.clip(1.-np.dot(a, b), 0., 2.))


@dataclass(frozen=True)
class IdentityResult:
    track_id: str
    person_id: str = ''
    state: str = 'UNCONFIRMED'
    distance: float = math.nan
    detail: str = ''


class Gallery:
    def __init__(self, features, stamp, size):
        self.anchor = unit(np.mean(features, axis=0))
        self.features = deque((v.copy() for v in features), maxlen=size)
        self.last_seen = stamp
        self.views = deque()  # owner-only extra viewpoints, collected while followed

    def score(self, feature):
        # Initial registration cannot drift with repeated online updates.
        base = max(distance(feature, self.anchor),
                   float(np.median([distance(feature, v) for v in self.features])))
        if self.views:
            return min(base, min(distance(feature, v) for v in self.views))
        return base


class IdentityManager:
    def __init__(self, config=None):
        self.cfg = config or IdentityConfig()
        self.session = uuid.uuid4().hex[:12]
        self.counter = 0
        self.galleries, self.bindings, self.pending = {}, {}, {}
        self.unverified = set()
        self.excluded = {}  # track -> {identity: holder track seen at the same time}
        self.pinned = ''  # owner identity: never expires, may collect extra views
        self.last_identity = {}  # track -> identity it was last bound to
        self.swap_votes = {}  # (track, excluded identity) -> consecutive swap-evidence count
        self.track_seen = {}  # track -> last observation time (exclusions survive brief misses)
        self.last_stamp = None
        self.dimension = None

    def suspend(self):
        """Drop current links; retained identities require fresh multi-frame confirmation."""
        self.bindings.clear(); self.pending.clear(); self.unverified.clear(); self.excluded.clear()
        self.last_identity.clear(); self.swap_votes.clear(); self.track_seen.clear()

    def reset(self):
        self.suspend(); self.galleries.clear(); self.last_stamp = None; self.dimension = None
        self.pinned = ''

    def pin(self, person_id):
        """Mark the followed owner: exempt from retention, eligible for extra views."""
        if person_id not in self.galleries:
            return False
        if self.pinned != person_id:
            self.unpin()
            self.pinned = person_id
            self.galleries[person_id].views = deque(maxlen=max(1, self.cfg.owner_views))
        return True

    def unpin(self):
        if self.pinned in self.galleries:
            self.galleries[self.pinned].views.clear()
        self.pinned = ''

    def forget(self, person_id):
        """Remove the released owner gallery and invalidate all current links."""
        self.galleries.pop(person_id, None)
        if self.pinned == person_id:
            self.unpin()
        # Pending confirmations/exclusions may refer to this identity too.
        self.suspend()

    def update(self, stamp, observations, overlapped=(), focus=None):
        """observations: {epoch:track_id: unit descriptor or None (bad crop)}.

        Every visible track participates in one-to-one competition. An occluded
        bound track reserves its identity but is not published as verified.
        overlapped: tracks whose crop was rejected for overlapping another person.
        """
        if not math.isfinite(stamp) or stamp <= 0:
            raise ValueError('Require positive observation time')
        if self.last_stamp is not None:
            if stamp <= self.last_stamp:
                self.reset()
            elif stamp-self.last_stamp > self.cfg.max_gap_s:
                self.suspend()
        self.last_stamp = stamp
        for person_id, g in list(self.galleries.items()):
            if stamp-g.last_seen > self.cfg.retention_s and person_id != self.pinned:
                del self.galleries[person_id]
        self.bindings = {t: p for t, p in self.bindings.items()
                         if t in observations and p in self.galleries}
        self.pending = {t: p for t, p in self.pending.items() if t in observations}
        self.unverified.intersection_update(self.bindings)
        self.track_seen.update((t, stamp) for t in observations)
        self.track_seen = {t: s for t, s in self.track_seen.items() if stamp-s <= self.cfg.max_gap_s}
        self.excluded = {t: e for t, e in self.excluded.items() if t in self.track_seen}
        self.last_identity = {t: p for t, p in self.last_identity.items() if t in self.track_seen}
        descriptors = {}
        for track, feature in observations.items():
            if feature is not None:
                feature = unit(feature)
                if self.dimension is not None and feature.size != self.dimension:
                    raise ValueError('Descriptor dimension changed')
                self.dimension = feature.size
                descriptors[track] = feature
        scores = {t: {p: g.score(f) for p, g in self.galleries.items()}
                  for t, f in descriptors.items()}
        # A contradictory descriptor breaks the mapping immediately, never after
        # an EMA update has already contaminated the stored appearance.
        verify = self.cfg.verify_distance or self.cfg.match_distance
        for t, p in list(self.bindings.items()):
            if t in scores and scores[t][p] > verify:
                del self.bindings[t]; self.pending.pop(t, None)
        # An exclusion lapses only on repeated swap evidence: its holder now shows the
        # identity this track last held, not the excluded one. A holder that merely left,
        # or a single noisy frame, keeps it.
        votes = {}
        for u, held in self.excluded.items():
            mine = self.last_identity.get(u)
            for q, holder in list(held.items()):
                s = scores.get(holder, {})
                if mine in s and q in s and s[mine] <= self.cfg.match_distance and s[q] > verify:
                    votes[u, q] = self.swap_votes.get((u, q), 0)+1
                    if votes[u, q] >= self.cfg.confirm_frames:
                        del held[q]; del votes[u, q]
        self.swap_votes = votes
        owners = {p: t for t, p in self.bindings.items()}
        results = []
        for track in sorted(observations):
            if track not in descriptors:
                if not self.cfg.reconfirm_on_overlap_only or track in overlapped:
                    self.unverified.add(track)
                self.pending.pop(track, None)
                results.append(IdentityResult(track, state='UNVERIFIED', detail='appearance quality rejected'))
                continue
            feature = descriptors[track]
            bound = self.bindings.get(track)
            if bound and track not in self.unverified and bound_verified(scores, track, bound, verify, self.cfg.margin):
                g = self.galleries[bound]; g.last_seen = stamp
                if scores[track][bound] <= self.cfg.update_distance:
                    g.features.append(feature.copy())
                results.append(IdentityResult(track, bound, 'VERIFIED', scores[track][bound],
                                              'current appearance verified'))
                continue
            ranked = sorted(scores[track].items(), key=lambda x: (x[1], x[0]))
            best, score = ranked[0] if ranked else ('', math.inf)
            row_ambiguous = len(ranked) > 1 and ranked[1][1]-score < self.cfg.margin
            # Also reject two current people competing for the same identity.
            competitors = [v[best] for t, v in scores.items() if t != track] if best else []
            column_ambiguous = bool(competitors and min(competitors)-score < self.cfg.margin)
            matched = bool(best and score <= self.cfg.match_distance)
            new = not ranked or score >= self.cfg.new_identity_distance
            excluded = best in self.excluded.get(track, ())
            if (matched and (row_ambiguous or column_ambiguous or excluded
                             or (best in owners and owners[best] != track))) or not (matched or new):
                self.unverified.add(track)
                self.pending.pop(track, None)
                results.append(IdentityResult(track, state='AMBIGUOUS', distance=score,
                                              detail='distance/margin/one-to-one gate'))
                continue
            # Similar simultaneous new people must not create indistinguishable galleries.
            if new and any(distance(feature, f) < self.cfg.new_identity_distance
                           for t, f in descriptors.items() if t != track):
                self.unverified.add(track)
                self.pending.pop(track, None)
                results.append(IdentityResult(track, state='AMBIGUOUS', detail='similar new people'))
                continue
            candidate = best if matched else 'NEW'
            pending = self.pending.get(track)
            if (pending is None or pending[0] != candidate
                    or any(distance(feature, f) > self.cfg.update_distance for f in pending[1])):
                pending = (candidate, deque(maxlen=self.cfg.confirm_frames), [-math.inf])
                self.pending[track] = pending
            if stamp-pending[2][0] >= self.cfg.confirm_min_interval_s:
                pending[1].append(feature.copy()); pending[2][0] = stamp
            if len(pending[1]) < self.cfg.confirm_frames:
                results.append(IdentityResult(track, state='PENDING', distance=score if ranked else math.nan,
                                              detail='waiting for consistent distinct observations'))
                continue
            if new:
                if len(self.galleries) >= self.cfg.max_identities:
                    results.append(IdentityResult(track, state='UNCONFIRMED', detail='gallery capacity reached'))
                    continue
                self.counter += 1
                best = f'{self.session}:person_{self.counter:04d}'
                self.galleries[best] = Gallery(list(pending[1]), stamp, self.cfg.gallery_size)
            else:
                self.galleries[best].last_seen = stamp
            self.bindings[track] = best; owners[best] = track; self.last_identity[track] = best
            self.unverified.discard(track)
            self.pending.pop(track, None)
            results.append(IdentityResult(track, best, 'REGISTERED' if new else 'REASSOCIATED',
                                          score if ranked else math.nan, 'multi-frame confirmation'))
        if self.cfg.owner_views and self.pinned and focus in descriptors and focus not in overlapped:
            # Extra owner views only from the followed, unoccluded track the owner was verified on.
            if any(r.track_id == focus and r.person_id == self.pinned and r.state == 'VERIFIED' for r in results):
                g = self.galleries[self.pinned]
                f = descriptors[focus]
                if all(distance(f, v) >= self.cfg.owner_view_novelty for v in list(g.views)+list(g.features)):
                    g.views.append(f.copy())
        if self.cfg.co_visible_exclusion:
            for t, p in self.bindings.items():
                for u in observations:
                    if u != t:
                        self.excluded.setdefault(u, {})[p] = t
        return results


def bound_verified(scores, track, identity, limit, margin):
    """Bound-track check: within limit and clear of other identities and other tracks."""
    own = scores[track][identity]
    return (own <= limit
            and all(v-own >= margin for q, v in scores[track].items() if q != identity)
            and all(v[identity]-own >= margin for u, v in scores.items() if u != track))
