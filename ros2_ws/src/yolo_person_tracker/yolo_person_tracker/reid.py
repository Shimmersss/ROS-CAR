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

    def score(self, feature):
        # Initial registration cannot drift with repeated online updates.
        return max(distance(feature, self.anchor),
                   float(np.median([distance(feature, v) for v in self.features])))


class IdentityManager:
    def __init__(self, config=None):
        self.cfg = config or IdentityConfig()
        self.session = uuid.uuid4().hex[:12]
        self.counter = 0
        self.galleries, self.bindings, self.pending = {}, {}, {}
        self.unverified = set()
        self.last_stamp = None
        self.dimension = None

    def suspend(self):
        """Drop current links; retained identities require fresh multi-frame confirmation."""
        self.bindings.clear(); self.pending.clear(); self.unverified.clear()

    def reset(self):
        self.suspend(); self.galleries.clear(); self.last_stamp = None; self.dimension = None

    def update(self, stamp, observations):
        """observations: {epoch:track_id: unit descriptor or None (bad crop)}.

        Every visible track participates in one-to-one competition. An occluded
        bound track reserves its identity but is not published as verified.
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
            if stamp-g.last_seen > self.cfg.retention_s:
                del self.galleries[person_id]
        self.bindings = {t: p for t, p in self.bindings.items()
                         if t in observations and p in self.galleries}
        self.pending = {t: p for t, p in self.pending.items() if t in observations}
        self.unverified.intersection_update(self.bindings)
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
        for t, p in list(self.bindings.items()):
            if t in scores and scores[t][p] > self.cfg.match_distance:
                del self.bindings[t]; self.pending.pop(t, None)
        owners = {p: t for t, p in self.bindings.items()}
        results = []
        for track in sorted(observations):
            if track not in descriptors:
                self.unverified.add(track)
                self.pending.pop(track, None)
                results.append(IdentityResult(track, state='UNVERIFIED', detail='appearance quality rejected'))
                continue
            feature = descriptors[track]
            ranked = sorted(scores[track].items(), key=lambda x: (x[1], x[0]))
            best, score = ranked[0] if ranked else ('', math.inf)
            row_ambiguous = len(ranked) > 1 and ranked[1][1]-score < self.cfg.margin
            # Also reject two current people competing for the same identity.
            competitors = [v[best] for t, v in scores.items() if t != track] if best else []
            column_ambiguous = bool(competitors and min(competitors)-score < self.cfg.margin)
            matched = bool(best and score <= self.cfg.match_distance)
            new = not ranked or score >= self.cfg.new_identity_distance
            if (matched and (row_ambiguous or column_ambiguous
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
            if matched and self.bindings.get(track) == best and track not in self.unverified:
                g = self.galleries[best]; g.last_seen = stamp
                if score <= self.cfg.update_distance:
                    g.features.append(feature.copy())
                results.append(IdentityResult(track, best, 'VERIFIED', score, 'current appearance verified'))
                continue
            candidate = best if matched else 'NEW'
            pending = self.pending.get(track)
            if (pending is None or pending[0] != candidate
                    or any(distance(feature, f) > self.cfg.update_distance for f in pending[1])):
                pending = (candidate, deque(maxlen=self.cfg.confirm_frames))
                self.pending[track] = pending
            pending[1].append(feature.copy())
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
            self.bindings[track] = best; owners[best] = track
            self.unverified.discard(track)
            self.pending.pop(track, None)
            results.append(IdentityResult(track, best, 'REGISTERED' if new else 'REASSOCIATED',
                                          score if ranked else math.nan, 'multi-frame confirmation'))
        return results
