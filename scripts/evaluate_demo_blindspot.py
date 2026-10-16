#!/usr/bin/env python3
"""Offline-only bottom-edge descent candidates, not a runtime fall detector.

Pairs real cached height/box observations with the previously validated baseline.
Missing people never become confirmed falls. No pixels or depth are extrapolated.
"""
import argparse
from collections import deque
from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path

from evaluate_demo_falls import SEGMENTS
from evaluate_falls import aggregate, falls_and_actions, label_times, score
from evaluate_demo_fall_refinement import regressions
from replay_falls import combine_timelines


@dataclass(frozen=True)
class EdgeCandidate:
    margin_px: float = 48.
    drop_m: float = .3
    window_s: float = .8
    max_gap_s: float = .3
    hold_s: float = 6.
    early: bool = False

    def __post_init__(self):
        if not isinstance(self.early, bool):
            raise ValueError('early must be boolean')
        for name in ('margin_px','drop_m','window_s','max_gap_s','hold_s'):
            value = getattr(self, name)
            if isinstance(value, bool) or not math.isfinite(value) or value <= 0:
                raise ValueError(f'{name} must be finite and positive')
        if self.margin_px >= 480 or self.max_gap_s > self.window_s:
            raise ValueError('invalid image margin or observation window')


class EdgeWatch:
    """Diagnostic suspicion only. A falling top and downward box motion must agree."""
    def __init__(self, config):
        self.cfg = config
        self.tracks = {}
        self.stamp = None
        self.epoch = None

    def update(self, stamp, people, height_of):
        cfg = self.cfg
        if self.stamp is not None and (stamp <= self.stamp or stamp-self.stamp > cfg.max_gap_s):
            self.tracks.clear()
        self.stamp = stamp
        present = {p['id']: p for p in people if p['id'] is not None}
        epochs = {str(ident).split(':')[0] for ident in present}
        if epochs and epochs != {self.epoch}:
            self.tracks.clear()
            if len(epochs) != 1:
                return []
            self.epoch = next(iter(epochs))
        output = []
        for ident, person in present.items():
            box = tuple(person['box'])
            top = height_of(ident)
            state = self.tracks.setdefault(ident, dict(history=deque(), seen=stamp, armed=None, active=None, box=box))
            if stamp-state['seen'] > cfg.max_gap_s:
                state['history'].clear()
                state['armed'] = None
            state['seen'], state['box'] = stamp, box
            usable = (top is not None and math.isfinite(top) and top > 0
                      and box[1] > 15 and box[0] > 8 and box[2] < 632)
            if not usable:
                state['history'].clear()
                state['armed'] = None
                continue
            if top >= .9:
                state['active'] = None
            history = state['history']
            history.append((stamp, top, box[1]))
            while history and stamp-history[0][0] > cfg.window_s:
                history.popleft()
            descent = (len(history) >= 3 and box[3] >= 480-cfg.margin_px and top < .9
                       and any(h >= .8 and h-top >= cfg.drop_m and box[1]-y >= 38.4
                               for _,h,y in history))
            state['armed'] = stamp if descent else None
            if cfg.early and descent:
                state['active'] = stamp
        for ident, state in list(self.tracks.items()):
            if ident not in present and state['armed'] is not None:
                if stamp-state['armed'] <= cfg.max_gap_s:
                    state['active'] = stamp
                state['armed'] = None
            if state['active'] is not None and stamp-state['active'] <= cfg.hold_s:
                output.append(dict(t=stamp, id=ident, state='UNKNOWN', phase=1, basis='edge_watch',
                    detail='offline candidate: measured descent near bottom; suspicion only', height=0.))
            if stamp-state['seen'] > cfg.hold_s:
                del self.tracks[ident]
        return output


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source-root', type=Path, required=True)
    parser.add_argument('--baseline', type=Path, required=True,
                        help='validated output of evaluate_demo_fall_refinement.py')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    manifest = json.loads((args.baseline/'inputs-sha256.json').read_text())
    variants = {'baseline': None}
    variants.update({f'{mode}-edge{edge}-drop{drop}': EdgeCandidate(margin_px=edge, drop_m=drop, early=mode=='early')
                     for mode in ('lost', 'early') for edge in (24.,48.) for drop in (.3,.4,.5)})
    totals, per_event = {n:{} for n in variants}, {n:[] for n in variants}
    for segment in SEGMENTS:
        key = segment.split('-')[0]
        base = args.source_root/'artifacts/fall-eval-20261010'
        recording = args.source_root/f'data/recordings/gemini-20261010-{segment}'
        frame_path = base/'poses'/segment/'frames.jsonl'
        height_path = base/'replay'/key/'heights-geometry.jsonl'
        for path in (frame_path,height_path,recording/'labels.csv',recording/'color_timestamps.csv'):
            if hashlib.sha256(path.read_bytes()).hexdigest() != manifest.get(str(path)):
                raise ValueError(f'baseline input changed: {path}')
        frames = [json.loads(line) for line in frame_path.read_text().splitlines()]
        heights = {(r['frame'],r['id']):r['top'] for r in map(json.loads,height_path.read_text().splitlines())}
        baseline_path = args.baseline/'selected'/key/'timeline-combined.jsonl'
        baseline = [json.loads(line) for line in baseline_path.read_text().splitlines()]
        manifest[str(baseline_path)] = hashlib.sha256(baseline_path.read_bytes()).hexdigest()
        falls,actions = falls_and_actions(label_times(recording))
        for name,config in variants.items():
            candidate = []
            if config is not None:
                watch = EdgeWatch(config)
                for frame in frames:
                    candidate.extend(watch.update(frame['time_s'],frame['people'],
                        lambda ident: heights.get((frame['frame'],ident))))
            rows,stray = score(combine_timelines(baseline,candidate),falls,actions)
            totals[name][key] = dict(aggregate(rows), unlabelled_alarms=len(stray),candidate_rows=len(candidate))
            per_event[name].extend(dict(row,segment=key) for row in rows)
            if candidate:
                target=args.output/name
                target.mkdir(parents=True,exist_ok=True)
                (target/f'{key}.jsonl').write_text(''.join(json.dumps(row)+'\n' for row in candidate))
        print('scored',key,flush=True)
    summary={name:{field:sum(row[field] for row in segments.values()) for field in (
        'falls','fall_suspected','fall_confirmed','actions','false_suspected','false_confirmed','unlabelled_alarms','candidate_rows')}
        for name,segments in totals.items()}
    report=dict(scope='offline only; source recordings are not a 1.3 m moving-camera blindspot acceptance',
                totals=summary,per_segment=totals,
                regressions={name:regressions(per_event['baseline'],events) for name,events in per_event.items()})
    (args.output/'comparison.json').write_text(json.dumps(report,indent=2))
    (args.output/'per-event.json').write_text(json.dumps(per_event,indent=2))
    (args.output/'inputs-sha256.json').write_text(json.dumps(manifest,indent=2))
    print(json.dumps(summary,indent=2))


if __name__=='__main__':
    main()
