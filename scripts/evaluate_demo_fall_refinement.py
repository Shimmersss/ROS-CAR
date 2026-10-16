#!/usr/bin/env python3
"""A2a/A4 paired ablations on all eight labelled recordings; offline diagnostics only.

Rebuild depth joints once per recording, then reuse identical measured depth/pose inputs.
All candidates use the same labels and frames. No held-out accuracy claim is made.
"""
import argparse
from collections import Counter
from dataclasses import replace
import hashlib
import json
from pathlib import Path

import replay_falls as replay
from evaluate_demo_falls import ROOT, SEGMENTS
from evaluate_falls import aggregate, falls_and_actions, label_times, score


class CachedContinuation(replay.Continuation):
    """Memoize measurements only, never tracker decisions, across the paired ablations."""
    def __init__(self, *args):
        super().__init__(*args)
        self.cache = {}

    def top(self, box):
        key = (self.frame['frame'], tuple(box))
        if key not in self.cache:
            self.cache[key] = super().top(box)
        return self.cache[key]


def regressions(baseline, candidate):
    """Compare individual events, not totals that could conceal exchanged misses."""
    key = lambda row: (row['segment'], row['kind'], row['onset'], row.get('action', ''))
    before = {key(row): row for row in baseline}
    after = {key(row): row for row in candidate}
    if before.keys() != after.keys():
        raise ValueError('comparison must contain the same labelled events')
    return [dict(event=identity, level=level) for identity, row in before.items()
            for level in ('suspected', 'confirmed')
            if (row['kind'] == 'fall' and row[level] and not after[identity][level])
            or (row['kind'] == 'action' and not row[level] and after[identity][level])]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source-root', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    base = args.source_root/'artifacts/fall-eval-20261010'
    profile = ROOT/'ros2_ws/src/perception_bringup/config/pose_fixed_27cm_demo.yaml'
    selected = replay.load_height_config(profile)
    candidates = dict(baseline=replace(selected, drop_min_m=.4, confirm_max_m=0.),
                      drop_only=replace(selected, confirm_max_m=0.),
                      confirmation_only=replace(selected, drop_min_m=.4), selected=selected)
    totals = {name: Counter() for name in candidates}
    per_event = {name: [] for name in candidates}
    manifest = {}
    pose, spatial = replay.load_configs(profile)
    for segment in SEGMENTS:
        key = segment.split('-')[0]
        recording = args.source_root/f'data/recordings/gemini-20261010-{segment}'
        poses = base/'poses'/segment/'frames.jsonl'
        geometry_path = base/'geometry'/f'{key}.jsonl'
        heights_path = base/'replay'/key/'heights-geometry.jsonl'
        for path in (profile, poses, geometry_path, heights_path, recording/'labels.csv',
                     recording/'device.json', recording/'timestamps.csv', recording/'color_timestamps.csv'):
            manifest[str(path)] = hashlib.sha256(path.read_bytes()).hexdigest()
        frames = [json.loads(line) for line in poses.read_text().splitlines()]
        replay.recompute_depth(recording, frames, pose.confidence, profile)
        replay.VARIANT.clear()
        pose_rows = replay.replay(frames, {}, '3d-config', pose, spatial, None)
        postures = {(row['t'], row['id']): row['state'] for row in pose_rows}
        heights = {(r['frame'], r['id']): r['top']
                   for r in map(json.loads, heights_path.read_text().splitlines())}
        geometry = {r['frame']: (replay.np.array(r['up']), r['camera_height_m'])
                    for r in map(json.loads, geometry_path.read_text().splitlines()) if r['up'] is not None}
        continuation = CachedContinuation(recording, lambda f: geometry.get(f['frame']))
        falls, actions = falls_and_actions(label_times(recording))
        for name, config in candidates.items():
            replay.VARIANT.clear()
            replay.VARIANT.update(handover_s=pose.handover_s, veto2d=True)
            height_rows = replay.replay_height(frames, heights, continuation, postures, height_config=config)
            timeline = replay.combine_timelines(pose_rows, height_rows)
            rows, stray = score(timeline, falls, actions)
            counts = aggregate(rows)
            totals[name].update({field: counts[field] for field in (
                'falls', 'fall_suspected', 'fall_confirmed', 'actions', 'false_suspected', 'false_confirmed')})
            totals[name]['unlabelled_alarms'] += len(stray)
            per_event[name].extend(dict(row, segment=key) for row in rows)
            output = args.output/name/key
            output.mkdir(parents=True, exist_ok=True)
            for mode, timeline_rows in (('height', height_rows), ('combined', timeline)):
                (output/f'timeline-{mode}.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in timeline_rows))
        print('evaluated', key, flush=True)
    failures = regressions(per_event['baseline'], per_event['selected'])
    report = dict(scope='Static-camera diagnostic replay, not hardware acceptance or held-out evaluation',
                  results=totals, regressions=failures)
    (args.output/'comparison.json').write_text(json.dumps(report, indent=2))
    (args.output/'per-event.json').write_text(json.dumps(per_event, indent=2))
    (args.output/'inputs-sha256.json').write_text(json.dumps(manifest, indent=2))
    print(json.dumps(report, indent=2), flush=True)
    if failures or totals['selected']['unlabelled_alarms'] > totals['baseline']['unlabelled_alarms']:
        raise SystemExit('selected candidate regresses labelled events or unlabelled alarms')


if __name__ == '__main__':
    main()
