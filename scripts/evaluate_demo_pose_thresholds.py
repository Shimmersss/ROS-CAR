#!/usr/bin/env python3
"""T2.6 paired offline keypoint/bottom-edge sweep. No detector or runtime changes.

Cached boxes/keypoints are fixed; depth joints are rebuilt at each threshold. Posture and height
vetoes are recomputed. Height geometry is diagnostic, not calibrated acceptance.
"""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
from evaluate_demo_falls import ROOT, SEGMENTS
from replay_falls import recompute_depth, depth_cache_metadata


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source-root', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    base = args.source_root/'artifacts/fall-eval-20261010'
    profile = ROOT/'ros2_ws/src/perception_bringup/config/pose_fixed_27cm_demo.yaml'
    manifest = {str(profile): hashlib.sha256(profile.read_bytes()).hexdigest()}
    totals = {}
    args.output.mkdir(parents=True, exist_ok=True)
    for confidence in (.5, .35):
        with tempfile.TemporaryDirectory(prefix=f'rebuilt-{confidence}-', dir=args.output) as temporary:
            rebuilt = {}
            for segment in SEGMENTS:
                recording = args.source_root/f'data/recordings/gemini-20261010-{segment}'
                frames = [json.loads(line) for line in (base/'poses'/segment/'frames.jsonl').read_text().splitlines()]
                target = Path(temporary)/segment
                target.mkdir(parents=True, exist_ok=True)
                frames = recompute_depth(recording, frames, confidence, profile)
                (target/'frames.jsonl').write_text(''.join(json.dumps(row)+'\n' for row in frames))
                (target/'rebuilt-depth.json').write_text(json.dumps(depth_cache_metadata(target, profile, confidence)))
                rebuilt[segment] = target
                print('rebuilt', confidence, segment, flush=True)
            for edge in (0, 8, 15):
                name = f'confidence-{confidence}-bottom-{edge}'
                pairs = []
                for segment in SEGMENTS:
                    key = segment.split('-')[0]
                    recording = args.source_root/f'data/recordings/gemini-20261010-{segment}'
                    poses = base/'poses'/segment
                    geometry = base/'geometry'/f'{key}.jsonl'
                    output = args.output/name/key
                    output.mkdir(parents=True, exist_ok=True)
                    sources = [poses/'frames.jsonl', geometry, recording/'labels.csv',
                               recording/'device.json', recording/'color_timestamps.csv',
                               recording/'timestamps.csv']
                    caches = ('floor.jsonl', 'heights-geometry.jsonl')
                    sources += [base/'replay'/key/cache for cache in caches]
                    for source in sources:
                        manifest[str(source)] = hashlib.sha256(source.read_bytes()).hexdigest()
                    for cache in caches:
                        shutil.copyfile(base/'replay'/key/cache, output/cache)
                    try:
                        subprocess.run([sys.executable, str(ROOT/'scripts/replay_falls.py'),
                            str(recording), str(rebuilt[segment]), '--output', str(output), '--modes', '3d-config', 'height',
                            '--pose-config', str(profile), '--pose-confidence', str(confidence),
                            '--height-postures', '3d-config', '--bottom-edge-px', str(edge),
                            '--variant', 'handover=1.5,veto2d,lost=6,sit=3,side=8,cont=6,drop=0.4,confirm_max=0',
                            '--geometry', str(geometry)], check=True)
                    finally:
                        for cache in caches:
                            (output/cache).unlink(missing_ok=True)
                    pairs.extend(['--pair', str(recording), str(output)])
                evaluation = args.output/name/'evaluation'
                subprocess.run([sys.executable, str(ROOT/'scripts/evaluate_falls.py'),
                                *pairs, '--output', str(evaluation)], check=True)
                summary = json.loads((evaluation/'summary.json').read_text())
                modes = {}
                for mode, recordings in summary.items():
                    counts = modes.setdefault(mode.split('/')[-1], Counter())
                    for row in recordings.values():
                        for key in ('falls', 'fall_suspected', 'fall_confirmed', 'actions',
                                    'false_suspected', 'false_confirmed'):
                            counts[key] += row.get(key) or 0
                        counts['unlabelled_alarms'] += len(row['unlabelled_alarms_s'])
                totals[name] = modes
                print(name, json.dumps(modes), flush=True)
    (args.output/'inputs-sha256.json').write_text(json.dumps(manifest, indent=2))
    (args.output/'comparison.json').write_text(json.dumps(totals, indent=2))


if __name__ == '__main__':
    main()
