#!/usr/bin/env python3
"""Compare fixed-camera height continuation on existing 10-10 labelled recordings.

Reuses cached poses/heights for the SAME recordings (no confidence sweep). This is
static-camera diagnostic replay, NOT verification of odometry gating or physical motion.
Those require test_demo_stationary_runtime.py and field tests respectively.
"""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys

ROOT=Path(__file__).resolve().parents[1]
SEGMENTS=('A1a-fall-side-1.9m-part2','A1b-fall-toward-1.9m','A1c-fall-walking-away',
          'A2a-fall-back-stumble-collapse','A2b-fall-backward-sit-chair','A3-fall-3m','A4','A5-daily-postures')

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source-root',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=True)
    base=args.source_root/'artifacts/fall-eval-20261010'
    manifest={};totals={}
    for name,variant in [('baseline','handover=1.5,veto2d,lost=6,sit=3,side=8'),
                         ('continuation','handover=1.5,veto2d,lost=6,sit=3,side=8,cont=6')]:
        pairs=[]
        for segment in SEGMENTS:
            key=segment.split('-')[0]
            recording=args.source_root/f'data/recordings/gemini-20261010-{segment}'
            poses=base/'poses'/segment;geometry=base/'geometry'/f'{key}.jsonl'
            output=args.output/name/key;output.mkdir(parents=True,exist_ok=True)
            for source in (poses/'frames.jsonl',geometry,recording/'labels.csv',recording/'device.json'):
                manifest[str(source)]=hashlib.sha256(source.read_bytes()).hexdigest()
            for cache in ('floor.jsonl','heights-geometry.jsonl'):
                source=base/'replay'/key/cache
                manifest[str(source)]=hashlib.sha256(source.read_bytes()).hexdigest()
                shutil.copyfile(source,output/cache)
            subprocess.run([sys.executable,str(ROOT/'scripts/replay_falls.py'),str(recording),str(poses),
                '--output',str(output),'--modes','height','--variant',variant,'--geometry',str(geometry)],check=True)
            for cache in ('floor.jsonl','heights-geometry.jsonl'):(output/cache).unlink()
            pairs.extend(['--pair',str(recording),str(output)])
        evaluation=args.output/name/'evaluation'
        subprocess.run([sys.executable,str(ROOT/'scripts/evaluate_falls.py'),*pairs,'--output',str(evaluation)],check=True)
        summary=json.loads((evaluation/'summary.json').read_text());counts=Counter()
        for recordings in summary.values():
            for row in recordings.values():
                for key in ('falls','fall_suspected','fall_confirmed','actions','false_suspected','false_confirmed'):
                    counts[key]+=row.get(key) or 0
                counts['unlabelled_alarms']+=len(row['unlabelled_alarms_s'])
        totals[name]=dict(counts)
    (args.output/'inputs-sha256.json').write_text(json.dumps(manifest,indent=2,ensure_ascii=False))
    (args.output/'comparison.json').write_text(json.dumps(dict(
        scope='Static-camera diagnostic replay; no recorded odometry; not a gated-node or physical validation',
        pose_confidence='unchanged cached poses; no T2.6 confidence sweep',results=totals),indent=2,ensure_ascii=False))
    print(json.dumps(totals,ensure_ascii=False),flush=True)

if __name__=='__main__':main()
