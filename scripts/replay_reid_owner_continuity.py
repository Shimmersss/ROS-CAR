#!/usr/bin/env python3
"""Compare owner continuity reconfirmation using cached real descriptors and ROS events.

Inputs: analysis/{descriptors.jsonl,timeline.jsonl,descriptor-manifest.json}.
Reuses recorded inference events; reconstructs focus from OwnerState as an approximation.
This is not a closed-loop vehicle replay.
No model download, ROS publication, or motion authorization.
"""
import argparse
from collections import Counter
from dataclasses import replace
import hashlib
import json
from pathlib import Path
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'ros2_ws/src/yolo_person_tracker'))
from yolo_person_tracker.reid import IdentityConfig, IdentityManager


def compare(root):
    descriptors = root/'descriptors.jsonl'
    frames = {r['stamp_ns']: r for r in map(json.loads, descriptors.read_text().splitlines())}
    events = list(map(json.loads, (root/'timeline.jsonl').read_text().splitlines()))
    metadata = json.loads((root/'descriptor-manifest.json').read_text())
    if hashlib.sha256(descriptors.read_bytes()).hexdigest() != metadata['descriptor_sha256']:
        raise ValueError('Descriptor cache hash differs from manifest')
    config = IdentityConfig(**{k: v for k, v in metadata['quality_config'].items()
                               if k in vars(IdentityConfig())})
    report = {'descriptor_sha256': hashlib.sha256(descriptors.read_bytes()).hexdigest(),
              'model_sha256': metadata['model_sha256'],
              'scope': 'Recorded inference events; focus approximated from OwnerState; empty initial gallery. '
                       'Association comparison only, not identification accuracy or closed-loop recovery.'}
    for enabled in (False, True):
        mode = 'candidate' if enabled else 'baseline'
        manager = IdentityManager(replace(config, owner_continuity_verify=enabled))
        focus, owner = None, ''
        counts, positives, output = Counter(), Counter(), []
        valid_events = 0
        for event in events:
            data = event['data']
            if event['topic'] == '/perception/owner_state':
                focus = data['target_id'] or None
                if data['person_id'] and owner:
                    manager.pin(owner)
                continue
            if event['topic'] != '/perception/person_identities':
                continue
            if not data['valid']:
                if data['detail'] not in ('Stale appearance observations',
                                          'Waiting for fresh matching RGB/Pose'):
                    manager.suspend()
                continue
            valid_events += 1
            stamp = data['header']['stamp']
            stamp_ns = stamp['sec']*10**9+stamp['nanosec']
            frame = frames[stamp_ns]  # Missing exact pairs must fail, never use nearest RGB.
            observations = {t: np.asarray(f) if f is not None else None
                            for t, f in frame['observations'].items()}
            overlaps = {t for t, reason in frame['reasons'].items()
                        if reason in ('overlapping people', 'pose-masked overlapping crop')}
            for result in manager.update(stamp_ns*1e-9, observations, overlaps, focus):
                counts[result.state] += 1
                if result.person_id:
                    positives[result.track_id] += 1
                    # Map the recorded owner ID into this independent replay's ID namespace.
                    if not owner and result.track_id == focus:
                        owner = result.person_id
                output.append(dict(stamp_ns=stamp_ns, t=event['t'], track=result.track_id,
                                   person=result.person_id, state=result.state,
                                   distance=result.distance if np.isfinite(result.distance) else None,
                                   box=frame['boxes'][result.track_id]))
        (root/(mode+'-continuity.jsonl')).write_text(
            ''.join(json.dumps(row)+'\n' for row in output))
        gallery = manager.galleries.get(owner)
        report[mode] = dict(counts=dict(counts), positive_tracks=dict(positives),
                            valid_events=valid_events, missing_exact_pairs=0,
                            owner_views=len(gallery.views) if gallery else 0)
    (root/'owner-continuity-comparison.json').write_text(json.dumps(report, indent=2))
    return report


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('analysis', type=Path)
    print(json.dumps(compare(parser.parse_args().analysis), indent=2))
