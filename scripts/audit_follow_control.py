#!/usr/bin/env python3
"""Audit recorded TargetState inputs; proposed commands only, no ROS/motion output."""
import argparse
from collections import Counter
from dataclasses import replace
import json
import math
from pathlib import Path
import statistics
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'ros2_ws/src/astra_body_adapter'))
from astra_body_adapter.follow_control import FollowConfig, compute_command, observation_is_fresh


def stamp(value):
    return value['sec']*10**9+value['nanosec']


def audit(path):
    events = [json.loads(line) for line in path.read_text().splitlines()]
    poses = {stamp(e['data']['header']['stamp']): e['data'] for e in events
             if e['topic'] == '/perception/person_states' and e['data']['valid']}
    old = FollowConfig(target_distance_m=1.3, max_linear_mps=.2, distance_gain=1., bearing_gain=1.6)
    new = replace(old, ackermann=True, min_turn_radius_m=.4)
    counts, sources = Counter(), Counter()
    distances, residuals, radii = [], [], []
    for event in events:
        if event['topic'] != '/perception/target_state': continue
        d = event['data']; counts['target_messages'] += 1
        if not (d['position_valid'] and d['status'] == 2 and not d['is_simulated']
                and math.isfinite(d['horizontal_distance_m']) and math.isfinite(d['bearing_rad'])
                and observation_is_fresh(event['stamp']*1e-9, stamp(d['header']['stamp'])*1e-9,
                                         stamp(d['observation_stamp'])*1e-9, d['measurement_age_s'],
                                         .5, d['source'], 'yolo')):
            continue
        distance, bearing = d['horizontal_distance_m'], d['bearing_rad']
        distances.append(distance); counts['fresh_valid'] += 1
        counts['beyond_1_3m' if distance > 1.3 else 'within_1_3m'] += 1
        for label, cfg in (('legacy', old), ('ackermann', new)):
            v, w = compute_command(distance, bearing, True, cfg)
            counts[label+'_forward'] += v > 0
            counts[label+'_yaw_only'] += v == 0 and w != 0
            if label == 'ackermann' and w:
                radii.append(v/abs(w))
        people = poses.get(stamp(d['observation_stamp']), {}).get('persons', [])
        person = next((p for p in people if p.get('track_id') == d['target_id']), None)
        if person:
            counts['exact_pose_track_pairs'] += 1
            sources[person['body_depth_source']] += 1
            if person['body_depth_valid']:
                residuals.append(abs(person['body_depth_m']-d['position']['z']))
    return dict(recording=str(path), scope='Fresh recorded targets, ungated command proposals. '
                'No authority/fall/vehicle simulation and no absolute range calibration.',
                counts=dict(counts), body_depth_sources=dict(sources),
                distance_min_m=min(distances) if distances else None,
                distance_median_m=statistics.median(distances) if distances else None,
                distance_max_m=max(distances) if distances else None,
                body_z_filter_difference_median_m=statistics.median(residuals) if residuals else None,
                body_z_filter_difference_max_m=max(residuals) if residuals else None,
                minimum_proposed_turn_radius_m=min(radii) if radii else None)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('timeline', type=Path)
    print(json.dumps(audit(parser.parse_args().timeline), indent=2))
