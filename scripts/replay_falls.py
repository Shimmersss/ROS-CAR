#!/usr/bin/env python3
"""Replay cached poses through the fall state machine in several modes and list fall events.

Modes (offline experiment; the *_confirmed flags are forced on here, never in deployment):
  2d        pixel rules only, upright confirmed
  3d-fixed  3D rules with one gravity/camera height per segment (median of stable RANSAC fits,
            standing in for a manual mount calibration)
  3d-floor  3D rules with this frame's stable RANSAC floor (pose3d_up_source=floor)
  3d-geometry  3D rules with this frame's gravity/camera height from --geometry
  height    body-top height cue; per-frame --geometry when given, else the 3d-fixed geometry
Inputs: a recording plus frames.jsonl written by analyze_gemini_recording.py for it.
Host-receive colour/depth pairing only; not exposure-synchronised.
"""
import argparse
from dataclasses import fields
import json
import math
from pathlib import Path
import sys

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'ros2_ws/src/yolo_person_tracker'))
sys.path.insert(0, str(ROOT/'scripts'))
from yolo_person_tracker.floor_plane import FloorConfig, FloorTracker, camera_angles, fit_floor  # noqa: E402
from yolo_person_tracker.pose import JointJumpGate, LABELS, PoseConfig  # noqa: E402
from yolo_person_tracker.pose3d import EnhancedPostureTracker, Pose3DConfig  # noqa: E402
from yolo_person_tracker.registration import Registration  # noqa: E402
from yolo_person_tracker.height_fall import (HeightFallConfig, HeightFallTracker, body_heights,  # noqa: E402
                                             at_side_edge, clipped_at_top, skeleton_veto)
from gemini_recording import load_depth_m, read_rows  # noqa: E402

MODES = ('2d', '3d-fixed', '3d-floor', '3d-geometry', 'height')
VARIANT = {}   # rule switches under evaluation, set from --variant
HEIGHT_KEYS = dict(upright='upright_min_m', low='low_max_m', drop='drop_min_m', recovery='recovery_s', confirm='confirm_s',
                   hint='hint_upright_min_m', lost='lost_hold_s', sit='sit_hold_s', side='side_edge_px')
IMAGE_WIDTH = 640   # recorder colour stream (record_gemini_mac.py)
MERGE_S = 1.5   # detections closer than this belong to the same event


def load_configs():
    params = yaml.safe_load((ROOT/'ros2_ws/src/perception_bringup/config/pose.yaml').read_text())['/**']['ros__parameters']
    pose = {f.name: params['pose_'+f.name] for f in fields(PoseConfig) if 'pose_'+f.name in params}
    spatial = {f.name: params['pose3d_'+f.name] for f in fields(Pose3DConfig) if 'pose3d_'+f.name in params}
    return PoseConfig(**dict(pose, upright_confirmed=True)), spatial


def floor_fits(recording, frames, cache_path):
    """Per analysed frame: stable floor (up, height) or None; cached as JSON lines."""
    if cache_path.exists():
        rows = [json.loads(line) for line in cache_path.read_text().splitlines()]
        return {r['frame']: r for r in rows}
    reg = Registration(json.loads((recording/'device.json').read_text()))
    k = reg.color
    intrinsics = (k['fx'], k['fy'], k['cx'], k['cy'])
    depths = {r['frame']: r for r in read_rows(recording/'timestamps.csv')}
    blank = np.zeros((k['height'], k['width'], 3), np.uint8)
    tracker, out = FloorTracker(FloorConfig()), {}
    with cache_path.open('w') as handle:
        for f in frames:
            row = dict(frame=f['frame'], valid=False, stable=False)
            if f['depth_matched']:
                _, aligned = reg.apply(blank, load_depth_m(recording, depths[f['depth_frame']]))
                fit = fit_floor(aligned, intrinsics, [p['box'] for p in f['people']])
                stable, _ = tracker.update(f['time_s']+1., fit)
                row.update(valid=fit.valid, stable=bool(stable and fit.valid), reason=fit.reason)
                if fit.valid:
                    pitch, roll = camera_angles(fit.up)
                    row.update(up=list(fit.up), height=fit.height_m, pitch=pitch, roll=roll, rms=fit.rms_m)
            else:
                tracker.reset()
            out[row['frame']] = row
            handle.write(json.dumps(row)+'\n')
    return out


def person_heights(recording, frames, geometry_of, cache_path):
    """Body top height per tracked person and frame (cached); geometry_of(frame) -> (up, camera
    height) or None, in which case the frame is unmeasured."""
    if cache_path.exists():
        return {(r['frame'], r['id']): r['top'] for r in map(json.loads, cache_path.read_text().splitlines())}
    reg = Registration(json.loads((recording/'device.json').read_text()))
    k = reg.color
    intrinsics = (k['fx'], k['fy'], k['cx'], k['cy'])
    depths = {r['frame']: r for r in read_rows(recording/'timestamps.csv')}
    blank = np.zeros((k['height'], k['width'], 3), np.uint8)
    out = {}
    with cache_path.open('w') as handle:
        for f in frames:
            if not f['depth_matched']:
                continue
            tracked = [p for p in f['people'] if p['id'] is not None]
            geometry = geometry_of(f)
            if not tracked or geometry is None:
                continue
            up, camera_height = geometry
            _, aligned = reg.apply(blank, load_depth_m(recording, depths[f['depth_frame']]))
            for p in tracked:
                measured = body_heights(aligned, p['box'], intrinsics, up, camera_height)
                top = None if measured is None else measured[0]
                out[(f['frame'], p['id'])] = top
                handle.write(json.dumps(dict(frame=f['frame'], id=p['id'], top=top))+'\n')
    return out


def replay_height(frames, heights):
    overrides = {k: VARIANT[k] for k in HEIGHT_KEYS.values() if k in VARIANT}
    if 'veto_aspect' in VARIANT:
        overrides['veto_min_aspect'] = VARIANT['veto_aspect'] or 1e-6
    tracker, boxes, timeline, epoch = HeightFallTracker(HeightFallConfig(
        transition_s=VARIANT.get('transition_s', 2.), **overrides)), {}, [], None
    for f in frames:
        stamp = f['time_s']+1.
        present = {p['id']: tuple(p['box']) for p in f['people'] if p['id'] is not None}
        epochs = {i.split(':')[0] for i in present}
        if epochs and epoch not in epochs:
            tracker.reset(); boxes.clear(); epoch = min(epochs)
        if VARIANT.get('handover_s'):
            tracker.handover(stamp, present, boxes, VARIANT['handover_s'])
        states = {p['id']: p.get('state') for p in f['people'] if p['id'] is not None}
        for ident, box in present.items():
            boxes[ident] = box
            # veto2d[=<r>]: a skeleton upright label vetoes only in a box r times taller than wide
            # (default HeightFallConfig.veto_min_aspect; veto2d=0 is the ungated veto).
            upright = VARIANT.get('veto2d', False) and states.get(ident) in ('STANDING', 'SITTING_CROUCHING')
            phase, detail = tracker.update(ident, stamp, heights.get((f['frame'], ident)),
                                           upright_hint=upright and skeleton_veto(True, box, tracker.cfg),
                                           posture_upright=upright and states.get(ident) == 'STANDING',
                                           clipped=clipped_at_top(box, tracker.cfg),
                                           side=at_side_edge(box, IMAGE_WIDTH, tracker.cfg))
            state = 'FALLEN' if phase == 2 else 'LYING' if 'drop' in detail or 'low' in detail else 'UNKNOWN'
            timeline.append(dict(t=f['time_s'], id=ident, state=state, phase=phase, basis='height',
                                 detail=detail[:160], height=float(box[3]-box[1])))
    return timeline


def replay(frames, floors, mode, pose, spatial, fixed, geometry=None):
    from dataclasses import replace
    spatial = dict(spatial)
    if VARIANT.get('knee_fallback'):
        spatial['knee_fallback'] = True
    pose = replace(pose, gap_pause=VARIANT.get('gap_pause', pose.gap_pause),
                   handover_s=VARIANT.get('handover_s', pose.handover_s),
                   transition_s=VARIANT.get('transition_s', pose.transition_s),
                   confirm_lying_only=VARIANT.get('confirm_lying_only', pose.confirm_lying_only))
    if mode == '2d':
        s = Pose3DConfig(**dict(spatial, enabled=False, gravity_confirmed=False, ground_confirmed=False))
    elif mode == '3d-fixed':
        up, height = fixed
        s = Pose3DConfig(**dict(spatial, up_source='static', gravity_confirmed=True, ground_confirmed=True,
                                up_x=up[0], up_y=up[1], up_z=up[2], camera_height_m=height))
    else:
        s = Pose3DConfig(**dict(spatial, up_source='floor', gravity_confirmed=True, ground_confirmed=True))
    tracker, gates, epoch, timeline = EnhancedPostureTracker(pose, s), {}, None, []
    for f in frames:
        stamp = f['time_s']+1.
        if mode == '3d-geometry':
            gravity = geometry.get(f['frame'])
        else:
            floor = floors.get(f['frame'], {})
            gravity = (np.array(floor['up']), floor['height']) if floor.get('stable') else None
        present = {p['id']: tuple(p['box']) for p in f['people'] if p['id'] is not None}
        epochs = {i.split(':')[0] for i in present}
        if epochs and epoch not in epochs:
            tracker.reset(); gates.clear(); epoch = min(epochs)
        for new, old in tracker.handover(stamp, present).items():
            if old in gates:
                gates[new] = gates.pop(old)
        for p in f['people']:
            if p['id'] is None:
                continue
            xyz = np.array([[np.nan if v is None else v for v in point] for point in p['joints3d']], float)
            gate = gates.setdefault(p['id'], JointJumpGate(pose.joint_jump_m, pose.joint_jump_speed_mps, pose.max_gap_s))
            xyz, _ = gate.apply(xyz, stamp)
            state, phase, detail = tracker.update(p['id'], stamp, tuple(p['box']), tuple(map(tuple, p['keypoints'])),
                                                  xyz, gravity)
            basis = detail.split(';')[0].replace('basis=', '')
            timeline.append(dict(t=f['time_s'], id=p['id'], state=LABELS[state], phase=phase, basis=basis, detail=detail[:160],
                                 height=float(p['box'][3]-p['box'][1])))
    return timeline


def events(timeline, phase):
    """Merge per-track detections at >= phase into events [(start, end, ids, basis)]."""
    hits = sorted((r['t'], r['id'], r['basis']) for r in timeline if r['phase'] >= phase)
    merged = []
    for t, ident, basis in hits:
        if merged and t-merged[-1]['end'] <= MERGE_S:
            merged[-1]['end'] = t
            merged[-1]['ids'].add(ident)
            merged[-1]['basis'].add(basis)
        else:
            merged.append(dict(start=t, end=t, ids={ident}, basis={basis}))
    return [dict(start=round(e['start'], 2), end=round(e['end'], 2), ids=sorted(e['ids']), basis=sorted(e['basis']))
            for e in merged]


def summarize_floor(floors):
    stable = [r for r in floors.values() if r.get('stable')]
    if not stable:
        return dict(stable_fraction=0.)
    pick = lambda key: float(np.median([r[key] for r in stable]))
    up = np.median([r['up'] for r in stable], axis=0)
    return dict(stable_fraction=round(len(stable)/max(1, len(floors)), 3), camera_height_m=round(pick('height'), 3),
                pitch_up_deg=round(pick('pitch'), 2), roll_deg=round(pick('roll'), 2),
                up=[round(float(v), 5) for v in up/np.linalg.norm(up)], rms_m=round(pick('rms'), 4))


def plot(path, timelines, title):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    order = ['UNKNOWN', 'STANDING', 'SITTING_CROUCHING', 'LYING', 'FALLEN']
    fig, axes = plt.subplots(len(timelines), 1, figsize=(12, 2.2*len(timelines)), sharex=True)
    for ax, (mode, timeline) in zip(np.atleast_1d(axes), timelines.items()):
        if timeline:
            main = max({r['id'] for r in timeline}, key=lambda i: sum(r['height'] for r in timeline if r['id'] == i))
            rows = [r for r in timeline if r['id'] == main]
            t = [r['t'] for r in rows]
            ax.step(t, [order.index(r['state']) for r in rows], where='post', lw=1, color='0.3')
            for phase, colour in ((1, 'orange'), (2, 'red')):
                ax.scatter([r['t'] for r in rows if r['phase'] == phase], [order.index(r['state']) for r in rows if r['phase'] == phase],
                           s=6, color=colour, label='suspected' if phase == 1 else 'confirmed')
            three = [r['t'] for r in rows if r['basis'] == '3d']
            if three:
                ax.scatter(three, [-0.6]*len(three), s=2, marker='|', color='tab:blue', label='3D basis')
        ax.set_yticks(range(len(order)), order, fontsize=7)
        ax.set_ylim(-1, len(order))
        ax.set_title(f'{title} — {mode}', fontsize=9, loc='left')
        ax.grid(axis='x', alpha=.3)
    np.atleast_1d(axes)[0].legend(fontsize=7, loc='upper right')
    np.atleast_1d(axes)[-1].set_xlabel('time since first frame (s)')
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('recording', type=Path)
    parser.add_argument('poses', type=Path, help='directory containing frames.jsonl for this recording')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--modes', nargs='+', choices=MODES, default=list(MODES))
    parser.add_argument('--geometry', type=Path,
                        help='per-frame gravity/camera height from estimate_camera_geometry.py (height, 3d-geometry)')
    parser.add_argument('--variant', default='baseline',
                        help="baseline, or comma list of gap_pause, knee_fallback, confirm_lying_only, veto2d, "
                             "handover=<s>, transition=<s>, upright=<m>, low=<m>, drop=<m>, recovery=<s>, confirm=<s>, hint=<m>, lost=<s>, sit=<s>, side=<px>")
    args = parser.parse_args()
    for item in filter(None, args.variant.split(',')):
        if item == 'baseline':
            continue
        key, _, value = item.partition('=')
        if key == 'handover':
            VARIANT['handover_s'] = float(value or 1.5)
        elif key == 'transition':
            VARIANT['transition_s'] = float(value)
        elif key in HEIGHT_KEYS:
            VARIANT[HEIGHT_KEYS[key]] = float(value)
        elif key in ('gap_pause', 'knee_fallback', 'confirm_lying_only', 'veto2d'):
            VARIANT[key] = True
            if key == 'veto2d' and value:
                VARIANT['veto_aspect'] = float(value)
        else:
            parser.error(f'unknown variant {item}')
    args.output.mkdir(parents=True, exist_ok=True)
    frames = [json.loads(line) for line in (args.poses/'frames.jsonl').read_text().splitlines()]
    pose, spatial = load_configs()
    floors = floor_fits(args.recording, frames, args.output/'floor.jsonl')
    floor = summarize_floor(floors)
    fixed = (floor['up'], floor['camera_height_m']) if 'up' in floor else None
    geometry = {}
    if args.geometry:
        for row in map(json.loads, args.geometry.read_text().splitlines()):
            if row['up'] is not None:
                geometry[row['frame']] = (np.array(row['up']), row['camera_height_m'])
    timelines, report = {}, dict(recording=args.recording.name, frames=len(frames), floor=floor, variant=args.variant,
                                 geometry=str(args.geometry) if args.geometry else None, modes={})
    for mode in args.modes:
        if mode in ('3d-fixed',) and fixed is None:
            report['modes'][mode] = 'skipped: no stable floor fit'
            continue
        if mode == '3d-geometry' and not geometry:
            report['modes'][mode] = 'skipped: no --geometry'
            continue
        if mode == 'height':
            if geometry:
                heights = person_heights(args.recording, frames, lambda f: geometry.get(f['frame']),
                                         args.output/'heights-geometry.jsonl')
            elif fixed is not None:
                heights = person_heights(args.recording, frames, lambda f: fixed, args.output/'heights.jsonl')
            else:
                report['modes'][mode] = 'skipped: no stable floor fit'
                continue
            timeline = replay_height(frames, heights)
        else:
            timeline = replay(frames, floors, mode, pose, spatial, fixed, geometry)
        timelines[mode] = timeline
        three = sum(r['basis'] == '3d' for r in timeline)
        report['modes'][mode] = dict(
            suspected_events=events(timeline, 1), confirmed_events=events(timeline, 2),
            basis_3d_fraction=round(three/max(1, len(timeline)), 3))
        with (args.output/f'timeline-{mode}.jsonl').open('w') as handle:
            for row in timeline:
                handle.write(json.dumps(row)+'\n')
    (args.output/'report.json').write_text(json.dumps(report, indent=2, ensure_ascii=False))
    if timelines:
        plot(args.output/'timeline.png', timelines, args.recording.name)
    print(json.dumps({m: (v if isinstance(v, str) else dict(suspected=len(v['suspected_events']), confirmed=len(v['confirmed_events']),
                                                           basis_3d=v['basis_3d_fraction']))
                      for m, v in report['modes'].items()}, ensure_ascii=False), '| floor', floor)


if __name__ == '__main__':
    main()
