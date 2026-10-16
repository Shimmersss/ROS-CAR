#!/usr/bin/env python3
"""Replay cached poses through the fall state machine in several modes and list fall events.

Modes (offline experiments; default confirmation overrides never affect deployment):
  3d-config explicit profile parameters, including its confirmation flags
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
from dataclasses import fields, replace
import json
import hashlib
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

MODES = ('3d-config', '2d', '3d-fixed', '3d-floor', '3d-geometry', 'height')
VARIANT = {}   # rule switches under evaluation, set from --variant
HEIGHT_KEYS = dict(upright='upright_min_m', low='low_max_m', drop='drop_min_m', recovery='recovery_s', confirm='confirm_s',
                   hint='hint_upright_min_m', lost='lost_hold_s', sit='sit_hold_s', side='side_edge_px',
                   cont='continue_s', cont_min='continue_min_m', confirm_max='confirm_max_m')
IMAGE_WIDTH = 640   # recorder colour stream (record_gemini_mac.py)
MERGE_S = 1.5   # detections closer than this belong to the same event


def load_configs(path=None):
    params = yaml.safe_load((path or ROOT/'ros2_ws/src/perception_bringup/config/pose.yaml').read_text())['/**']['ros__parameters']
    pose = {f.name: params['pose_'+f.name] for f in fields(PoseConfig) if 'pose_'+f.name in params}
    spatial = {f.name: params['pose3d_'+f.name] for f in fields(Pose3DConfig) if 'pose3d_'+f.name in params}
    return PoseConfig(**(pose if path else dict(pose, upright_confirmed=True))), spatial


def load_height_config(path):
    params = yaml.safe_load(path.read_text())['/**']['ros__parameters']
    return HeightFallConfig(**{f.name: params['height_'+f.name] for f in fields(HeightFallConfig)
                              if 'height_'+f.name in params})


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


class Continuation:
    """Body top inside continuation boxes for tracks the detector lost (HeightFallConfig.continue_s).

    Without sot the box is the track's last detected box (depth hold, static camera). With sot a
    VitTrack per lost track starts from that box on the previous analysed frame and follows it on
    analysed frames only, as the node would. Diagnostic replay; nothing here is cached."""

    def __init__(self, recording, geometry_of, sot=False):
        self.recording, self.geometry_of = recording, geometry_of
        self.reg = Registration(json.loads((recording/'device.json').read_text()))
        k = self.reg.color
        self.intrinsics = (k['fx'], k['fy'], k['cx'], k['cy'])
        self.depths = {r['frame']: r for r in read_rows(recording/'timestamps.csv')}
        self.blank = np.zeros((k['height'], k['width'], 3), np.uint8)
        self.sot, self.trackers, self.frame, self.image, self.previous = sot, {}, None, None, None
        if sot:
            import cv2
            from yolo_person_tracker.sot import VitTrack
            self.cap, self.n = cv2.VideoCapture(str(recording/'color.avi')), 0
            self.model = lambda: VitTrack(str(ROOT/'models/weights/object_tracking_vittrack_2023sep.onnx'))

    def advance(self, frame):
        """Call once per analysed frame, in order."""
        self.frame = frame
        if self.sot:
            while self.n < frame['frame']:
                ok, raw = self.cap.read()
                if not ok:
                    raise RuntimeError(f'{self.recording.name}: video ends before frame {frame["frame"]}')
                self.n += 1
            self.previous, self.image = self.image, cv2_remap(raw, self.reg)

    def box(self, ident, last_box):
        if not self.sot:
            return last_box
        tracker = self.trackers.get(ident)
        if tracker is None:
            if self.previous is None:
                return None
            x1, y1, x2, y2 = map(int, last_box)
            tracker = self.trackers[ident] = self.model()
            tracker.init(self.previous, (x1, y1, x2-x1, y2-y1))
        (x, y, w, h), _ = tracker.update(self.image)
        return (x, y, x+w, y+h)

    def stop(self, keep):
        for ident in [i for i in self.trackers if i not in keep]:
            del self.trackers[ident]

    def top(self, box):
        f = self.frame
        geometry = self.geometry_of(f)
        if box is None or not f['depth_matched'] or geometry is None:
            return None
        _, aligned = self.reg.apply(self.blank, load_depth_m(self.recording, self.depths[f['depth_frame']]))
        measured = body_heights(aligned, box, self.intrinsics, geometry[0], geometry[1])
        return None if measured is None else measured[0]


def cv2_remap(raw, reg):
    import cv2
    return cv2.remap(raw, *reg.maps, cv2.INTER_LINEAR)


def recompute_depth(recording, frames, confidence, profile):
    """Rebuild threshold-dependent joints from raw depth; never reuse filtered joints."""
    from yolo_person_tracker.fusion import FusionConfig, fuse_depth
    params = yaml.safe_load(profile.read_text())['/**']['ros__parameters']
    cfg = FusionConfig(**{f.name: params['fusion_'+f.name] for f in fields(FusionConfig)
                          if 'fusion_'+f.name in params})
    if not cfg.enabled:
        raise ValueError('depth rebuild currently requires fusion_enabled')
    reader = Continuation(recording, lambda frame: None)
    for frame in frames:
        aligned = None
        if frame['depth_matched']:
            _, aligned = reader.reg.apply(reader.blank,
                load_depth_m(recording, reader.depths[frame['depth_frame']]))
        for i, person in enumerate(frame['people']):
            xyz = np.full((17, 3), np.nan)
            if aligned is not None:
                xyz = fuse_depth(aligned, person['box'], person['keypoints'], reader.intrinsics,
                    confidence=confidence, min_fraction=params.get('depth_min_fraction', .08),
                    other_boxes=[p['box'] for j,p in enumerate(frame['people']) if i != j], config=cfg).joints
            person['joints3d'] = [[float(v) if np.isfinite(v) else None for v in joint] for joint in xyz]
    return frames


def depth_cache_metadata(poses, profile, confidence):
    return dict(confidence=confidence,
                profile_sha256=hashlib.sha256(profile.read_bytes()).hexdigest(),
                frames_sha256=hashlib.sha256((poses/'frames.jsonl').read_bytes()).hexdigest())


def depth_cache_matches(poses, profile, confidence):
    marker = poses/'rebuilt-depth.json'
    if not marker.exists():
        return False
    return json.loads(marker.read_text()) == depth_cache_metadata(poses, profile, confidence)


def bottom_clipped(box, pixels, image_height=480):
    """Offline candidate only: withhold low height evidence at the image bottom."""
    if not math.isfinite(pixels) or not 0 <= pixels < image_height:
        raise ValueError('bottom edge must be finite and within the image')
    return pixels > 0 and float(box[3]) >= image_height-pixels


def combine_timelines(pose, height):
    """Per-person max as in pose_node, retaining continuation-only people."""
    rows = {}
    for row in pose + height:
        key = (row['t'], row['id'])
        if key not in rows or row['phase'] > rows[key]['phase']:
            rows[key] = dict(row)
    return [rows[key] for key in sorted(rows)]


def replay_height(frames, heights, continuation=None, postures=None, bottom_edge_px=0., height_config=None):
    overrides = {k: VARIANT[k] for k in HEIGHT_KEYS.values() if k in VARIANT}
    if 'veto_aspect' in VARIANT:
        overrides['veto_min_aspect'] = VARIANT['veto_aspect'] or 1e-6
    config = height_config or HeightFallConfig()
    tracker, boxes, timeline, epoch = HeightFallTracker(replace(config,
        transition_s=VARIANT.get('transition_s', config.transition_s), **overrides)), {}, [], None
    for f in frames:
        stamp = f['time_s']+1.
        present = {p['id']: tuple(p['box']) for p in f['people'] if p['id'] is not None}
        epochs = {i.split(':')[0] for i in present}
        if epochs and epoch not in epochs:
            tracker.reset(); boxes.clear(); epoch = min(epochs)
        if VARIANT.get('handover_s'):
            tracker.handover(stamp, present, boxes, VARIANT['handover_s'])
        states = {p['id']: (p.get('state') if postures is None else postures[(f['time_s'],p['id'])])
                  for p in f['people'] if p['id'] is not None}
        for ident, box in present.items():
            boxes[ident] = box
            # veto2d[=<r>]: a skeleton upright label vetoes only in a box r times taller than wide
            # (default HeightFallConfig.veto_min_aspect; veto2d=0 is the ungated veto).
            upright = VARIANT.get('veto2d', False) and states.get(ident) in ('STANDING', 'SITTING_CROUCHING')
            phase, detail = tracker.update(ident, stamp, heights.get((f['frame'], ident)),
                                           upright_hint=upright and skeleton_veto(True, box, tracker.cfg),
                                           posture_upright=upright and states.get(ident) == 'STANDING',
                                           clipped=clipped_at_top(box, tracker.cfg) or bottom_clipped(box, bottom_edge_px),
                                           side=at_side_edge(box, IMAGE_WIDTH, tracker.cfg))
            state = 'FALLEN' if phase == 2 else 'LYING' if 'drop' in detail or 'low' in detail else 'UNKNOWN'
            timeline.append(dict(t=f['time_s'], id=ident, state=state, phase=phase, basis='height',
                                 detail=detail[:160], height=float(box[3]-box[1])))
        if continuation is not None:
            continuation.advance(f)
            keep = tracker.continuable(stamp, present)
            continuation.stop(keep)
            for ident in keep:
                # boxes keeps the last detected box: handover matches against it, never the
                # continuation box (a tracker box following someone up moved alarms onto them).
                box = continuation.box(ident, boxes[ident])
                if box is None or tracker.yields(ident, box, present):
                    continue
                top = continuation.top(box)
                phase, detail = tracker.update(ident, stamp, top, continued=True,
                                               clipped=bottom_clipped(box, bottom_edge_px),
                                               side=at_side_edge(boxes[ident], IMAGE_WIDTH, tracker.cfg))
                state = 'FALLEN' if phase == 2 else 'LYING' if 'drop' in detail or 'low' in detail else 'UNKNOWN'
                timeline.append(dict(t=f['time_s'], id=ident, state=state, phase=phase, basis='height',
                                     detail=('continued; '+detail)[:160], height=0.))
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
    if mode == '3d-config':
        s = Pose3DConfig(**spatial)
    elif mode == '2d':
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
    parser.add_argument('--modes', nargs='+', choices=MODES, default=[m for m in MODES if m != "3d-config"])
    parser.add_argument('--geometry', type=Path,
                        help='per-frame gravity/camera height from estimate_camera_geometry.py (height, 3d-geometry)')
    parser.add_argument('--variant', default='baseline',
                        help="baseline, or comma list of gap_pause, knee_fallback, confirm_lying_only, veto2d, "
                             "handover=<s>, transition=<s>, upright=<m>, low=<m>, drop=<m>, recovery=<s>, confirm=<s>, hint=<m>, lost=<s>, sit=<s>, side=<px>, "
                             "cont=<s>, cont_min=<m>, confirm_max=<m>, sot (continuation boxes from VitTrack instead of the last box)")
    parser.add_argument('--pose-config', type=Path, help='Explicit profile, preserves its confirmation flags')
    parser.add_argument('--pose-confidence', type=float, help='Keypoint threshold, not detector confidence')
    parser.add_argument('--height-postures', choices=('2d','3d-geometry','3d-config'),
                        help='Recompute height-veto posture using current threshold instead of cached state')
    parser.add_argument('--bottom-edge-px', type=float, default=0., help='Offline candidate: low bottom-clipped heights are unmeasured; 0 off')
    args = parser.parse_args()
    bottom_clipped((0,0,1,1),args.bottom_edge_px)
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
        elif key in ('gap_pause', 'knee_fallback', 'confirm_lying_only', 'veto2d', 'sot'):
            VARIANT[key] = True
            if key == 'veto2d' and value:
                VARIANT['veto_aspect'] = float(value)
        else:
            parser.error(f'unknown variant {item}')
    args.output.mkdir(parents=True, exist_ok=True)
    frames = [json.loads(line) for line in (args.poses/'frames.jsonl').read_text().splitlines()]
    pose, spatial = load_configs(args.pose_config)
    if args.pose_confidence is not None:
        pose = replace(pose, confidence=args.pose_confidence)
        if 'height' in args.modes and not args.height_postures:
            parser.error('--pose-confidence with height requires --height-postures (cached posture is invalid)')
    uses_3d = any(mode.startswith('3d-') for mode in args.modes + [args.height_postures or ''])
    if uses_3d and (args.pose_config is not None or args.pose_confidence is not None):
        profile = args.pose_config or ROOT/'ros2_ws/src/perception_bringup/config/pose.yaml'
        if not depth_cache_matches(args.poses, profile, pose.confidence):
            frames = recompute_depth(args.recording, frames, pose.confidence, profile)
    floors = floor_fits(args.recording, frames, args.output/'floor.jsonl')
    floor = summarize_floor(floors)
    fixed = (floor['up'], floor['camera_height_m']) if 'up' in floor else None
    geometry = {}
    if args.geometry:
        for row in map(json.loads, args.geometry.read_text().splitlines()):
            if row['up'] is not None:
                geometry[row['frame']] = (np.array(row['up']), row['camera_height_m'])
    posture_rows = None
    if args.height_postures:
        if args.height_postures == '3d-geometry' and not geometry:
            parser.error('3d-geometry height posture requires geometry')
        posture_rows = replay(frames, floors, args.height_postures, pose, spatial, fixed, geometry)
    postures = None if posture_rows is None else {(r['t'],r['id']):r['state'] for r in posture_rows}
    timelines, report = {}, dict(pose_confidence=pose.confidence,height_postures=args.height_postures,
                                 bottom_edge_px=args.bottom_edge_px,pose_config=str(args.pose_config),recording=args.recording.name, frames=len(frames), floor=floor, variant=args.variant,
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
                geometry_of = lambda f: geometry.get(f['frame'])
                heights = person_heights(args.recording, frames, geometry_of, args.output/'heights-geometry.jsonl')
            elif fixed is not None:
                geometry_of = lambda f: fixed
                heights = person_heights(args.recording, frames, geometry_of, args.output/'heights.jsonl')
            else:
                report['modes'][mode] = 'skipped: no stable floor fit'
                continue
            height_config = load_height_config(args.pose_config) if args.pose_config else HeightFallConfig()
            continue_s = VARIANT.get('continue_s', height_config.continue_s)
            if VARIANT.get('sot') and not continue_s:
                parser.error('sot needs cont=<s>')
            continuation = (Continuation(args.recording, geometry_of, VARIANT.get('sot', False))
                            if continue_s else None)
            timeline = replay_height(frames, heights, continuation, postures, args.bottom_edge_px,
                                     height_config)
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
    if posture_rows is not None and 'height' in timelines:
        combined = combine_timelines(posture_rows, timelines['height'])
        report['modes']['combined'] = dict(suspected_events=events(combined, 1),
            confirmed_events=events(combined, 2), basis_3d_fraction=None)
        timelines['combined'] = combined
        with (args.output/'timeline-combined.jsonl').open('w') as handle:
            for row in combined: handle.write(json.dumps(row)+'\n')
    (args.output/'report.json').write_text(json.dumps(report, indent=2, ensure_ascii=False))
    if timelines:
        plot(args.output/'timeline.png', timelines, args.recording.name)
    print(json.dumps({m: (v if isinstance(v, str) else dict(suspected=len(v['suspected_events']), confirmed=len(v['confirmed_events']),
                                                           basis_3d=v['basis_3d_fraction']))
                      for m, v in report['modes'].items()}, ensure_ascii=False), '| floor', floor)


if __name__ == '__main__':
    main()
