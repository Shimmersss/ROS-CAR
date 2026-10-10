#!/usr/bin/env python3
"""Per-frame camera gravity direction and height for a Gemini recording, without floor depth.

The 2026-10-09 corridor floor is glossy: its depth is mirrored wall, so floor-plane fits are
wrong, and the camera was moved during segments. This estimate uses neither:

  up      vanishing point of near-vertical line segments (lockers, door, walls) in each sampled
          colour frame (undistorted, CLAHE, LSD, person boxes masked, RANSAC on interpretation-plane
          normals), then the
          median over +-smooth_s; frames with few lines or a jump are left without geometry.
  height  standing people (pose cache state STANDING, both ankles confident): ankle rays scaled
          to the torso depth, height = ankle_height - ankle.up, median over +-height_window_s;
          falls back to the recording-wide median (or --default-height).

Writes JSON lines {t, up, camera_height_m, lines, height_samples}. Diagnostic only: assumes
vertical structure and an ankle height, and never sets any *_confirmed flag.
"""
import argparse
import csv
import json
from pathlib import Path

import cv2
import numpy as np


class VerticalVanishing:
    def __init__(self, K, dist, min_length=25., max_tilt_deg=20., margin_px=20, tolerance_deg=.7, seed=0):
        self.K, self.dist, self.Ki = K, dist, np.linalg.inv(K)
        self.min_length, self.max_tilt, self.margin = min_length, max_tilt_deg, margin_px
        self.tolerance = np.sin(np.radians(tolerance_deg))
        self.lsd = cv2.createLineSegmentDetector()
        self.clahe = cv2.createCLAHE(2., (8, 8))
        self.rng = np.random.default_rng(seed)

    def __call__(self, image, boxes=(), iterations=300, box_margin_px=10):
        """(up unit vector in the colour optical frame, inlier line count) or None.

        boxes: person boxes (raw pixels) whose lines are ignored; limbs near the camera
        otherwise outweigh the background structure."""
        gray = self.clahe.apply(cv2.cvtColor(cv2.undistort(image, self.K, self.dist), cv2.COLOR_BGR2GRAY))
        found = self.lsd.detect(gray)[0]
        if found is None:
            return None
        segs = found.reshape(-1, 4)
        h, w = gray.shape
        m = self.margin    # undistortion leaves straight black borders: ignore lines near them
        segs = segs[(segs[:, [0, 2]].min(1) > m) & (segs[:, [0, 2]].max(1) < w-m)
                    & (segs[:, [1, 3]].min(1) > m) & (segs[:, [1, 3]].max(1) < h-m)]
        if len(boxes):
            corners = np.array([[x1-box_margin_px, y1-box_margin_px, x2+box_margin_px, y2+box_margin_px]
                                for x1, y1, x2, y2 in boxes], float)
            # Box corners into the undistorted image, the frame the segments are in.
            pts = cv2.undistortPoints(corners.reshape(-1, 1, 2), self.K, self.dist, P=self.K).reshape(-1, 4)
            mid = (segs[:, :2]+segs[:, 2:])/2
            inside = np.zeros(len(segs), bool)
            for x1, y1, x2, y2 in pts:
                inside |= (mid[:, 0] >= x1) & (mid[:, 0] <= x2) & (mid[:, 1] >= y1) & (mid[:, 1] <= y2)
            segs = segs[~inside]
        d = segs[:, 2:]-segs[:, :2]
        length = np.hypot(d[:, 0], d[:, 1])
        keep = (length > self.min_length) & (np.degrees(np.arctan2(np.abs(d[:, 0]), np.abs(d[:, 1]))) < self.max_tilt)
        segs, length = segs[keep], length[keep]
        if len(segs) < 8:
            return None
        a = (self.Ki@np.c_[segs[:, :2], np.ones(len(segs))].T).T
        b = (self.Ki@np.c_[segs[:, 2:], np.ones(len(segs))].T).T
        normals = np.cross(a, b)
        normals /= np.linalg.norm(normals, axis=1, keepdims=True)   # a 3D vertical is orthogonal to each
        best = None
        for _ in range(iterations):
            i, j = self.rng.choice(len(normals), 2, replace=False)
            u = np.cross(normals[i], normals[j])
            if np.linalg.norm(u) < 1e-6:
                continue
            inliers = np.abs(normals@(u/np.linalg.norm(u))) < self.tolerance
            score = length[inliers].sum()
            if best is None or score > best[0]:
                best = (score, inliers)
        if best is None or best[1].sum() < 8:
            return None
        inliers = best[1]
        up = np.linalg.svd(normals[inliers]*length[inliers, None])[2][-1]
        return (-up if up[1] > 0 else up), int(inliers.sum())


def angles(up):
    """Pitch up (+ = camera tilted up) and roll in degrees for an up vector in the optical frame."""
    x, y, z = up
    return float(np.degrees(np.arcsin(np.clip(z, -1, 1)))), float(np.degrees(np.arctan2(x, -y)))


def smooth_up(samples, t, smooth_s, min_lines, max_spread_deg):
    near = [(u, n) for ts, u, n in samples if abs(ts-t) <= smooth_s and n >= min_lines]
    if len(near) < 2:
        return None
    ups = np.array([u for u, _ in near])
    up = np.median(ups, axis=0)
    up /= np.linalg.norm(up)
    spread = np.degrees(np.arccos(np.clip(ups@up, -1, 1)))
    # Camera being moved: neighbouring estimates disagree.
    return up if np.median(spread) <= max_spread_deg else None


def ankle_heights(frames, up_at, K, dist, ankle_height_m, min_confidence=.5):
    out = []
    for f in frames:
        up = up_at(f['time_s'])
        if up is None:
            continue
        for p in f['people']:
            if p.get('state') != 'STANDING' or not p.get('target') or not np.isfinite(p['target'][2]):
                continue
            kp = np.asarray(p['keypoints'], float)
            if kp[15, 2] < min_confidence or kp[16, 2] < min_confidence:
                continue
            rays = cv2.undistortPoints(kp[[15, 16], :2].reshape(-1, 1, 2), K, dist).reshape(-1, 2)
            points = np.c_[rays, np.ones(2)]*p['target'][2]
            out.append((f['time_s'], ankle_height_m-float((points@up).mean())))
    return out


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('recording', type=Path)
    parser.add_argument('poses', type=Path, help='frames.jsonl pose cache of the same recording')
    parser.add_argument('--output', type=Path, required=True, help='JSON lines, one per pose-cache frame')
    parser.add_argument('--stride', type=int, default=15, help='colour frames between vanishing-point samples')
    parser.add_argument('--smooth', type=float, default=1.5, help='+- seconds for the median gravity')
    parser.add_argument('--min-lines', type=int, default=20)
    parser.add_argument('--max-spread', type=float, default=3., help='deg; larger means the camera was moving')
    parser.add_argument('--ankle-height', type=float, default=.08)
    parser.add_argument('--height-window', type=float, default=5.)
    parser.add_argument('--default-height', type=float,
                        help='camera height when no standing ankles are near (default: recording-wide ankle median)')
    args = parser.parse_args()
    device = json.loads((args.recording/'device.json').read_text())
    c = device['color_intrinsic']
    K = np.array([[c['fx'], 0, c['cx']], [0, c['fy'], c['cy']], [0, 0, 1.]])
    dist = np.asarray(device['color_distortion'][:5], float)
    with (args.recording/'color_timestamps.csv').open(newline='') as handle:
        stamps = [int(r['host_monotonic_ns']) for r in csv.DictReader(handle)]
    frames = [json.loads(line) for line in args.poses.read_text().splitlines()]
    boxes_at = {f['frame']: [p['box'] for p in f['people']] for f in frames}
    vanishing, samples = VerticalVanishing(K, dist), []
    video, index = cv2.VideoCapture(str(args.recording/'color.avi')), 0
    while True:
        ok, image = video.read()
        if not ok:
            break
        if index % args.stride == 0:
            near = min(boxes_at, key=lambda k: abs(k-(index+1))) if boxes_at else None
            found = vanishing(image, boxes_at.get(near, []) if near and abs(near-index-1) <= 2 else [])
            if found:
                samples.append(((stamps[index]-stamps[0])/1e9, found[0], found[1]))
        index += 1

    def up_at(t):
        return smooth_up(samples, t, args.smooth, args.min_lines, args.max_spread)

    heights = ankle_heights(frames, up_at, K, dist, args.ankle_height)
    times = np.array([t for t, _ in heights]) if heights else np.zeros(0)
    values = np.array([h for _, h in heights]) if heights else np.zeros(0)
    fallback = args.default_height if args.default_height else (float(np.median(values)) if len(values) else None)
    rows, with_geometry = [], 0
    for f in frames:
        t = f['time_s']
        up = up_at(t)
        near = values[np.abs(times-t) <= args.height_window] if len(times) else values
        height = float(np.median(near)) if len(near) >= 5 else fallback
        row = dict(frame=f['frame'], t=round(t, 3), up=None, camera_height_m=None,
                   height_samples=int(len(near)))
        if up is not None and height is not None:
            pitch, roll = angles(up)
            row.update(up=[round(float(v), 5) for v in up], camera_height_m=round(height, 3),
                       pitch_up_deg=round(pitch, 2), roll_deg=round(roll, 2))
            with_geometry += 1
        rows.append(row)
    args.output.write_text(''.join(json.dumps(r)+'\n' for r in rows))
    valid = [r for r in rows if r['up'] is not None]
    summary = dict(recording=args.recording.name, frames=len(rows), with_geometry=with_geometry,
                   vanishing_samples=len(samples), ankle_samples=len(heights))
    if valid:
        for key in ('camera_height_m', 'pitch_up_deg', 'roll_deg'):
            v = np.array([r[key] for r in valid])
            summary[key] = dict(p5=round(float(np.percentile(v, 5)), 3), median=round(float(np.median(v)), 3),
                                p95=round(float(np.percentile(v, 95)), 3))
    print(json.dumps(summary))


if __name__ == '__main__':
    main()
