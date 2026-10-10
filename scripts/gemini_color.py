#!/usr/bin/env python3
"""Gemini colour capture on the Mac without the reduced-mode blue cast.

The colour camera (firmware RD3013) often gives a strong cyan cast in its 640x480, 1280x960 and
1280x720 modes, while 2592x1944 and 1920x1080 are always normal (docs/Gemini彩色偏蓝排查20261010.md).
`ColorCapture` opens 2592x1944, crops the central 2560x1920 and shrinks it 4x to 640x480, which
covers the factory 640x480 field of view. The small offset between that crop and the factory
640x480 mode is measured once (`measure`) and turned into colour intrinsics for the recording.

    .venv/bin/python scripts/gemini_color.py measure     # point at a static, textured scene
"""
import argparse
import json
from pathlib import Path
import sys
import time

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
FULL, OUT = (2592, 1944), (640, 480)
CROP = ((FULL[0]-4*OUT[0])//2, (FULL[1]-4*OUT[1])//2)          # (16, 12): 2560x1920 centre
FACTORY = ROOT/'ros2_ws/src/perception_bringup/config/gemini_AY2755200PW.json'
MAPPING = ROOT/'scripts/config/gemini_color_mode_AY2755200PW.json'
# Measured 2026-10-10 by SIFT on one static scene: native 640x480 pixel = s * full pixel + t.
DEFAULT_MAPPING = dict(scale=.2501, tx=4.1, ty=.4, source='default-20261010-single-scene')
CAST_RB = .5     # whole-frame mean R/B below this is the reduced-mode cast, not a scene colour


def rb_ratio(frame):
    b, _, r = frame.reshape(-1, 3).mean(0)
    return float(r/max(b, 1.))


def shrink(frame):
    """2592x1944 frame -> 640x480 (central 2560x1920, area-averaged)."""
    x, y = CROP
    return cv2.resize(frame[y:y+4*OUT[1], x:x+4*OUT[0]], OUT, interpolation=cv2.INTER_AREA)


def load_mapping(path=MAPPING):
    if Path(path).exists():
        return json.loads(Path(path).read_text())
    return dict(DEFAULT_MAPPING)


def mode_intrinsics(factory, mapping):
    """Factory 640x480 intrinsics expressed in shrink() output pixels.

    native = s*X + t and output = (X - crop)/4, so output = (native - t)/(4s) - crop/4. Distortion
    coefficients act on normalised coordinates and stay unchanged."""
    s, tx, ty = mapping['scale'], mapping['tx'], mapping['ty']
    k = 1/(4*s)
    return dict(width=OUT[0], height=OUT[1], fx=factory['fx']*k, fy=factory['fy']*k,
                cx=(factory['cx']-tx)*k-CROP[0]/4, cy=(factory['cy']-ty)*k-CROP[1]/4)


class ColorCapture:
    """mode 'full': 2592x1944 shrunk to 640x480 (normal colour, ~25 fps); 'native': 640x480 at 30 fps."""

    def __init__(self, camera=0, mode='full'):
        if mode not in ('full', 'native'):
            raise ValueError(f'unknown colour mode {mode}')
        self.mode = mode
        self.capture = cv2.VideoCapture(camera, cv2.CAP_AVFOUNDATION)
        if not self.capture.isOpened():
            raise RuntimeError('打不开彩色相机：请在“终端”里运行（需要摄像头权限），或换 --camera 序号')
        size = FULL if mode == 'full' else OUT
        self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, size[0])
        self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, size[1])
        self.capture.set(cv2.CAP_PROP_FPS, 30)    # 640x480 otherwise opens at 60 fps and too dark
        ok, frame = self.capture.read()
        if not ok or frame.shape[1::-1] != size:
            got = None if not ok else frame.shape[1::-1]
            self.release()
            raise RuntimeError(f'彩色相机输出 {got}，需要 {size[0]}x{size[1]}：检查 --camera 是否为 Gemini')
        self.fps = 25. if mode == 'full' else 30.

    def read(self):
        ok, frame = self.capture.read()
        if not ok:
            return False, None
        return True, shrink(frame) if self.mode == 'full' else frame

    def release(self):
        self.capture.release()


def check_cast(capture, seconds=1.):
    """Mean R/B over the first frames; raises when the reduced-mode cast is present."""
    ratios, end = [], time.monotonic()+seconds
    while time.monotonic() < end:
        ok, frame = capture.read()
        if ok:
            ratios.append(rb_ratio(frame))
    if not ratios:
        raise RuntimeError('彩色读帧失败')
    ratio = float(np.median(ratios))
    if ratio < CAST_RB:
        raise RuntimeError(f'彩色画面严重偏蓝（R/B {ratio:.2f} < {CAST_RB}）：相机处于偏色模式，'
                           '重新运行；仍偏蓝时拔插相机，或检查 --color-mode')
    return ratio


def grab(camera, size, seconds=2.5):
    cap = cv2.VideoCapture(camera, cv2.CAP_AVFOUNDATION)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, size[0])
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, size[1])
    cap.set(cv2.CAP_PROP_FPS, 30)
    frames, end = [], time.monotonic()+seconds
    while time.monotonic() < end:
        ok, frame = cap.read()
        if ok and frame.shape[1::-1] == tuple(size):
            frames.append(frame)
    cap.release()
    if len(frames) < 5:
        raise RuntimeError(f'{size[0]}x{size[1]} 取帧失败')
    # Median of the last frames reduces sensor noise; geometry does not depend on the colour cast.
    return cv2.cvtColor(np.median(np.stack(frames[-5:]), axis=0).astype(np.uint8), cv2.COLOR_BGR2GRAY)


def fit_mapping(native, full):
    """native = s*full + t from SIFT matches (similarity, RANSAC)."""
    s0 = OUT[0]/FULL[0]
    small = cv2.resize(full, (round(FULL[0]*s0), round(FULL[1]*s0)), interpolation=cv2.INTER_AREA)
    sift = cv2.SIFT_create(6000)
    k1, d1 = sift.detectAndCompute(small, None)
    k2, d2 = sift.detectAndCompute(native, None)
    if d1 is None or d2 is None:
        raise RuntimeError('画面纹理太少，换一个有纹理的静止场景')
    matches = [a for a, b in cv2.BFMatcher().knnMatch(d1, d2, k=2) if a.distance < .7*b.distance]
    if len(matches) < 30:
        raise RuntimeError(f'匹配点太少（{len(matches)}），换一个有纹理的静止场景')
    p1 = np.float32([k1[m.queryIdx].pt for m in matches])/s0
    p2 = np.float32([k2[m.trainIdx].pt for m in matches])
    a, inliers = cv2.estimateAffinePartial2D(p1, p2, ransacReprojThreshold=1.)
    inliers = inliers.ravel().astype(bool)
    scale, angle = float(np.hypot(a[0, 0], a[1, 0])), float(np.degrees(np.arctan2(a[1, 0], a[0, 0])))
    residual = np.linalg.norm(p1[inliers]@a[:, :2].T+a[:, 2]-p2[inliers], axis=1)
    return dict(scale=scale, tx=float(a[0, 2]), ty=float(a[1, 2]), angle_deg=angle, matches=len(matches),
                inliers=int(inliers.sum()), residual_px=float(np.median(residual)))


def measure(camera, rounds=3):
    results = []
    for i in range(rounds):
        native = grab(camera, OUT)
        full = grab(camera, FULL)
        fit = fit_mapping(native, full)
        print(f'第 {i+1} 次：scale {fit["scale"]:.5f} t=({fit["tx"]:.2f}, {fit["ty"]:.2f}) 旋转 {fit["angle_deg"]:.3f}° '
              f'内点 {fit["inliers"]}/{fit["matches"]} 残差中位 {fit["residual_px"]:.2f}px', flush=True)
        results.append(fit)
    keys = ('scale', 'tx', 'ty')
    spread = {k: float(np.ptp([r[k] for r in results])) for k in keys}
    if spread['scale'] > .001 or spread['tx'] > 1. or spread['ty'] > 1. or any(abs(r['angle_deg']) > .1 for r in results):
        raise RuntimeError(f'三次结果不一致 {spread}：相机或场景在动，固定后重测')
    mapping = {k: float(np.median([r[k] for r in results])) for k in keys}
    mapping.update(source='measured', measured_at=time.strftime('%Y-%m-%d %H:%M'), rounds=results,
                   note='native 640x480 pixel = scale * 2592x1944 pixel + (tx, ty); SIFT, static scene')
    return mapping


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest='command', required=True)
    m = sub.add_parser('measure', help='测量 2592x1944 与出厂 640x480 模式的几何关系并保存')
    m.add_argument('--camera', type=int, default=0)
    m.add_argument('--output', type=Path, default=MAPPING)
    args = parser.parse_args()
    try:
        mapping = measure(args.camera)
    except RuntimeError as exc:
        sys.exit(str(exc))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(mapping, indent=2, ensure_ascii=False)+'\n')
    factory = json.loads(FACTORY.read_text())['color_intrinsic']
    print('已保存', args.output)
    print('录制时使用的彩色内参：', {k: round(v, 3) for k, v in mode_intrinsics(factory, mapping).items()})


if __name__ == '__main__':
    main()
