#!/usr/bin/env python3
"""Live colour-cast check for the Gemini colour camera on the Mac (run from Terminal: needs camera
permission). Shows the mean B/G/R of the whole frame and of the centre patch (hold a sheet of white
paper there) and lets you switch white balance through standard UVC controls (scripts/uvc_controls.c).

--sweep runs without a window: auto white balance, then manual 2800-6500 K, saves a contact sheet
and restores the defaults.

Keys: a toggle auto white balance | [ / ] manual white balance -/+ 200 K | d defaults (auto)
      s save frame + values to artifacts/color-probe/ | q quit
"""
import argparse
from pathlib import Path
import subprocess
import sys
import time

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
SOURCE, TOOL = ROOT/'scripts/uvc_controls.c', ROOT/'artifacts/build/uvc_controls'
OUT = ROOT/'artifacts/color-probe'


def build_tool():
    if TOOL.exists() and TOOL.stat().st_mtime >= SOURCE.stat().st_mtime:
        return
    TOOL.parent.mkdir(parents=True, exist_ok=True)
    subprocess.run(['clang', '-O2', str(SOURCE), '-I/opt/homebrew/include/libusb-1.0', '-L/opt/homebrew/lib',
                    '-lusb-1.0', '-Wl,-rpath,/opt/homebrew/lib', '-o', str(TOOL)], check=True)


def controls(*sets):
    """Apply NAME=VALUE sets and return the current {name: value} of the colour camera."""
    args = [str(TOOL)]
    for item in sets:
        args += ['--set', item]
    text = subprocess.run(args, capture_output=True, text=True).stdout
    values = {}
    for line in text.splitlines():
        parts = line.replace(',', ' ').split()
        if len(parts) >= 2 and parts[0] in ('awb', 'wb', 'ae', 'exposure', 'gain') and parts[1].lstrip('-').isdigit():
            values[parts[0]] = int(parts[1])
    return values


def means(frame):
    h, w = frame.shape[:2]
    centre = frame[h//2-50:h//2+50, w//2-50:w//2+50]
    return frame.reshape(-1, 3).mean(0), centre.reshape(-1, 3).mean(0)


def verdict(bgr):
    b, g, r = bgr
    ratio = r/max(b, 1)
    return ratio, 'blue cast' if ratio < .7 else 'red/yellow cast' if ratio > 1.4 else 'neutral'


def sweep(camera):
    """Auto WB, then each manual temperature: settle, measure, save one contact sheet."""
    settings = [('auto', ('wb=4600', 'awb=1'))]+[(f'{k}K', ('awb=0', f'wb={k}')) for k in range(2800, 6600, 600)]
    tiles, rows = [], []
    try:
        for label, sets in settings:
            controls(*sets)
            end = time.monotonic()+2.
            while time.monotonic() < end:
                ok, frame = camera.read()
            if not ok:
                raise RuntimeError('读帧失败')
            whole, centre = means(frame)
            ratio, kind = verdict(whole)
            rows.append(f'{label:6s} frame B/G/R {whole.round(0)} R/B {ratio:.2f} {kind}; centre R/B {verdict(centre)[0]:.2f}')
            print(rows[-1], flush=True)
            tile = cv2.resize(frame, (320, 240))
            cv2.putText(tile, f'{label} R/B {ratio:.2f}', (6, 20), cv2.FONT_HERSHEY_SIMPLEX, .6, (0, 255, 255), 2)
            tiles.append(tile)
    finally:
        print('恢复默认：', controls('wb=4600', 'awb=1'), flush=True)
    while len(tiles) % 4:
        tiles.append(np.zeros_like(tiles[0]))
    sheet = np.vstack([np.hstack(tiles[i:i+4]) for i in range(0, len(tiles), 4)])
    stamp = time.strftime('%H%M%S')
    cv2.imwrite(str(OUT/f'sweep-{stamp}.jpg'), sheet)
    (OUT/f'sweep-{stamp}.txt').write_text('\n'.join(rows)+'\n')
    print(f'已保存 {OUT}/sweep-{stamp}.jpg', flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--camera', type=int, default=0)
    parser.add_argument('--sweep', action='store_true', help='measure auto and manual white balance, no window')
    args = parser.parse_args()
    build_tool()
    values = controls()
    if not values:
        sys.exit('读不到 UVC 控制：确认 Gemini 已连接（scripts/uvc_controls.c）')
    print('相机当前设置：', values, flush=True)
    camera = cv2.VideoCapture(args.camera, cv2.CAP_AVFOUNDATION)
    if not camera.isOpened():
        sys.exit('打不开彩色相机：请在“终端”里运行（需要摄像头权限），或换 --camera 序号')
    camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    OUT.mkdir(parents=True, exist_ok=True)
    if args.sweep:
        sweep(camera)
        camera.release()
        return
    title = 'colour check - q to quit'
    cv2.namedWindow(title)
    logged = 0.
    while True:
        ok, frame = camera.read()
        if not ok:
            print('读帧失败', flush=True)
            break
        whole, centre = means(frame)
        ratio, kind = verdict(whole)
        c_ratio, c_kind = verdict(centre)
        shown = frame.copy()
        h, w = shown.shape[:2]
        cv2.rectangle(shown, (w//2-50, h//2-50), (w//2+50, h//2+50), (0, 255, 255), 1)
        awb = 'auto' if values.get('awb') else f"manual {values.get('wb')}K"
        lines = [f'{w}x{h}  white balance {awb}',
                 f'frame B/G/R {whole[0]:.0f}/{whole[1]:.0f}/{whole[2]:.0f}  R/B {ratio:.2f} {kind}',
                 f'centre B/G/R {centre[0]:.0f}/{centre[1]:.0f}/{centre[2]:.0f}  R/B {c_ratio:.2f} {c_kind}',
                 'a auto WB | [ ] manual WB -/+200K | d defaults | s save | q quit']
        for i, line in enumerate(lines):
            cv2.putText(shown, line, (8, 22+20*i), cv2.FONT_HERSHEY_SIMPLEX, .5, (0, 0, 0), 3)
            cv2.putText(shown, line, (8, 22+20*i), cv2.FONT_HERSHEY_SIMPLEX, .5, (0, 255, 255), 1)
        cv2.imshow(title, shown)
        if time.monotonic()-logged > 2:
            logged = time.monotonic()
            print(f'{awb:14s} frame R/B {ratio:.2f} ({kind})  centre R/B {c_ratio:.2f} ({c_kind})', flush=True)
        key = cv2.waitKey(1) & 255
        if key in (ord('q'), 27):
            break
        if key == ord('a'):
            values = controls(f"awb={0 if values.get('awb') else 1}")
        elif key in (ord('['), ord(']')):
            wb = min(6500, max(2800, values.get('wb', 4600)+(200 if key == ord(']') else -200)))
            values = controls('awb=0', f'wb={wb}')
        elif key == ord('d'):
            values = controls('wb=4600', 'awb=1')
        elif key == ord('s'):
            stamp = time.strftime('%H%M%S')
            cv2.imwrite(str(OUT/f'frame-{stamp}.jpg'), frame)
            (OUT/f'frame-{stamp}.txt').write_text(f'{values}\nframe {whole.round(1)} centre {centre.round(1)}\n')
            print(f'已保存 {OUT}/frame-{stamp}.jpg', flush=True)
        if key in (ord('a'), ord('['), ord(']'), ord('d')):
            print('设置改为：', values, flush=True)
    camera.release()
    cv2.destroyAllWindows()
    print('退出时的相机设置：', controls(), '（按 d 可恢复默认自动白平衡）')


if __name__ == '__main__':
    main()
