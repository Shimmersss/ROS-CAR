#!/usr/bin/env python3
"""Record one Gemini segment on the Mac: SDK native depth (C++ helper) + UVC colour (OpenCV).

Run from Terminal (it needs camera permission). Q/Esc or Ctrl-C stops early. Output goes to
data/recordings/gemini-YYYYMMDD-<segment>/ and matches scripts/analyze_gemini_recording.py.
Host receive timestamps only: this is not exposure-synchronised RGB-D.
"""
import argparse
import csv
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import threading
import time
import zlib

import cv2
import numpy as np

from gemini_recording import DEPTH_SHAPE, depth_path, write_summary

ROOT = Path(__file__).resolve().parents[1]
SDK = Path(os.environ.get('ORBBEC_SDK_ROOT', Path.home()/'.local/share/roscar/orbbec-sdk-v1.10.16'))/'SDK'
SOURCE = ROOT/'scripts/record_gemini_depth.cpp'
BINARY = ROOT/'artifacts/build/record_gemini_depth'
# Measured on the 10-07 recording: zlib level 1 shrinks depth ~7x (500 KiB -> ~70 KiB/frame).
BYTES_PER_SECOND = {'zlib': 2.5e6+2e6, 'raw': 15.5e6+2e6}
NOTES = """# {name}

- 段号/名称：{segment}
- 日期时间：{when}
- 设备序列号：见 device.json
- 相机镜头中心离地高度（cm）：
- 俯仰角（°，抬头为正）：
- 横滚角（°）：
- 场地与地面材质：
- 参与者与衣着（上衣/裤子颜色）：
- 防护垫厚度（cm）、朝向（横放/纵放）、位置（距相机 m）：
- 椅子位置：
- 异常情况（误入画面、中断、重录等）：
"""


def build_helper():
    if BINARY.exists() and BINARY.stat().st_mtime >= SOURCE.stat().st_mtime:
        return
    if not (SDK/'lib').exists():
        sys.exit(f'找不到 Orbbec SDK v1.10.x：{SDK}（可设置 ORBBEC_SDK_ROOT）')
    BINARY.parent.mkdir(parents=True, exist_ok=True)
    subprocess.run(['clang++', '-std=c++17', '-O2', f'-I{SDK}/include', str(SOURCE), f'-L{SDK}/lib',
                    '-lOrbbecSDK', '-lz', f'-Wl,-rpath,{SDK}/lib', '-o', str(BINARY)], check=True)


class DepthProcess:
    """Runs the SDK helper and follows its progress lines."""

    def __init__(self, out, seconds, raw, serial):
        env = dict(os.environ, DYLD_LIBRARY_PATH=str(SDK/'lib'))
        args = [str(BINARY), str(out), str(seconds+15)]
        if raw:
            args.append('--raw')
        if serial:
            args += ['--serial', serial]
        # The SDK writes its Log directory relative to the working directory.
        self.proc = subprocess.Popen(args, cwd=out, env=env, stdout=subprocess.PIPE,
                                     stderr=subprocess.STDOUT, text=True, bufsize=1)
        self.out, self.started, self.frames, self.log = out, False, 0, []
        threading.Thread(target=self._follow, daemon=True).start()

    def _follow(self):
        for line in self.proc.stdout:
            line = line.strip()
            self.log.append(line)
            if line.startswith('DEPTH_STARTED'):
                self.started = True
            elif line.startswith(('DEPTH_FRAMES', 'DEPTH_STOPPING', 'DEPTH_DONE')):
                self.frames = int(line.split()[-1])

    def stop(self):
        (self.out/'stop').touch()
        try:
            self.proc.wait(timeout=8)
        except subprocess.TimeoutExpired:
            # Seen on 10-07: the SDK can hang on a USB timeout while stopping.
            self.proc.send_signal(signal.SIGTERM)
            try:
                self.proc.wait(timeout=4)
            except subprocess.TimeoutExpired:
                self.proc.kill()
                self.proc.wait()
                self.log.append('helper killed after stop timeout')
        (self.out/'stop').unlink(missing_ok=True)


def depth_preview(out, frame):
    try:
        path = depth_path(out, frame)
        data = path.read_bytes()
        if path.suffix == '.z':
            data = zlib.decompress(data)
        depth = np.frombuffer(data, np.uint16).reshape(DEPTH_SHAPE)
    except (OSError, ValueError, zlib.error):
        return None
    scaled = np.clip(depth.astype(np.float32)/5000.*255, 0, 255).astype(np.uint8)
    image = cv2.applyColorMap(scaled, cv2.COLORMAP_TURBO)
    image[depth == 0] = 0
    return cv2.resize(image, (320, 200))


# docs/补录视频要求20261008.md: code -> (directory name, default seconds).
PLAN = {
    'A0': ('A0-setup-check', 60), 'A1a': ('A1a-fall-side-1.9m', 120), 'A1b': ('A1b-fall-toward-1.9m', 120),
    'A1c': ('A1c-fall-walking-away', 120), 'A2': ('A2-fall-types-1.9m', 160), 'A3': ('A3-fall-3m', 120),
    'A4a': ('A4a-negatives-fast', 120), 'A4b': ('A4b-negatives-floor', 120), 'A5': ('A5-daily-postures', 120),
    'A6': ('A6-dark-pants', 120), 'A7': ('A7-occlusion-edge', 120), 'A8': ('A8-second-person', 170),
    'B1': ('B1-reid-distinct', 130), 'B2': ('B2-reid-similar', 130),
}


def resolve_segment(segment, root, day):
    """Plan code (case-insensitive) or a free name -> (unused directory, default seconds)."""
    code = next((k for k in PLAN if k.lower() == segment.lower()), None)
    name, seconds = PLAN[code] if code else (segment, 120)
    if not name or '/' in name or name.startswith('.'):
        raise ValueError(f'invalid segment name: {segment!r}')
    base = f'gemini-{day}-{name}'
    out, part = Path(root)/base, 1
    while out.exists() and any(out.iterdir()):
        part += 1
        out = Path(root)/f'{base}-part{part}'
    return out, seconds


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('segment', help='补录段号（如 A1a、B1，见 PLAN）或自定义段名；同名已存在时自动加 -part2…')
    parser.add_argument('--seconds', type=float, help='最长时长，≤170 秒；默认按段号设定，自定义段名为 120')
    parser.add_argument('--camera', type=int, default=0, help='AVFoundation 彩色相机序号')
    parser.add_argument('--root', type=Path, default=ROOT/'data/recordings')
    parser.add_argument('--serial', default='AY2755200PW', help='空字符串表示不核对序列号')
    parser.add_argument('--raw', action='store_true', help='深度不压缩（每段约 1.8 GB）')
    parser.add_argument('--no-preview', action='store_true')
    args = parser.parse_args()
    try:
        out, planned = resolve_segment(args.segment, args.root, time.strftime('%Y%m%d'))
    except ValueError as exc:
        parser.error(str(exc))
    if args.seconds is None:
        args.seconds = float(planned)
    if not 0 < args.seconds <= 170:
        parser.error('--seconds must be in (0, 170]')
    name = out.name
    print(f'本段目录：{out}（最长 {args.seconds:.0f} 秒）', flush=True)
    need = BYTES_PER_SECOND['raw' if args.raw else 'zlib']*args.seconds*1.5
    free = shutil.disk_usage(args.root if args.root.exists() else ROOT).free
    if free < need+2e9:
        sys.exit(f'磁盘不足：可用 {free/1e9:.1f} GB，本段预计需要 {need/1e9:.1f} GB 并保留 2 GB')
    build_helper()
    out.mkdir(parents=True, exist_ok=True)
    (out/'notes.md').write_text(NOTES.format(name=name, segment=name, when=time.strftime('%Y-%m-%d %H:%M')))

    camera = cv2.VideoCapture(args.camera, cv2.CAP_AVFOUNDATION)
    if not camera.isOpened():
        sys.exit('打不开彩色相机：请在“终端”里运行（需要摄像头权限），或换 --camera 序号')
    camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    camera.set(cv2.CAP_PROP_FPS, 30)
    ok, frame = camera.read()
    if not ok or frame.shape[:2] != (480, 640):
        sys.exit(f'彩色相机输出 {None if not ok else frame.shape[1::-1]}，需要 640×480：检查 --camera 是否为 Gemini')

    depth = DepthProcess(out, args.seconds, args.raw, args.serial)
    deadline = time.monotonic()+15
    while time.monotonic() < deadline and depth.proc.poll() is None and depth.frames == 0:
        time.sleep(.1)
    if depth.frames == 0:
        depth.stop()
        camera.release()
        sys.exit('深度流没有出帧：\n'+'\n'.join(depth.log[-10:]))
    print(f'深度已出帧，开始录制 {name}（最长 {args.seconds:.0f} 秒，Q/Esc 提前结束）', flush=True)

    if not args.no_preview:
        # Create the window before timing starts: the first imshow stalls ~0.2 s on macOS.
        cv2.namedWindow('Gemini recording - Q to stop')
        cv2.imshow('Gemini recording - Q to stop', frame)
        cv2.waitKey(1)
        for _ in range(3):
            camera.read()
    writer = cv2.VideoWriter(str(out/'color.avi'), cv2.VideoWriter_fourcc(*'MJPG'), 30, (640, 480))
    stopping = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stopping.set())
    start, count, preview_depth, preview_at = time.monotonic(), 0, None, 0.
    with (out/'color_timestamps.csv').open('w', newline='') as handle:
        rows = csv.writer(handle)
        rows.writerow(['frame', 'host_wall_ns', 'host_monotonic_ns'])
        while not stopping.is_set() and time.monotonic()-start < args.seconds:
            ok, frame = camera.read()
            wall, mono = time.time_ns(), time.monotonic_ns()
            if not ok:
                print('彩色读帧失败，停止', flush=True)
                break
            count += 1
            writer.write(frame)
            rows.writerow([count, wall, mono])
            if depth.proc.poll() is not None:
                print('深度进程意外退出，停止：\n'+'\n'.join(depth.log[-5:]), flush=True)
                break
            if not args.no_preview and count % 3 == 0:
                if time.monotonic()-preview_at > .5 and depth.frames > 1:
                    preview_depth, preview_at = depth_preview(out, depth.frames-1), time.monotonic()
                shown = frame.copy()
                elapsed = time.monotonic()-start
                text = f'{elapsed:5.1f}/{args.seconds:.0f}s  color {count}  depth {depth.frames}'
                cv2.putText(shown, text, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, .6, (0, 255, 255), 2)
                if preview_depth is not None:
                    shown[480-200:, 640-320:] = preview_depth
                cv2.imshow('Gemini recording - Q to stop', shown)
                if cv2.waitKey(1) & 255 in (ord('q'), 27):
                    break
    writer.release()
    camera.release()
    cv2.destroyAllWindows()
    depth.stop()
    summary = write_summary(out, {'segment': name, 'depth_helper_log': depth.log[-20:],
                                  'depth_storage': 'raw' if args.raw else 'zlib'})
    print(f'彩色 {summary["color_frames"]} 帧，深度 {summary["depth_rows"]} 帧，重叠 {summary["overlap_seconds"]} 秒，'
          f'最大间断 彩色 {summary["color_max_gap_seconds"]} / 深度 {summary["depth_max_gap_seconds"]} 秒，'
          f'占用 {summary["disk_bytes"]/1e9:.2f} GB')
    if summary['problems']:
        print('有问题：'+'；'.join(summary['problems']))
    print(f'请填写 {out/"notes.md"}，再用 scripts/label_recording.py 标注。')
    return 1 if summary['problems'] else 0


if __name__ == '__main__':
    sys.exit(main())
