#!/usr/bin/env python3
"""Record one Gemini segment on the Mac: SDK native depth (C++ helper) + UVC colour (OpenCV).

Run from Terminal (it needs camera permission). Q/Esc or Ctrl-C stops early. Output goes to
data/recordings/gemini-YYYYMMDD-<segment>/ and matches scripts/analyze_gemini_recording.py.
Host receive timestamps only: this is not exposure-synchronised RGB-D.

Colour defaults to 2592x1944 shrunk to 640x480 (~25 fps): the native 640x480 mode often has a
strong blue cast (docs/Gemini彩色偏蓝排查20261010.md). device.json then carries the matching
colour intrinsics; the factory ones are kept as factory_color_intrinsic.
"""
import json
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

from gemini_color import CAST_RB, ColorCapture, check_cast, load_mapping, mode_intrinsics, rb_ratio
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
- 相机镜头中心离地高度（cm）：{height}
- 俯仰角（°，抬头为正）：{pitch}
- 横滚角（°）：{roll}
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
# docs/重拍视频要求20261010.md: code -> (directory name, default seconds). A1 holds 5 falls of
# ~26 s each (stand 4 s, fall, lie 4 s, get up, stand 4 s, walk back).
PLAN = {
    'A0': ('A0-setup-check', 60), 'A1a': ('A1a-fall-side-1.9m', 150), 'A1b': ('A1b-fall-toward-1.9m', 150),
    'A1c': ('A1c-fall-walking-away', 150), 'A2a': ('A2a-fall-back-stumble-collapse', 150),
    'A2b': ('A2b-fall-backward-sit-chair', 150), 'A3': ('A3-fall-3m', 150),
    'A4a': ('A4a-negatives-fast', 170), 'A4b': ('A4b-negatives-floor', 150), 'A5': ('A5-daily-postures', 130),
    'A6': ('A6-dark-pants', 150), 'A7': ('A7-occlusion-edge', 120), 'A8': ('A8-second-person', 170),
    'B1': ('B1-reid-distinct', 130), 'B2': ('B2-reid-similar', 130),
}
SETUP_FIELDS = (('height_cm', '相机高度', 'cm'), ('pitch_deg', '俯仰', '°'), ('roll_deg', '横滚', '°'))


def setup_path(root, day):
    return Path(root)/f'camera-setup-{day}.json'


def camera_setup(root, day, values=None):
    """Today's measured camera mount: saves `values` (height_cm, pitch_deg, roll_deg) when given,
    otherwise returns the saved one or None. The camera must not move within a session."""
    path = setup_path(root, day)
    if values is not None:
        height, pitch, roll = map(float, values)
        if not (5 <= height <= 200 and -45 <= pitch <= 45 and -20 <= roll <= 20):
            raise ValueError(f'camera setup out of range: height {height} cm, pitch {pitch}°, roll {roll}°')
        setup = dict(height_cm=height, pitch_deg=pitch, roll_deg=roll, saved_at=time.strftime('%Y-%m-%d %H:%M'))
        Path(root).mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(setup, ensure_ascii=False, indent=1))
        return setup
    return json.loads(path.read_text()) if path.exists() else None


def notes_text(name, when, setup):
    def value(key):
        return '' if setup is None else f'{setup[key]:g}'
    return NOTES.format(name=name, segment=name, when=when, height=value('height_cm'),
                        pitch=value('pitch_deg'), roll=value('roll_deg'))


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


def patch_device_json(out, mode, mapping):
    """Record the colour capture mode; in full mode replace color_intrinsic with the shrunk-crop one."""
    path = out/'device.json'
    if not path.exists():
        return
    device = json.loads(path.read_text())
    capture = dict(mode=mode, native_cast_threshold_rb=CAST_RB)
    if mode == 'full':
        device.setdefault('factory_color_intrinsic', device['color_intrinsic'])
        device['color_intrinsic'] = mode_intrinsics(device['factory_color_intrinsic'], mapping)
        capture.update(source_size=[2592, 1944], crop=[16, 12, 2560, 1920], output_size=[640, 480],
                       mapping=mapping)
    device['color_capture'] = capture
    path.write_text(json.dumps(device))


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('segment', nargs='?', help='补录段号（如 A1a、B1，见 PLAN）或自定义段名；同名已存在时自动加 -part2…')
    parser.add_argument('--set-camera', nargs=3, type=float, metavar=('HEIGHT_CM', 'PITCH_DEG', 'ROLL_DEG'),
                        help='记录今天实测的镜头离地高度、俯仰（抬头为正）、横滚，之后各段自动写入 notes.md')
    parser.add_argument('--seconds', type=float, help='最长时长，≤170 秒；默认按段号设定，自定义段名为 120')
    parser.add_argument('--camera', type=int, default=0, help='AVFoundation 彩色相机序号')
    parser.add_argument('--root', type=Path, default=ROOT/'data/recordings')
    parser.add_argument('--serial', default='AY2755200PW', help='空字符串表示不核对序列号')
    parser.add_argument('--raw', action='store_true', help='深度不压缩（每段约 1.8 GB）')
    parser.add_argument('--no-preview', action='store_true')
    parser.add_argument('--color-mode', choices=('full', 'native'), default='full',
                        help='full：2592x1944 裁剪缩小到 640x480（默认，颜色正常，约 25 fps）；native：原 640x480 模式（常偏蓝）')
    parser.add_argument('--allow-color-cast', action='store_true', help='开录前检测到严重偏蓝也继续（不建议）')
    args = parser.parse_args()
    day = time.strftime('%Y%m%d')
    if args.set_camera:
        try:
            setup = camera_setup(args.root, day, args.set_camera)
        except ValueError as exc:
            parser.error(str(exc))
        print('已记录今天的相机架设：'+'，'.join(f'{label} {setup[k]:g}{unit}' for k, label, unit in SETUP_FIELDS)
              + f'（{setup_path(args.root, day)}）。场次中途不要再动相机。')
        if not args.segment:
            return 0
    if not args.segment:
        parser.error('需要段号，或只用 --set-camera 记录相机架设')
    setup = camera_setup(args.root, day)
    if setup is None:
        print('提醒：今天还没记录相机架设，notes.md 里高度/俯仰/横滚为空。实测后运行：'
              'record_gemini_mac.py --set-camera 高度cm 俯仰° 横滚°', flush=True)
    else:
        print('相机架设：'+'，'.join(f'{label} {setup[k]:g}{unit}' for k, label, unit in SETUP_FIELDS), flush=True)
    try:
        out, planned = resolve_segment(args.segment, args.root, day)
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
    (out/'notes.md').write_text(notes_text(name, time.strftime('%Y-%m-%d %H:%M'), setup))

    camera = None
    try:
        camera = ColorCapture(args.camera, args.color_mode)
        ratio = check_cast(camera)
    except RuntimeError as exc:
        if camera is None or not args.allow_color_cast:
            if camera is not None:
                camera.release()
            shutil.rmtree(out)      # only notes.md so far; the next run reuses the name
            sys.exit(str(exc))
        print(f'警告：{exc}（--allow-color-cast，继续录制）', flush=True)
        ratio = float('nan')
    mapping = load_mapping()
    print(f'彩色模式 {args.color_mode}，开录前 R/B {ratio:.2f}'
          + (f'，几何映射 {mapping.get("source")}' if args.color_mode == 'full' else ''), flush=True)
    ok, frame = camera.read()
    if not ok:
        camera.release()
        sys.exit('彩色读帧失败')

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
    writer = cv2.VideoWriter(str(out/'color.avi'), cv2.VideoWriter_fourcc(*'MJPG'), camera.fps, (640, 480))
    stopping = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stopping.set())
    start, count, preview_depth, preview_at, cast_frames = time.monotonic(), 0, None, 0., 0
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
            if count % 15 == 0 and rb_ratio(frame) < CAST_RB:
                cast_frames += 1
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
                if rb_ratio(frame) < CAST_RB:
                    cv2.putText(shown, 'BLUE CAST - stop and restart', (10, 50), cv2.FONT_HERSHEY_SIMPLEX, .7,
                                (0, 0, 255), 2)
                if preview_depth is not None:
                    shown[480-200:, 640-320:] = preview_depth
                cv2.imshow('Gemini recording - Q to stop', shown)
                if cv2.waitKey(1) & 255 in (ord('q'), 27):
                    break
    writer.release()
    camera.release()
    cv2.destroyAllWindows()
    depth.stop()
    patch_device_json(out, args.color_mode, mapping)
    summary = write_summary(out, {'segment': name, 'depth_helper_log': depth.log[-20:],
                                  'depth_storage': 'raw' if args.raw else 'zlib', 'color_mode': args.color_mode,
                                  'color_rb_start': ratio, 'color_cast_samples': cast_frames, 'camera_setup': setup})
    if cast_frames:
        summary['problems'].append(f'录制中 {cast_frames} 个抽样帧严重偏蓝')
        (out/'capture-summary.json').write_text(json.dumps(summary, indent=2, ensure_ascii=False))
    print(f'彩色 {summary["color_frames"]} 帧，深度 {summary["depth_rows"]} 帧，重叠 {summary["overlap_seconds"]} 秒，'
          f'最大间断 彩色 {summary["color_max_gap_seconds"]} / 深度 {summary["depth_max_gap_seconds"]} 秒，'
          f'占用 {summary["disk_bytes"]/1e9:.2f} GB')
    if summary['problems']:
        print('有问题：'+'；'.join(summary['problems']))
    print(f'请填写 {out/"notes.md"}，再用 scripts/label_recording.py 标注。')
    return 1 if summary['problems'] else 0


if __name__ == '__main__':
    sys.exit(main())
