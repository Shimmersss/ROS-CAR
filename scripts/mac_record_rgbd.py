#!/usr/bin/env python3
"""Record synchronized Astra/OpenNI RGB-D frames on macOS.

The recorder deliberately requires depth-to-color registration.  It never
silently falls back to an RGB-only video, because such a recording cannot be
used to diagnose the body's missing-depth problem.
"""

from __future__ import annotations

import argparse
import csv
import json
import os
import sys
import time
from pathlib import Path


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Record registered RGB-D frames from an OpenNI2 camera")
    parser.add_argument("output", type=Path, help="output directory")
    parser.add_argument("--seconds", type=float, default=30.0, help="recording duration (default: 30)")
    parser.add_argument("--openni-path", type=Path, help="OpenNI2 redist directory (or use OPENNI2_REDIST)")
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=int, default=30)
    return parser


def _load_openni(path: Path | None):
    if path:
        os.environ["OPENNI2_REDIST"] = str(path)
    try:
        from openni import openni2  # type: ignore
    except ImportError as exc:
        raise RuntimeError(
            "缺少 OpenNI2 Python 绑定。Astra S 是旧 OpenNI 设备，请安装与 macOS/当前 Python ABI "
            "匹配的 openni 包，并设置 OPENNI2_REDIST；不要用 RGB-only 录制代替。"
        ) from exc
    return openni2


def _set_mode(stream, openni2, width: int, height: int, fps: int, pixel_format):
    stream.set_video_mode(
        openni2.VideoMode(
            pixelFormat=pixel_format,
            resolutionX=width,
            resolutionY=height,
            fps=fps,
        )
    )


def record(args: argparse.Namespace) -> int:
    openni2 = _load_openni(args.openni_path)
    args.output.mkdir(parents=True, exist_ok=False)
    (args.output / "color").mkdir()
    (args.output / "depth").mkdir()

    openni2.initialize(str(args.openni_path) if args.openni_path else None)
    device = None
    depth_stream = None
    color_stream = None
    try:
        device = openni2.Device.open_any()
        depth_stream = device.create_depth_stream()
        color_stream = device.create_color_stream()
        _set_mode(depth_stream, openni2, args.width, args.height, args.fps, openni2.PIXEL_FORMAT_DEPTH_1_MM)
        _set_mode(color_stream, openni2, args.width, args.height, args.fps, openni2.PIXEL_FORMAT_RGB888)
        try:
            device.set_image_registration_mode(openni2.IMAGE_REGISTRATION_DEPTH_TO_COLOR)
        except Exception as exc:  # registration is a hard requirement
            raise RuntimeError(
                "相机不支持深度到彩色配准；请确认 Astra S 使用 OpenNI2 驱动，或改用 ROS 中已标定的 RGB-D 录制。"
            ) from exc

        depth_stream.start()
        color_stream.start()
        metadata = {
            "format": "roscar-mac-rgbd-v1",
            "camera": "OpenNI2 device (Astra S expected)",
            "registered_depth_to_color": True,
            "width": args.width,
            "height": args.height,
            "fps": args.fps,
            "depth_encoding": "uint16 millimeters, PNG",
            "color_encoding": "RGB8, PNG",
            "host_clock": "time.monotonic_ns",
        }
        (args.output / "metadata.json").write_text(json.dumps(metadata, ensure_ascii=False, indent=2) + "\n")
        with (args.output / "timestamps.csv").open("w", newline="") as csv_file:
            writer = csv.writer(csv_file)
            writer.writerow(["frame", "host_monotonic_ns", "depth_timestamp", "color_timestamp"])
            deadline = time.monotonic() + args.seconds
            frame_id = 0
            while time.monotonic() < deadline:
                depth_frame = depth_stream.read_frame()
                color_frame = color_stream.read_frame()
                depth = depth_frame.get_buffer_as_uint16()
                color = color_frame.get_buffer_as_uint8()
                expected_depth = args.width * args.height
                expected_color = expected_depth * 3
                if len(depth) != expected_depth or len(color) != expected_color:
                    raise RuntimeError(f"帧尺寸异常: depth={len(depth)}, color={len(color)}")
                # Pillow and numpy are already present on this Mac; PNG keeps the
                # recording compact while preserving the 16-bit depth samples.
                from PIL import Image  # type: ignore
                import numpy as np  # type: ignore

                frame_id += 1
                Image.fromarray(np.frombuffer(bytes(color), dtype=np.uint8).reshape(args.height, args.width, 3), "RGB").save(
                    args.output / "color" / f"{frame_id:06d}.png"
                )
                Image.fromarray(np.frombuffer(bytes(depth), dtype="<u2").reshape(args.height, args.width), "I;16").save(
                    args.output / "depth" / f"{frame_id:06d}.png"
                )
                writer.writerow([frame_id, time.monotonic_ns(), getattr(depth_frame, "timestamp", 0), getattr(color_frame, "timestamp", 0)])
                if frame_id % max(1, args.fps) == 0:
                    csv_file.flush()
                    print(f"已录制 {frame_id} 帧", flush=True)
        print(f"录制完成: {args.output} ({frame_id} 帧)")
        return 0
    finally:
        for stream in (depth_stream, color_stream):
            if stream is not None:
                try:
                    stream.stop()
                except Exception:
                    pass
                try:
                    stream.close()
                except Exception:
                    pass
        if device is not None:
            try:
                device.close()
            except Exception:
                pass
        try:
            openni2.unload()
        except Exception:
            pass


def main() -> int:
    args = _parser().parse_args()
    try:
        return record(args)
    except Exception as exc:
        print(f"RGB-D 录制未开始: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
