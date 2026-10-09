"""Shared reader/summary for Mac Gemini recordings (color.avi + N.depth[.z] + CSV timestamps).

Host receive times only: colour and depth are paired by nearest host time, which is a
diagnostic pairing, not verified exposure synchronisation.
"""
import csv
import json
from pathlib import Path
import zlib

import numpy as np

DEPTH_SHAPE = (400, 640)
DEPTH_BYTES = DEPTH_SHAPE[0]*DEPTH_SHAPE[1]*2


def depth_path(recording, frame):
    recording = Path(recording)
    raw = recording/f'{frame}.depth'
    return raw if raw.exists() else recording/f'{frame}.depth.z'


def load_depth_raw(recording, frame):
    """uint16 depth units (multiply by scale_mm for millimetres)."""
    path = depth_path(recording, frame)
    data = path.read_bytes()
    if path.suffix == '.z':
        data = zlib.decompress(data)
    if len(data) != DEPTH_BYTES:
        raise ValueError(f'{path.name}: {len(data)} bytes, expected {DEPTH_BYTES}')
    return np.frombuffer(data, dtype=np.uint16).reshape(DEPTH_SHAPE)


def load_depth_m(recording, row):
    return load_depth_raw(recording, row['frame']).astype(np.float32)*float(row['scale_mm'])/1000


def read_rows(path):
    with Path(path).open(newline='') as handle:
        return list(csv.DictReader(handle))


def _span(times_ns):
    if len(times_ns) < 2:
        return 0., 0.
    times = np.sort(np.asarray(times_ns, dtype=np.int64))
    return float(times[-1]-times[0])/1e9, float(np.max(np.diff(times)))/1e9


def summarize(recording, check_depth_files=20):
    """Counts, durations, overlap and largest gaps; a sample of depth files is fully decoded."""
    recording = Path(recording)
    colors = read_rows(recording/'color_timestamps.csv') if (recording/'color_timestamps.csv').exists() else []
    depths = read_rows(recording/'timestamps.csv') if (recording/'timestamps.csv').exists() else []
    color_ns = [int(r['host_wall_ns']) for r in colors]
    depth_ns = [int(r['host_wall_ns']) for r in depths]
    color_s, color_gap = _span(color_ns)
    depth_s, depth_gap = _span(depth_ns)
    overlap = 0.
    if color_ns and depth_ns:
        overlap = max(0., (min(max(color_ns), max(depth_ns))-max(min(color_ns), min(depth_ns)))/1e9)
    missing = [r['frame'] for r in depths if not depth_path(recording, r['frame']).exists()]
    bad = []
    if depths and check_depth_files:
        for index in np.linspace(0, len(depths)-1, min(check_depth_files, len(depths))).astype(int):
            row = depths[int(index)]
            try:
                load_depth_raw(recording, row['frame'])
            except (OSError, ValueError, zlib.error) as exc:
                bad.append(f'{row["frame"]}: {exc}')
    disk = sum(p.stat().st_size for p in recording.iterdir() if p.is_file())
    summary = dict(
        color_frames=len(colors), depth_rows=len(depths), depth_files_missing=len(missing),
        depth_files_bad_in_sample=bad, color_capture_seconds=round(color_s, 3),
        depth_capture_seconds=round(depth_s, 3), overlap_seconds=round(overlap, 3),
        color_max_gap_seconds=round(color_gap, 3), depth_max_gap_seconds=round(depth_gap, 3),
        disk_bytes=disk,
        note='Host receive timestamps only, not verified exposure synchronization.')
    problems = []
    if not colors or not depths:
        problems.append('a stream recorded no frames')
    if missing or bad:
        problems.append('missing or unreadable depth files')
    if color_gap > 1. or depth_gap > 1.:
        problems.append('capture gap longer than 1 s')
    summary['problems'] = problems
    return summary


def write_summary(recording, extra=None):
    summary = summarize(recording)
    summary.update(extra or {})
    Path(recording, 'capture-summary.json').write_text(json.dumps(summary, indent=2, ensure_ascii=False))
    return summary
