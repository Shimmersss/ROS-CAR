#!/usr/bin/env python3
"""Score replay_falls.py timelines against labels.csv: per-fall detection and latency, per-action
false alarms, and detections outside any labelled event. Small samples: initial results only."""
import argparse
import csv
import json
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parent))
from gemini_recording import read_rows  # noqa: E402

FALL_EVENTS = ('fall_onset', 'impact', 'lying_start', 'getup_start', 'stand_stable')
PRE_S, POST_S = .5, 10.   # detection window around a labelled fall / action


def label_times(recording):
    """labels.csv rows with t = seconds since the first colour frame (the replay time base)."""
    colours = read_rows(Path(recording)/'color_timestamps.csv')
    first = int(colours[0]['host_monotonic_ns'])
    rows = []
    for row in read_rows(Path(recording)/'labels.csv'):
        frame = int(row['frame'])
        rows.append(dict(row, t=(int(colours[frame-1]['host_monotonic_ns'])-first)/1e9))
    return rows


def falls_and_actions(labels):
    falls, actions, current, opened = [], [], {}, {}
    for row in sorted(labels, key=lambda r: r['t']):
        if row['event'] in FALL_EVENTS:
            if row['event'] == 'fall_onset':
                current = dict(direction=row['direction'], person=row['person'], position=row['position'])
            current[row['event']] = row['t']
            if row['event'] == 'stand_stable' and 'fall_onset' in current:
                falls.append(current)
                current = {}
        elif row['event'] == 'action_start':
            opened[row['notes']] = row
        elif row['event'] == 'action_end' and row['notes'] in opened:
            start = opened.pop(row['notes'])
            actions.append(dict(action=row['notes'], start=start['t'], end=row['t']))
    return falls, actions


def first_hit(timeline, start, end, phase):
    hits = [r for r in timeline if start <= r['t'] <= end and r['phase'] >= phase]
    return min(hits, key=lambda r: r['t']) if hits else None


def score(timeline, falls, actions):
    rows, covered = [], []
    for fall in falls:
        start = fall['fall_onset']-PRE_S
        end = fall.get('stand_stable', fall.get('impact', start)+POST_S)
        covered.append((start, end))
        result = dict(kind='fall', direction=fall['direction'], person=fall['person'], position=fall['position'],
                      onset=round(fall['fall_onset'], 2))
        for phase, name in ((1, 'suspected'), (2, 'confirmed')):
            hit = first_hit(timeline, start, end, phase)
            result[name] = hit is not None
            impact = fall.get('impact', fall['fall_onset'])
            result[f'{name}_latency_s'] = round(hit['t']-impact, 2) if hit else None
            result[f'{name}_basis'] = hit['basis'] if hit else None
        rows.append(result)
    for action in actions:
        start, end = action['start']-PRE_S, action['end']+3.
        covered.append((start, end))
        rows.append(dict(kind='action', action=action['action'], onset=round(action['start'], 2),
                         suspected=first_hit(timeline, start, end, 1) is not None,
                         confirmed=first_hit(timeline, start, end, 2) is not None))
    stray = sorted({round(r['t'], 1) for r in timeline if r['phase'] >= 1
                    and not any(a <= r['t'] <= b for a, b in covered)})
    return rows, stray


def aggregate(rows):
    falls = [r for r in rows if r['kind'] == 'fall']
    actions = [r for r in rows if r['kind'] == 'action']
    out = dict(falls=len(falls), actions=len(actions))
    for name in ('suspected', 'confirmed'):
        hits = [r for r in falls if r[name]]
        latencies = sorted(r[f'{name}_latency_s'] for r in hits)
        out[f'fall_{name}'] = len(hits)
        out[f'{name}_latency_median_s'] = latencies[len(latencies)//2] if latencies else None
        out[f'false_{name}'] = sum(r[name] for r in actions)
    return out


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--pair', nargs=2, action='append', metavar=('RECORDING', 'REPLAY_DIR'), required=True,
                        help='recording with labels.csv and the replay_falls.py output directory')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    report, table = {}, []
    for recording, replay in args.pair:
        recording, replay = Path(recording), Path(replay)
        if not (recording/'labels.csv').exists():
            print(f'skip {recording.name}: no labels.csv')
            continue
        falls, actions = falls_and_actions(label_times(recording))
        for timeline_path in sorted(replay.glob('timeline-*.jsonl')):
            mode = timeline_path.stem.replace('timeline-', '')
            timeline = [json.loads(line) for line in timeline_path.read_text().splitlines()]
            rows, stray = score(timeline, falls, actions)
            for row in rows:
                table.append(dict(row, recording=recording.name, mode=mode))
            entry = report.setdefault(mode, {})
            entry[recording.name] = dict(aggregate(rows), unlabelled_detections_s=stray)
    with (args.output/'per-event.csv').open('w', newline='') as handle:
        columns = sorted({k for r in table for k in r})
        writer = csv.DictWriter(handle, fieldnames=columns)
        writer.writeheader()
        writer.writerows(table)
    (args.output/'summary.json').write_text(json.dumps(report, indent=2, ensure_ascii=False))
    for mode, recordings in report.items():
        total = {k: sum(v[k] for v in recordings.values() if isinstance(v.get(k), int))
                 for k in ('falls', 'fall_suspected', 'fall_confirmed', 'actions', 'false_suspected', 'false_confirmed')}
        print(mode, json.dumps(total))


if __name__ == '__main__':
    main()
