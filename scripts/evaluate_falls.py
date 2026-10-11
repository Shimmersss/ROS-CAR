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
HOLD_S = 1.               # a person's last phase stays in the alarm this long


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
            current['auto'] = current.get('auto', False) or row.get('notes') == 'auto'
            if row['event'] == 'stand_stable' and 'fall_onset' in current:
                if not current.pop('auto'):     # unreviewed candidates from fall_candidates.py are not ground truth
                    falls.append(current)
                current = {}
        elif row['event'] == 'action_start':
            opened[row['notes']] = row
        elif row['event'] == 'action_end' and row['notes'] in opened:
            start = opened.pop(row['notes'])
            actions.append(dict(action=row['notes'], start=start['t'], end=row['t']))
    return falls, actions


def alarm_levels(timeline, hold_s=HOLD_S):
    """[(t, level, row)]: the highest phase among people seen within hold_s.

    People are not detected in every frame, so each one keeps its last phase for hold_s. Before
    a timestamp's rows are applied, the level from holds still live at that time is emitted (row
    None) whenever it dropped, so an alarm that expired during a gap with no rows is not mistaken
    for one that continued when the person reappears."""
    by_time = {}
    for row in timeline:
        by_time.setdefault(row['t'], []).append(row)
    last, out = {}, []
    for t in sorted(by_time):
        expired = max((r['phase'] for r in last.values() if t-r['t'] <= hold_s), default=0)
        if out and expired < out[-1][1]:
            out.append((t, expired, None))
        for row in by_time[t]:
            last[row.get('id')] = row
        live = [r for r in last.values() if t-r['t'] <= hold_s]
        top = max(live, key=lambda r: r['phase'])
        out.append((t, top['phase'], top))
    return out


def onsets(levels, phase):
    """Rows where the alarm rises to >= phase. An alarm raised before a fall started does not count
    for that fall: each fall must start its own alarm."""
    out, previous = [], 0
    for _, level, row in levels:
        if level >= phase > previous:
            out.append(row)
        previous = level
    return out


def first_hit(rises, start, end):
    hits = [r for r in rises if start <= r['t'] <= end]
    return hits[0] if hits else None


def score(timeline, falls, actions):
    rows, covered = [], []
    levels = alarm_levels(timeline)
    rises = {phase: onsets(levels, phase) for phase in (1, 2)}
    for fall in falls:
        start = fall['fall_onset']-PRE_S
        end = fall.get('stand_stable', fall.get('impact', start)+POST_S)
        covered.append((start, end))
        result = dict(kind='fall', direction=fall['direction'], person=fall['person'], position=fall['position'],
                      onset=round(fall['fall_onset'], 2))
        before = [(t, level) for t, level, _ in levels if t < start]
        for phase, name in ((1, 'suspected'), (2, 'confirmed')):
            hit = first_hit(rises[phase], start, end)
            result[name] = hit is not None
            impact = fall.get('impact', fall['fall_onset'])
            result[f'{name}_latency_s'] = round(hit['t']-impact, 2) if hit else None
            result[f'{name}_basis'] = hit['basis'] if hit else None
            # Alarm left over from an earlier event when this fall started (not counted as a hit).
            # Still held when this fall's window opens (holds expire after HOLD_S without rows).
            result[f'{name}_carried_in'] = bool(before) and before[-1][1] >= phase and start-before[-1][0] <= HOLD_S
        rows.append(result)
    for action in actions:
        start, end = action['start']-PRE_S, action['end']+3.
        covered.append((start, end))
        rows.append(dict(kind='action', action=action['action'], onset=round(action['start'], 2),
                         suspected=first_hit(rises[1], start, end) is not None,
                         confirmed=first_hit(rises[2], start, end) is not None))
    stray = [round(r['t'], 1) for r in rises[1] if not any(a <= r['t'] <= b for a, b in covered)]
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
        out[f'{name}_carried_in'] = sum(r[f'{name}_carried_in'] for r in falls)
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
            mode = f"{replay.name}/{timeline_path.stem.replace('timeline-', '')}"
            timeline = [json.loads(line) for line in timeline_path.read_text().splitlines()]
            rows, stray = score(timeline, falls, actions)
            for row in rows:
                table.append(dict(row, recording=recording.name, mode=mode))
            entry = report.setdefault(mode, {})
            entry[recording.name] = dict(aggregate(rows), unlabelled_alarms_s=stray)
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
