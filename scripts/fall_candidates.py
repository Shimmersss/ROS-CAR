#!/usr/bin/env python3
"""Pre-fill labels.csv with candidate falls for review in label_recording.py (notes=auto).

A candidate is a stretch where someone is low: body top from replay_falls.py --modes height below
--low-m, a wide short box in the pose cache, or any height-cue alarm. Each becomes the five fall
events: onset 0.6 s before the low stretch (or after the previous one), impact where it starts,
lying_start 0.5 s later, getup_start at its end (often late: kneeling is still low), stand_stable at the first upright second after it.

Candidates come from the detector's own signals, so a fall it never saw low is not proposed:
the reviewer still watches the whole segment. evaluate_falls.py ignores candidates until each
fall is confirmed (y) in the label tool; slow sit/lie-downs are deleted (x) and labelled as
actions (b/e). Existing reviewed labels are kept; earlier candidates are replaced.
"""
import argparse
import csv
import json
from pathlib import Path
import re

import numpy as np

from label_recording import AUTO, COLUMNS, FALL_EVENTS

TOP = re.compile(r'top=(-?[\d.]+)m')


def low_times(timeline, frames, low_m, wide_aspect, upright_m=1.2, tall_aspect=1.8):
    """[(t, low, upright)] over every replay time: someone low / someone clearly standing.

    Low is geometric only (body top, wide box): an alarm stays raised until recovery, so it would
    stretch the low span past the get-up. Alarms are added separately by alarm_spans().

    Sitting on the floor keeps the head near 0.8-0.9 m and is often a 2D STANDING, so standing
    needs the body top above upright_m or a STANDING box at least tall_aspect times its width."""
    out = {}
    for row in timeline:
        low, upright = out.get(row['t'], (False, False))
        match = TOP.search(row.get('detail', ''))
        top = float(match.group(1)) if match else None
        low |= top is not None and top < low_m
        upright |= top is not None and top >= upright_m
        out[row['t']] = (low, upright)
    for f in frames:
        low, upright = out.get(f['time_s'], (False, False))
        for p in f['people']:
            x1, y1, x2, y2 = p['box']
            low |= (x2-x1) > wide_aspect*(y2-y1)
            upright |= p.get('state') == 'STANDING' and (y2-y1) >= tall_aspect*(x2-x1)
        out[f['time_s']] = (low, upright)
    return sorted((t, *v) for t, v in out.items())


def alarm_spans(timeline, spans, length_s=.6):
    """Alarm rising edges not inside a geometric low span (e.g. height unmeasured), as short spans."""
    raised, extra = False, []
    for t in sorted({row['t'] for row in timeline}):
        now = any(r['phase'] >= 1 for r in timeline_at(timeline, t))
        if now and not raised and not any(a-1. <= t <= b+1. for a, b in spans+extra):
            extra.append((t, t+length_s))
        raised = now
    return sorted(spans+extra)


def timeline_at(timeline, t, _index={}):
    key = id(timeline)
    if key not in _index:
        _index.clear()
        _index[key] = {}
        for row in timeline:
            _index[key].setdefault(row['t'], []).append(row)
    return _index[key].get(t, [])


def intervals(samples, merge_s, min_s):
    spans = []
    for t, low, _ in samples:
        if not low:
            continue
        if spans and t-spans[-1][1] <= merge_s:
            spans[-1][1] = t
        else:
            spans.append([t, t])
    return [(a, b) for a, b in spans if b-a >= min_s]


def first_stable(samples, after, until, stable_s):
    run = None
    for t, low, upright in samples:
        if t <= after:
            continue
        if t > until:
            return None
        run = (run if run is not None else t) if upright and not low else None
        if run is not None and t-run >= stable_s:
            return run
    return None


def candidates(samples, spans, lead_s=.6, settle_s=.5, stable_s=1., max_wait_s=8.):
    """One candidate per fall: low stretches that start before the person stands again are one fall."""
    out, previous_end, i = [], 0., 0
    while i < len(spans):
        a, b = spans[i]
        while True:
            stable = first_stable(samples, b, b+max_wait_s, stable_s)
            nxt = spans[i+1][0] if i+1 < len(spans) else None
            if nxt is not None and nxt < b+max_wait_s and (stable is None or nxt < stable):
                i += 1
                b = spans[i][1]
                continue
            break
        i += 1
        onset = max(previous_end+.1, a-lead_s)
        stable = stable if stable is not None else b+3.
        if i < len(spans):
            stable = min(stable, spans[i][0]-lead_s-.2)
        # The body top crosses low_m about when it reaches the mat (10-10 A1a: within 0.2 s).
        out.append(dict(fall_onset=onset, impact=max(a, onset+.1), lying_start=min(a+settle_s, (a+b)/2), getup_start=b,
                        stand_stable=max(stable, b+.1)))
        previous_end = out[-1]['stand_stable']
    return out


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('recording', type=Path)
    parser.add_argument('poses', type=Path, help='directory with frames.jsonl')
    parser.add_argument('timeline', type=Path, help='timeline-height.jsonl from replay_falls.py')
    parser.add_argument('--low-m', type=float, default=.65, help='body top below this counts as low')
    parser.add_argument('--wide-aspect', type=float, default=1.2)
    parser.add_argument('--upright-m', type=float, default=1.2, help='body top above this counts as standing')
    parser.add_argument('--merge', type=float, default=1., help='s: join low stretches closer than this')
    parser.add_argument('--min', type=float, default=.6, help='s: shortest low stretch proposed')
    parser.add_argument('--dry-run', action='store_true')
    args = parser.parse_args()
    timeline = [json.loads(line) for line in args.timeline.read_text().splitlines()]
    frames = [json.loads(line) for line in (args.poses/'frames.jsonl').read_text().splitlines()]
    samples = low_times(timeline, frames, args.low_m, args.wide_aspect, args.upright_m)
    found = candidates(samples, alarm_spans(timeline, intervals(samples, args.merge, args.min)))
    with (args.recording/'color_timestamps.csv').open(newline='') as handle:
        colours = list(csv.DictReader(handle))
    mono = np.array([int(r['host_monotonic_ns']) for r in colours])
    rel = (mono-mono[0])/1e9
    segment = args.recording.name.split('-')[2]
    kept, path = [], args.recording/'labels.csv'
    if path.exists():
        with path.open(newline='') as handle:
            kept = [r for r in csv.DictReader(handle) if r['notes'] != AUTO]
    rows = list(kept)
    for i, c in enumerate(found, 1):
        print(f'{i:2d}  onset {c["fall_onset"]:6.1f}s  lying {c["lying_start"]:6.1f}-{c["getup_start"]:6.1f}s'
              f'  stable {c["stand_stable"]:6.1f}s')
        for event in FALL_EVENTS:
            frame = int(np.argmin(np.abs(rel-c[event])))
            rows.append(dict(segment=segment, t_s=f'{rel[frame]:.3f}', event=event, person='A', position='',
                             direction='', notes=AUTO, frame=frame+1, host_wall_ns=colours[frame]['host_wall_ns']))
    print(f'{args.recording.name}: {len(found)} candidates, {len(kept)} reviewed labels kept')
    if args.dry_run:
        return
    rows.sort(key=lambda r: int(r['frame']))
    with path.open('w', newline='') as handle:
        writer = csv.DictWriter(handle, fieldnames=COLUMNS)
        writer.writeheader()
        writer.writerows({k: r[k] for k in COLUMNS} for r in rows)


if __name__ == '__main__':
    main()
