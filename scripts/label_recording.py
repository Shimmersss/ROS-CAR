#!/usr/bin/env python3
"""Frame-accurate event labelling for a Gemini recording (docs/补录视频要求20261008.md §9).

Keys
  space  play/pause          a / d  previous / next frame     z / c  -/+ 10 frames
  m      start marker (t=0, the raised-hands frame)
  1-5    fall events: fall_onset, impact, lying_start, getup_start, stand_stable
  b / e  action_start / action_end (action name cycles with n)
  r      cycle direction     n  cycle action     p  cycle person     k  cycle position
  u      undo last label     s  save             q  save and quit    h  toggle help
"""
import argparse
import csv
from dataclasses import dataclass, field
import json
from pathlib import Path
import sys

FALL_EVENTS = ('fall_onset', 'impact', 'lying_start', 'getup_start', 'stand_stable')
DIRECTIONS = ('side_left', 'side_right', 'toward', 'away', 'walking_away', 'walking', 'collapse', 'backward_sit',
              'chair', 'two_stage', '')
ACTIONS = ('fast_chair_sit', 'fast_floor_sit', 'fast_lie_down', 'stumble_recover', 'kneel', 'pushup', 'jump',
           'sofa_lie', 'squat',
           'chair_sit', 'pick_up', 'tie_shoes', 'slow_floor_sit', 'slow_lie_down', 'turn',
           'enroll_front_turn', 'cross_occlusion', 'exit_reenter', 'full_occlusion', 'swap_back', 'other')
PERSONS = ('A', 'B', 'A+B')
POSITIONS = ('1.9m', '3m', '1.2m', '2m', '2.5m', '4m', '')
COLUMNS = ('segment', 't_s', 'event', 'person', 'position', 'direction', 'notes', 'frame', 'host_wall_ns')


def cycle(options, value):
    return options[(options.index(value)+1) % len(options)] if value in options else options[0]


@dataclass
class LabelSession:
    """Window-independent labelling state; frame indices are 0-based into color_timestamps."""
    segment: str
    wall_ns: list
    mono_ns: list
    frame: int = 0
    marker: int = None
    person: str = 'A'
    position: str = '1.9m'
    direction: str = 'side_left'
    action: str = ACTIONS[0]
    playing: bool = False
    labels: list = field(default_factory=list)

    def seconds(self, frame):
        origin = self.marker if self.marker is not None else 0
        return (self.mono_ns[frame]-self.mono_ns[origin])/1e9

    def step(self, delta):
        self.frame = max(0, min(len(self.wall_ns)-1, self.frame+delta))

    def add(self, event, direction='', notes=''):
        self.labels.append(dict(segment=self.segment, event=event, person=self.person, position=self.position,
                                direction=direction, notes=notes, frame=self.frame))

    def key(self, char):
        """Apply one key; returns 'quit' or 'save' when the caller must act."""
        if char == ' ':
            self.playing = not self.playing
        elif char in 'adzc':
            self.playing = False
            self.step({'a': -1, 'd': 1, 'z': -10, 'c': 10}[char])
        elif char == 'm':
            self.marker = self.frame
        elif char in '12345':
            self.add(FALL_EVENTS[int(char)-1], direction=self.direction)
        elif char == 'b':
            self.add('action_start', notes=self.action)
        elif char == 'e':
            self.add('action_end', notes=self.action)
        elif char == 'r':
            self.direction = cycle(DIRECTIONS, self.direction)
        elif char == 'n':
            self.action = cycle(ACTIONS, self.action)
        elif char == 'p':
            self.person = cycle(PERSONS, self.person)
        elif char == 'k':
            self.position = cycle(POSITIONS, self.position)
        elif char == 'u' and self.labels:
            self.labels.pop()
        elif char == 's':
            return 'save'
        elif char == 'q':
            return 'quit'
        return None

    def rows(self):
        out = []
        for label in sorted(self.labels, key=lambda item: item['frame']):
            out.append(dict(label, t_s=f'{self.seconds(label["frame"]):.3f}',
                            host_wall_ns=self.wall_ns[label['frame']], frame=label['frame']+1))
        return out

    def problems(self):
        """Fall events must run 1..5 in order per fall; actions must open and close."""
        issues = []
        if self.marker is None:
            issues.append('start marker (m) not set: times are relative to the first frame')
        expected = 0
        for row in self.rows():
            if row['event'] in FALL_EVENTS:
                index = FALL_EVENTS.index(row['event'])
                if index != expected:
                    issues.append(f'frame {row["frame"]}: {row["event"]} out of order (expected {FALL_EVENTS[expected]})')
                expected = (index+1) % len(FALL_EVENTS)
        if expected:
            issues.append(f'last fall incomplete (missing {FALL_EVENTS[expected]})')
        open_actions = {}
        for row in self.rows():
            if row['event'] == 'action_start':
                open_actions[row['notes']] = open_actions.get(row['notes'], 0)+1
            elif row['event'] == 'action_end':
                if not open_actions.get(row['notes']):
                    issues.append(f'frame {row["frame"]}: action_end without start ({row["notes"]})')
                else:
                    open_actions[row['notes']] -= 1
        issues += [f'{name}: action_start without end' for name, n in open_actions.items() if n]
        return issues

    def save(self, recording):
        recording = Path(recording)
        with (recording/'labels.csv').open('w', newline='') as handle:
            writer = csv.DictWriter(handle, fieldnames=COLUMNS)
            writer.writeheader()
            for row in self.rows():
                writer.writerow({k: row[k] for k in COLUMNS})
        meta = dict(marker_frame=None if self.marker is None else self.marker+1,
                    marker_host_wall_ns=None if self.marker is None else self.wall_ns[self.marker])
        (recording/'labels-meta.json').write_text(json.dumps(meta, indent=2))

    def load(self, recording):
        recording = Path(recording)
        meta = recording/'labels-meta.json'
        if meta.exists():
            marker = json.loads(meta.read_text()).get('marker_frame')
            self.marker = None if marker is None else int(marker)-1
        if (recording/'labels.csv').exists():
            with (recording/'labels.csv').open(newline='') as handle:
                for row in csv.DictReader(handle):
                    self.labels.append(dict(segment=row['segment'], event=row['event'], person=row['person'],
                                            position=row['position'], direction=row['direction'],
                                            notes=row['notes'], frame=int(row['frame'])-1))


def run(recording, segment):
    import cv2
    with (recording/'color_timestamps.csv').open(newline='') as handle:
        rows = list(csv.DictReader(handle))
    session = LabelSession(segment, [int(r['host_wall_ns']) for r in rows], [int(r['host_monotonic_ns']) for r in rows])
    session.load(recording)
    video = cv2.VideoCapture(str(recording/'color.avi'))
    if not video.isOpened():
        sys.exit('无法打开 color.avi')
    position, image, help_on = -1, None, True
    title = f'label {recording.name} - h for help'
    cv2.namedWindow(title, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(title, 960, 720)
    while True:
        if session.frame != position:
            if session.frame != position+1:
                video.set(cv2.CAP_PROP_POS_FRAMES, session.frame)
            ok, image = video.read()
            if not ok:
                session.frame = max(0, position)
                session.playing = False
                continue
            position = session.frame
        shown = image.copy()
        last = session.rows()[-4:]
        lines = [f'frame {session.frame+1}/{len(rows)}  t={session.seconds(session.frame):.2f}s'
                 f'{"" if session.marker is not None else " (no marker)"}{"  PLAY" if session.playing else ""}',
                 f'person {session.person}  pos {session.position}  dir {session.direction or "-"}  action {session.action}']
        lines += [f'  {r["t_s"]}s {r["event"]} {r["direction"] or r["notes"]}' for r in last]
        if help_on:
            lines += ['space play | a/d +-1 | z/c +-10 | m marker | 1-5 fall events | b/e action',
                      'r dir | n action | p person | k pos | u undo | s save | q quit | h help']
        for i, line in enumerate(lines):
            cv2.putText(shown, line, (8, 20+18*i), cv2.FONT_HERSHEY_SIMPLEX, .5, (0, 255, 255), 1)
        cv2.imshow(title, shown)
        key = cv2.waitKey(33 if session.playing else 0) & 255
        if key == 255:
            if session.playing:
                session.step(1)
            continue
        char = chr(key).lower() if key < 128 else ''
        if char == 'h':
            help_on = not help_on
            continue
        if session.playing and not char:
            session.step(1)
        action = session.key(char) if char else None
        if action in ('save', 'quit'):
            session.save(recording)
            print(f'已保存 {len(session.labels)} 条标注到 {recording/"labels.csv"}')
            for issue in session.problems():
                print('  注意：'+issue)
            if action == 'quit':
                break
    cv2.destroyAllWindows()


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('recording', type=Path)
    parser.add_argument('--segment', help='默认取目录名中日期之后的部分的第一段，例如 A1')
    args = parser.parse_args()
    parts = args.recording.name.split('-')
    segment = args.segment or (parts[2] if len(parts) > 2 else args.recording.name)
    run(args.recording, segment)


if __name__ == '__main__':
    main()
