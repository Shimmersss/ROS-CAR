#!/usr/bin/env python3
"""Explicit WAV collection for a labeled benchmark; never launches ROS/motion.

Run only when a microphone is available. Pause the project's shared capture
before using this standalone recorder. Optional playback supports AEC/double-
talk measurements; this script does not itself implement AEC.
"""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import time


def collect(args):
    if not 1 <= args.seconds <= 60:
        raise ValueError('Record 1..60 seconds')
    args.output.mkdir(parents=True, exist_ok=True)
    wav = args.output / (args.id + '.wav')
    if Path(args.id).name != args.id or not args.id or wav.exists():
        raise ValueError('Unique simple sample ID required; existing recordings are never overwritten')
    recorder = subprocess.Popen(['arecord', '-q', '-D', args.device, '-f', 'S16_LE', '-r', '16000',
                                 '-c', '1', '-t', 'wav', '-d', str(args.seconds), str(wav)], stderr=subprocess.PIPE)
    playback = None
    try:
        if args.playback_wav:
            time.sleep(min(1, args.seconds / 4))
            playback = subprocess.Popen(['aplay', '-q', '-D', args.playback_device, str(args.playback_wav)])
        _, errors = recorder.communicate(timeout=args.seconds + 5)
        if recorder.returncode:
            raise RuntimeError(errors.decode(errors='replace'))
        row = dict(id=args.id, wav=wav.name, reference_text=args.reference_text,
                   expected_intent=args.expected_intent, sample_type=args.sample_type,
                   condition=args.condition, distance_m=args.distance_m,
                   speech_start_s=None, speech_end_s=None, stop_word_end_s=None,
                   sha256=hashlib.sha256(wav.read_bytes()).hexdigest(),
                   playback_wav=str(args.playback_wav) if args.playback_wav else None,
                   annotation_status='Timing labels require manual annotation')
        with (args.output / 'manifest.jsonl').open('a') as stream:
            stream.write(json.dumps(row, ensure_ascii=False) + '\n')
        return wav
    finally:
        for process in (recorder, playback):
            if process and process.poll() is None:
                process.terminate()
                try:
                    process.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    process.kill(); process.wait(timeout=2)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--id', required=True)
    parser.add_argument('--reference-text', required=True)
    parser.add_argument('--expected-intent', required=True, help='e.g. MOVE:FORWARD, TURN:LEFT, STOP or NONE')
    parser.add_argument('--condition', required=True)
    parser.add_argument('--sample-type', choices=['human', 'negative', 'noise', 'robot_tts', 'double_talk'], required=True)
    parser.add_argument('--distance-m', type=float)
    parser.add_argument('--device', default='plughw:CARD=XFMDPV0018,DEV=0')
    parser.add_argument('--seconds', type=int, default=8)
    parser.add_argument('--playback-wav', type=Path)
    parser.add_argument('--playback-device', default='plughw:CARD=Device,DEV=0')
    print(collect(parser.parse_args()))
