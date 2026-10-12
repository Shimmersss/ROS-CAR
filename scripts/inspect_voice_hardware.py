#!/usr/bin/env python3
"""Read-only ALSA/USB inventory. Does not open capture or configure the DSP."""
import argparse
import json
from pathlib import Path
import shutil
import subprocess


def inspect():
    commands = [['arecord', '-l'], ['arecord', '-L'], ['aplay', '-l'], ['lsusb'],
                ['amixer', '-c', 'XFMDPV0018', 'contents'],
                ['udevadm', 'info', '--query=property', '--name=/dev/wheeltec_mic']]
    rows = []
    for command in commands:
        if not shutil.which(command[0]):
            rows.append(dict(command=command, available=False)); continue
        try:
            result = subprocess.run(command, capture_output=True, text=True, timeout=15)
            rows.append(dict(command=command, available=True, returncode=result.returncode,
                             stdout=result.stdout, stderr=result.stderr))
        except subprocess.TimeoutExpired:
            rows.append(dict(command=command, available=True, error='timeout'))
    return dict(scope='Inventory only; no recording, DSP configuration or motion', commands=rows,
                asound_cards=Path('/proc/asound/cards').read_text() if Path('/proc/asound/cards').exists() else None,
                unknown=['raw USB hardware channels', 'beamforming/NS DSP settings', 'AEC reference path', 'continuous DOA output'])


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(inspect(), ensure_ascii=False, indent=2) + '\n')
    print(args.output)
