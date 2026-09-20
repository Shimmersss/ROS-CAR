#!/usr/bin/env python3
"""Allowlisted one-way sync. Defaults to dry-run; no deletion or remote execution."""
import argparse
from pathlib import Path
import re
import shlex
import subprocess


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', required=True, help='user@hostname or user@IPv4')
    parser.add_argument('--dest', required=True, help='absolute dedicated project directory')
    parser.add_argument('--apply', action='store_true')
    args = parser.parse_args()
    if not re.fullmatch(r'[A-Za-z0-9_][A-Za-z0-9_.-]*@[A-Za-z0-9][A-Za-z0-9.-]*', args.host):
        parser.error('host 必须为 user@hostname 或 user@IPv4')
    if not re.fullmatch(r'/[A-Za-z0-9_./-]+', args.dest):
        parser.error('dest 必须为不含空格的绝对项目路径')
    dest = Path(args.dest)
    if '..' in dest.parts or len(dest.parts) < 4 or dest.name in {'.', '..'}:
        parser.error('请指定如 /home/user/ROSCAR 的专用项目子目录')
    root = Path(__file__).resolve().parents[1]
    paths = ['README.md', 'AGENTS.md', 'WORKLOG.md', '人体跟随感知方案.md',
             'ros2_ws/src', 'scripts', 'tests', 'docs', 'foxglove',
             'deploy/README.md', 'deploy/humble-test.Dockerfile', 'deploy/chassis-test.Dockerfile', 'deploy/systemd',
             'models/manifest.json', 'models/weights/README.md',
             'data/catalog', 'data/recordings/README.md']
    cmd = ['rsync', '-azR', '--itemize-changes',
           '--exclude=__pycache__', '--exclude=*.pyc', '--exclude=.DS_Store',
           '--exclude=local.*', '--exclude=.env', '--exclude=.env.*']
    if not args.apply:
        cmd.append('--dry-run')
    cmd.extend(paths + [f'{args.host}:{args.dest.rstrip("/")}/'])
    print(('实际同步' if args.apply else '同步预览') + ': ' + shlex.join(cmd), flush=True)
    subprocess.run(cmd, cwd=root, check=True)


if __name__ == '__main__':
    main()
