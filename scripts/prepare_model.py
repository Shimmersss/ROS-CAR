#!/usr/bin/env python3
"""Prepare a manifest-pinned model; only this explicit command may download weights."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import tempfile
import urllib.request


def main():
    root = Path(__file__).resolve().parents[1]
    manifest = json.loads((root / 'models/manifest.json').read_text())
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model', default=manifest['default_model'],
                        choices=[item['name'] for item in manifest['models']])
    args = parser.parse_args()
    entry = next(item for item in manifest['models'] if item['name'] == args.model)
    target = root / 'models' / entry['local_path']

    def verify(path):
        return (path.stat().st_size == entry['size_bytes']
                and hashlib.sha256(path.read_bytes()).hexdigest() == entry['sha256'])

    if target.exists():
        if verify(target):
            print(f'模型已存在且 SHA-256 一致：{target}')
            return
        raise SystemExit('目标已有不同内容，未覆盖。')
    target.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile(dir=target.parent, suffix='.part', delete=False) as file:
        temporary = Path(file.name)
    try:
        if 'url' in entry:
            with urllib.request.urlopen(entry['url'], timeout=60) as response, temporary.open('wb') as out:
                shutil.copyfileobj(response, out)
        else:
            source = root / 'JP6.2_wheeltec_ros2_src_20260903/ultralytics_ros2/model' / entry['name']
            shutil.copyfile(source, temporary)
        if not verify(temporary):
            raise SystemExit('模型大小或 SHA-256 与清单不符，未安装。')
        temporary.replace(target)
    finally:
        temporary.unlink(missing_ok=True)
    print(f'已准备并校验：{target}；尚未加载推理。')


if __name__ == '__main__':
    main()
