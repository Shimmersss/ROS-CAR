#!/usr/bin/env python3
"""Copy verified model bytes from the user's local vendor archive; never load weights."""
import hashlib
import json
from pathlib import Path
import shutil


def main():
    root = Path(__file__).resolve().parents[1]
    entry = json.loads((root / 'models/manifest.json').read_text())['models'][0]
    source = root / 'JP6.2_wheeltec_ros2_src_20260903/ultralytics_ros2/model/yolo11n.pt'
    target = root / 'models' / entry['local_path']
    if not source.is_file():
        raise SystemExit('原厂模型未找到，请恢复本地厂商资料目录。')
    if hashlib.sha256(source.read_bytes()).hexdigest() != entry['sha256']:
        raise SystemExit('原厂模型与清单哈希不符，未复制。')
    if target.exists():
        if hashlib.sha256(target.read_bytes()).hexdigest() == entry['sha256']:
            print('模型已存在且哈希一致。')
            return
        raise SystemExit('目标已有不同内容，未覆盖。')
    target.parent.mkdir(parents=True, exist_ok=True)
    shutil.copyfile(source, target)
    print(f'已复制并校验：{target}；尚未加载推理。')


if __name__ == '__main__':
    main()
