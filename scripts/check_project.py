#!/usr/bin/env python3
"""Dependency-free structural checks; ROS build/runtime are tested separately."""
import ast
import json
from pathlib import Path
import xml.etree.ElementTree as ET


def main():
    root = Path(__file__).resolve().parents[1]
    packages = root / 'ros2_ws/src'
    names = set()
    for xml in packages.glob('*/package.xml'):
        if (xml.parent / 'COLCON_IGNORE').exists():
            continue
        tree = ET.parse(xml).getroot()
        name = tree.findtext('name')
        assert name == xml.parent.name and name not in names, xml
        names.add(name)
    assert names == {
        'person_interfaces', 'astra_body_adapter', 'yolo_person_tracker',
        'perception_bringup', 'bodyreader_msg', 'xfyun_speech',
        'deepseek_ros2', 'voice_command_router', 'red_object_tracker', 'motion_guard', 'navigation_bringup', 'roscar_interfaces', 'roscar_api',
    }
    python_files = list(packages.rglob('*.py')) + list((root / 'scripts').glob('*.py')) + list((root / 'tests').glob('*.py'))
    for path in python_files:
        ast.parse(path.read_text(encoding='utf-8'), filename=str(path))
    for path in (root / 'models').glob('*.json'):
        json.loads(path.read_text(encoding='utf-8'))
    for path in (root / 'data/catalog').glob('*.json'):
        json.loads(path.read_text(encoding='utf-8'))
    for path in (root / 'docs/diagrams').glob('*.svg'):
        ET.parse(path)
    for package in packages.iterdir():
        if (package / 'setup.py').exists():
            assert (package / 'resource' / package.name).exists()
            assert (package / 'setup.cfg').is_file()
    print(f'结构检查通过：{len(names)} 个 ROS 包，{len(python_files)} 个 Python 文件。')
    print('这不代表 ROS 编译或硬件验证通过。')


if __name__ == '__main__':
    main()
