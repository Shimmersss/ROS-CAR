"""The interface package must be discovered as ROS 2 ament, not generic CMake."""
from pathlib import Path
import xml.etree.ElementTree as ET


def test_ros_interface_packages_export_ament_build_type():
    source_root = Path(__file__).resolve().parents[2]
    manifests = []
    for manifest in source_root.glob('*/package.xml'):
        if (manifest.parent / 'COLCON_IGNORE').exists():
            continue
        package = ET.parse(manifest).getroot()
        if any(group.text == 'rosidl_interface_packages'
               for group in package.findall('member_of_group')):
            manifests.append(manifest)
            assert package.findtext('export/build_type') == 'ament_cmake', str(manifest)
    assert manifests
