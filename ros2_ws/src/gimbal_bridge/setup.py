from glob import glob
from setuptools import find_packages, setup

package_name = 'gimbal_bridge'
setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ROSCAR maintainers',
    maintainer_email='maintainer@example.invalid',
    description='Serial bridge, URDF and MCU simulator for the pan-tilt camera gimbal.',
    license='Proprietary',
    entry_points={'console_scripts': ['bridge = gimbal_bridge.bridge_node:main',
                                      'mock_mcu = gimbal_bridge.mock_mcu:main']},
)
