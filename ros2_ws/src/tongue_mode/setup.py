from glob import glob
from setuptools import find_packages, setup
setup(name='tongue_mode', version='0.1.0', packages=find_packages(),
      data_files=[('share/ament_index/resource_index/packages', ['resource/tongue_mode']),
                  ('share/tongue_mode', ['package.xml']),
                  ('share/tongue_mode/launch', glob('launch/*.launch.py')),
                  ('share/tongue_mode/config', glob('config/*.yaml'))],
      install_requires=['setuptools'], license='Proprietary',
      entry_points={'console_scripts': ['mode_node = tongue_mode.node:main']})
