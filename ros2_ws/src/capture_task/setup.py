from glob import glob
from setuptools import find_packages, setup
setup(name='capture_task', version='0.1.0', packages=find_packages(),
      data_files=[('share/ament_index/resource_index/packages', ['resource/capture_task']),
                  ('share/capture_task', ['package.xml']),
                  ('share/capture_task/launch', glob('launch/*.launch.py')),
                  ('share/capture_task/config', glob('config/*.yaml'))],
      install_requires=['setuptools'], license='Proprietary',
      entry_points={'console_scripts': [
          'executor_node = capture_task.executor_node:main',
          'bridge_node = capture_task.bridge_node:main',
          'fake_tts_node = capture_task.fake_tts_node:main',
          'synthetic_camera_node = capture_task.synthetic_camera_node:main']})
