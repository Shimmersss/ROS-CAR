from glob import glob
from setuptools import setup
setup(name='motion_guard', version='0.1.0', packages=['motion_guard'],
      data_files=[('share/ament_index/resource_index/packages',['resource/motion_guard']),
                  ('share/motion_guard',['package.xml']),
                  ('share/motion_guard/config',glob('config/*.yaml')),
                  ('share/motion_guard/launch',glob('launch/*.launch.py'))],
      install_requires=['setuptools'], license='Proprietary',
      maintainer='ROSCAR maintainers', maintainer_email='maintainer@example.invalid',
      description='Fail-closed velocity gate',
      entry_points={'console_scripts':['guard = motion_guard.node:main']})
