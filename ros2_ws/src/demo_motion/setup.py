from glob import glob
from setuptools import setup
setup(name='demo_motion', version='0.0.1', packages=['demo_motion'],
      data_files=[('share/ament_index/resource_index/packages', ['resource/demo_motion']),
                  ('share/demo_motion', ['package.xml']),
                  ('share/demo_motion/launch', glob('launch/*.launch.py')),
                  ('share/demo_motion/config', glob('config/*.yaml'))],
      install_requires=['setuptools'], license='Proprietary',
      maintainer='ROSCAR', maintainer_email='maintainer@example.invalid',
      description='Demo-branch only relative motion executor', entry_points={'console_scripts': [
          'relative_motion = demo_motion.relative_motion_node:main']})
