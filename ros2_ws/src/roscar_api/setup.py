from glob import glob
from setuptools import setup
setup(name='roscar_api', version='0.1.0', packages=['roscar_api'],
      data_files=[('share/ament_index/resource_index/packages', ['resource/roscar_api']),
                  ('share/roscar_api', ['package.xml']),
                  ('share/roscar_api/launch', glob('launch/*.launch.py'))],
      install_requires=['setuptools'], license='Proprietary',
      maintainer='ROSCAR', maintainer_email='maintainer@example.invalid',
      description='Public API examples', entry_points={'console_scripts':[
          'chassis = roscar_api.chassis:main',
          'detections = roscar_api.observe:detections_main',
          'radar = roscar_api.observe:radar_main',
          'target = roscar_api.target:main']})
