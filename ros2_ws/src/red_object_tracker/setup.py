from setuptools import find_packages, setup
setup(name='red_object_tracker', version='0.1.0', packages=find_packages(exclude=['test']),
      data_files=[('share/ament_index/resource_index/packages', ['resource/red_object_tracker']),
                  ('share/red_object_tracker', ['package.xml'])],
      install_requires=['setuptools'], maintainer='ROSCAR maintainers',
      maintainer_email='maintainer@example.invalid', license='Proprietary',
      description='Red component tracking with registered RGB-D.',
      entry_points={'console_scripts': ['tracker = red_object_tracker.node:main']})
