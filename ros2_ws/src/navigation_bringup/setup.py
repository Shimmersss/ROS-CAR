from glob import glob
from setuptools import setup
setup(name='navigation_bringup',version='0.1.0',packages=['navigation_bringup'],
      data_files=[('share/ament_index/resource_index/packages',['resource/navigation_bringup']),
                  ('share/navigation_bringup',['package.xml'])]+[
          ('share/navigation_bringup/'+d,glob(d+'/*')) for d in ('config','launch','behavior_trees')],
      install_requires=['setuptools'],license='Proprietary',maintainer='ROSCAR maintainers',
      maintainer_email='maintainer@example.invalid',description='Guarded Nav2 navigation',
      entry_points={'console_scripts':['odom_tf = navigation_bringup.odom_tf:main',
              'follow_goal = navigation_bringup.follow_goal:main',
              'velocity_adapter = navigation_bringup.velocity_adapter:main']})
