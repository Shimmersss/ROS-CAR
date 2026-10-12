from glob import glob
from setuptools import find_packages, setup
setup(name='execution_mode_manager', version='0.1.0', packages=find_packages(),
      data_files=[('share/ament_index/resource_index/packages', ['resource/execution_mode_manager']),
                  ('share/execution_mode_manager', ['package.xml']),
                  ('share/execution_mode_manager/config', glob('config/*.yaml'))],
      install_requires=['setuptools'], license='Proprietary',
      entry_points={'console_scripts': ['manager_node = execution_mode_manager.node:main']})
