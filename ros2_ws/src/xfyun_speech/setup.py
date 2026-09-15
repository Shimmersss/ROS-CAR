from glob import glob
from setuptools import find_packages, setup


package_name = 'xfyun_speech'

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
    description='iFLYTEK streaming ASR and TTS nodes for ROS 2.',
    license='Proprietary',
    entry_points={
        'console_scripts': [
            'asr_node = xfyun_speech.asr_node:main',
            'tts_node = xfyun_speech.tts_node:main',
        ],
    },
)
