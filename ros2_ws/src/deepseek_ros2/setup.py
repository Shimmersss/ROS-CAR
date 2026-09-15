from setuptools import find_packages, setup


package_name = 'deepseek_ros2'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ROSCAR maintainers',
    maintainer_email='maintainer@example.invalid',
    description='DeepSeek chat bridge for ROS 2 voice text topics.',
    license='Proprietary',
    entry_points={
        'console_scripts': [
            'chat_node = deepseek_ros2.chat_node:main',
        ],
    },
)
