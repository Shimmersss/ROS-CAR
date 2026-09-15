from setuptools import find_packages, setup


package_name = 'voice_command_router'

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
    description='Allowlisted robot command router and optional Jetson GPIO buzzer.',
    license='Proprietary',
    entry_points={
        'console_scripts': [
            'router_node = voice_command_router.router_node:main',
            'buzzer_gpio_node = voice_command_router.buzzer_gpio_node:main',
        ],
    },
)
