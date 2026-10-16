"""Demo only. Start the relative motion executor next to an already running motion_guard.

Do not start it alongside another /chassis/cmd_vel publisher (e.g. the voice motion adapter):
the guard requires exactly one publisher on the selected request topic.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = LaunchConfiguration('config')
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=PathJoinSubstitution(
            [FindPackageShare('demo_motion'), 'config', 'relative_motion.yaml'])),
        Node(package='demo_motion', executable='relative_motion', name='demo_relative_motion',
             output='screen', parameters=[config]),
    ])
