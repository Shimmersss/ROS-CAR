import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
import launch_ros.actions

def generate_launch_description():
    call_recognition = Node(
        package="double_lidar_fusion",
        executable="lidar_fusion",
        output='screen',
        parameters=[
            {'scan1_topic':'/scan1'},
            {'scan2_topic':'/scan2'},
            {'frame_id':'laser'},
            {'scan_topic': 'scan'},
            {'lidar1_rotate':45.0},
            {'lidar1_xoffset':0.3},
            {'lidar1_yoffset':0.235},

            {'lidar2_rotate':-135.0},
            {'lidar2_xoffset':-0.3},
            {'lidar2_yoffset':-0.235}
        ],
    )

    return LaunchDescription(
        [call_recognition]
    )
