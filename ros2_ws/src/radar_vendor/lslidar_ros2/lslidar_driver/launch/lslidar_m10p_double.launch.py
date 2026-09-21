import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import LifecycleNode
from launch import LaunchDescription

def generate_launch_description():
    pkg_dir = get_package_share_directory('lslidar_driver')
    left_config = os.path.join(pkg_dir, 'config',  'lslidar_m10p_left.yaml')
    right_config = os.path.join(pkg_dir, 'config', 'lslidar_m10p_right.yaml')

    left_node = LifecycleNode(
        package='lslidar_driver',
        executable='lslidar_driver_node',
        name='lslidar_driver_node',
        namespace='lidar_left',
        output='screen',
        emulate_tty=True,
        parameters=[left_config]
    )
    
    right_node = LifecycleNode(
        package='lslidar_driver',
        executable='lslidar_driver_node',
        name='lslidar_driver_node',
        namespace='lidar_right',
        output='screen',
        emulate_tty=True,
        parameters=[right_config]
    )

    return LaunchDescription([left_node, right_node])