"""Request producer + gate. Never starts sensors, chassis, or auto-arms."""
from pathlib import Path
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def start(context):
    path=LaunchConfiguration('safety_config').perform(context)
    c=yaml.safe_load(Path(path).read_text())['/**']['ros__parameters']
    motion=LaunchConfiguration('motion_enabled').perform(context)=='true'
    source=LaunchConfiguration('expected_source').perform(context) or c['expected_source']
    target_frame=LaunchConfiguration('target_frame').perform(context) or c['target_frame']
    return [Node(package='astra_body_adapter',executable='person_follower',output='screen',
                 parameters=[{'enabled':True,'expected_source':source,'base_frame':c['base_frame'],
                    'performance_enabled':LaunchConfiguration('performance_enabled').perform(context)=='true',
                    'max_linear_mps':float(c['max_linear_mps']),
                    'max_angular_rps':float(c['max_angular_rps']),
                    'target_distance_m':float(LaunchConfiguration('target_distance_m').perform(context))}]),
            Node(package='motion_guard',executable='guard',output='screen',
                 parameters=[path,{'motion_enabled':motion,'expected_source':source,'target_frame':target_frame}])]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('safety_config',default_value=str(Path(
            get_package_share_directory('motion_guard'))/'config/safety.yaml')),
        DeclareLaunchArgument('motion_enabled',default_value='false',choices=['true','false']),
        DeclareLaunchArgument('expected_source',default_value=''),
        DeclareLaunchArgument('target_frame',default_value=''),
        DeclareLaunchArgument('performance_enabled',default_value='true',choices=['true','false']),
        DeclareLaunchArgument('target_distance_m',default_value='1.0'),
        OpaqueFunction(function=start)])
