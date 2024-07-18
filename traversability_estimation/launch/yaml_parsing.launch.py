from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    traversability_estimation_dir = get_package_share_directory('traversability_estimation')
    
    robot_yaml = os.path.join(traversability_estimation_dir, 'config', 'robot.yaml')
    robot_footprint_yaml = os.path.join(traversability_estimation_dir, 'config', 'robot_footprint_parameter.yaml')
    robot_filter_yaml = os.path.join(traversability_estimation_dir, 'config', 'robot_filter_parameter.yaml')
    return LaunchDescription([
        Node(
            package='traversability_estimation',
            executable='yaml_parsing_test',
            name='yaml_parsing_node',
            output='screen',
            parameters=[robot_footprint_yaml],
        )
    ])

