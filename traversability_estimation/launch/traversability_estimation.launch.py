import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    traversability_estimation_dir = get_package_share_directory('traversability_estimation')
    
    robot_yaml = os.path.join(traversability_estimation_dir, 'config', 'robot.yaml')
    robot_footprint_yaml = os.path.join(traversability_estimation_dir, 'config', 'robot_footprint_parameter.yaml')
    robot_filter_yaml = os.path.join(traversability_estimation_dir, 'config', 'robot_filter_parameter.yaml')

    return LaunchDescription([
        Node(
            package='traversability_estimation',
            executable='traversability_estimation_node',
            name='traversability_estimation',
            output='screen',
            parameters=[robot_yaml, robot_footprint_yaml, robot_filter_yaml],
        ),
    ])

#             prefix=['gdb -ex run --args'],