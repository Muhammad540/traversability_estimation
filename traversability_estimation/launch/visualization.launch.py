import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    traversability_estimation_dir = get_package_share_directory('traversability_estimation')
    
    visualization_param = os.path.join(traversability_estimation_dir, 'config', 'visualization', 'traversability.yaml')

    return LaunchDescription([
        Node(
            package='grid_map_visualization',
            executable='grid_map_visualization',
            name='traversability_map_visualization',
            output='screen',
            parameters=[visualization_param]
        ),
    ])


