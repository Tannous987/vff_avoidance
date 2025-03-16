from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the path to the configuration file
    config_file = os.path.join(
        get_package_share_directory('vff_avoidance'),
        'config',
        'AvoidanceNodeConfig.yaml'
    )

    return LaunchDescription([
        Node(
            package='vff_avoidance',
            executable='avoidance_vff',
            name='avoidance_node',
            output='screen',
            parameters=[config_file]  # Use the absolute path to the config file
        )
    ])
