from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('realsense2_camera'),
        'config',
        'multicam_config_file.yaml'
        )

    return LaunchDescription([
        Node(
            package="realsense2_camera",
            node_executable="realsense_node",
            node_name="realsense_node",
            output="screen",
            parameters=[config]
        )
    ])