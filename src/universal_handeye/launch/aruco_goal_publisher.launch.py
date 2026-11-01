from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(
            get_package_share_directory('universal_handeye'),
            'config',
            'goal_publisher.yaml'
        ),
        description='Path to the ArUco goal publisher configuration file'
    )

    goal_publisher_node = Node(
        package='universal_handeye',
        executable='goal_publisher_node',
        name='aruco_goal_publisher',
        output='screen',
        emulate_tty=True,
        parameters=[LaunchConfiguration('config_file')],
        respawn=False
    )

    return LaunchDescription([
        config_file_arg,
        goal_publisher_node
    ])
