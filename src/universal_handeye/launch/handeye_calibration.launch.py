from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Declare the config file argument
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(
            get_package_share_directory('universal_handeye'),
            'config',
            'handeye.yaml'
        ),
        description='Path to the hand-eye calibration configuration file'
    )

    # Hand-eye calibration node
    handeye_node = Node(
        package='universal_handeye',
        executable='handeye_node',
        name='universal_handeye',
        output='screen',
        emulate_tty=True,
        parameters=[LaunchConfiguration('config_file')],
        respawn=False
    )

    return LaunchDescription([
        config_file_arg,
        handeye_node
    ])
