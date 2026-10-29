import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('battery_communication')
    default_config_path = os.path.join(pkg_share, 'config', 'battery_config.yaml')

    config_arg = DeclareLaunchArgument(
        'config',
        default_value=default_config_path,
        description='Path to battery communication config YAML file'
    )

    port_arg = DeclareLaunchArgument(
        'port',
        default_value='/dev/battery_can',
        description='Serial port for battery CAN adapter'
    )

    baudrate_arg = DeclareLaunchArgument(
        'baudrate',
        default_value='2000000',
        description='Serial baudrate'
    )

    battery_node = Node(
        package='battery_communication',
        executable='battery_node.py',
        name='battery_communication_node',
        output='screen',
        parameters=[
            LaunchConfiguration('config'),
            {
                'port': LaunchConfiguration('port'),
                'baudrate': LaunchConfiguration('baudrate')
            }
        ]
    )

    return LaunchDescription([
        config_arg,
        port_arg,
        baudrate_arg,
        battery_node
    ])
