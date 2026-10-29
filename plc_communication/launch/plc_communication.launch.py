import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('plc_communication')
    default_config_path = os.path.join(pkg_share, 'config', 'plc_config.yaml')

    config_arg = DeclareLaunchArgument(
        'config',
        default_value=default_config_path,
        description='Path to PLC communication config YAML file'
    )

    plc_ip_arg = DeclareLaunchArgument(
        'plc_ip',
        default_value='192.168.1.21',
        description='Siemens PLC IP Address'
    )

    poll_rate_arg = DeclareLaunchArgument(
        'poll_rate_ms',
        default_value='100',
        description='Polling rate in milliseconds'
    )

    plc_node = Node(
        package='plc_communication',
        executable='plc_bridge_node',
        name='plc_communication_node',
        output='screen',
        parameters=[
            LaunchConfiguration('config'),
            {
                'plc_ip': LaunchConfiguration('plc_ip'),
                'poll_rate_ms': LaunchConfiguration('poll_rate_ms'),
            }
        ]
    )

    return LaunchDescription([
        config_arg,
        plc_ip_arg,
        poll_rate_arg,
        plc_node
    ])
