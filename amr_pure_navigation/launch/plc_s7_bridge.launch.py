from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg = get_package_share_directory('amr_pure_navigation')

    return LaunchDescription([

        DeclareLaunchArgument(
            'plc_ip',
            default_value='192.168.1.21',   # ← your PLC IP
            description='S7-1200 PLC IP address'),

        DeclareLaunchArgument(
            'rack',
            default_value='0',
            description='PLC rack (S7-1200 = 0)'),

        DeclareLaunchArgument(
            'slot',
            default_value='1',
            description='PLC slot (S7-1200 = 1)'),

        DeclareLaunchArgument(
            'station_map_file',
            default_value=os.path.join(pkg, 'config', 'station_map.yaml'),
            description='Path to station_map.yaml'),

        Node(
            package    = 'amr_pure_navigation',
            executable = 'plc_s7_bridge',
            name       = 'plc_s7_bridge',
            output     = 'screen',
            parameters = [{
                'plc_ip':           LaunchConfiguration('plc_ip'),
                'rack':             LaunchConfiguration('rack'),
                'slot':             LaunchConfiguration('slot'),
                'station_map_file': LaunchConfiguration('station_map_file'),
            }],
        ),
    ])





# from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument
# from launch.substitutions import LaunchConfiguration
# from launch_ros.actions import Node
# import os
# from ament_index_python.packages import get_package_share_directory


# def generate_launch_description():

#     # Same package as navigation — amr_pure_navigation
#     pkg = get_package_share_directory('amr_pure_navigation')

#     return LaunchDescription([

#         DeclareLaunchArgument(
#             'plc_ip',
#             default_value='192.168.1.21',
#             description='S7-1200 PLC IP address'),

#         DeclareLaunchArgument(
#             'rack',
#             default_value='0',
#             description='PLC rack number (usually 0)'),

#         DeclareLaunchArgument(
#             'slot',
#             default_value='1',
#             description='PLC slot number (S7-1200 = 1)'),

#         DeclareLaunchArgument(
#             'station_map_file',
#             default_value=os.path.join(pkg, 'config', 'station_map.yaml'),
#             description='Path to station_map.yaml'),

#         Node(
#             package    = 'amr_pure_navigation',   # same package
#             executable = 'plc_s7_bridge',          # built by CMakeLists
#             name       = 'plc_s7_bridge',
#             output     = 'screen',
#             parameters = [{
#                 'plc_ip':           LaunchConfiguration('plc_ip'),
#                 'rack':             LaunchConfiguration('rack'),
#                 'slot':             LaunchConfiguration('slot'),
#                 'station_map_file': LaunchConfiguration('station_map_file'),
#             }],
#         ),
#     ])
