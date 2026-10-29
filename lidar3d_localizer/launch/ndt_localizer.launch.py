import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg = get_package_share_directory('lidar3d_localizer')
    config = os.path.join(pkg, 'config', 'ndt_localizer.yaml')

    return LaunchDescription([

        DeclareLaunchArgument('pcd_map',
            default_value='/home/taikisha/turn_ws/scans.pcd'),

        DeclareLaunchArgument('map_offset_x',   default_value='0.0'),
        DeclareLaunchArgument('map_offset_y',   default_value='0.0'),
        DeclareLaunchArgument('map_offset_yaw', default_value='0.0'),

        DeclareLaunchArgument('use_sim_time', default_value='false'),

        LogInfo(msg='\n'
            '╔══════════════════════════════════════════════════════════╗\n'
            '║  3D NDT LOCALIZER — Run alongside navigation.launch.py ║\n'
            '║  Corrects AMCL drift using 3D PCD map scan matching    ║\n'
            '╚══════════════════════════════════════════════════════════╝\n'),

        Node(
            package='lidar3d_localizer',
            executable='ndt_localizer',
            name='ndt_localizer',
            output='screen',
            parameters=[
                config,
                {
                    'pcd_map_path':    LaunchConfiguration('pcd_map'),
                    'map_offset_x':    LaunchConfiguration('map_offset_x'),
                    'map_offset_y':    LaunchConfiguration('map_offset_y'),
                    'map_offset_yaw':  LaunchConfiguration('map_offset_yaw'),
                    'use_sim_time':    LaunchConfiguration('use_sim_time'),
                },
            ],
        ),
    ])
