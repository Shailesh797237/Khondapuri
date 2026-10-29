# ================================================================
#  obstacle_detector_3d.launch.py
#
#  Standalone launch file for the 3D obstacle detector.
#
#  This can be launched ALONGSIDE your existing navigation.launch.py
#  without any modifications to the existing stack.
#
#  USAGE:
#    ros2 launch lidar3d_obstacle_monitor obstacle_detector_3d.launch.py
#
#  OPTIONAL ARGS:
#    pcd_map:=/path/to/your/map.pcd
#    live_cloud_topic:=/livox/lidar
#
#  PREREQUISITES:
#    1. Robot hardware + Livox driver must be running
#    2. navigation.launch.py must be running (provides AMCL TF)
#    3. The .pcd map must exist (saved from dual_mapping_launch.py)
# ================================================================

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_dir = get_package_share_directory('lidar3d_obstacle_monitor')

    config_file = os.path.join(pkg_dir, 'config', 'obstacle_detector_3d.yaml')

    # ── Launch Arguments ──────────────────────────────────────────
    declare_pcd_map = DeclareLaunchArgument(
        'pcd_map',
        default_value='/home/taikisha/turn_ws/scans.pcd',
        description='Path to the 3D PCD map saved by FAST-LIO2')

    declare_live_topic = DeclareLaunchArgument(
        'live_cloud_topic',
        default_value='/livox/lidar',
        description='Live PointCloud2 topic from Livox Mid-360')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock')

    # ── Startup Banner ────────────────────────────────────────────
    startup_banner = LogInfo(msg='\n\n'
        '╔══════════════════════════════════════════════════════════════╗\n'
        '║     3D OBSTACLE DETECTOR — Using PCD Map                   ║\n'
        '║                                                            ║\n'
        '║  This node detects obstacles at ALL heights:               ║\n'
        '║    • Ground-level (already covered by 2D scan)             ║\n'
        '║    • MID-level (between scan and ceiling)                  ║\n'
        '║    • OVERHEAD (top surface / hanging objects)              ║\n'
        '║                                                            ║\n'
        '║  Publishes:                                                ║\n'
        '║    /obstacle_3d_overhead  → Bool (TRUE = overhead danger)  ║\n'
        '║    /obstacle_3d_cloud     → PointCloud2 (RViz)            ║\n'
        '║    /known_map_cloud       → PointCloud2 (RViz)            ║\n'
        '║    /obstacle_3d_marker    → Marker (RViz bounding box)    ║\n'
        '╚══════════════════════════════════════════════════════════════╝\n'
    )

    # ── 3D Obstacle Detector Node ─────────────────────────────────
    obstacle_detector_node = Node(
        package='lidar3d_obstacle_monitor',
        executable='obstacle_detector_3d',
        name='obstacle_detector_3d',
        output='screen',
        parameters=[
            config_file,
            {
                'pcd_map_path':     LaunchConfiguration('pcd_map'),
                'live_cloud_topic': LaunchConfiguration('live_cloud_topic'),
                'use_sim_time':     LaunchConfiguration('use_sim_time'),
            },
        ],
    )

    return LaunchDescription([
        declare_pcd_map,
        declare_live_topic,
        declare_use_sim_time,
        startup_banner,
        obstacle_detector_node,
    ])
