# ================================================================
#  dual_mapping_launch.py
#
#  SIMULTANEOUS 2D + 3D MAPPING  (NO RVIZ)
#
#  This launch file runs BOTH mapping pipelines at the same time:
#
#  ┌─────────────────────────────────────────────────────────┐
#  │  3D MAPPING (FAST-LIO2)                                │
#  │    Input:  /livox/lidar_synced (PointCloud2)            │
#  │           /imu/data (IMU)                               │
#  │    Output: /cloud_registered (3D point cloud map)       │
#  │           /Odometry (LIO odometry)                      │
#  │           PCD file saved on Ctrl+C                      │
#  ├─────────────────────────────────────────────────────────┤
#  │  PointCloud → LaserScan Converter                       │
#  │    Input:  /livox/lidar_synced (PointCloud2)            │
#  │    Output: /scan (LaserScan for 2D SLAM)                │
#  ├─────────────────────────────────────────────────────────┤
#  │  2D MAPPING (slam_toolbox)                              │
#  │    Input:  /scan (LaserScan)                            │
#  │    Output: /map (OccupancyGrid — 2D nav map)            │
#  │           map → odom TF                                 │
#  └─────────────────────────────────────────────────────────┘
#
#  PREREQUISITES:
#    1. On ROBOT: ros2 launch my_robot_bringup my_robot.launch.py
#       (starts hardware + Livox driver + restamper + IMU)
#    2. Install pointcloud_to_laserscan:
#       sudo apt install ros-humble-pointcloud-to-laserscan
#    3. Install slam_toolbox:
#       sudo apt install ros-humble-slam-toolbox
#    4. FAST-LIO2 must be built in your workspace
#
#  USAGE:
#    ros2 launch my_robot_bringup dual_mapping_launch.py
#
#  NOTE: RViz2 is NOT launched in this file. To visualize, run
#  RViz2 separately in another terminal, e.g.:
#    ros2 run rviz2 rviz2 -d <path_to_your_rviz_config>
#
#  AFTER MAPPING:
#    - 2D map: Save via slam_toolbox plugin in RViz, or:
#        ros2 run nav2_map_server map_saver_cli -f ~/ekf_ws/my_map
#    - 3D map: PCD auto-saved when you press Ctrl+C
#    - Use both maps for navigation (2D for Nav2 + 3D for obstacle avoidance)
# ================================================================

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from nav2_common.launch import HasNodeParams


def generate_launch_description():

    pkg_dir = get_package_share_directory('my_robot_bringup')

    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')

    # ── Default config files ──────────────────────────────────────
    default_slam_params = os.path.join(
        pkg_dir, 'config', 'mapper_params_online_async.yaml')

    fastlio2_config = os.path.join(
        pkg_dir, 'config', 'fastlio2_mid360.yaml')

    pointcloud_to_scan_config = os.path.join(
        pkg_dir, 'config', 'pointcloud_to_laserscan.yaml')

    # ── Launch Arguments ──────────────────────────────────────────
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock (false for real robot)')

    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=default_slam_params,
        description='Full path to slam_toolbox params file')

    # ── Check if params file contains slam_toolbox params ─────────
    has_node_params = HasNodeParams(
        source_file=params_file, node_name='slam_toolbox')

    actual_params_file = PythonExpression([
        '"', params_file, '" if ', has_node_params,
        ' else "', default_slam_params, '"'])

    log_param_change = LogInfo(
        msg=['provided params_file ', params_file,
             ' does not contain slam_toolbox parameters. Using default: ',
             default_slam_params],
        condition=UnlessCondition(has_node_params))

    # ══════════════════════════════════════════════════════════════
    #  STARTUP BANNER
    # ══════════════════════════════════════════════════════════════
    startup_banner = LogInfo(msg='\n\n'
        '╔══════════════════════════════════════════════════════════════╗\n'
        '║     DUAL MAPPING — 2D + 3D Simultaneous Mapping (No RViz) ║\n'
        '║                                                            ║\n'
        '║  3D MAP (FAST-LIO2):                                      ║\n'
        '║    ← /livox/lidar_synced + /imu/data                      ║\n'
        '║    → /cloud_registered (3D point cloud)                    ║\n'
        '║    → PCD file saved on Ctrl+C                              ║\n'
        '║                                                            ║\n'
        '║  PointCloud → LaserScan:                                   ║\n'
        '║    ← /livox/lidar_synced (PointCloud2)                    ║\n'
        '║    → /scan (2D LaserScan for SLAM)                        ║\n'
        '║                                                            ║\n'
        '║  2D MAP (slam_toolbox):                                   ║\n'
        '║    ← /scan (LaserScan)                                    ║\n'
        '║    → /map (OccupancyGrid for Nav2)                        ║\n'
        '║                                                            ║\n'
        '║  Drive the robot around to build both maps!                ║\n'
        '║  Press Ctrl+C when done.                                   ║\n'
        '╚══════════════════════════════════════════════════════════════╝\n'
    )

    # ══════════════════════════════════════════════════════════════
    #  NODE 1: PointCloud2 → LaserScan Converter
    #
    #  Converts the MID-360's 3D point cloud into a 2D laser scan
    #  so that slam_toolbox can use it for 2D OccupancyGrid mapping.
    #
    #  Only the ground-level slice of the point cloud is extracted
    #  (configurable height range in the YAML config).
    #
    #  NOTE: This node publishes with BEST_EFFORT QoS (SensorDataQoS),
    #  but slam_toolbox subscribes with RELIABLE QoS → INCOMPATIBLE!
    #  So we publish to /scan_raw and relay via scan_qos_bridge below.
    # ══════════════════════════════════════════════════════════════
    pointcloud_to_laserscan_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        output='screen',
        parameters=[
            pointcloud_to_scan_config,
            {'use_sim_time': use_sim_time},
        ],
        remappings=[
            ('cloud_in', '/livox/lidar'),
            ('scan', '/scan_raw'),           # → publish to /scan_raw (BEST_EFFORT)
        ],
    )

    # ══════════════════════════════════════════════════════════════
    #  NODE 1b: QoS Bridge — /scan_raw (BEST_EFFORT) → /scan (RELIABLE)
    #
    #  slam_toolbox requires RELIABLE QoS on /scan, but
    #  pointcloud_to_laserscan only publishes BEST_EFFORT.
    #  This relay node bridges the two QoS policies.
    # ══════════════════════════════════════════════════════════════
    scan_qos_bridge = Node(
        package='my_robot_bringup',
        executable='scan_qos_relay.py',
        name='scan_qos_bridge',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # ══════════════════════════════════════════════════════════════
    #  NODE 2: slam_toolbox (2D Mapping)
    #
    #  Subscribes to /scan and creates a 2D OccupancyGrid (/map).
    #  This is the same node used in online_async_launch.py.
    #  The 2D map is used by Nav2 for path planning & costmaps.
    # ══════════════════════════════════════════════════════════════
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            actual_params_file,
            {'use_sim_time': use_sim_time},
        ],
    )

    # ══════════════════════════════════════════════════════════════
    #  NODE 3: FAST-LIO2 (3D Mapping)
    #
    #  Subscribes to /livox/lidar_synced (PointCloud2) and
    #  /imu/data (IMU) to build a 3D point cloud map.
    #  Publishes /cloud_registered and saves PCD on shutdown.
    # ══════════════════════════════════════════════════════════════
    fastlio2_node = Node(
        package='fast_lio',
        executable='fastlio_mapping',
        name='fast_lio',
        output='screen',
        parameters=[
            fastlio2_config,
            {'use_sim_time': use_sim_time},
        ],
    )

    # ══════════════════════════════════════════════════════════════
    #  NODE 4: Map-to-Camera_Init Static TF Bridge
    #
    #  Connects the 2D map frame and 3D camera_init frame.
    #  This links the disconnected TF trees so both maps can be
    #  visualized simultaneously (e.g. if RViz is opened separately)
    #  under the "map" Fixed Frame.
    # ══════════════════════════════════════════════════════════════
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_camera_init_bridge',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'camera_init'],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # ══════════════════════════════════════════════════════════════
    #  Build Launch Description
    # ══════════════════════════════════════════════════════════════
    ld = LaunchDescription()

    # Launch arguments
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_params_file)

    # Info
    ld.add_action(log_param_change)
    ld.add_action(startup_banner)

    # Nodes — all run simultaneously
    ld.add_action(pointcloud_to_laserscan_node)   # PointCloud2 → /scan_raw (BEST_EFFORT)
    ld.add_action(scan_qos_bridge)                 # /scan_raw → /scan (RELIABLE)
    ld.add_action(slam_toolbox_node)               # /scan → 2D /map
    ld.add_action(fastlio2_node)                   # PointCloud2 + IMU → 3D map
    ld.add_action(static_tf_node)                  # Connect 2D + 3D TF trees

    return ld

