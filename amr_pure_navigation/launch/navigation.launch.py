import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import LaunchConfigurationEquals
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    pkg = get_package_share_directory('amr_pure_navigation')

    use_sim_time   = LaunchConfiguration('use_sim_time')
    map_yaml       = LaunchConfiguration('map')
    waypoints_file = LaunchConfiguration('waypoints_file')
    missions_folder = LaunchConfiguration('missions_folder')
    params_file    = LaunchConfiguration('params_file')
    plc_enabled    = LaunchConfiguration('plc_enabled', default='true')

    return LaunchDescription([

        DeclareLaunchArgument('use_sim_time', default_value='false'),

        DeclareLaunchArgument(
            'map',
            default_value='/home/taikisha/turn_ws/install/share/amr_pure_navigation/map'),

        DeclareLaunchArgument(
            'waypoints_file',
            default_value='',
            description='Path to single waypoints yaml (backward compatible)'),

        DeclareLaunchArgument(
            'missions_folder',
            default_value='/home/taikisha/turn_ws/install/share/amr_pure_navigation/config/missions',
            description='Path to folder with mission .yaml files (each file = one mission)'),

        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(pkg, 'config', 'amcl_map_params.yaml')),

        DeclareLaunchArgument(
            'plc_enabled',
            default_value='true',
            description='Enable PLC integration for station triggers'),

        # ── 1. Map Server ──────────────────────────────────────────
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[
                params_file,
                {'use_sim_time': use_sim_time},
                {'yaml_filename': map_yaml},
            ],
        ),

        # ── 2. AMCL ───────────────────────────────────────────────
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[
                params_file,
                {'use_sim_time': use_sim_time},
            ],
        ),

        # ── 3. Lifecycle Manager ───────────────────────────────────
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time},
                {'autostart':    True},
                {'node_names':   ['map_server', 'amcl']},
                {'bond_timeout': 60.0},
                {'bond_respawn_max_duration': 60.0},
            ],
        ),

        # ── 4. Navigation + TCP Bridge (starts 30 sec after AMCL) ──────
        TimerAction(period=30.0, actions=[
            Node(
                package='amr_pure_navigation',
                executable='waypoint_navigator',
                name='waypoint_navigator',
                output='screen',
                parameters=[{
                    'waypoints_file':    waypoints_file,
                    'missions_folder':   missions_folder,
                    'amcl_topic':        '/amcl_pose',
                    # 'odom_topic':        '/diff_drive_controller/odom',
                    'odom_topic':        '/odometry/filtered',
                    'cmd_vel_topic':     '/cmd_vel',
                    'use_amcl':          True,
                    'linear_speed':      0.30,
                    'angular_speed':     0.15,
                    'drive_correction':  0.15,   # keeps RPM equal = straight line
                    'xy_tolerance':      0.10,
                    'yaw_tolerance':     0.08,
                    'heading_tolerance': 0.10,
                    'stop_on_obstacle':  True,
                    'obstacle_stop_dist':0.40,
                    'loop_waypoints':    False,
                    'plc_enabled':       plc_enabled,
                    'use_sim_time':      use_sim_time,
                }],
            ),
            # TCP Mission Bridge — port 5000 for external mission control
            # Using ExecuteProcess instead of Node to work around ROS2 Humble
            # launch bug that silently drops this executable
            ExecuteProcess(
                cmd=[
                    'ros2', 'run', 'amr_pure_navigation', 'tcp_mission_bridge',
                    '--ros-args', '-p', 'tcp_port:=5000',
                ],
                output='screen',
            ),
        ]),

        # ── 5. PLC Station Monitor ─────────────────────────────────
        #  CONTINUOUSLY monitors DB1.DBX0.4 and publishes on /plc_trigger
        #  When station reached + bit=TRUE → publishes /plc_done
        #  Always runs with navigation stack (no condition)
        TimerAction(period=30.0, actions=[
            Node(
                package='amr_pure_navigation',
                executable='plc_station_monitor',
                name='plc_station_monitor',
                output='screen',
                parameters=[{
                    'plc_ip':     '192.168.1.21',
                    'rack':       0,
                    'slot':       1,
                    'plc_db':     1,        # DB number  → DB1
                    'plc_byte':   0,        # byte offset → byte 0
                    'plc_bit':    4,        # bit index  → bit 4  = DB1.DBX0.4
                    'poll_rate':  10.0,     # check 10 times/sec
                }],
            ),
        ]),

        # ── 6. PLC Lidar Obstacle Monitor ──────────────────────────────
        #  CONTINUOUSLY monitors DB1.DBX1.5 (front lidar obstacle bit)
        #  Publishes on /lidar_obstacle (Bool) every poll cycle
        #  waypoint_navigator subscribes → ramps speed down/up
        TimerAction(period=30.0, actions=[
            Node(
                package='amr_pure_navigation',
                executable='plc_lidar_monitor',
                name='plc_lidar_monitor',
                output='screen',
                parameters=[{
                    'plc_ip':     '192.168.1.21',
                    'rack':       0,
                    'slot':       1,
                    'plc_db':     1,        # DB number  → DB1
                    'plc_byte':   1,        # byte offset → byte 1
                    'plc_bit':    5,        # bit index  → bit 5  = DB1.DBX1.5
                    'poll_rate':  10.0,     # check 10 times/sec
                }],
            ),
        ]),

        # ── 7. PLC Emergency Button Monitor ────────────────────────────
        #  CONTINUOUSLY monitors DB1.DBX0.5 (emergency button)
        #  PLC bit TRUE = normal, FALSE = emergency pressed (INVERTED)
        #  Publishes on /emergency_stop (Bool) every poll cycle
        #  waypoint_navigator subscribes → stops robot in 0.2s
        TimerAction(period=30.0, actions=[
            Node(
                package='amr_pure_navigation',
                executable='plc_emergency_monitor',
                name='plc_emergency_monitor',
                output='screen',
                parameters=[{
                    'plc_ip':     '192.168.1.21',
                    'rack':       0,
                    'slot':       1,
                    'plc_db':     1,        # DB number  → DB1
                    'plc_byte':   0,        # byte offset → byte 0
                    'plc_bit':    5,        # bit index  → bit 5  = DB1.DBX0.5
                    'poll_rate':  10.0,     # check 10 times/sec
                }],
            ),
        ]),

        # ── 8. Pose Persistence — Auto-Localization ────────────────────
        #  Saves AMCL pose to file every 5 seconds.
        #  On startup: restores last known pose → no manual localization!
        Node(
            package='amr_pure_navigation',
            executable='pose_persistence',
            name='pose_persistence',
            output='screen',
            parameters=[{
                'pose_file':      '/home/taikisha/turn_ws/src/amr_pure_navigation/config/last_robot_pose.yaml',
                'save_interval':  8.0,
                'max_pose_age':   720.0,
                'restore_delay':  3.0,
            }],
        ),
        # ── 9. Mission Publisher ───────────────────────────────────────
        #  Publishes available missions in JSON format to /available_missions
        #  for Web UI and fleet management
        Node(
            package='amr_pure_navigation',
            executable='mission_publisher',
            name='mission_publisher',
            output='screen',
            parameters=[{
                'mission_path': missions_folder,
            }],
        ),

    ])






# # ================================================================
# #  navigation_launch.py
# #
# #  Starts:
# #    1. map_server        — serves your map
# #    2. amcl              — localizes robot on map (no drift!)
# #    3. lifecycle_manager — manages map_server + amcl
# #    4. waypoint_navigator— YOUR custom navigation node
# #
# #  After launching:
# #    → Open RViz
# #    → Click "2D Pose Estimate"
# #    → Click on map where robot actually is
# #    → Robot will start moving through waypoints automatically
# # ================================================================

# import os
# from ament_index_python.packages import get_package_share_directory
# from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument, TimerAction
# from launch.substitutions import LaunchConfiguration
# from launch_ros.actions import Node


# def generate_launch_description():

#     pkg = get_package_share_directory('amr_pure_navigation')

#     use_sim_time   = LaunchConfiguration('use_sim_time')
#     params_file    = LaunchConfiguration('params_file')
#     map_yaml       = LaunchConfiguration('map')
#     waypoints_file = LaunchConfiguration('waypoints_file')

#     # Only AMCL + map_server are lifecycle managed
#     lifecycle_nodes = ['map_server', 'amcl']

#     remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

#     return LaunchDescription([

#         # ── Arguments ─────────────────────────────────────────────
#         DeclareLaunchArgument(
#             'use_sim_time',
#             default_value='false'),

#         DeclareLaunchArgument(
#             'params_file',
#             default_value=os.path.join(pkg, 'config', 'amcl_map_params.yaml'),
#             description='AMCL + map_server params'),

#         DeclareLaunchArgument(
#             'map',
#             default_value='/home/shailesh/ekf_ws/src/my_robot_description/rviz/taikisha.yaml',
#             description='Path to map yaml'),

#         DeclareLaunchArgument(
#             'waypoints_file',
#             default_value='/home/shailesh/ekf_ws/src/amr_pure_navigation/config/waypoints.yaml',
#             description='Path to waypoints yaml'),

#         # ── 1. Map Server ──────────────────────────────────────────
#         Node(
#             package='nav2_map_server',
#             executable='map_server',
#             name='map_server',
#             output='screen',
#             parameters=[
#                 params_file,
#                 {'use_sim_time': use_sim_time},
#                 {'yaml_filename': map_yaml},
#             ],
#             remappings=remappings,
#         ),

#         # ── 2. AMCL ───────────────────────────────────────────────
#         Node(
#             package='nav2_amcl',
#             executable='amcl',
#             name='amcl',
#             output='screen',
#             parameters=[
#                 params_file,
#                 {'use_sim_time': use_sim_time},
#             ],
#             remappings=remappings,
#         ),

#         # ── 3. Lifecycle Manager — only map_server + amcl ──────────
#         Node(
#             package='nav2_lifecycle_manager',
#             executable='lifecycle_manager',
#             name='lifecycle_manager_localization',
#             output='screen',
#             parameters=[
#                 {'use_sim_time': use_sim_time},
#                 {'autostart':    True},
#                 {'node_names':   lifecycle_nodes},
#             ],
#         ),

#         # ── 4. Your Custom Navigator ───────────────────────────────
#         # Delayed 5 sec — lets AMCL + map fully initialize first
#         TimerAction(
#             period=5.0,
#             actions=[
#                 Node(
#                     package='amr_pure_navigation',
#                     executable='waypoint_navigator',
#                     name='waypoint_navigator',
#                     output='screen',
#                     parameters=[{
#                         'waypoints_file':    waypoints_file,

#                         # Topic names
#                         'amcl_topic':        '/amcl_pose',
#                         'odom_topic':        '/diff_drive_controller/odom',
#                         'cmd_vel_topic':     '/cmd_vel',

#                         # Speed
#                         'linear_speed':      0.20,
#                         'angular_speed':     0.40,

#                         # Tolerances
#                         'xy_tolerance':      0.10,
#                         'yaw_tolerance':     0.08,
#                         'heading_tolerance': 0.10,

#                         # Obstacle
#                         'stop_on_obstacle':  True,
#                         'obstacle_stop_dist':0.40,

#                         # Loop
#                         'loop_waypoints':    False,

#                         'use_sim_time':      use_sim_time,
#                     }],
#                 ),
#             ],
#         ),

#     ])
