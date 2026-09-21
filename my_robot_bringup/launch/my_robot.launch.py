# import os

# import xacro
# from launch import LaunchDescription
# from launch.actions import IncludeLaunchDescription, TimerAction
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from launch_ros.actions import Node
# from ament_index_python.packages import get_package_share_directory


# def generate_launch_description():

#     # -------------------------------
#     # Package paths
#     # -------------------------------
#     description_pkg = get_package_share_directory('my_robot_description')
#     bringup_pkg = get_package_share_directory('my_robot_bringup')
#     gazebo_pkg = get_package_share_directory('gazebo_ros')

#     # -------------------------------
#     # File paths
#     # -------------------------------
#     urdf_path = os.path.join(description_pkg, 'urdf', 'my_robot.urdf.xacro')
#     rviz_config_path = os.path.join(description_pkg, 'rviz', 'urdf_config.rviz')
#     controllers_path = os.path.join(bringup_pkg, 'config', 'my_robot_controllers.yaml')

#     # -------------------------------
#     # Process URDF/Xacro
#     # -------------------------------
#     doc = xacro.process_file(urdf_path)
#     robot_desc = doc.toxml()
#     robot_description = {'robot_description': robot_desc}

#     # -------------------------------
#     # Launch Gazebo
#     # -------------------------------
#     gazebo = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(
#             os.path.join(gazebo_pkg, 'launch', 'gazebo.launch.py')
#         ),
#         launch_arguments={
#             'world': os.path.join(gazebo_pkg, 'worlds', 'empty.world')
#         }.items()
#     )

#     # -------------------------------
#     # Robot State Publisher
#     # -------------------------------
#     rsp_node = Node(
#         package='robot_state_publisher',
#         executable='robot_state_publisher',
#         output='screen',
#         parameters=[robot_description]
#     )

#     # -------------------------------
#     # ros2_control node
#     # -------------------------------
#     ros2_control_node = Node(
#         package='controller_manager',
#         executable='ros2_control_node',
#         parameters=[
#             robot_description,
#             controllers_path
#         ],
#         output='screen'
#     )

#     # -------------------------------
#     # Spawn robot in Gazebo (delay to ensure Gazebo is running)
#     # -------------------------------
#     spawn_robot = Node(
#         package='gazebo_ros',
#         executable='spawn_entity.py',
#         arguments=['-entity', 'my_robot', '-topic', 'robot_description', '-x', '0', '-y', '0', '-z', '0.1'],
#         output='screen'
#     )

#     # -------------------------------
#     # Controller spawners
#     # -------------------------------
#     joint_state_broadcaster_spawner = TimerAction(
#         period=3.0,
#         actions=[
#             Node(
#                 package='controller_manager',
#                 executable='spawner',
#                 arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
#                 output='screen'
#             )
#         ]
#     )

#     diff_drive_controller_spawner = TimerAction(
#         period=5.0,
#         actions=[
#             Node(
#                 package='controller_manager',
#                 executable='spawner',
#                 arguments=['diff_drive_controller', '--controller-manager', '/controller_manager'],
#                 output='screen'
#             )
#         ]
#     )

#     # -------------------------------
#     # RViz
#     # -------------------------------
#     rviz_node = Node(
#         package='rviz2',
#         executable='rviz2',
#         arguments=['-d', rviz_config_path],
#         output='screen'
#     )

#     # -------------------------------
#     # Return LaunchDescription
#     # -------------------------------
#     return LaunchDescription([
#         rsp_node,
#         gazebo,
#         ros2_control_node,
#         spawn_robot,
#         joint_state_broadcaster_spawner,
#         diff_drive_controller_spawner,
#         rviz_node
#     ])


# import os

# from launch import LaunchDescription
# from launch.actions import IncludeLaunchDescription
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from launch_ros.actions import Node
# from launch.substitutions import Command
# from launch_ros.parameter_descriptions import ParameterValue
# from ament_index_python.packages import get_package_share_directory


# def generate_launch_description():

#     # Package paths
#     description_pkg = get_package_share_directory('my_robot_description')
#     bringup_pkg = get_package_share_directory('my_robot_bringup')
#     ros_gz_pkg = get_package_share_directory('ros_gz_sim')

#     urdf_path = os.path.join(description_pkg, 'urdf', 'my_robot.urdf.xacro')
#     rviz_config_path = os.path.join(description_pkg, 'rviz', 'urdf_config.rviz')
#     bridge_config_path = os.path.join(bringup_pkg, 'config', 'gz_bridge.yaml')
#     controllers_path = os.path.join(bringup_pkg, 'config', 'my_robot_controllers.yaml')

#     robot_description = ParameterValue(
#         Command(['xacro ', urdf_path]),
#         value_type=str
#     )
#     return LaunchDescription([

#         # Robot State Publisher
#         Node(
#             package='robot_state_publisher',
#             executable='robot_state_publisher',
#             output='screen',
#             parameters=[{
#                 'robot_description': robot_description
#             }]
#         ),

#         # Launch Gazebo (ros_gz_sim)
#         IncludeLaunchDescription(
#             PythonLaunchDescriptionSource(
#                 os.path.join(ros_gz_pkg, 'launch', 'gz_sim.launch.py')
#             ),
#             launch_arguments={'gz_args': 'empty.sdf -r'}.items(),
#         ),

#         # Spawn robot into Gazebo
#         Node(
#             package='ros_gz_sim',
#             executable='create',
#             arguments=['-name', 'my_robot', '-topic', 'robot_description'],
#             output='screen'
#         ),

#         # Bridge
#         Node(
#             package='ros_gz_bridge',
#             executable='parameter_bridge',
#             parameters=[bridge_config_path],
#             output='screen'
#         ),

#         # ros2_control node
#         Node(
#             package='controller_manager',
#             executable='ros2_control_node',
#             parameters=[
#                 {'robot_description': robot_description},
#                 controllers_path
#             ],
#             remappings=[('/cmd_vel','/diff_drive_controller/cmd_vel_unstamped', '/cmd_vel')],
#             output='screen'
#         ),

#         # Joint State Broadcaster
#         Node(
#             package='controller_manager',
#             executable='spawner',
#             arguments=['joint_state_broadcaster',
#                        '--controller-manager', '/controller_manager'],
#             output='screen'
#         ),

#         # Diff Drive Controller
#         Node(
#             package='controller_manager',
#             executable='spawner',
#             arguments=['diff_drive_controller',
#                        '--controller-manager', '/controller_manager'],
#             output='screen'
#         ),

#         # RViz
#         Node(
#             package='rviz2',
#             executable='rviz2',
#             arguments=['-d', rviz_config_path],
#             output='screen'
#         ),

#     ])












from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
import os


def generate_launch_description():

    pkg_share = FindPackageShare('my_robot_description')

    urdf_file = PathJoinSubstitution(
        [pkg_share, 'urdf', 'my_robot.urdf.xacro']
    )

    rviz_config_file = PathJoinSubstitution(
        [pkg_share, 'rviz', 'urdf_config.rviz']
    )

    controllers_file = PathJoinSubstitution(
        [FindPackageShare('my_robot_bringup'),
         'config',
         'my_robot_controllers.yaml']
    )

    twist_mux_params = PathJoinSubstitution(
        [FindPackageShare('my_robot_bringup'),
         'config',
         'twist_mux.yaml']
    )

    ekf_config = PathJoinSubstitution(
        [FindPackageShare('my_robot_bringup'),
         'config',
         'ekf.yaml']
    )

    # ── Robot Description ──────────────────────────────────────────────────────
    robot_description = ParameterValue(
        Command([
            FindExecutable(name='xacro'),
            ' ',
            urdf_file
        ]),
        value_type=str
    )

    # ── Twist Mux ──────────────────────────────────────────────────────────────
    twist_mux = Node(
        package='twist_mux',
        executable='twist_mux',
        parameters=[twist_mux_params, {'use_sim_time': False}],
        remappings=[('/cmd_vel_out', '/diff_drive_controller/cmd_vel_unstamped')],
        output='screen'
    )

    # ── Xsens MTi-320 IMU Driver ───────────────────────────────────────────────
    xsens_imu = Node(
        package='xsens_mti_ros2_driver',
        executable='xsens_mti_node',
        name='xsens_mti_node',
        output='screen',
        # Publishes:
        #   /imu/data          ← used by EKF
        #   /imu/mag
        #   /filter/quaternion
        #   /filter/euler
        #   /filter/free_acceleration
    )

    # ── Static TF: base_link → imu_link ───────────────────────────────────────
    # Adjust x y z to match physical mounting position of MTi-320 on your robot
    imu_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='imu_tf_publisher',
        arguments=[
            '0.0', '0.0', '0.1',   # x y z offset from base_link (adjust!)
            '0',    '0',   '0',    # roll pitch yaw (adjust if IMU is rotated)
            'base_link',
            'imu_link'
        ],
        output='screen'
    )

    # ── EKF Node ───────────────────────────────────────────────────────────────
    # Fuses: /diff_drive_controller/odom  +  /imu/data
    # Output: /odometry/filtered  (use THIS in Nav2 instead of raw wheel odom)
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config],
        remappings=[
            ('odometry/filtered', '/odometry/filtered')
        ]
    )

    return LaunchDescription([

        # ── Robot State Publisher ──────────────────────────────────────────────
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description
            }]
        ),

        # ── ROS2 Control Node ──────────────────────────────────────────────────
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[
                {'robot_description': robot_description},
                controllers_file
            ],
            output='screen'
        ),

        # ── Controller Spawners ────────────────────────────────────────────────
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=[
                'joint_state_broadcaster',
                '--controller-manager',
                '/controller_manager'
            ],
            output='screen'
        ),

        Node(
            package='controller_manager',
            executable='spawner',
            arguments=[
                'diff_drive_controller',
                '--controller-manager',
                '/controller_manager'
            ],
            output='screen'
        ),

        # ── Twist Mux ──────────────────────────────────────────────────────────
        twist_mux,

        # ── Xsens MTi-320 IMU ──────────────────────────────────────────────────

        # ── IMU Static TF ──────────────────────────────────────────────────────
        imu_tf,

        # ── EKF (fuses wheel odom + IMU) ───────────────────────────────────────
        ekf_node,

        # ── RViz (optional) ────────────────────────────────────────────────────
        # Node(
        #     package='rviz2',
        #     executable='rviz2',
        #     arguments=['-d', rviz_config_file],
        #     output='screen'
        # )
    ])

