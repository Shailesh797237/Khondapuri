# ================================================================
#  livox_3d_bringup.launch.py
#
#  Launches Livox MID-360 3D LiDAR + PointCloud2 to LaserScan converter
#  for 2D AMCL Navigation & Obstacle Avoidance.
# ================================================================

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    livox_pkg = get_package_share_directory('livox_ros_driver2')
    bringup_pkg = get_package_share_directory('my_robot_bringup')
    pointcloud_to_scan_config = os.path.join(bringup_pkg, 'config', 'pointcloud_to_laserscan.yaml')

    # 1. Livox MID-360 LiDAR Driver (xfer_format=0 -> sensor_msgs/PointCloud2)
    livox_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(livox_pkg, 'launch_ROS2', 'msg_MID360_launch.py')
        )
    )

    # 2. PointCloud2 -> LaserScan Converter (/livox/lidar -> /scan_raw)
    pointcloud_to_laserscan_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        output='screen',
        parameters=[pointcloud_to_scan_config],
        remappings=[
            ('cloud_in', '/livox/lidar'),
            ('scan', '/scan_raw'),
        ],
    )

    # 3. Scan QoS Relay (/scan_raw [BEST_EFFORT] -> /scan [RELIABLE])
    scan_qos_bridge = Node(
        package='my_robot_bringup',
        executable='scan_qos_relay.py',
        name='scan_qos_bridge',
        output='screen',
    )

    # 4. Static TF: base_link -> livox_frame
    #    CRITICAL: Without this, pointcloud_to_laserscan cannot transform
    #    the Livox cloud into the robot frame, causing scan to rotate with robot!
    livox_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='livox_tf_publisher',
        arguments=[
            '0', '0', '0.334',     # x y z offset from base_link
            '0', '0', '0',         # roll pitch yaw
            'base_link',
            'livox_frame'
        ],
        output='screen',
    )

    return LaunchDescription([
        livox_driver,
        livox_tf,
        pointcloud_to_laserscan_node,
        scan_qos_bridge,
    ])
