# ================================================================
#  triple_lidar_bringup.launch.py
#
#  Launches 3 LiDARs (Livox MID-360 3D + 2x SICK 2D Front/Back)
#  and merges them into a single 360° /scan topic for 2D Mapping & Navigation.
# ================================================================

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    bringup_pkg = get_package_share_directory('my_robot_bringup')
    livox_pkg   = get_package_share_directory('livox_ros_driver2')
    
    pointcloud_to_scan_config = os.path.join(bringup_pkg, 'config', 'pointcloud_to_laserscan.yaml')

    # 1. Livox MID-360 LiDAR Driver (3D PointCloud on /livox/lidar)
    livox_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(livox_pkg, 'launch_ROS2', 'msg_MID360_launch.py')
        )
    )

    # 2. PointCloud2 → LaserScan Converter (/livox/lidar → /livox/scan)
    pointcloud_to_laserscan_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        output='screen',
        parameters=[pointcloud_to_scan_config],
        remappings=[
            ('cloud_in', '/livox/lidar'),
            ('scan', '/livox/scan'),
        ],
    )

    # 3. Triple Scan Merger Node
    # Merges: /livox/scan (Livox 3D slice), /front/scan (SICK Front), /back/scan (SICK Back)
    # Outputs: /scan (unified 360° LaserScan in base_link frame)
    triple_scan_merger_node = Node(
        package='dual_scan_merger',
        executable='triple_scan_merger',
        name='triple_scan_merger',
        output='screen',
        parameters=[{
            'topic_livox': '/livox/scan',
            'topic_front': '/front/scan',
            'topic_back': '/back/scan',
            'output_topic': '/scan',
            'output_frame': 'base_link',
            'max_range': 30.0,
            'min_range': 0.15,
            'robot_body_radius': 0.45,
            'angle_increment': 0.00436332,
        }]
    )

    return LaunchDescription([
        livox_driver,
        pointcloud_to_laserscan_node,
        triple_scan_merger_node,
    ])
