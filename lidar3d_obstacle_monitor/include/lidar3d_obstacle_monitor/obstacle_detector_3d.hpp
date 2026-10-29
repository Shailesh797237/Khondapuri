#pragma once

// ================================================================
//  obstacle_detector_3d.hpp
//
//  3D Obstacle Detection using a pre-built PCD map.
//
//  This node:
//    1. Loads a .pcd map (from FAST-LIO2) at startup
//    2. Builds an OcTree (3D voxel grid) of the known environment
//    3. Subscribes to live 3D point cloud from Livox Mid-360
//    4. Transforms live cloud into map frame (using TF from AMCL)
//    5. Crops to a corridor around the robot
//    6. Compares live points against the OcTree
//    7. Points NOT in the known map = dynamic obstacles
//    8. Filters by height to detect overhead obstacles
//    9. Publishes obstacle alerts and visualization clouds
//
//  DOES NOT modify any existing packages!
// ================================================================

#include <string>
#include <memory>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/octree/octree_search.h>

namespace lidar3d_obstacle_monitor
{

class ObstacleDetector3D : public rclcpp::Node
{
public:
  explicit ObstacleDetector3D(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // ── Callbacks ──────────────────────────────────────────────────
  void liveCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void processCloud();

  // ── Helpers ────────────────────────────────────────────────────
  bool loadPCDMap(const std::string & path);
  void buildOcTree();
  void publishMapCloud();

  // ── ROS interfaces ────────────────────────────────────────────
  // Subscribers
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr  live_cloud_sub_;

  // Publishers
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr               obstacle_3d_pub_;       // /obstacle_3d_overhead (Bool)
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr     obstacle_cloud_pub_;    // /obstacle_3d_cloud (PointCloud2)
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr     known_map_pub_;         // /known_map_cloud (PointCloud2)
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr   marker_pub_;            // /obstacle_3d_marker (Marker)

  // Timer
  rclcpp::TimerBase::SharedPtr  process_timer_;

  // TF2
  std::shared_ptr<tf2_ros::Buffer>            tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // ── PCL data ──────────────────────────────────────────────────
  pcl::PointCloud<pcl::PointXYZI>::Ptr  map_cloud_;           // loaded PCD map
  pcl::PointCloud<pcl::PointXYZI>::Ptr  latest_live_cloud_;   // latest incoming cloud

  using OcTreeT = pcl::octree::OctreePointCloudSearch<pcl::PointXYZI>;
  std::shared_ptr<OcTreeT>             map_octree_;           // spatial index of known map

  std::mutex  cloud_mutex_;
  bool        map_loaded_       {false};
  bool        map_published_    {false};
  bool        new_cloud_ready_  {false};

  // ── Parameters ────────────────────────────────────────────────
  std::string pcd_map_path_;
  std::string live_cloud_topic_;
  std::string robot_frame_;
  std::string map_frame_;

  double octree_resolution_;      // voxel size for OcTree (meters)
  double detection_radius_;       // corridor radius around robot (meters)
  double forward_distance_;       // how far ahead to check (meters)
  double min_overhead_height_;    // min height above base_link for overhead detection
  double max_overhead_height_;    // max height to check
  double min_ground_height_;      // min height for any obstacle (filter ground)
  int    min_obstacle_points_;    // min new points to trigger obstacle
  double check_rate_;             // Hz to process clouds
  double voxel_neighbor_radius_;  // search radius to match live→map points
};

}  // namespace lidar3d_obstacle_monitor
