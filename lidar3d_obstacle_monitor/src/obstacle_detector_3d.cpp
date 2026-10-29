// ================================================================
//  obstacle_detector_3d.cpp
//
//  3D Obstacle Detection using pre-built PCD map + live point cloud.
//
//  ALGORITHM:
//    1. Load .pcd map → build OcTree (3D spatial index)
//    2. For each live cloud:
//       a) Transform to map frame (using AMCL TF)
//       b) Crop to detection corridor around robot
//       c) For each live point, search OcTree:
//          - If point IS in known map → static (wall/floor/ceiling) → ignore
//          - If point is NOT in known map → dynamic obstacle!
//       d) Filter dynamic obstacles by height:
//          - Ground level (below min_ground_height_) → ignore
//          - Overhead (above min_overhead_height_)  → overhead obstacle!
//          - Mid-level (between ground and overhead) → regular obstacle
//       e) If enough new points detected → publish obstacle alert
//
//  TOPICS PUBLISHED:
//    /obstacle_3d_overhead  (Bool)        — TRUE when overhead obstacle in path
//    /obstacle_3d_cloud     (PointCloud2) — detected obstacle points (for RViz)
//    /known_map_cloud       (PointCloud2) — loaded PCD map (published once)
//    /obstacle_3d_marker    (Marker)      — bounding box around obstacles
// ================================================================

#include "lidar3d_obstacle_monitor/obstacle_detector_3d.hpp"

#include <pcl/io/pcd_io.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"
#include <tf2_eigen/tf2_eigen.hpp>

namespace lidar3d_obstacle_monitor
{

ObstacleDetector3D::ObstacleDetector3D(const rclcpp::NodeOptions & options)
: Node("obstacle_detector_3d", options)
{
  // ── Declare parameters ──────────────────────────────────────────
  declare_parameter<std::string>("pcd_map_path",     "/home/taikisha/turn_ws/scans.pcd");
  declare_parameter<std::string>("live_cloud_topic", "/livox/lidar");
  declare_parameter<std::string>("robot_frame",      "base_link");
  declare_parameter<std::string>("map_frame",        "map");

  declare_parameter<double>("octree_resolution",     0.15);   // 15 cm voxels
  declare_parameter<double>("detection_radius",      2.0);    // 2m around robot
  declare_parameter<double>("forward_distance",      3.0);    // 3m ahead
  declare_parameter<double>("min_overhead_height",   1.0);    // above 1m = overhead
  declare_parameter<double>("max_overhead_height",   4.0);    // don't check above 4m
  declare_parameter<double>("min_ground_height",     0.15);   // below 0.15m = ground noise
  declare_parameter<int>   ("min_obstacle_points",   10);     // need 10+ new points
  declare_parameter<double>("check_rate",            5.0);    // process at 5 Hz
  declare_parameter<double>("voxel_neighbor_radius", 0.20);   // match tolerance

  // ── Read parameters ─────────────────────────────────────────────
  get_parameter("pcd_map_path",        pcd_map_path_);
  get_parameter("live_cloud_topic",    live_cloud_topic_);
  get_parameter("robot_frame",         robot_frame_);
  get_parameter("map_frame",           map_frame_);
  get_parameter("octree_resolution",   octree_resolution_);
  get_parameter("detection_radius",    detection_radius_);
  get_parameter("forward_distance",    forward_distance_);
  get_parameter("min_overhead_height", min_overhead_height_);
  get_parameter("max_overhead_height", max_overhead_height_);
  get_parameter("min_ground_height",   min_ground_height_);
  get_parameter("min_obstacle_points", min_obstacle_points_);
  get_parameter("check_rate",          check_rate_);
  get_parameter("voxel_neighbor_radius", voxel_neighbor_radius_);

  // ── TF2 ─────────────────────────────────────────────────────────
  tf_buffer_   = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // ── Publishers ──────────────────────────────────────────────────
  obstacle_3d_pub_    = create_publisher<std_msgs::msg::Bool>(
    "/obstacle_3d_overhead", 10);

  obstacle_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
    "/obstacle_3d_cloud", rclcpp::SensorDataQoS());

  known_map_pub_      = create_publisher<sensor_msgs::msg::PointCloud2>(
    "/known_map_cloud", rclcpp::QoS(1).transient_local().reliable());

  marker_pub_         = create_publisher<visualization_msgs::msg::Marker>(
    "/obstacle_3d_marker", 10);

  // ── Load PCD map ────────────────────────────────────────────────
  map_cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  latest_live_cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();

  if (loadPCDMap(pcd_map_path_)) {
    buildOcTree();
    RCLCPP_INFO(get_logger(),
      "✅ 3D map loaded: %zu points, OcTree resolution: %.2fm",
      map_cloud_->size(), octree_resolution_);
  } else {
    RCLCPP_ERROR(get_logger(),
      "❌ Failed to load PCD map: %s — node will run but NO obstacle detection!",
      pcd_map_path_.c_str());
  }

  // ── Subscriber — live point cloud ───────────────────────────────
  //    Use SensorDataQoS (BEST_EFFORT) to match Livox driver output
  live_cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    live_cloud_topic_,
    rclcpp::SensorDataQoS(),
    std::bind(&ObstacleDetector3D::liveCloudCallback, this, std::placeholders::_1));

  // ── Processing timer ────────────────────────────────────────────
  double period_ms = 1000.0 / check_rate_;
  process_timer_ = create_wall_timer(
    std::chrono::milliseconds(static_cast<int>(period_ms)),
    std::bind(&ObstacleDetector3D::processCloud, this));

  RCLCPP_INFO(get_logger(),
    "\n╔══════════════════════════════════════════════════════════════╗\n"
    "║     3D OBSTACLE DETECTOR — Overhead & Full-Height          ║\n"
    "║                                                            ║\n"
    "║  PCD Map:    %s\n"
    "║  Live Topic: %s\n"
    "║  Check Rate: %.1f Hz                                       ║\n"
    "║  Overhead:   %.1fm — %.1fm above %s              ║\n"
    "║  Corridor:   %.1fm radius, %.1fm forward                  ║\n"
    "║                                                            ║\n"
    "║  Publishes:                                                ║\n"
    "║    /obstacle_3d_overhead  (Bool)                           ║\n"
    "║    /obstacle_3d_cloud     (PointCloud2 — RViz)             ║\n"
    "║    /known_map_cloud       (PointCloud2 — RViz)             ║\n"
    "╚══════════════════════════════════════════════════════════════╝",
    pcd_map_path_.c_str(), live_cloud_topic_.c_str(),
    check_rate_,
    min_overhead_height_, max_overhead_height_, robot_frame_.c_str(),
    detection_radius_, forward_distance_);
}


// ══════════════════════════════════════════════════════════════════
//  Load the .pcd map from disk
// ══════════════════════════════════════════════════════════════════
bool ObstacleDetector3D::loadPCDMap(const std::string & path)
{
  if (pcl::io::loadPCDFile<pcl::PointXYZI>(path, *map_cloud_) == -1) {
    return false;
  }

  if (map_cloud_->empty()) {
    RCLCPP_WARN(get_logger(), "PCD map loaded but is EMPTY!");
    return false;
  }

  // Downsample the map for faster OcTree operations
  pcl::VoxelGrid<pcl::PointXYZI> vg;
  pcl::PointCloud<pcl::PointXYZI>::Ptr filtered(
    new pcl::PointCloud<pcl::PointXYZI>());
  vg.setInputCloud(map_cloud_);
  vg.setLeafSize(
    static_cast<float>(octree_resolution_),
    static_cast<float>(octree_resolution_),
    static_cast<float>(octree_resolution_));
  vg.filter(*filtered);

  RCLCPP_INFO(get_logger(),
    "PCD map: %zu raw points → %zu after voxel downsampling (%.2fm)",
    map_cloud_->size(), filtered->size(), octree_resolution_);

  map_cloud_ = filtered;
  map_loaded_ = true;
  return true;
}


// ══════════════════════════════════════════════════════════════════
//  Build OcTree spatial index from the loaded map
// ══════════════════════════════════════════════════════════════════
void ObstacleDetector3D::buildOcTree()
{
  map_octree_ = std::make_shared<OcTreeT>(octree_resolution_);
  map_octree_->setInputCloud(map_cloud_);
  map_octree_->addPointsFromInputCloud();
  RCLCPP_INFO(get_logger(), "OcTree built with resolution %.3fm", octree_resolution_);
}


// ══════════════════════════════════════════════════════════════════
//  Publish the known map cloud (once, for RViz visualization)
// ══════════════════════════════════════════════════════════════════
void ObstacleDetector3D::publishMapCloud()
{
  if (!map_loaded_ || map_published_) return;

  sensor_msgs::msg::PointCloud2 ros_cloud;
  pcl::toROSMsg(*map_cloud_, ros_cloud);
  ros_cloud.header.frame_id = map_frame_;
  ros_cloud.header.stamp = this->now();
  known_map_pub_->publish(ros_cloud);
  map_published_ = true;

  RCLCPP_INFO(get_logger(),
    "📡 Published known map cloud (%zu points) on /known_map_cloud",
    map_cloud_->size());
}


// ══════════════════════════════════════════════════════════════════
//  Callback: store latest live cloud
// ══════════════════════════════════════════════════════════════════
void ObstacleDetector3D::liveCloudCallback(
  const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  // Publish map cloud on first live cloud received (ensures TF is ready)
  if (!map_published_ && map_loaded_) {
    publishMapCloud();
  }

  std::lock_guard<std::mutex> lock(cloud_mutex_);

  // Convert ROS msg → PCL cloud
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(
    new pcl::PointCloud<pcl::PointXYZI>());
  pcl::fromROSMsg(*msg, *cloud);

  latest_live_cloud_ = cloud;
  new_cloud_ready_ = true;
}


// ══════════════════════════════════════════════════════════════════
//  Timer callback: process the latest live cloud
// ══════════════════════════════════════════════════════════════════
void ObstacleDetector3D::processCloud()
{
  if (!map_loaded_ || !new_cloud_ready_) return;

  // ── Grab the latest cloud ──────────────────────────────────────
  pcl::PointCloud<pcl::PointXYZI>::Ptr live_cloud;
  {
    std::lock_guard<std::mutex> lock(cloud_mutex_);
    live_cloud = latest_live_cloud_;
    new_cloud_ready_ = false;
  }

  if (!live_cloud || live_cloud->empty()) return;

  // ── Get robot position in map frame ────────────────────────────
  geometry_msgs::msg::TransformStamped tf_robot_to_map;
  try {
    tf_robot_to_map = tf_buffer_->lookupTransform(
      map_frame_, robot_frame_,
      tf2::TimePointZero,
      std::chrono::milliseconds(100));
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
      "Cannot get TF %s→%s: %s", robot_frame_.c_str(), map_frame_.c_str(), ex.what());
    return;
  }

  double robot_x = tf_robot_to_map.transform.translation.x;
  double robot_y = tf_robot_to_map.transform.translation.y;
  double robot_z = tf_robot_to_map.transform.translation.z;

  // Get robot yaw for forward direction
  auto & q = tf_robot_to_map.transform.rotation;
  double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  double robot_yaw = std::atan2(siny_cosp, cosy_cosp);
  (void)robot_yaw;  // reserved for directional corridor filtering

  // ── Transform live cloud into map frame ────────────────────────
  // Get transform from cloud's frame to map
  geometry_msgs::msg::TransformStamped tf_cloud_to_map;
  try {
    // The live cloud frame is typically "livox_frame" or the LiDAR frame
    std::string cloud_frame = live_cloud->header.frame_id;
    if (cloud_frame.empty()) {
      cloud_frame = "livox_frame";
    }
    tf_cloud_to_map = tf_buffer_->lookupTransform(
      map_frame_, cloud_frame,
      tf2::TimePointZero,
      std::chrono::milliseconds(100));
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
      "Cannot get cloud→map TF: %s", ex.what());
    return;
  }

  // Apply transform manually (faster than tf2_sensor_msgs for PCL clouds)
  Eigen::Affine3d transform = tf2::transformToEigen(tf_cloud_to_map);
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in_map(
    new pcl::PointCloud<pcl::PointXYZI>());
  pcl::transformPointCloud(*live_cloud, *cloud_in_map, transform);

  // ── Crop to detection corridor around robot ────────────────────
  // Use a box: robot_x ± detection_radius, robot_y ± detection_radius,
  //            robot_z + min_ground_height → robot_z + max_overhead_height
  pcl::CropBox<pcl::PointXYZI> crop;
  crop.setInputCloud(cloud_in_map);

  // Extend the box forward in the robot's facing direction
  double extend_x = std::max(detection_radius_, forward_distance_);
  crop.setMin(Eigen::Vector4f(
    static_cast<float>(robot_x - detection_radius_),
    static_cast<float>(robot_y - detection_radius_),
    static_cast<float>(robot_z + min_ground_height_),
    1.0f));
  crop.setMax(Eigen::Vector4f(
    static_cast<float>(robot_x + extend_x),
    static_cast<float>(robot_y + detection_radius_),
    static_cast<float>(robot_z + max_overhead_height_),
    1.0f));

  pcl::PointCloud<pcl::PointXYZI>::Ptr cropped(
    new pcl::PointCloud<pcl::PointXYZI>());
  crop.filter(*cropped);

  if (cropped->empty()) {
    // No points in detection zone — no obstacle
    std_msgs::msg::Bool msg;
    msg.data = false;
    obstacle_3d_pub_->publish(msg);
    return;
  }

  // ── Compare against known map OcTree ───────────────────────────
  pcl::PointCloud<pcl::PointXYZI>::Ptr obstacle_points(
    new pcl::PointCloud<pcl::PointXYZI>());
  pcl::PointCloud<pcl::PointXYZI>::Ptr overhead_points(
    new pcl::PointCloud<pcl::PointXYZI>());

  std::vector<int> indices;
  std::vector<float> distances;

  for (const auto & pt : cropped->points) {
    if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) {
      continue;
    }

    // Check if this point exists in the known map
    // Use radius search: if no known map point within voxel_neighbor_radius_ → NEW point
    indices.clear();
    distances.clear();

    int found = map_octree_->radiusSearch(
      pt, static_cast<float>(voxel_neighbor_radius_),
      indices, distances, 1);  // only need 1 neighbor

    if (found == 0) {
      // This point is NOT in the known map → dynamic obstacle!
      double height_above_robot = pt.z - robot_z;

      obstacle_points->push_back(pt);

      // Check if it's an overhead obstacle
      if (height_above_robot >= min_overhead_height_ &&
          height_above_robot <= max_overhead_height_)
      {
        overhead_points->push_back(pt);
      }
    }
  }

  // ── Publish obstacle alert ─────────────────────────────────────
  bool overhead_detected = static_cast<int>(overhead_points->size()) >= min_obstacle_points_;
  bool any_obstacle      = static_cast<int>(obstacle_points->size()) >= min_obstacle_points_;

  std_msgs::msg::Bool obstacle_msg;
  obstacle_msg.data = overhead_detected;
  obstacle_3d_pub_->publish(obstacle_msg);

  // ── Publish obstacle cloud for RViz ────────────────────────────
  if (any_obstacle) {
    sensor_msgs::msg::PointCloud2 ros_obstacle;
    pcl::toROSMsg(*obstacle_points, ros_obstacle);
    ros_obstacle.header.frame_id = map_frame_;
    ros_obstacle.header.stamp = this->now();
    obstacle_cloud_pub_->publish(ros_obstacle);

    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
      "⚠ 3D obstacles: %zu total, %zu overhead (above %.1fm)",
      obstacle_points->size(), overhead_points->size(), min_overhead_height_);
  }

  // ── Publish bounding box marker for overhead obstacles ─────────
  if (overhead_detected) {
    // Calculate bounding box of overhead obstacles
    float min_x = std::numeric_limits<float>::max();
    float min_y = std::numeric_limits<float>::max();
    float min_z = std::numeric_limits<float>::max();
    float max_x = std::numeric_limits<float>::lowest();
    float max_y = std::numeric_limits<float>::lowest();
    float max_z = std::numeric_limits<float>::lowest();

    for (const auto & pt : overhead_points->points) {
      min_x = std::min(min_x, pt.x);
      min_y = std::min(min_y, pt.y);
      min_z = std::min(min_z, pt.z);
      max_x = std::max(max_x, pt.x);
      max_y = std::max(max_y, pt.y);
      max_z = std::max(max_z, pt.z);
    }

    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = map_frame_;
    marker.header.stamp = this->now();
    marker.ns = "overhead_obstacle";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = (min_x + max_x) / 2.0;
    marker.pose.position.y = (min_y + max_y) / 2.0;
    marker.pose.position.z = (min_z + max_z) / 2.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = std::max(0.1, static_cast<double>(max_x - min_x));
    marker.scale.y = std::max(0.1, static_cast<double>(max_y - min_y));
    marker.scale.z = std::max(0.1, static_cast<double>(max_z - min_z));
    marker.color.r = 1.0f;
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;
    marker.color.a = 0.4f;
    marker.lifetime = rclcpp::Duration::from_seconds(0.5);

    marker_pub_->publish(marker);

    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
      "🚨 OVERHEAD OBSTACLE DETECTED! %zu points at height %.1f–%.1fm",
      overhead_points->size(),
      static_cast<double>(min_z) - robot_z,
      static_cast<double>(max_z) - robot_z);
  }
}

}  // namespace lidar3d_obstacle_monitor


// ── main ──────────────────────────────────────────────────────────
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<lidar3d_obstacle_monitor::ObstacleDetector3D>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
