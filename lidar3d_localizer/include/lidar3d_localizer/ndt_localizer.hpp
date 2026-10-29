#pragma once

// ================================================================
//  ndt_localizer.hpp
//
//  3D NDT Scan-Matching Localizer
//
//  Continuously matches live 3D point cloud from Livox Mid-360
//  against a pre-built PCD map using NDT (Normal Distributions
//  Transform) algorithm.
//
//  When the NDT pose differs from AMCL's pose beyond a threshold,
//  it publishes a correction on /initialpose to fix AMCL.
//
//  This keeps the robot properly localized during navigation
//  by using FULL 3D map data (floors, walls, ceiling, top surfaces).
//
//  DOES NOT modify any existing packages!
// ================================================================

#include <string>
#include <memory>
#include <mutex>
#include <atomic>
#include <deque>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/string.hpp"

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/ndt.h>

#include <Eigen/Dense>

namespace lidar3d_localizer
{

class NdtLocalizer : public rclcpp::Node
{
public:
  explicit NdtLocalizer(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // ── Callbacks ──────────────────────────────────────────────────
  void liveCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void amclCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void matchingLoop();

  // ── Helpers ────────────────────────────────────────────────────
  bool loadPCDMap(const std::string & path);
  void prepareNdtTarget();
  void publishInitialPose(double x, double y, double z, double yaw);
  void publishStatus(const std::string & msg);

  // ── Live map accumulation ──────────────────────────────────────
  void accumulateAlignedCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr & aligned);
  void mergeAccumulatedIntoMap();
  void saveMapToDisk();

  Eigen::Matrix4f poseToMatrix(double x, double y, double z, double yaw);
  void matrixToPose(const Eigen::Matrix4f & mat,
                    double & x, double & y, double & z, double & yaw);

  // ── ROS interfaces ────────────────────────────────────────────
  // Subscribers
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr            live_cloud_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr                  odom_sub_;

  // Publishers
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_pub_;  // → /initialpose
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr ndt_pose_pub_;      // → /ndt_pose
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr                       ndt_score_pub_;     // → /ndt_score
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr                aligned_cloud_pub_; // → /ndt_aligned_cloud
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr                map_cloud_pub_;     // → /ndt_map_cloud
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr                        status_pub_;        // → /ndt_status

  // Timer
  rclcpp::TimerBase::SharedPtr  match_timer_;

  // TF2
  std::shared_ptr<tf2_ros::Buffer>            tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // ── PCL data ──────────────────────────────────────────────────
  pcl::PointCloud<pcl::PointXYZI>::Ptr  map_cloud_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr  latest_live_cloud_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr  accumulated_cloud_;  // buffer for new scans

  pcl::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI> ndt_;

  std::mutex  cloud_mutex_;
  std::mutex  accum_mutex_;
  bool        map_loaded_         {false};
  bool        map_cloud_published_{false};
  std::atomic<bool> new_cloud_ready_{false};

  // ── Map accumulation tracking ──────────────────────────────────
  int    accum_count_       {0};   // scans accumulated since last merge
  int    total_accum_count_ {0};   // total scans accumulated ever
  int    merges_since_save_ {0};   // merges since last disk save
  size_t initial_map_size_  {0};   // original map point count

  // ── AMCL pose tracking ────────────────────────────────────────
  double amcl_x_     {0.0};
  double amcl_y_     {0.0};
  double amcl_yaw_   {0.0};
  bool   amcl_received_ {false};
  std::mutex amcl_mutex_;

  // ── Odometry for pose prediction ──────────────────────────────
  double odom_x_     {0.0};
  double odom_y_     {0.0};
  double odom_yaw_   {0.0};
  bool   odom_received_ {false};
  bool   robot_moving_   {false};
  std::mutex odom_mutex_;

  // ── NDT tracking ──────────────────────────────────────────────
  double ndt_x_      {0.0};
  double ndt_y_      {0.0};
  double ndt_z_      {0.0};
  double ndt_yaw_    {0.0};
  bool   ndt_initialized_ {false};
  double last_fitness_score_ {1.0};

  int    match_count_  {0};
  int    correction_count_ {0};

  // ── Consistency buffer for stable corrections ──────────────────
  struct NdtReading {
    double x, y, yaw, fitness;
    rclcpp::Time stamp;
  };
  std::deque<NdtReading> consistency_buffer_;
  static constexpr size_t consistency_window_ = 4;  // need 4 consistent readings

  // ── Parameters ────────────────────────────────────────────────
  std::string pcd_map_path_;
  std::string map_yaml_path_;
  std::string live_cloud_topic_;
  std::string robot_frame_;
  std::string map_frame_;

  // Automatic YAML Origin Parsing
  bool loadYamlOrigin(const std::string & yaml_path);

  // NDT parameters
  double ndt_resolution_;         // NDT grid cell size
  double ndt_step_size_;          // NDT optimization step
  double ndt_epsilon_;            // NDT convergence epsilon
  int    ndt_max_iterations_;     // max iterations per alignment

  // Scan processing
  double scan_voxel_size_;        // downsample live scan before matching
  double map_voxel_size_;         // downsample map before NDT target
  double match_rate_;             // Hz — how often to run NDT

  // Map Alignment Offset (align 3D PCD map with 2D YAML map)
  double map_offset_x_   {0.0};
  double map_offset_y_   {0.0};
  double map_offset_z_   {0.0};
  double map_offset_yaw_ {0.0};

  // Correction thresholds
  double drift_xy_threshold_;     // meters — correct AMCL if drift exceeds this
  double drift_yaw_threshold_;    // radians — correct AMCL if yaw drift exceeds this
  double good_fitness_threshold_; // NDT score below this = good match
  double bad_fitness_threshold_;  // NDT score above this = reject match
  double min_correction_interval_;// seconds — don't spam /initialpose
  rclcpp::Time last_correction_time_;

  // ── Candidate Voxel Observation Grid for Persistent Map Filtering ──
  struct VoxelKey {
    int ix, iy, iz;
    bool operator==(const VoxelKey & o) const {
      return ix == o.ix && iy == o.iy && iz == o.iz;
    }
  };

  struct VoxelKeyHash {
    std::size_t operator()(const VoxelKey & k) const {
      return ((std::hash<int>()(k.ix) ^ (std::hash<int>()(k.iy) << 1)) >> 1) ^ (std::hash<int>()(k.iz) << 2);
    }
  };

  struct VoxelObs {
    float x_sum{0.0f}, y_sum{0.0f}, z_sum{0.0f}, intensity_sum{0.0f};
    int count{0};
  };

  std::unordered_map<VoxelKey, VoxelObs, VoxelKeyHash> candidate_voxels_;

  // Live map accumulation parameters
  bool   map_accumulate_enabled_       {true};
  bool   accumulate_require_stopped_   {true};
  bool   accumulate_require_consistent_{true};
  int    min_accum_observations_       {10};   // require 10 distinct observations before promoting voxel
  int    accumulate_interval_          {10};   // process buffer every N scans
  int    map_save_interval_            {5};    // save to disk every N merges
  double map_accumulate_voxel_         {0.15}; // voxel size for candidate map
};

}  // namespace lidar3d_localizer
