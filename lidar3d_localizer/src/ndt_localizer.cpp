// ================================================================
//  ndt_localizer.cpp — 3D NDT Scan-Matching Localizer
//
//  Continuously matches live Livox cloud against saved PCD map.
//  Corrects AMCL via /initialpose when drift detected.
// ================================================================

#include "lidar3d_localizer/ndt_localizer.hpp"

#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <yaml-cpp/yaml.h>
#include <fstream>

namespace lidar3d_localizer
{

// ── Helper: yaw from quaternion ──────────────────────────────────
static double quatToYaw(double qx, double qy, double qz, double qw)
{
  return std::atan2(2.0*(qw*qz + qx*qy), 1.0 - 2.0*(qy*qy + qz*qz));
}

NdtLocalizer::NdtLocalizer(const rclcpp::NodeOptions & options)
: Node("ndt_localizer", options)
{
  // ── Declare all parameters ─────────────────────────────────────
  declare_parameter<std::string>("pcd_map_path",     "/home/taikisha/turn_ws/scans.pcd");
  declare_parameter<std::string>("map_yaml",         "");
  declare_parameter<std::string>("live_cloud_topic", "/livox/lidar");
  declare_parameter<std::string>("robot_frame",      "base_link");
  declare_parameter<std::string>("map_frame",        "map");

  declare_parameter<double>("ndt_resolution",      1.0);
  declare_parameter<double>("ndt_step_size",       0.1);
  declare_parameter<double>("ndt_epsilon",         0.01);
  declare_parameter<int>   ("ndt_max_iterations",  30);

  declare_parameter<double>("scan_voxel_size",     0.3);
  declare_parameter<double>("map_voxel_size",      0.2);
  declare_parameter<double>("match_rate",          2.0);

  declare_parameter<double>("map_offset_x",        0.0);
  declare_parameter<double>("map_offset_y",        0.0);
  declare_parameter<double>("map_offset_z",        0.0);
  declare_parameter<double>("map_offset_yaw",      0.0);

  declare_parameter<double>("drift_xy_threshold",      0.25);
  declare_parameter<double>("drift_yaw_threshold",     0.15);
  declare_parameter<double>("good_fitness_threshold",  0.5);
  declare_parameter<double>("bad_fitness_threshold",   2.0);
  declare_parameter<double>("min_correction_interval", 3.0);

  // Live map accumulation parameters
  declare_parameter<bool>  ("map_accumulate_enabled",        true);
  declare_parameter<bool>  ("accumulate_require_stopped",    true);
  declare_parameter<bool>  ("accumulate_require_consistent", true);
  declare_parameter<int>   ("min_accum_observations",        10);
  declare_parameter<int>   ("accumulate_interval",           10);
  declare_parameter<int>   ("map_save_interval",             5);
  declare_parameter<double>("map_accumulate_voxel",          0.15);

  // ── Read parameters ────────────────────────────────────────────
  get_parameter("pcd_map_path",        pcd_map_path_);
  get_parameter("map_yaml",            map_yaml_path_);
  get_parameter("live_cloud_topic",    live_cloud_topic_);
  get_parameter("robot_frame",         robot_frame_);
  get_parameter("map_frame",           map_frame_);
  get_parameter("ndt_resolution",      ndt_resolution_);
  get_parameter("ndt_step_size",       ndt_step_size_);
  get_parameter("ndt_epsilon",         ndt_epsilon_);
  get_parameter("ndt_max_iterations",  ndt_max_iterations_);
  get_parameter("scan_voxel_size",     scan_voxel_size_);
  get_parameter("map_voxel_size",      map_voxel_size_);
  get_parameter("match_rate",          match_rate_);
  // Read map_offset parameters (0.0 by default)
  get_parameter("map_offset_x",        map_offset_x_);
  get_parameter("map_offset_y",        map_offset_y_);
  get_parameter("map_offset_z",        map_offset_z_);
  get_parameter("map_offset_yaw",      map_offset_yaw_);

  get_parameter("drift_xy_threshold",      drift_xy_threshold_);
  get_parameter("drift_yaw_threshold",     drift_yaw_threshold_);
  get_parameter("good_fitness_threshold",  good_fitness_threshold_);
  get_parameter("bad_fitness_threshold",   bad_fitness_threshold_);
  get_parameter("min_correction_interval", min_correction_interval_);

  get_parameter("map_accumulate_enabled",        map_accumulate_enabled_);
  get_parameter("accumulate_require_stopped",    accumulate_require_stopped_);
  get_parameter("accumulate_require_consistent", accumulate_require_consistent_);
  get_parameter("min_accum_observations",        min_accum_observations_);
  get_parameter("accumulate_interval",           accumulate_interval_);
  get_parameter("map_save_interval",             map_save_interval_);
  get_parameter("map_accumulate_voxel",          map_accumulate_voxel_);

  last_correction_time_ = this->now();

  // ── TF2 ────────────────────────────────────────────────────────
  tf_buffer_   = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // ── Publishers ─────────────────────────────────────────────────
  initial_pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "/initialpose", 10);
  ndt_pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "/ndt_pose", 10);
  ndt_score_pub_ = create_publisher<std_msgs::msg::Float64>(
    "/ndt_score", 10);
  aligned_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
    "/ndt_aligned_cloud", 10);
  map_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
    "/ndt_map_cloud", rclcpp::QoS(1).transient_local().reliable());
  status_pub_ = create_publisher<std_msgs::msg::String>(
    "/ndt_status", 10);

  // ── Load PCD map & prepare NDT ─────────────────────────────────
  map_cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  latest_live_cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  accumulated_cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();

  if (loadPCDMap(pcd_map_path_)) {
    prepareNdtTarget();
    initial_map_size_ = map_cloud_->size();
    RCLCPP_INFO(get_logger(), "✅ PCD map loaded & NDT target ready");
  } else {
    RCLCPP_ERROR(get_logger(), "❌ Failed to load PCD map: %s", pcd_map_path_.c_str());
  }

  // ── Subscribers ────────────────────────────────────────────────
  live_cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    live_cloud_topic_, rclcpp::SensorDataQoS(),
    std::bind(&NdtLocalizer::liveCloudCallback, this, std::placeholders::_1));

  amcl_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "/amcl_pose", 10,
    std::bind(&NdtLocalizer::amclCallback, this, std::placeholders::_1));

  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    "/odometry/filtered", 10,
    std::bind(&NdtLocalizer::odomCallback, this, std::placeholders::_1));

  // ── Matching timer ─────────────────────────────────────────────
  double period_ms = 1000.0 / match_rate_;
  match_timer_ = create_wall_timer(
    std::chrono::milliseconds(static_cast<int>(period_ms)),
    std::bind(&NdtLocalizer::matchingLoop, this));

  RCLCPP_INFO(get_logger(),
    "\n╔══════════════════════════════════════════════════════════════╗\n"
    "║     3D NDT LOCALIZER — Continuous Scan Matching             ║\n"
    "║                                                             ║\n"
    "║  PCD Map:       %s\n"
    "║  Live Topic:    %s\n"
    "║  Match Rate:    %.1f Hz                                     ║\n"
    "║  NDT Resolution:%.1fm                                       ║\n"
    "║  Drift Threshold: %.2fm / %.2f rad                          ║\n"
    "║                                                             ║\n"
    "║  → Corrects AMCL via /initialpose when drift detected       ║\n"
    "║  → Monitor: /ndt_pose, /ndt_score, /ndt_status              ║\n"
    "╚══════════════════════════════════════════════════════════════╝",
    pcd_map_path_.c_str(), live_cloud_topic_.c_str(),
    match_rate_, ndt_resolution_,
    drift_xy_threshold_, drift_yaw_threshold_);
}


// ══════════════════════════════════════════════════════════════════
//  Auto-parse 2D map YAML file for origin alignment
// ══════════════════════════════════════════════════════════════════
bool NdtLocalizer::loadYamlOrigin(const std::string & yaml_path)
{
  try {
    YAML::Node config = YAML::LoadFile(yaml_path);
    if (config["origin"]) {
      std::vector<double> origin = config["origin"].as<std::vector<double>>();
      if (origin.size() >= 3) {
        // If manual map_offset was not set, use origin from YAML
        if (std::abs(map_offset_x_) < 1e-4 && std::abs(map_offset_y_) < 1e-4) {
          map_offset_x_   = origin[0];
          map_offset_y_   = origin[1];
          map_offset_yaw_ = origin[2];
        }
        RCLCPP_INFO(get_logger(),
          "📄 Parsed 2D Map YAML '%s': origin=[%.2f, %.2f, %.2f]",
          yaml_path.c_str(), origin[0], origin[1], origin[2]);
        return true;
      }
    }
  } catch (const std::exception & e) {
    RCLCPP_WARN(get_logger(), "Failed to read 2D map YAML '%s': %s", yaml_path.c_str(), e.what());
  }
  return false;
}


// ══════════════════════════════════════════════════════════════════
//  Load PCD map from disk
// ══════════════════════════════════════════════════════════════════
bool NdtLocalizer::loadPCDMap(const std::string & path)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr raw(new pcl::PointCloud<pcl::PointXYZI>());
  if (pcl::io::loadPCDFile<pcl::PointXYZI>(path, *raw) == -1) return false;
  if (raw->empty()) return false;

  // Apply map offset to align 3D PCD map with 2D YAML map if set
  if (std::abs(map_offset_x_) > 1e-4 || std::abs(map_offset_y_) > 1e-4 ||
      std::abs(map_offset_z_) > 1e-4 || std::abs(map_offset_yaw_) > 1e-4)
  {
    Eigen::Matrix4f transform = poseToMatrix(map_offset_x_, map_offset_y_, map_offset_z_, map_offset_yaw_);
    pcl::transformPointCloud(*raw, *raw, transform);
    RCLCPP_INFO(get_logger(), "📐 Applied 3D Map Offset: x=%.2f, y=%.2f, z=%.2f, yaw=%.1f°",
      map_offset_x_, map_offset_y_, map_offset_z_, map_offset_yaw_ * 180.0 / M_PI);
  }

  // Downsample map
  pcl::VoxelGrid<pcl::PointXYZI> vg;
  vg.setInputCloud(raw);
  vg.setLeafSize(
    static_cast<float>(map_voxel_size_),
    static_cast<float>(map_voxel_size_),
    static_cast<float>(map_voxel_size_));
  vg.filter(*map_cloud_);

  RCLCPP_INFO(get_logger(), "PCD: %zu raw → %zu after voxel (%.2fm)",
    raw->size(), map_cloud_->size(), map_voxel_size_);

  map_loaded_ = true;
  return true;
}


// ══════════════════════════════════════════════════════════════════
//  Prepare NDT with map as target
// ══════════════════════════════════════════════════════════════════
void NdtLocalizer::prepareNdtTarget()
{
  ndt_.setResolution(static_cast<float>(ndt_resolution_));
  ndt_.setStepSize(ndt_step_size_);
  ndt_.setTransformationEpsilon(ndt_epsilon_);
  ndt_.setMaximumIterations(ndt_max_iterations_);
  ndt_.setInputTarget(map_cloud_);

  RCLCPP_INFO(get_logger(), "NDT target set: res=%.2f, step=%.2f, eps=%.3f, iter=%d",
    ndt_resolution_, ndt_step_size_, ndt_epsilon_, ndt_max_iterations_);
}


// ══════════════════════════════════════════════════════════════════
//  Callbacks
// ══════════════════════════════════════════════════════════════════
void NdtLocalizer::liveCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  // Periodically publish map cloud for RViz (every 5 seconds)
  static rclcpp::Time last_map_pub(0, 0, RCL_ROS_TIME);
  if (map_loaded_ && (this->now() - last_map_pub).seconds() > 5.0) {
    sensor_msgs::msg::PointCloud2 ros_map;
    pcl::toROSMsg(*map_cloud_, ros_map);
    ros_map.header.frame_id = map_frame_;
    ros_map.header.stamp = this->now();
    map_cloud_pub_->publish(ros_map);
    last_map_pub = this->now();
  }

  std::lock_guard<std::mutex> lock(cloud_mutex_);
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::fromROSMsg(*msg, *cloud);
  latest_live_cloud_ = cloud;
  new_cloud_ready_ = true;
}

void NdtLocalizer::amclCallback(
  const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(amcl_mutex_);
  amcl_x_   = msg->pose.pose.position.x;
  amcl_y_   = msg->pose.pose.position.y;
  amcl_yaw_ = quatToYaw(
    msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
    msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
  amcl_received_ = true;
}

void NdtLocalizer::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(odom_mutex_);
  odom_x_   = msg->pose.pose.position.x;
  odom_y_   = msg->pose.pose.position.y;
  odom_yaw_ = quatToYaw(
    msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
    msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);

  double linear_spd  = std::hypot(msg->twist.twist.linear.x, msg->twist.twist.linear.y);
  double angular_spd = std::abs(msg->twist.twist.angular.z);
  robot_moving_ = (linear_spd > 0.03 || angular_spd > 0.05);

  odom_received_ = true;
}


// ══════════════════════════════════════════════════════════════════
//  Pose ↔ Matrix helpers
// ══════════════════════════════════════════════════════════════════
Eigen::Matrix4f NdtLocalizer::poseToMatrix(double x, double y, double z, double yaw)
{
  Eigen::Matrix4f mat = Eigen::Matrix4f::Identity();
  mat(0,0) =  std::cos(yaw); mat(0,1) = -std::sin(yaw);
  mat(1,0) =  std::sin(yaw); mat(1,1) =  std::cos(yaw);
  mat(0,3) = static_cast<float>(x);
  mat(1,3) = static_cast<float>(y);
  mat(2,3) = static_cast<float>(z);
  return mat;
}

void NdtLocalizer::matrixToPose(const Eigen::Matrix4f & mat,
                                double & x, double & y, double & z, double & yaw)
{
  x   = static_cast<double>(mat(0,3));
  y   = static_cast<double>(mat(1,3));
  z   = static_cast<double>(mat(2,3));
  yaw = std::atan2(static_cast<double>(mat(1,0)), static_cast<double>(mat(0,0)));
}


// ══════════════════════════════════════════════════════════════════
//  Main matching loop (runs at match_rate_ Hz)
// ══════════════════════════════════════════════════════════════════
void NdtLocalizer::matchingLoop()
{
  if (!map_loaded_ || !new_cloud_ready_.load()) return;

  // ── Grab latest cloud ──────────────────────────────────────────
  pcl::PointCloud<pcl::PointXYZI>::Ptr live_cloud;
  {
    std::lock_guard<std::mutex> lock(cloud_mutex_);
    live_cloud = latest_live_cloud_;
    new_cloud_ready_ = false;
  }
  if (!live_cloud || live_cloud->empty()) return;

  // ── Get robot pose in map frame via TF ─────────────────────────
  // ── Downsample live cloud in sensor frame ─────────────────────
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ds(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::VoxelGrid<pcl::PointXYZI> vg;
  vg.setInputCloud(live_cloud);
  vg.setLeafSize(
    static_cast<float>(scan_voxel_size_),
    static_cast<float>(scan_voxel_size_),
    static_cast<float>(scan_voxel_size_));
  vg.filter(*cloud_ds);

  if (cloud_ds->size() < 100) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
      "Too few points after downsample: %zu", cloud_ds->size());
    return;
  }

  // ── Get CURRENT robot/sensor pose in map frame via TF ─────────
  geometry_msgs::msg::TransformStamped tf_cloud_to_map;
  try {
    std::string cloud_frame = live_cloud->header.frame_id;
    if (cloud_frame.empty()) cloud_frame = "livox_frame";
    tf_cloud_to_map = tf_buffer_->lookupTransform(
      map_frame_, cloud_frame, tf2::TimePointZero,
      std::chrono::milliseconds(100));
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
      "TF not ready: %s", ex.what());
    return;
  }

  // ── Build initial guess from real-time TF pose ─────────────────
  Eigen::Affine3d tf_eigen = tf2::transformToEigen(tf_cloud_to_map);
  Eigen::Matrix4f init_guess = (tf_eigen.matrix()).cast<float>();

  // ── Run NDT alignment ─────────────────────────────────────────
  ndt_.setInputSource(cloud_ds);
  pcl::PointCloud<pcl::PointXYZI>::Ptr aligned(new pcl::PointCloud<pcl::PointXYZI>());
  ndt_.align(*aligned, init_guess);

  bool converged = ndt_.hasConverged();
  double fitness  = ndt_.getFitnessScore();
  match_count_++;

  // ── Publish fitness score ──────────────────────────────────────
  std_msgs::msg::Float64 score_msg;
  score_msg.data = fitness;
  ndt_score_pub_->publish(score_msg);

  if (!converged || fitness > bad_fitness_threshold_) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
      "NDT match #%d: %s (score=%.3f) — REJECTED",
      match_count_, converged ? "converged" : "FAILED", fitness);
    publishStatus("NDT_REJECTED");
    return;
  }

  // ── Extract matched pose ───────────────────────────────────────
  Eigen::Matrix4f result = ndt_.getFinalTransformation();
  double nx, ny, nz, nyaw;
  matrixToPose(result, nx, ny, nz, nyaw);

  ndt_x_   = nx;
  ndt_y_   = ny;
  ndt_z_   = nz;
  ndt_yaw_ = nyaw;
  ndt_initialized_ = true;
  last_fitness_score_ = fitness;

  // ── Publish NDT pose ───────────────────────────────────────────
  {
    geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;
    pose_msg.header.frame_id = map_frame_;
    pose_msg.header.stamp = this->now();
    pose_msg.pose.pose.position.x = nx;
    pose_msg.pose.pose.position.y = ny;
    pose_msg.pose.pose.position.z = nz;
    pose_msg.pose.pose.orientation.z = std::sin(nyaw / 2.0);
    pose_msg.pose.pose.orientation.w = std::cos(nyaw / 2.0);
    ndt_pose_pub_->publish(pose_msg);
  }

  // ── Publish aligned cloud for RViz ─────────────────────────────
  {
    sensor_msgs::msg::PointCloud2 aligned_msg;
    pcl::toROSMsg(*aligned, aligned_msg);
    aligned_msg.header.frame_id = map_frame_;
    aligned_msg.header.stamp = this->now();
    aligned_cloud_pub_->publish(aligned_msg);
  }

  // ── Live map accumulation: add aligned cloud to buffer ─────────
  if (map_accumulate_enabled_ && converged && fitness < good_fitness_threshold_) {
    accumulateAlignedCloud(aligned);
  }

  // ── Check drift from AMCL ─────────────────────────────────────
  if (!amcl_received_) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
      "NDT #%d OK (score=%.3f, pos=%.2f,%.2f) — waiting for AMCL",
      match_count_, fitness, nx, ny);
    publishStatus("NDT_OK_NO_AMCL");
    return;
  }

  double dx, dy, dyaw;
  {
    std::lock_guard<std::mutex> lock(amcl_mutex_);
    dx   = nx - amcl_x_;
    dy   = ny - amcl_y_;
    dyaw = std::atan2(std::sin(nyaw - amcl_yaw_), std::cos(nyaw - amcl_yaw_));
  }
  double dist = std::sqrt(dx*dx + dy*dy);

  bool good_match = (fitness < good_fitness_threshold_);

  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
    "NDT #%d: score=%.3f | NDT(%.2f,%.2f,%.1f°) vs AMCL | drift=%.3fm, %.1f°%s",
    match_count_, fitness, nx, ny, nyaw*180.0/M_PI,
    dist, std::abs(dyaw)*180.0/M_PI,
    good_match ? " ✅" : " ⚠");

  // ── CORRECTION LOGIC: Only correct when STOPPED + consistent ───
  bool is_moving = false;
  {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    is_moving = robot_moving_;
  }

  // Store reading in consistency buffer (ring buffer of last N)
  if (good_match) {
    NdtReading r;
    r.x = nx;  r.y = ny;  r.yaw = nyaw;  r.fitness = fitness;
    r.stamp = this->now();
    consistency_buffer_.push_back(r);
    if (consistency_buffer_.size() > consistency_window_) {
      consistency_buffer_.pop_front();
    }
  } else {
    // Bad match — clear the consistency buffer
    consistency_buffer_.clear();
  }

  // NEVER correct while the robot is actively driving/rotating.
  // This prevents TF jumps that cause the navigator's distance to "teleport".
  if (is_moving) {
    if (good_match) {
      publishStatus("MATCHED_MOVING");
    } else {
      publishStatus("WEAK_MATCH");
    }
    return;
  }

  // Robot is STOPPED — check if we should correct AMCL
  if (!good_match) {
    publishStatus("WEAK_MATCH");
    return;
  }

  // Need enough consistent readings before correcting
  if (consistency_buffer_.size() < consistency_window_) {
    publishStatus("MATCHED_BUFFERING");
    return;
  }

  // Check that all readings in the buffer agree (cluster check)
  double avg_x = 0.0, avg_y = 0.0, avg_yaw = 0.0;
  for (const auto & r : consistency_buffer_) {
    avg_x   += r.x;
    avg_y   += r.y;
    avg_yaw += r.yaw;
  }
  avg_x   /= consistency_buffer_.size();
  avg_y   /= consistency_buffer_.size();
  avg_yaw /= consistency_buffer_.size();

  bool all_consistent = true;
  for (const auto & r : consistency_buffer_) {
    double rdist = std::hypot(r.x - avg_x, r.y - avg_y);
    double ryaw  = std::abs(std::atan2(std::sin(r.yaw - avg_yaw), std::cos(r.yaw - avg_yaw)));
    if (rdist > 0.15 || ryaw > 0.087) {  // 15cm, ~5° spread max
      all_consistent = false;
      break;
    }
  }

  if (!all_consistent) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
      "NDT readings inconsistent — not correcting");
    publishStatus("MATCHED_INCONSISTENT");
    return;
  }

  // Use the cluster average for correction (more stable than single reading)
  double corr_dist = std::hypot(avg_x - amcl_x_, avg_y - amcl_y_);
  double corr_dyaw;
  {
    std::lock_guard<std::mutex> lock(amcl_mutex_);
    corr_dyaw = std::abs(std::atan2(std::sin(avg_yaw - amcl_yaw_), std::cos(avg_yaw - amcl_yaw_)));
  }

  if (corr_dist > drift_xy_threshold_ || corr_dyaw > drift_yaw_threshold_) {
    double elapsed = (this->now() - last_correction_time_).seconds();
    if (elapsed >= min_correction_interval_) {
      publishInitialPose(avg_x, avg_y, nz, avg_yaw);
      correction_count_++;
      last_correction_time_ = this->now();
      consistency_buffer_.clear();  // Reset after correction

      RCLCPP_WARN(get_logger(),
        "🔧 CORRECTION #%d → /initialpose (drift=%.3fm, %.1f°, avg of %zu readings, STOPPED)",
        correction_count_, corr_dist, corr_dyaw*180.0/M_PI, consistency_window_);
      publishStatus("CORRECTED");
    }
  } else {
    publishStatus("MATCHED");
  }
}


// ══════════════════════════════════════════════════════════════════
//  Live Map Accumulation
// ══════════════════════════════════════════════════════════════════
// ══════════════════════════════════════════════════════════════════
//  Live Map Accumulation (Multi-Observation Consensus Filter)
// ══════════════════════════════════════════════════════════════════
void NdtLocalizer::accumulateAlignedCloud(
  const pcl::PointCloud<pcl::PointXYZI>::Ptr & aligned)
{
  if (!aligned || aligned->empty()) return;

  // 1. Strict Filter: Never accumulate when robot is moving/rotating if enabled
  if (accumulate_require_stopped_) {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    if (robot_moving_) {
      // Robot is driving or rotating — skip accumulation to prevent motion blur/ghosting
      return;
    }
  }

  // 2. Strict Filter: Require NDT pose to be confirmed consistent in consistency buffer
  if (accumulate_require_consistent_) {
    if (consistency_buffer_.size() < consistency_window_) {
      return;
    }
  }

  // 3. Add point cloud into Candidate Voxel Grid
  {
    std::lock_guard<std::mutex> lock(accum_mutex_);
    float inv_voxel = 1.0f / static_cast<float>(map_accumulate_voxel_);

    for (const auto & pt : aligned->points) {
      if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) continue;

      VoxelKey key;
      key.ix = static_cast<int>(std::floor(pt.x * inv_voxel));
      key.iy = static_cast<int>(std::floor(pt.y * inv_voxel));
      key.iz = static_cast<int>(std::floor(pt.z * inv_voxel));

      auto & obs = candidate_voxels_[key];
      obs.x_sum += pt.x;
      obs.y_sum += pt.y;
      obs.z_sum += pt.z;
      obs.intensity_sum += pt.intensity;
      obs.count++;
    }
  }

  accum_count_++;
  total_accum_count_++;

  // 4. Time to promote persistent voxels into the main map?
  if (accum_count_ >= accumulate_interval_) {
    mergeAccumulatedIntoMap();
    accum_count_ = 0;
    merges_since_save_++;

    // Time to save to disk?
    if (merges_since_save_ >= map_save_interval_) {
      saveMapToDisk();
      merges_since_save_ = 0;
    }
  }
}


void NdtLocalizer::mergeAccumulatedIntoMap()
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr promoted_cloud(new pcl::PointCloud<pcl::PointXYZI>());

  {
    std::lock_guard<std::mutex> lock(accum_mutex_);
    if (candidate_voxels_.empty()) return;

    for (auto it = candidate_voxels_.begin(); it != candidate_voxels_.end(); ) {
      if (it->second.count >= min_accum_observations_) {
        // Voxel has been observed persistently across N distinct scan cycles!
        pcl::PointXYZI pt;
        pt.x = it->second.x_sum / it->second.count;
        pt.y = it->second.y_sum / it->second.count;
        pt.z = it->second.z_sum / it->second.count;
        pt.intensity = it->second.intensity_sum / it->second.count;

        promoted_cloud->points.push_back(pt);

        // Remove promoted voxel so we don't re-add it
        it = candidate_voxels_.erase(it);
      } else {
        ++it;
      }
    }
  }

  if (promoted_cloud->empty()) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 10000,
      "⏳ Map Accumulator: Candidate voxels filtering... (waiting for %d consistent observations)",
      min_accum_observations_);
    return;
  }

  promoted_cloud->width = promoted_cloud->points.size();
  promoted_cloud->height = 1;
  promoted_cloud->is_dense = true;

  size_t before_size = map_cloud_->size();

  // Merge promoted persistent points into map_cloud_
  *map_cloud_ += *promoted_cloud;

  // Voxel filter to keep global map density bounded
  pcl::PointCloud<pcl::PointXYZI>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::VoxelGrid<pcl::PointXYZI> vg;
  vg.setInputCloud(map_cloud_);
  vg.setLeafSize(
    static_cast<float>(map_accumulate_voxel_),
    static_cast<float>(map_accumulate_voxel_),
    static_cast<float>(map_accumulate_voxel_));
  vg.filter(*filtered);

  // Replace map and re-set NDT target
  map_cloud_ = filtered;
  ndt_.setInputTarget(map_cloud_);

  RCLCPP_INFO(get_logger(),
    "🗺️ MAP UPDATED & VERIFIED: %zu → %zu pts (promoted %zu persistent voxels, min_obs=%d) | total scans: %d",
    before_size, map_cloud_->size(), promoted_cloud->size(),
    min_accum_observations_, total_accum_count_);
}


void NdtLocalizer::saveMapToDisk()
{
  if (!map_cloud_ || map_cloud_->empty()) return;

  try {
    // Atomic save: write to temp, then rename
    std::string tmp_path = pcd_map_path_ + ".tmp";
    pcl::io::savePCDFileBinary(tmp_path, *map_cloud_);

    // Rename atomically
    if (std::rename(tmp_path.c_str(), pcd_map_path_.c_str()) == 0) {
      RCLCPP_INFO(get_logger(),
        "💾 MAP SAVED: %s (%zu points)",
        pcd_map_path_.c_str(), map_cloud_->size());
    } else {
      RCLCPP_WARN(get_logger(), "⚠ Failed to rename temp file to %s", pcd_map_path_.c_str());
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "❌ Map save failed: %s", e.what());
  }
}


// ══════════════════════════════════════════════════════════════════
//  Publish /initialpose to correct AMCL
// ══════════════════════════════════════════════════════════════════
void NdtLocalizer::publishInitialPose(double x, double y, double /*z*/, double yaw)
{
  geometry_msgs::msg::PoseWithCovarianceStamped msg;
  msg.header.frame_id = map_frame_;
  msg.header.stamp = this->now();
  msg.pose.pose.position.x = x;
  msg.pose.pose.position.y = y;
  msg.pose.pose.position.z = 0.0;  // AMCL is 2D
  msg.pose.pose.orientation.z = std::sin(yaw / 2.0);
  msg.pose.pose.orientation.w = std::cos(yaw / 2.0);

  // Small covariance — we're confident in NDT result
  msg.pose.covariance[0]  = 0.05;   // x
  msg.pose.covariance[7]  = 0.05;   // y
  msg.pose.covariance[35] = 0.02;   // yaw

  initial_pose_pub_->publish(msg);
}


void NdtLocalizer::publishStatus(const std::string & status)
{
  std_msgs::msg::String msg;
  msg.data = status;
  status_pub_->publish(msg);
}

}  // namespace lidar3d_localizer


// ── main ─────────────────────────────────────────────────────────
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<lidar3d_localizer::NdtLocalizer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
