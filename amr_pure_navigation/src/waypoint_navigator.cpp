// ================================================================
//  waypoint_navigator.cpp  v8  —  Smooth Curve-Through Waypoints
//
//  type: waypoint → LOOK-AHEAD: when within lookahead_radius, skip
//                   to next WP without stopping → smooth curve
//  type: station  → stop at position, rotate to theta, wait for PLC
//
//  mission: forward → rotate to face goal → drive forward
//  mission: reverse → rotate BACK to goal → drive REVERSE
//
//  WAYPOINTS YAML EXAMPLE:
//    waypoints:
//      - x: -10.0
//        y:  10.0
//        theta: 0.0
//        type: waypoint
//        mission: forward    ← robot passes through without stopping
//
//      - x:  20.0
//        y:  10.0
//        theta: 0.0
//        type: station
//        mission: forward    ← robot STOPS here, rotates to theta
//
//  CYCLE CONTROL via /set_cycles topic:
//    ros2 topic pub --once /set_cycles std_msgs/msg/Int32 "data: 1"
//    ros2 topic pub --once /set_cycles std_msgs/msg/Int32 "data: -1"  # forever
//    ros2 topic pub --once /set_cycles std_msgs/msg/Int32 "data: 0"   # stop
//
//  CHECK STATUS:
//    ros2 topic echo /nav_status
// ================================================================

#include "amr_pure_navigation/waypoint_navigator.hpp"

namespace amr_pure_navigation
{

// ─── Constructor ──────────────────────────────────────────────────
WaypointNavigator::WaypointNavigator(const rclcpp::NodeOptions & options)
: Node("waypoint_navigator", options)
{
  declare_parameter<std::string>("waypoints_file",     "");
  declare_parameter<std::string>("missions_folder",    "");
  declare_parameter<std::string>("odom_topic",         "/odometry/filtered");
  declare_parameter<std::string>("cmd_vel_topic",      "/diff_drive_controller/cmd_vel");
  declare_parameter<std::string>("amcl_topic",         "/amcl_pose");
  declare_parameter<double>     ("linear_speed",       0.20);
  declare_parameter<double>     ("angular_speed",      0.40);
  declare_parameter<double>     ("drive_correction",   0.15);
  declare_parameter<double>     ("xy_tolerance",       0.10);
  declare_parameter<double>     ("lookahead_radius",   0.50);
  declare_parameter<double>     ("yaw_tolerance",      0.08);
  declare_parameter<double>     ("heading_tolerance",  0.10);
  declare_parameter<double>     ("obstacle_stop_dist", 0.40);
  declare_parameter<bool>       ("stop_on_obstacle",   true);
  declare_parameter<bool>       ("use_amcl",           true);
  declare_parameter<bool>       ("plc_enabled",        false);

  std::vector<std::string> default_tags = {
    "home",
    "tag1", "tag2", "tag3", "tag4", "tag5",
    "demo1", "demo2", "demo3", "demo4", "demo5", "demo6", "demo7", "demo8", "demo9", "demo10",
    "tag6", "tag7", "tag8", "tag9", "tag10", "tag11", "tag12", "tag13", "tag14", "tag15", "tag16", "tag17", "tag18", "tag19", "tag20"
  };
  declare_parameter<std::vector<std::string>>("ordered_tags", default_tags);

  get_parameter("waypoints_file",    waypoints_file_);
  get_parameter("missions_folder",   missions_folder_);
  get_parameter("odom_topic",        odom_topic_);
  get_parameter("cmd_vel_topic",     cmd_vel_topic_);
  get_parameter("amcl_topic",        amcl_topic_);
  get_parameter("linear_speed",      linear_speed_);
  get_parameter("angular_speed",     angular_speed_);
  get_parameter("drive_correction",  drive_correction_);
  get_parameter("xy_tolerance",      xy_tolerance_);
  get_parameter("lookahead_radius",  lookahead_radius_);
  get_parameter("yaw_tolerance",     yaw_tolerance_);
  get_parameter("heading_tolerance", heading_tolerance_);
  get_parameter("obstacle_stop_dist",obstacle_stop_dist_);
  get_parameter("stop_on_obstacle",  stop_on_obstacle_);
  get_parameter("use_amcl",          use_amcl_);
  get_parameter("plc_enabled",       plc_enabled_);
  get_parameter("ordered_tags",      ordered_tags_);

  // ── Load waypoints: try missions folder first, then single waypoints file ──
  bool waypoints_ready = false;

  if (!missions_folder_.empty()) {
    // Multi-mission mode: scan folder for .yaml files
    if (loadMissionsFolder(missions_folder_)) {
      RCLCPP_INFO(get_logger(), "✅ Missions folder loaded: %s", missions_folder_.c_str());
      RCLCPP_INFO(get_logger(), "   Available missions: %zu", missions_map_.size());
      for (const auto & [name, wps] : missions_map_) {
        RCLCPP_INFO(get_logger(), "     📋 %-20s (%zu waypoints)", name.c_str(), wps.size());
      }
      RCLCPP_INFO(get_logger(), "   Select mission: ros2 topic pub --once /load_mission std_msgs/msg/String \"data: 'tag1_to_tag5'\"");
      waypoints_ready = true;  // missions loaded, but no waypoints active yet — that's OK
    } else {
      RCLCPP_ERROR(get_logger(), "Failed to load missions from folder: %s", missions_folder_.c_str());
    }
  }

  if (!waypoints_ready && !waypoints_file_.empty()) {
    // Single waypoints file mode (backward compatible)
    if (loadWaypoints(waypoints_file_)) {
      waypoints_ready = true;
    }
  }

  if (!waypoints_ready && missions_folder_.empty()) {
    RCLCPP_FATAL(get_logger(), "Cannot load waypoints: no missions_folder or waypoints_file provided!");
    init_failed_ = true;
    return;
  }

  RCLCPP_INFO(get_logger(), "=============================================");
  RCLCPP_INFO(get_logger(), "  AMR Navigation v8 - Smooth Curve-Through");
  if (missions_loaded_) {
    RCLCPP_INFO(get_logger(), "  Mode           : MULTI-MISSION");
    RCLCPP_INFO(get_logger(), "  Missions       : %zu available", missions_map_.size());
    RCLCPP_INFO(get_logger(), "  Active mission : %s",
      current_mission_name_.empty() ? "NONE (select via /load_mission)" : current_mission_name_.c_str());
  } else {
    RCLCPP_INFO(get_logger(), "  Mode           : SINGLE WAYPOINTS FILE");
    RCLCPP_INFO(get_logger(), "  Waypoints      : %zu",    waypoints_.size());
  }
  RCLCPP_INFO(get_logger(), "  Odom topic     : %s",     odom_topic_.c_str());
  RCLCPP_INFO(get_logger(), "  Position src   : %s",     use_amcl_ ? "AMCL map frame" : "Odom only");
  RCLCPP_INFO(get_logger(), "  Linear speed   : %.2f m/s",   linear_speed_);
  RCLCPP_INFO(get_logger(), "  Angular speed  : %.2f rad/s", angular_speed_);
  RCLCPP_INFO(get_logger(), "  Lookahead rad  : %.2f m",     lookahead_radius_);
  RCLCPP_INFO(get_logger(), "  XY tolerance   : %.2f m",     xy_tolerance_);
  RCLCPP_INFO(get_logger(), "  Lidar ramp dn  : %.1f sec",   ramp_down_duration_);
  RCLCPP_INFO(get_logger(), "  Lidar ramp up  : %.1f sec",   ramp_up_duration_);
  RCLCPP_INFO(get_logger(), "=============================================");
  if (missions_loaded_) {
    RCLCPP_INFO(get_logger(), "  Step 1: Select mission:");
    RCLCPP_INFO(get_logger(), "    ros2 topic pub --once /load_mission std_msgs/msg/String \"data: 'tag1_to_tag5'\"");
    RCLCPP_INFO(get_logger(), "  Step 2: Start cycles:");
    RCLCPP_INFO(get_logger(), "    ros2 topic pub --once /set_cycles std_msgs/msg/Int32 \"data: 1\"");
  } else {
    RCLCPP_INFO(get_logger(), "  Waiting for /set_cycles command...");
    RCLCPP_INFO(get_logger(), "  ros2 topic pub --once /set_cycles std_msgs/msg/Int32 \"data: 1\"");
  }
  RCLCPP_INFO(get_logger(), "=============================================");

  cmd_vel_pub_      = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);
  status_pub_       = create_publisher<std_msgs::msg::String>("/nav_status",       10);
  station_pub_      = create_publisher<std_msgs::msg::String>("/station_reached", 10);
  mission_list_pub_ = create_publisher<std_msgs::msg::String>("/mission_list",    10);
  queue_status_pub_ = create_publisher<std_msgs::msg::String>("/queue_status",    10);

  // Subscribe to /plc_done (plc_bridge tells us when action is complete)
  plc_done_sub_ = create_subscription<std_msgs::msg::String>(
    "/plc_done", 10,
    std::bind(&WaypointNavigator::plcDoneCallback, this, std::placeholders::_1));

  // Subscribe to /plc_trigger (continuous PLC bit value from plc_station_monitor)
  plc_trigger_sub_ = create_subscription<std_msgs::msg::Bool>(
    "/plc_trigger", 10,
    std::bind(&WaypointNavigator::plcTriggerCallback, this, std::placeholders::_1));

  // Subscribe to /lidar_obstacle (continuous PLC bit DB1.DBX0.0 from plc_lidar_monitor)
  lidar_obstacle_sub_ = create_subscription<std_msgs::msg::Bool>(
    "/lidar_obstacle", 10,
    std::bind(&WaypointNavigator::lidarObstacleCallback, this, std::placeholders::_1));

  // Subscribe to /emergency_stop (continuous PLC bit DB1.DBX0.5 from plc_emergency_monitor)
  emergency_stop_sub_ = create_subscription<std_msgs::msg::Bool>(
    "/emergency_stop", 10,
    std::bind(&WaypointNavigator::emergencyStopCallback, this, std::placeholders::_1));

  cycles_sub_ = create_subscription<std_msgs::msg::Int32>(
    "/set_cycles", 10,
    std::bind(&WaypointNavigator::cyclesCallback, this, std::placeholders::_1));

  // Subscribe to /load_mission (String) -- dynamically switch mission at runtime
  load_mission_sub_ = create_subscription<std_msgs::msg::String>(
    "/load_mission", 10,
    std::bind(&WaypointNavigator::loadMissionCallback, this, std::placeholders::_1));

  // Subscribe to /queue_missions (String) -- dynamic mission queue
  //   Send comma-separated mission names: "tag3,tag7,tag9,tag1,home"
  //   Robot chains through them automatically, remembering position
  queue_missions_sub_ = create_subscription<std_msgs::msg::String>(
    "/queue_missions", 10,
    std::bind(&WaypointNavigator::queueMissionsCallback, this, std::placeholders::_1));

  if (use_amcl_) {
    amcl_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      amcl_topic_, 10,
      std::bind(&WaypointNavigator::amclCallback, this, std::placeholders::_1));
  }

  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    odom_topic_, 10,
    std::bind(&WaypointNavigator::odomCallback, this, std::placeholders::_1));

  scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
    "/scan", 10,
    std::bind(&WaypointNavigator::scanCallback, this, std::placeholders::_1));

  timer_ = create_wall_timer(
    std::chrono::milliseconds(50),
    std::bind(&WaypointNavigator::controlLoop, this));
}

// ─── PLC Done Callback ───────────────────────────────────────────
// Called when plc_bridge finishes station action → robot continues
void WaypointNavigator::plcDoneCallback(const std_msgs::msg::String::SharedPtr msg)
{
  RCLCPP_INFO(get_logger(), "✅ /plc_done received for station: [%s] → resuming", msg->data.c_str());
  plc_done_      = true;
  waiting_for_plc_ = false;
}

// ─── PLC Trigger Callback ────────────────────────────────────────
// Called continuously by plc_station_monitor with live PLC bit value
void WaypointNavigator::plcTriggerCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  plc_trigger_value_ = msg->data;
}

// ─── Lidar Obstacle Callback ────────────────────────────────────
// Called continuously by plc_lidar_monitor with front lidar obstacle bit
// DB1.DBX0.0: TRUE = obstacle detected, FALSE = obstacle cleared
void WaypointNavigator::lidarObstacleCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  bool new_state = msg->data;

  // Detect state transitions
  if (new_state != lidar_obstacle_active_) {
    lidar_obstacle_active_ = new_state;
    ramp_start_time_       = this->now();
    ramp_timer_initialized_ = true;

    if (new_state) {
      RCLCPP_WARN(get_logger(),
        "⚠ LIDAR OBSTACLE DETECTED → ramping speed DOWN over %.1f sec",
        ramp_down_duration_);
      publishStatus("LIDAR_OBSTACLE - ramping down");
    } else {
      RCLCPP_INFO(get_logger(),
        "✅ LIDAR OBSTACLE CLEARED → ramping speed UP over %.1f sec",
        ramp_up_duration_);
      publishStatus("LIDAR_CLEAR - ramping up");
    }
  }
}

// ─── Update Speed Ramp ─────────────────────────────────────────
// Called every control loop tick (20 Hz) to update speed_ramp_factor_
// based on lidar obstacle state.
//
// When obstacle detected: factor ramps from current → 0 over ramp_down_duration_
// When obstacle cleared:  factor ramps from current → 1 over ramp_up_duration_
//
void WaypointNavigator::updateSpeedRamp()
{
  if (!ramp_timer_initialized_) return;

  double elapsed = (this->now() - ramp_start_time_).seconds();

  if (lidar_obstacle_active_) {
    // Ramp DOWN: 1.0 → 0.0 over ramp_down_duration_
    double progress = std::min(elapsed / ramp_down_duration_, 1.0);
    speed_ramp_factor_ = std::max(0.0, 1.0 - progress);

    if (speed_ramp_factor_ <= 0.001) {
      speed_ramp_factor_ = 0.0;
    }

    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
      "⬇ LIDAR RAMP DOWN: factor=%.2f  elapsed=%.2fs",
      speed_ramp_factor_, elapsed);

  } else {
    // Ramp UP: 0.0 → 1.0 over ramp_up_duration_
    double progress = std::min(elapsed / ramp_up_duration_, 1.0);
    speed_ramp_factor_ = std::min(1.0, progress);

    if (speed_ramp_factor_ >= 0.999) {
      speed_ramp_factor_ = 1.0;
    }

    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
      "⬆ LIDAR RAMP UP: factor=%.2f  elapsed=%.2fs",
      speed_ramp_factor_, elapsed);
  }
}

// ─── Emergency Stop Callback ────────────────────────────────────
// Called continuously by plc_emergency_monitor with emergency button state
// DB1.DBX0.5: TRUE=normal, FALSE=emergency (inverted by monitor)
// Topic /emergency_stop: TRUE = emergency active, FALSE = normal
void WaypointNavigator::emergencyStopCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  //bool new_state = msg->data;
  bool new_state = false; // msg->data; // TEMPORARILY BYPASSED FOR TESTING

  if (new_state != emergency_active_) {
    emergency_active_ = new_state;
    emergency_ramp_start_ = this->now();
    emergency_ramp_initialized_ = true;

    if (new_state) {
      RCLCPP_ERROR(get_logger(),
        "⛔ EMERGENCY STOP PRESSED → stopping in %.1f sec!",
        emergency_ramp_down_);
      publishStatus("EMERGENCY_STOP - stopping!");
    } else {
      RCLCPP_INFO(get_logger(),
        "✅ EMERGENCY RELEASED → resuming in %.1f sec",
        emergency_ramp_up_);
      publishStatus("EMERGENCY_RELEASED - resuming");
    }
  }
}

// ─── Update Emergency Ramp ─────────────────────────────────────
// Called every control loop tick (20 Hz).
// Emergency pressed:  factor ramps 1.0 → 0.0 in 0.2s (VERY FAST)
// Emergency released: factor ramps 0.0 → 1.0 in 1.0s (smooth resume)
void WaypointNavigator::updateEmergencyRamp()
{
  if (!emergency_ramp_initialized_) return;

  double elapsed = (this->now() - emergency_ramp_start_).seconds();

  if (emergency_active_) {
    // Ramp DOWN: 1.0 → 0.0 in 0.2s
    double progress = std::min(elapsed / emergency_ramp_down_, 1.0);
    emergency_ramp_factor_ = std::max(0.0, 1.0 - progress);
    if (emergency_ramp_factor_ <= 0.001) emergency_ramp_factor_ = 0.0;

    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 500,
      "⛔ EMERGENCY RAMP DOWN: factor=%.2f  elapsed=%.2fs",
      emergency_ramp_factor_, elapsed);
  } else {
    // Ramp UP: 0.0 → 1.0 in 1.0s
    double progress = std::min(elapsed / emergency_ramp_up_, 1.0);
    emergency_ramp_factor_ = std::min(1.0, progress);
    if (emergency_ramp_factor_ >= 0.999) emergency_ramp_factor_ = 1.0;

    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
      "✅ EMERGENCY RAMP UP: factor=%.2f  elapsed=%.2fs",
      emergency_ramp_factor_, elapsed);
  }
}

// ─── Cycles Callback ──────────────────────────────────────────────
void WaypointNavigator::cyclesCallback(const std_msgs::msg::Int32::SharedPtr msg)
{
  int requested = msg->data;

  if (requested == 0) {
    stopRobot();
    state_ = State::ALL_DONE;
    max_cycles_ = 0; loop_forever_ = false;
    RCLCPP_WARN(get_logger(), "🛑 STOP command! Robot halted.");
    publishStatus("STOPPED by user");
    return;
  }

  if (requested == -1) {
    if (waypoints_.empty()) {
      RCLCPP_ERROR(get_logger(), "❌ No waypoints loaded! Select a mission first:");
      RCLCPP_ERROR(get_logger(), "   ros2 topic pub --once /load_mission std_msgs/msg/String \"data: 'mission_A'\"");
      return;
    }
    loop_forever_ = true; max_cycles_ = -1;
    cycles_done_ = 0; wp_index_ = 0;
    if (pose_received_) {
      state_ = State::ROTATE_TO_HEADING;
      RCLCPP_INFO(get_logger(), "🔁 LOOP FOREVER started!");
    }
    publishStatus("RUNNING - loop forever");
    return;
  }

  if (requested > 0) {
    if (waypoints_.empty()) {
      RCLCPP_ERROR(get_logger(), "❌ No waypoints loaded! Select a mission first:");
      RCLCPP_ERROR(get_logger(), "   ros2 topic pub --once /load_mission std_msgs/msg/String \"data: 'mission_A'\"");
      return;
    }
    // ── Robot is currently running — add more cycles on top ───
    if (state_ != State::IDLE && state_ != State::ALL_DONE) {
      max_cycles_ = cycles_done_ + requested;
      loop_forever_ = false;
      RCLCPP_INFO(get_logger(), "🔄 %d more cycle(s) added. Total=%d", requested, max_cycles_);
      char buf[64]; snprintf(buf, sizeof(buf), "RUNNING - %d more added", requested);
      publishStatus(buf);
      return;
    }

    // ── Fresh start (IDLE or ALL_DONE — both reset fully) ─────
    loop_forever_ = false;
    max_cycles_   = requested;
    cycles_done_  = 0;       // ← CRITICAL: reset cycle counter
    wp_index_     = 0;       // ← CRITICAL: reset waypoint index

    if (pose_received_) {
      state_ = State::ROTATE_TO_HEADING;
      RCLCPP_INFO(get_logger(), "🚀 Starting %d cycle(s)! (fresh start)", requested);
    } else {
      state_ = State::IDLE;  // stay IDLE until pose arrives
      RCLCPP_INFO(get_logger(), "✅ %d cycle(s) queued. Waiting for pose...", requested);
    }
    char buf[64]; snprintf(buf, sizeof(buf), "STARTING %d cycles", requested);
    publishStatus(buf);
  }
}

// ─── AMCL Callback ────────────────────────────────────────────────
void WaypointNavigator::amclCallback(
  const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  if (init_failed_) return;
  robot_x_   = msg->pose.pose.position.x;
  robot_y_   = msg->pose.pose.position.y;
  robot_yaw_ = quaternionToYaw(msg->pose.pose.orientation);

  if (!pose_received_) {
    pose_received_ = true;
    RCLCPP_INFO(get_logger(), "✅ AMCL active! x=%.2f y=%.2f yaw=%.1f°",
      robot_x_, robot_y_, robot_yaw_ * 180.0 / M_PI);
    RCLCPP_INFO(get_logger(), "   Now send /set_cycles to start:");
    RCLCPP_INFO(get_logger(), "   ros2 topic pub --once /set_cycles std_msgs/msg/Int32 \"data: 1\"");

    // Only start if user already sent /set_cycles before AMCL was ready
    if (max_cycles_ != 0 && state_ == State::IDLE) {
      state_ = State::ROTATE_TO_HEADING;
      RCLCPP_INFO(get_logger(), "🚀 Cycles were queued — starting now!");
    }
    // Otherwise robot stays IDLE and waits for /set_cycles
  }
}

// ─── Odom Callback ────────────────────────────────────────────────
void WaypointNavigator::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  if (init_failed_) return;
  if (use_amcl_ && pose_received_) return;

  robot_x_   = msg->pose.pose.position.x;
  robot_y_   = msg->pose.pose.position.y;
  robot_yaw_ = quaternionToYaw(msg->pose.pose.orientation);

  if (!pose_received_) {
    pose_received_ = true;
    RCLCPP_INFO(get_logger(), "✅ Odom active! x=%.2f y=%.2f", robot_x_, robot_y_);
    if (max_cycles_ != 0 && state_ == State::IDLE)
      state_ = State::ROTATE_TO_HEADING;
  }
}

// ─── Scan Callback ────────────────────────────────────────────────
void WaypointNavigator::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  if (!stop_on_obstacle_) return;
  obstacle_ahead_ = false;

  // For REVERSE: check rear sector (±30° from back)
  // For FORWARD: check front sector (±30° from front)
  // We store both
  obstacle_front_ = false;
  obstacle_rear_  = false;

  int n = static_cast<int>(msg->ranges.size());
  for (int i = 0; i < n; ++i) {
    float angle = msg->angle_min + i * msg->angle_increment;
    float r     = msg->ranges[i];
    if (!std::isfinite(r) || r < 0.01f || r >= static_cast<float>(obstacle_stop_dist_)) continue;

    // Front sector ±30°
    if (angle >= -0.524f && angle <= 0.524f)
      obstacle_front_ = true;

    // Rear sector ±30° (around ±PI)
    if (std::abs(std::abs(angle) - M_PI) < 0.524f)
      obstacle_rear_ = true;
  }
}

// ─── Helpers ──────────────────────────────────────────────────────
double WaypointNavigator::quaternionToYaw(const geometry_msgs::msg::Quaternion & q)
{
  return std::atan2(
    2.0 * (q.w * q.z + q.x * q.y),
    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

double WaypointNavigator::normalizeAngle(double a)
{
  while (a >  M_PI) a -= 2.0 * M_PI;
  while (a < -M_PI) a += 2.0 * M_PI;
  return a;
}

void WaypointNavigator::stopRobot()
{
  cmd_vel_pub_->publish(geometry_msgs::msg::Twist());
}

void WaypointNavigator::publishStatus(const std::string & text)
{
  std_msgs::msg::String msg;
  msg.data = text;
  status_pub_->publish(msg);
}

bool WaypointNavigator::isOnlyRotation(const Waypoint & prev, const Waypoint & curr)
{
  return (std::abs(curr.x - prev.x) < xy_tolerance_ &&
          std::abs(curr.y - prev.y) < xy_tolerance_);
}

const char * WaypointNavigator::stateStr()
{
  switch (state_) {
    case State::IDLE:              return "IDLE";
    case State::ROTATE_TO_HEADING: return "ROT_HEADING";
    case State::DRIVE_STRAIGHT:    return "DRIVE_FWD";
    case State::DRIVE_REVERSE:     return "DRIVE_REV";
    case State::ROTATE_TO_THETA:   return "ROT_THETA";
    case State::WAIT_PLC:          return "WAIT_PLC";
    case State::WAYPOINT_DONE:     return "WP_DONE";
    case State::CYCLE_DONE:        return "CYCLE_DONE";
    case State::ALL_DONE:          return "ALL_DONE";
  }
  return "UNKNOWN";
}

int WaypointNavigator::getTagIndex(const std::string & name)
{
  std::string clean = name;
  std::transform(clean.begin(), clean.end(), clean.begin(), ::tolower);
  clean.erase(std::remove_if(clean.begin(), clean.end(), [](unsigned char c) {
    return std::isspace(c) || c == '#';
  }), clean.end());

  for (size_t i = 0; i < ordered_tags_.size(); ++i) {
    std::string tag = ordered_tags_[i];
    std::transform(tag.begin(), tag.end(), tag.begin(), ::tolower);
    tag.erase(std::remove_if(tag.begin(), tag.end(), [](unsigned char c) {
      return std::isspace(c) || c == '#';
    }), tag.end());
    if (clean == tag) {
      return static_cast<int>(i);
    }
  }
  return -1;
}

std::string WaypointNavigator::getTagName(int index)
{
  if (index >= 0 && index < static_cast<int>(ordered_tags_.size())) {
    return ordered_tags_[index];
  }
  return "";
}

// ─── Load Waypoints ───────────────────────────────────────────────
bool WaypointNavigator::loadWaypoints(const std::string & path)
{
  if (path.empty()) { RCLCPP_ERROR(get_logger(), "waypoints_file is empty!"); return false; }
  std::ifstream f(path);
  if (!f.good()) { RCLCPP_ERROR(get_logger(), "Cannot open: %s", path.c_str()); return false; }
  f.seekg(0, std::ios::end);
  if (f.tellg() == 0) { RCLCPP_ERROR(get_logger(), "File empty: %s", path.c_str()); return false; }
  f.close();

  try {
    YAML::Node root = YAML::LoadFile(path);
    YAML::Node wps  = root["waypoints"];
    if (!wps || !wps.IsSequence()) {
      RCLCPP_ERROR(get_logger(), "No 'waypoints' list in YAML"); return false;
    }
    for (size_t i = 0; i < wps.size(); ++i) {
      Waypoint w;
      w.x       = wps[i]["x"].as<double>();
      w.y       = wps[i]["y"].as<double>();
      w.theta   = wps[i]["theta"].as<double>();
      w.type         = wps[i]["type"]         ? wps[i]["type"].as<std::string>()         : "waypoint";
      w.mission      = wps[i]["mission"]      ? wps[i]["mission"].as<std::string>()      : "forward";
      w.rotation     = wps[i]["rotation"]     ? wps[i]["rotation"].as<std::string>()     : "auto";
      w.name         = wps[i]["name"]         ? wps[i]["name"].as<std::string>()         : "";
      w.plc_feedback = wps[i]["plc_feedback"] ? wps[i]["plc_feedback"].as<std::string>() : "";
      w.speed        = wps[i]["speed"]        ? wps[i]["speed"].as<double>()             : 0.0;
      waypoints_.push_back(w);

      RCLCPP_INFO(get_logger(),
        "  WP[%02zu] %-10s %-8s rot=%-5s x=%.2f y=%.2f theta=%.1f° spd=%s",
        i, w.type.c_str(), w.mission.c_str(), w.rotation.c_str(),
        w.x, w.y, w.theta * 180.0 / M_PI,
        w.speed > 0.0 ? (std::to_string(w.speed) + " m/s").c_str() : "default");
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "YAML error: %s", e.what()); return false;
  }
  return !waypoints_.empty();
}

// ─── Load Missions Folder ─────────────────────────────────────────
// Scans a folder for .yaml files. Each file = one mission.
// File name (without .yaml) becomes the mission ID.
//
// Folder structure:
//   missions/
//     tag1_to_tag5.yaml      ← mission ID: "tag1_to_tag5"
//     tag5_to_tag8.yaml      ← mission ID: "tag5_to_tag8"
//     home.yaml              ← mission ID: "home"
//
// Each .yaml file uses the SAME format as waypoints.yaml:
//   waypoints:
//     - x: 0.0
//       y: 0.0
//       theta: 0.0
//       type: waypoint
//       mission: forward
//       ...
bool WaypointNavigator::loadMissionsFolder(const std::string & folder_path)
{
  namespace fs = std::filesystem;

  if (folder_path.empty()) {
    RCLCPP_ERROR(get_logger(), "missions_folder is empty!");
    return false;
  }

  if (!fs::exists(folder_path) || !fs::is_directory(folder_path)) {
    RCLCPP_ERROR(get_logger(), "Missions folder does not exist: %s", folder_path.c_str());
    return false;
  }

  missions_map_.clear();

  // Collect and sort .yaml files for consistent ordering
  std::vector<fs::path> yaml_files;
  for (const auto & entry : fs::directory_iterator(folder_path)) {
    if (entry.is_regular_file()) {
      std::string ext = entry.path().extension().string();
      if (ext == ".yaml" || ext == ".yml") {
        yaml_files.push_back(entry.path());
      }
    }
  }
  std::sort(yaml_files.begin(), yaml_files.end());

  for (const auto & file_path : yaml_files) {
    // Mission name = filename without extension
    std::string mission_name = file_path.stem().string();

    if (mission_name == "tag_sequence") {
      try {
        YAML::Node root = YAML::LoadFile(file_path.string());
        if (root["ordered_tags"] && root["ordered_tags"].IsSequence()) {
          ordered_tags_.clear();
          for (size_t i = 0; i < root["ordered_tags"].size(); ++i) {
            ordered_tags_.push_back(root["ordered_tags"][i].as<std::string>());
          }
          RCLCPP_INFO(get_logger(), "  ℹ️ Loaded tag sequence from: %s",
            file_path.filename().c_str());
        }
      } catch (const std::exception & e) {
        RCLCPP_ERROR(get_logger(), "Failed to parse tag_sequence.yaml: %s", e.what());
      }
      continue;
    }

    try {
      YAML::Node root = YAML::LoadFile(file_path.string());
      YAML::Node wps  = root["waypoints"];

      if (!wps || !wps.IsSequence() || wps.size() == 0) {
        RCLCPP_WARN(get_logger(), "  ⚠ File '%s' has no 'waypoints' list — skipping",
          file_path.filename().c_str());
        continue;
      }

      std::vector<Waypoint> mission_waypoints;
      for (size_t i = 0; i < wps.size(); ++i) {
        Waypoint w;
        w.x            = wps[i]["x"].as<double>();
        w.y            = wps[i]["y"].as<double>();
        w.theta        = wps[i]["theta"].as<double>();
        w.type         = wps[i]["type"]         ? wps[i]["type"].as<std::string>()         : "waypoint";
        w.mission      = wps[i]["mission"]      ? wps[i]["mission"].as<std::string>()      : "forward";
        w.rotation     = wps[i]["rotation"]     ? wps[i]["rotation"].as<std::string>()     : "auto";
        w.name         = wps[i]["name"]         ? wps[i]["name"].as<std::string>()         : "";
        w.plc_feedback = wps[i]["plc_feedback"] ? wps[i]["plc_feedback"].as<std::string>() : "";
        w.speed        = wps[i]["speed"]        ? wps[i]["speed"].as<double>()             : 0.0;
        mission_waypoints.push_back(w);
      }

      missions_map_[mission_name] = mission_waypoints;
      RCLCPP_INFO(get_logger(), "  ✅ %-25s : %zu waypoints  (%s)",
        mission_name.c_str(), mission_waypoints.size(), file_path.filename().c_str());

    } catch (const std::exception & e) {
      RCLCPP_WARN(get_logger(), "  ⚠ Error reading '%s': %s — skipping",
        file_path.filename().c_str(), e.what());
    }
  }

  if (missions_map_.empty()) {
    RCLCPP_ERROR(get_logger(), "No valid mission files found in: %s", folder_path.c_str());
    return false;
  }

  missions_loaded_ = true;
  RCLCPP_INFO(get_logger(), "  ─────────────────────────────────────────");
  RCLCPP_INFO(get_logger(), "  Total: %zu missions loaded from folder", missions_map_.size());
  return true;
}

// ─── Select Mission ──────────────────────────────────────────────
// Switch the active waypoints to a named mission
bool WaypointNavigator::selectMission(const std::string & mission_name)
{
  auto it = missions_map_.find(mission_name);
  if (it == missions_map_.end()) {
    RCLCPP_ERROR(get_logger(), "❌ Mission '%s' not found!", mission_name.c_str());
    RCLCPP_ERROR(get_logger(), "   Available missions:");
    for (const auto & [name, wps] : missions_map_) {
      RCLCPP_ERROR(get_logger(), "     - %s (%zu waypoints)", name.c_str(), wps.size());
    }
    return false;
  }

  // Stop robot and reset navigation state
  stopRobot();
  state_       = State::IDLE;
  wp_index_    = 0;
  max_cycles_  = 0;
  cycles_done_ = 0;
  loop_forever_ = false;

  // Load new waypoints
  waypoints_ = it->second;
  waypoint_mission_names_.assign(waypoints_.size(), mission_name);
  current_mission_name_ = mission_name;
  queue_active_ = false;

  RCLCPP_INFO(get_logger(), "═══════════════════════════════════════════");
  RCLCPP_INFO(get_logger(), "  🔄 MISSION LOADED: %s", mission_name.c_str());
  RCLCPP_INFO(get_logger(), "  Waypoints: %zu", waypoints_.size());
  for (size_t i = 0; i < waypoints_.size(); ++i) {
    const auto & w = waypoints_[i];
    RCLCPP_INFO(get_logger(),
      "  WP[%02zu] %-10s %-8s rot=%-5s x=%.2f y=%.2f theta=%.1f°",
      i, w.type.c_str(), w.mission.c_str(), w.rotation.c_str(),
      w.x, w.y, w.theta * 180.0 / M_PI);
  }
  RCLCPP_INFO(get_logger(), "  ➤ Now send: ros2 topic pub --once /set_cycles std_msgs/msg/Int32 \"data: 1\"");
  RCLCPP_INFO(get_logger(), "═══════════════════════════════════════════");

  publishStatus("MISSION_LOADED: " + mission_name);
  return true;
}

// ─── Publish Mission List ─────────────────────────────────────────
// Publishes available mission names on /mission_list as comma-separated string
void WaypointNavigator::publishMissionList()
{
  std_msgs::msg::String msg;
  std::string list;
  for (const auto & [name, wps] : missions_map_) {
    if (!list.empty()) list += ",";
    list += name + "(" + std::to_string(wps.size()) + "wp)";
  }
  msg.data = list;
  mission_list_pub_->publish(msg);
}

// ─── Load Mission Callback ───────────────────────────────────────
// Called when user publishes on /load_mission:
//   ros2 topic pub --once /load_mission std_msgs/msg/String "data: 'tag1_to_tag5'"
//   ros2 topic pub --once /load_mission std_msgs/msg/String "data: 'list'"
//
// The robot will:
//   1. Stop current navigation (if moving)
//   2. Load the new waypoints
//   3. AUTO-START 1 cycle immediately (no need for /set_cycles!)
void WaypointNavigator::loadMissionCallback(const std_msgs::msg::String::SharedPtr msg)
{
  std::string requested = msg->data;

  // Special command: "list" → show available missions
  if (requested == "list") {
    RCLCPP_INFO(get_logger(), "═══════════════════════════════════════════");
    RCLCPP_INFO(get_logger(), "  📋 AVAILABLE MISSIONS:");
    for (const auto & [name, wps] : missions_map_) {
      std::string marker = (name == current_mission_name_) ? " ← ACTIVE" : "";
      RCLCPP_INFO(get_logger(), "     - %-25s (%zu waypoints)%s",
        name.c_str(), wps.size(), marker.c_str());
    }
    RCLCPP_INFO(get_logger(), "═══════════════════════════════════════════");
    publishMissionList();
    return;
  }

  if (!missions_loaded_) {
    RCLCPP_ERROR(get_logger(), "❌ No missions folder loaded! Cannot switch missions.");
    RCLCPP_ERROR(get_logger(), "   Start with missions_folder parameter to enable this feature.");
    return;
  }

  // If robot is currently moving, stop it first
  if (state_ != State::IDLE && state_ != State::ALL_DONE) {
    RCLCPP_WARN(get_logger(), "⚠ Robot is currently moving — stopping before mission switch");
    stopRobot();
  }

  if (selectMission(requested)) {
    // AUTO-START: immediately begin 1 cycle after loading
    if (pose_received_) {
      max_cycles_   = 1;
      cycles_done_  = 0;
      wp_index_     = 0;
      loop_forever_ = false;
      state_        = State::ROTATE_TO_HEADING;
      RCLCPP_INFO(get_logger(), "🚀 Mission '%s' AUTO-STARTED! (1 cycle)", requested.c_str());
      publishStatus("RUNNING: " + requested);
    } else {
      RCLCPP_INFO(get_logger(), "Mission '%s' loaded -- waiting for AMCL pose, then will start.",
        requested.c_str());
    }
  }
}

// --- Queue Missions Callback -----------------------------------------------
// Called when user publishes on /queue_missions:
//   ros2 topic pub --once /queue_missions std_msgs/msg/String "data: 'tag3,tag7,tag9,tag1,home'"
//
// The robot will:
//   1. Stop current navigation (if moving)
//   2. Parse the comma-separated list of mission names
//   3. Validate all missions exist
//   4. Start the FIRST mission immediately
//   5. When each mission completes, automatically load and start the next one
//   6. Robot remembers its position between missions (no reset needed)
//
// Special commands:
//   "status" -> show current queue status
//   "cancel" -> cancel current queue, stop robot
//   "clear"  -> alias for cancel
//   "add:tag5,tag6" -> append to existing queue without stopping
void WaypointNavigator::queueMissionsCallback(const std_msgs::msg::String::SharedPtr msg)
{
  std::string input = msg->data;

  // --- Special command: status ---
  if (input == "status") {
    RCLCPP_INFO(get_logger(), "========================================");
    if (queue_active_) {
      RCLCPP_INFO(get_logger(), "  QUEUE STATUS: ACTIVE");
      if (wp_index_ < waypoint_mission_names_.size()) {
        RCLCPP_INFO(get_logger(), "  Current mission : %s", waypoint_mission_names_[wp_index_].c_str());
      }
      RCLCPP_INFO(get_logger(), "  Completed waypoints : %zu / %zu", wp_index_, waypoints_.size());
    } else {
      RCLCPP_INFO(get_logger(), "  QUEUE STATUS: IDLE (no queue active)");
    }
    RCLCPP_INFO(get_logger(), "========================================");

    // Publish status on topic
    std_msgs::msg::String qmsg;
    if (queue_active_) {
      std::string current_name = (wp_index_ < waypoint_mission_names_.size()) ? waypoint_mission_names_[wp_index_] : "unknown";
      qmsg.data = "ACTIVE: " + current_name + " (WP " + std::to_string(wp_index_) + "/" + std::to_string(waypoints_.size()) + ")";
    } else {
      qmsg.data = "IDLE";
    }
    queue_status_pub_->publish(qmsg);
    return;
  }

  // --- Special command: cancel/clear ---
  if (input == "cancel" || input == "clear") {
    if (queue_active_) {
      RCLCPP_WARN(get_logger(), "QUEUE CANCELLED!");
      queue_active_ = false;
      stopRobot();
      state_ = State::ALL_DONE;
      publishStatus("QUEUE_CANCELLED");

      std_msgs::msg::String qmsg;
      qmsg.data = "CANCELLED";
      queue_status_pub_->publish(qmsg);
    } else {
      RCLCPP_INFO(get_logger(), "No queue active to cancel.");
    }
    return;
  }

  // --- Special command: add:mission1,mission2 ---
  bool append_mode = false;
  if (input.rfind("add:", 0) == 0) {
    input = input.substr(4);
    append_mode = true;
  }

  if (!missions_loaded_) {
    RCLCPP_ERROR(get_logger(), "No missions folder loaded! Cannot queue missions.");
    return;
  }

  // Parse requested missions (support both comma-separated list and "start to end" range syntax)
  struct TagStep {
    std::string name;
    std::string dir; // "forward", "reverse", or ""
  };
  std::vector<TagStep> tag_steps;
  std::vector<std::string> requested_missions;
  bool is_range_cmd = false;
  
  std::string lower_input = input;
  std::transform(lower_input.begin(), lower_input.end(), lower_input.begin(), ::tolower);
  size_t to_pos = lower_input.find(" to ");

  if (to_pos != std::string::npos) {
    is_range_cmd = true;
    std::string start_str = input.substr(0, to_pos);
    std::string end_str   = input.substr(to_pos + 4);

    // Trim whitespace from start_str
    size_t s_start = start_str.find_first_not_of(" \t\n\r");
    size_t s_end   = start_str.find_last_not_of(" \t\n\r");
    if (s_start != std::string::npos && s_end != std::string::npos) {
      start_str = start_str.substr(s_start, s_end - s_start + 1);
    }

    // Trim whitespace from end_str
    size_t e_start = end_str.find_first_not_of(" \t\n\r");
    size_t e_end   = end_str.find_last_not_of(" \t\n\r");
    if (e_start != std::string::npos && e_end != std::string::npos) {
      end_str = end_str.substr(e_start, e_end - e_start + 1);
    }

    int start_idx = getTagIndex(start_str);
    int end_idx   = getTagIndex(end_str);

    if (start_idx >= 0 && end_idx >= 0) {
      // Find nearest tag to current robot pose if available
      int robot_tag_idx = -1;
      if (pose_received_) {
        double min_dist = std::numeric_limits<double>::max();
        for (size_t i = 0; i < ordered_tags_.size(); ++i) {
          std::string tag_name = ordered_tags_[i];
          auto it = missions_map_.find(tag_name);
          if (it != missions_map_.end() && !it->second.empty()) {
            double tx = it->second.front().x;
            double ty = it->second.front().y;
            double dist = std::hypot(tx - robot_x_, ty - robot_y_);
            if (dist < min_dist) {
              min_dist = dist;
              robot_tag_idx = static_cast<int>(i);
            }
          }
        }
        if (min_dist > 3.0) {
          robot_tag_idx = -1;
        }
      }

      int cur_idx = (robot_tag_idx >= 0) ? robot_tag_idx : start_idx;
      RCLCPP_INFO(get_logger(), "📍 Position-aware Routing: Robot nearest tag = %s (idx %d), Range = %s (idx %d) -> %s (idx %d)",
        (robot_tag_idx >= 0 ? getTagName(robot_tag_idx).c_str() : "unknown"), cur_idx,
        start_str.c_str(), start_idx, end_str.c_str(), end_idx);

      // Segment 1: from robot current position (cur_idx) to requested start_idx
      if (cur_idx < start_idx) {
        for (int i = cur_idx + 1; i <= start_idx; ++i) {
          tag_steps.push_back({getTagName(i), "forward"});
        }
      } else if (cur_idx > start_idx) {
        for (int i = cur_idx - 1; i >= start_idx; --i) {
          tag_steps.push_back({getTagName(i), "reverse"});
        }
      }

      // Segment 2: from requested start_idx to requested end_idx
      if (start_idx < end_idx) {
        for (int i = start_idx + 1; i <= end_idx; ++i) {
          tag_steps.push_back({getTagName(i), "forward"});
        }
      } else if (start_idx > end_idx) {
        for (int i = start_idx - 1; i >= end_idx; --i) {
          tag_steps.push_back({getTagName(i), "reverse"});
        }
      } else if (tag_steps.empty()) {
        tag_steps.push_back({getTagName(start_idx), "forward"});
      }

      for (const auto & step : tag_steps) {
        requested_missions.push_back(step.name);
      }
    } else {
      RCLCPP_ERROR(get_logger(), "Invalid range format or tags: '%s' (idx: %d) or '%s' (idx: %d)",
        start_str.c_str(), start_idx, end_str.c_str(), end_idx);
      return;
    }
  } else {
    // Parse comma-separated mission names
    std::istringstream iss(input);
    std::string token;
    while (std::getline(iss, token, ',')) {
      // Trim whitespace
      size_t start = token.find_first_not_of(" \t\n\r");
      size_t end   = token.find_last_not_of(" \t\n\r");
      if (start != std::string::npos && end != std::string::npos) {
        std::string name = token.substr(start, end - start + 1);
        tag_steps.push_back({name, ""});
        requested_missions.push_back(name);
      }
    }
  }

  if (requested_missions.empty()) {
    RCLCPP_ERROR(get_logger(), "Empty mission queue! Send comma-separated names or range syntax:");
    RCLCPP_ERROR(get_logger(), "  ros2 topic pub --once /queue_missions std_msgs/msg/String \"data: 'tag3 to tag7'\"");
    RCLCPP_ERROR(get_logger(), "  ros2 topic pub --once /queue_missions std_msgs/msg/String \"data: 'tag3,tag7,tag1,home'\"");
    return;
  }

  // Validate all mission names exist
  std::vector<std::string> invalid;
  for (const auto & name : requested_missions) {
    if (missions_map_.find(name) == missions_map_.end()) {
      invalid.push_back(name);
    }
  }
  if (!invalid.empty()) {
    RCLCPP_ERROR(get_logger(), "INVALID mission names in queue:");
    for (const auto & name : invalid) {
      RCLCPP_ERROR(get_logger(), "  '%s' -- NOT FOUND", name.c_str());
    }
    RCLCPP_ERROR(get_logger(), "Available missions:");
    for (const auto & [name, wps] : missions_map_) {
      RCLCPP_ERROR(get_logger(), "  - %s (%zu waypoints)", name.c_str(), wps.size());
    }
    return;
  }

  // --- Append mode: add to existing queue ---
  if (append_mode && queue_active_) {
    for (const auto & step : tag_steps) {
      auto & wps = missions_map_[step.name];
      for (const auto & orig_w : wps) {
        Waypoint w = orig_w;
        if (!step.dir.empty()) {
          w.mission = step.dir;
        }
        waypoints_.push_back(w);
        waypoint_mission_names_.push_back(step.name);
      }
    }
    queue_total_ += requested_missions.size();
    RCLCPP_INFO(get_logger(), "========================================");
    RCLCPP_INFO(get_logger(), "  APPENDED %zu mission(s) to queue", requested_missions.size());
    RCLCPP_INFO(get_logger(), "  New queue total: %zu", queue_total_);
    RCLCPP_INFO(get_logger(), "========================================");

    std_msgs::msg::String qmsg;
    qmsg.data = "APPENDED: " + std::to_string(requested_missions.size()) +
                " missions (total: " + std::to_string(queue_total_) + ")";
    queue_status_pub_->publish(qmsg);
    return;
  }

  // --- New queue: stop current and start fresh ---
  if (state_ != State::IDLE && state_ != State::ALL_DONE) {
    RCLCPP_WARN(get_logger(), "Robot is currently moving -- stopping for new mission queue");
    stopRobot();
  }

  // Clear previous
  waypoints_.clear();
  waypoint_mission_names_.clear();
  last_tracked_mission_.clear();
  wp_index_ = 0;
  cycles_done_ = 0;
  max_cycles_ = 1;
  loop_forever_ = false;

  // Build unified path with per-step direction overriding
  for (const auto & step : tag_steps) {
    auto & wps = missions_map_[step.name];
    for (const auto & orig_w : wps) {
      Waypoint w = orig_w;
      if (!step.dir.empty()) {
        w.mission = step.dir;
      }
      waypoints_.push_back(w);
      waypoint_mission_names_.push_back(step.name);
    }
  }

  // ── Skip starting waypoint if robot is already standing at/near it ──
  // Prevents turning around to visit a waypoint that the robot is already at or slightly past.
  if (pose_received_ && waypoints_.size() > 1) {
    double dist_to_start = std::hypot(waypoints_.front().x - robot_x_, waypoints_.front().y - robot_y_);
    if (dist_to_start < 1.5) {
      RCLCPP_INFO(get_logger(),
        "  ⏭ Skipping start WP '%s' (%.2f, %.2f) — robot already at start location (dist=%.2fm)",
        waypoint_mission_names_.front().c_str(), waypoints_.front().x, waypoints_.front().y, dist_to_start);
      waypoints_.erase(waypoints_.begin());
      waypoint_mission_names_.erase(waypoint_mission_names_.begin());
    }
  }

  // ── Override intermediate station waypoints to "waypoint" pass-through ──
  // Only the FINAL destination in a multi-waypoint route acts as a station stop.
  // All intermediate waypoints along the route pass through smoothly without pausing for PLC.
  if (waypoints_.size() > 1) {
    for (size_t i = 0; i < waypoints_.size() - 1; ++i) {
      if (waypoints_[i].type == "station") {
        RCLCPP_INFO(get_logger(),
          "  🔄 Intermediate WP[%zu] '%s' converted station → waypoint (pass-through mode)",
          i, waypoint_mission_names_[i].c_str());
        waypoints_[i].type = "waypoint";
      }
    }
  }
  queue_total_  = requested_missions.size();
  queue_active_ = true;

  // Log the full queue plan
  RCLCPP_INFO(get_logger(), "========================================");
  RCLCPP_INFO(get_logger(), "  DYNAMIC MISSION QUEUE (%s)",
    is_range_cmd ? "POSITION-AWARE RANGE" : "CONCATENATED");
  RCLCPP_INFO(get_logger(), "  Total missions: %zu", queue_total_);
  RCLCPP_INFO(get_logger(), "  Plan:");
  for (size_t i = 0; i < tag_steps.size(); ++i) {
    const auto & step = tag_steps[i];
    auto it = missions_map_.find(step.name);
    RCLCPP_INFO(get_logger(), "    [%zu] %s (%zu waypoints, dir=%s)",
      i + 1, step.name.c_str(), it != missions_map_.end() ? it->second.size() : 0,
      step.dir.empty() ? "default" : step.dir.c_str());
  }
  RCLCPP_INFO(get_logger(), "========================================");

  // Publish queue status
  std_msgs::msg::String qmsg;
  std::string plan;
  for (const auto & name : requested_missions) {
    if (!plan.empty()) plan += " -> ";
    plan += name;
  }
  qmsg.data = "QUEUE_STARTED: " + plan;
  queue_status_pub_->publish(qmsg);

  // Start
  if (pose_received_) {
    state_ = State::ROTATE_TO_HEADING;
  }
}

// --- Main Control Loop  20 Hz ---------------------------------------------
void WaypointNavigator::controlLoop()
{
  if (init_failed_ || !pose_received_) return;

  // ── Update lidar obstacle speed ramp every tick ────────────────
  updateSpeedRamp();
  // ── Update emergency stop ramp every tick ───────────────────
  updateEmergencyRamp();

  if (queue_active_ && wp_index_ < waypoint_mission_names_.size()) {
    std::string current_mission = waypoint_mission_names_[wp_index_];
    if (current_mission != last_tracked_mission_) {
      last_tracked_mission_ = current_mission;
      RCLCPP_INFO(get_logger(), "QUEUE: Now executing mission '%s'", current_mission.c_str());
      publishStatus("RUNNING: " + current_mission);

      std_msgs::msg::String qmsg;
      qmsg.data = "RUNNING: " + current_mission;
      queue_status_pub_->publish(qmsg);
    }
  }

  if (state_ == State::IDLE)     { stopRobot(); return; }
  if (state_ == State::ALL_DONE) {
    stopRobot();
    // Stay here — cyclesCallback will reset state when user sends new command
    return;
  }

  if (wp_index_ >= waypoints_.size()) {
    if (state_ != State::CYCLE_DONE) {
      state_ = State::CYCLE_DONE;
      return;   // first time: set state and come back next tick
    }
    // state IS CYCLE_DONE — fall through to handler below
  }

  // -- CYCLE DONE -----------------------------------------------------------
  if (state_ == State::CYCLE_DONE) {
    cycles_done_++;
    if (loop_forever_ || cycles_done_ < max_cycles_) {
      wp_index_ = 0;
      state_    = State::ROTATE_TO_HEADING;
      RCLCPP_INFO(get_logger(), loop_forever_ ?
        "Cycle %d done -- looping!" : "Cycle %d/%d done -- next cycle!",
        cycles_done_, max_cycles_);
    } else {
      if (queue_active_) {
        queue_active_ = false;
        state_ = State::ALL_DONE;
        stopRobot();
        RCLCPP_INFO(get_logger(), "ALL %zu QUEUED MISSIONS COMPLETE!", queue_total_);

        // Publish final queue status
        std_msgs::msg::String qmsg;
        qmsg.data = "QUEUE_COMPLETE: " + std::to_string(queue_total_) + " missions done";
        queue_status_pub_->publish(qmsg);
        publishStatus("QUEUE_COMPLETE: all " + std::to_string(queue_total_) + " missions done");
      } else {
        state_ = State::ALL_DONE;
        stopRobot();
        RCLCPP_INFO(get_logger(), "ALL %d CYCLE(S) DONE!", cycles_done_);
      }
    }
    return;
  }

  const Waypoint & wp = waypoints_[wp_index_];
  const bool is_reverse = (wp.mission == "reverse");

  double dx            = wp.x - robot_x_;
  double dy            = wp.y - robot_y_;
  double distance      = std::hypot(dx, dy);
  double angle_to_goal = std::atan2(dy, dx);

  // For reverse: robot needs its BACK to face goal
  // So target heading = angle_to_goal + PI
  double heading_target = is_reverse
    ? normalizeAngle(angle_to_goal + M_PI)
    : angle_to_goal;

  double heading_error = normalizeAngle(heading_target - robot_yaw_);
  double yaw_error     = normalizeAngle(wp.theta - robot_yaw_);

  geometry_msgs::msg::Twist cmd;

  // 1 second log
  if (++log_counter_ >= 20) {
    log_counter_ = 0;
    RCLCPP_INFO(get_logger(),
      "[%s | %s | WP%zu] pos(%.2f,%.2f,%.1f°)→(%.2f,%.2f,%.1f°) dist=%.2fm",
      stateStr(), is_reverse ? "REV" : "FWD", wp_index_,
      robot_x_, robot_y_, robot_yaw_ * 180.0 / M_PI,
      wp.x, wp.y, wp.theta * 180.0 / M_PI, distance);
  }

  switch (state_)
  {
    // ════════════════════════════════════════════════════════════
    // ROTATE_TO_HEADING
    // Forward:  rotate until robot FACES the goal
    // Reverse:  rotate until robot BACK is toward the goal
    // ════════════════════════════════════════════════════════════
    case State::ROTATE_TO_HEADING:
    {
      Waypoint prev = (wp_index_ > 0) ?
        waypoints_[wp_index_ - 1] :
        Waypoint{robot_x_, robot_y_, robot_yaw_};

      if (isOnlyRotation(prev, wp)) {
        state_ = State::ROTATE_TO_THETA;
        RCLCPP_INFO(get_logger(), "[WP%zu] Pure rotation → %.1f°",
          wp_index_, wp.theta * 180.0 / M_PI);
        break;
      }

      if (distance < xy_tolerance_) {
        stopRobot(); state_ = State::ROTATE_TO_THETA;
        break;
      }

      if (std::abs(heading_error) < heading_tolerance_) {
        stopRobot();
        // Go to correct drive state based on mission
        state_ = is_reverse ? State::DRIVE_REVERSE : State::DRIVE_STRAIGHT;
        RCLCPP_INFO(get_logger(),
          "[WP%zu] ✓ Heading aligned (%.1f°). %s → dist=%.2fm",
          wp_index_, heading_error * 180.0 / M_PI,
          is_reverse ? "REVERSING" : "DRIVING FORWARD", distance);
        break;
      }

      // Rotate in place
      cmd.linear.x  = 0.0;
      cmd.angular.z = std::clamp(2.0 * heading_error, -angular_speed_, angular_speed_);
      cmd_vel_pub_->publish(cmd);

      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
        "[WP%zu] %s Rotating  err=%.1f°  cmd.z=%.3f",
        wp_index_, is_reverse ? "⬅" : "➡",
        heading_error * 180.0 / M_PI, cmd.angular.z);
      break;
    }

    // ════════════════════════════════════════════════════════════
    // DRIVE_STRAIGHT — Forward mission
    //
    // SMOOTH CURVE-THROUGH for waypoints:
    //   When distance < lookahead_radius_ AND type == waypoint,
    //   advance to next WP immediately without stopping.
    //   The heading correction naturally curves the robot.
    //
    // STATION type: stop at exact xy_tolerance_.
    // ════════════════════════════════════════════════════════════
    case State::DRIVE_STRAIGHT:
    {
      // ── WAYPOINT look-ahead: smooth curve-through ──────────
      if (wp.type == "waypoint" && distance < lookahead_radius_) {
        // Advance to next WP WITHOUT stopping — robot curves through
        wp_index_++;
        if (wp_index_ < waypoints_.size()) {
          const Waypoint & nwp = waypoints_[wp_index_];
          RCLCPP_INFO(get_logger(),
            "[WP%zu] ⤵ Curve-through (%.2fm away) → next WP[%zu] [%s/%s]",
            wp_index_ - 1, distance, wp_index_,
            nwp.type.c_str(), nwp.mission.c_str());

          // If next waypoint is a different mission direction, must re-rotate
          if (nwp.mission != wp.mission) {
            state_ = State::ROTATE_TO_HEADING;
          }
          // Otherwise stay in DRIVE_STRAIGHT — heading correction steers us
          // No stopRobot()! Robot keeps moving → smooth curve!
        } else {
          state_ = State::CYCLE_DONE;
        }
        break;
      }

      // ── STATION type: stop at exact position ───────────────
      if (wp.type == "station" && distance < xy_tolerance_) {
        stopRobot(); state_ = State::ROTATE_TO_THETA;
        RCLCPP_INFO(get_logger(), "[WP%zu] ✓ Station reached — stopping & rotating", wp_index_);
        break;
      }

      if (obstacle_front_) {
        stopRobot();
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
          "[WP%zu] ⚠ Obstacle FRONT! Waiting...", wp_index_);
        break;
      }

      if (std::abs(heading_error) > 1.05) {
        stopRobot(); state_ = State::ROTATE_TO_HEADING;
        RCLCPP_WARN(get_logger(), "[WP%zu] Off course %.1f° — re-rotating",
          wp_index_, heading_error * 180.0 / M_PI);
        break;
      }

      // ── Check if lidar obstacle has fully ramped down to 0 ────
      if (lidar_obstacle_active_ && speed_ramp_factor_ <= 0.0) {
        stopRobot();
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
          "[WP%zu] ⚠ LIDAR obstacle — speed ramped to ZERO, waiting...", wp_index_);
        break;
      }

      // ── Check if emergency stop has fully ramped down to 0 ────
      if (emergency_active_ && emergency_ramp_factor_ <= 0.0) {
        stopRobot();
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000,
          "[WP%zu] ⛔ EMERGENCY STOP — robot STOPPED!", wp_index_);
        break;
      }

      // Use per-waypoint speed if specified, otherwise default linear_speed_
      double base_speed = (wp.speed > 0.0) ? wp.speed : linear_speed_;
      double speed = base_speed * speed_ramp_factor_ * emergency_ramp_factor_;
      // Station: slow down near goal for precise stop
      if (wp.type == "station" && distance < 0.30)
        speed = std::max(0.05, base_speed * (distance / 0.30)) * speed_ramp_factor_ * emergency_ramp_factor_;
      // Waypoint: maintain speed — we'll curve through

      cmd.linear.x  = speed;   // POSITIVE = forward
      cmd.angular.z = std::clamp(0.5 * heading_error, -drive_correction_, drive_correction_);
      cmd_vel_pub_->publish(cmd);

      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
        "[WP%zu] ▲ FORWARD  dist=%.2fm  spd=%.2f  lidar=%.0f%%  emg=%.0f%%  err=%.1f°",
        wp_index_, distance, speed,
        speed_ramp_factor_ * 100.0, emergency_ramp_factor_ * 100.0,
        heading_error * 180.0 / M_PI);
      break;
    }

    // ════════════════════════════════════════════════════════════
    // DRIVE_REVERSE — Reverse mission
    //
    // SMOOTH CURVE-THROUGH: same look-ahead as DRIVE_STRAIGHT
    //
    // KEY FIX for straight-line reverse:
    //   During reversing, we track angle_to_goal directly.
    //   The robot's back needs to stay pointed at the goal.
    //   back_error = angle_to_goal - (robot_yaw + PI)
    //              = angle_to_goal - robot_yaw - PI
    //   Correction = +back_error  (same sign as forward)
    //   because diff drive angular.z turns left/right the same
    //   regardless of whether robot moves forward or backward.
    // ════════════════════════════════════════════════════════════
    case State::DRIVE_REVERSE:
    {
      // ── WAYPOINT look-ahead: smooth curve-through ──────────
      if (wp.type == "waypoint" && distance < lookahead_radius_) {
        wp_index_++;
        if (wp_index_ < waypoints_.size()) {
          const Waypoint & nwp = waypoints_[wp_index_];
          RCLCPP_INFO(get_logger(),
            "[WP%zu] ⤵ Curve-through REV (%.2fm away) → next WP[%zu] [%s/%s]",
            wp_index_ - 1, distance, wp_index_,
            nwp.type.c_str(), nwp.mission.c_str());

          if (nwp.mission != wp.mission) {
            state_ = State::ROTATE_TO_HEADING;
          }
          // Otherwise stay in DRIVE_REVERSE — back_error correction steers us
        } else {
          state_ = State::CYCLE_DONE;
        }
        break;
      }

      // ── STATION type: stop at exact position ───────────────
      if (wp.type == "station" && distance < xy_tolerance_) {
        stopRobot(); state_ = State::ROTATE_TO_THETA;
        RCLCPP_INFO(get_logger(), "[WP%zu] ✓ Station reached (reverse) — stopping & rotating", wp_index_);
        break;
      }

      if (obstacle_rear_) {
        stopRobot();
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
          "[WP%zu] ⚠ Obstacle REAR! Waiting...", wp_index_);
        break;
      }

      // Compute how much robot's BACK has drifted from goal direction
      // back_facing = robot_yaw + PI  (direction robot's back points)
      // back_error  = angle_to_goal - back_facing
      double back_facing = normalizeAngle(robot_yaw_ + M_PI);
      double back_error  = normalizeAngle(angle_to_goal - back_facing);

      // Re-rotate if back drifted too much from goal
      if (std::abs(back_error) > 1.05) {
        stopRobot(); state_ = State::ROTATE_TO_HEADING;
        RCLCPP_WARN(get_logger(),
          "[WP%zu] Reverse off course %.1f° — re-rotating",
          wp_index_, back_error * 180.0 / M_PI);
        break;
      }

      // ── Check if lidar obstacle has fully ramped down to 0 ────
      if (lidar_obstacle_active_ && speed_ramp_factor_ <= 0.0) {
        stopRobot();
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
          "[WP%zu] ⚠ LIDAR obstacle — speed ramped to ZERO (REV), waiting...", wp_index_);
        break;
      }

      // ── Check if emergency stop has fully ramped down to 0 ────
      if (emergency_active_ && emergency_ramp_factor_ <= 0.0) {
        stopRobot();
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000,
          "[WP%zu] ⛔ EMERGENCY STOP — robot STOPPED! (REV)", wp_index_);
        break;
      }

      // Use per-waypoint speed if specified, otherwise default linear_speed_
      double base_speed = (wp.speed > 0.0) ? wp.speed : linear_speed_;
      double speed = base_speed * speed_ramp_factor_ * emergency_ramp_factor_;
      // Station: slow down near goal for precise stop
      if (wp.type == "station" && distance < 0.30)
        speed = std::max(0.05, base_speed * (distance / 0.30)) * speed_ramp_factor_ * emergency_ramp_factor_;
      // Waypoint: maintain speed — we'll curve through

      // linear.x  = NEGATIVE  (reverse direction)
      // angular.z = +back_error correction  (same sign as forward!)
      cmd.linear.x  = -speed;
      cmd.angular.z = std::clamp(0.5 * back_error, -drive_correction_, drive_correction_);
      cmd_vel_pub_->publish(cmd);

      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
        "[WP%zu] ▼ REVERSE  dist=%.2fm  spd=%.2f  lidar=%.0f%%  emg=%.0f%%  back_err=%.1f°",
        wp_index_, distance, speed,
        speed_ramp_factor_ * 100.0, emergency_ramp_factor_ * 100.0,
        back_error * 180.0 / M_PI);
      break;
    }

    // ════════════════════════════════════════════════════════════
    // ROTATE_TO_THETA — rotate to final heading of waypoint
    //
    // rotation field in waypoints.yaml:
    //   "auto" = shortest path (default)
    //   "cw"   = clockwise       (angular.z always NEGATIVE)
    //   "ccw"  = anticlockwise   (angular.z always POSITIVE)
    // ════════════════════════════════════════════════════════════
    case State::ROTATE_TO_THETA:
    {
      if (std::abs(yaw_error) < yaw_tolerance_) {
        stopRobot(); state_ = State::WAYPOINT_DONE;
        RCLCPP_INFO(get_logger(),
          "[WP%zu] ✅ COMPLETE  x=%.2f y=%.2f theta=%.1f° [%s/%s/rot=%s]",
          wp_index_, wp.x, wp.y, wp.theta * 180.0 / M_PI,
          wp.type.c_str(), wp.mission.c_str(), wp.rotation.c_str());
        break;
      }

      // ── Determine angular.z sign based on rotation field ─────
      double rot_cmd = 0.0;

      if (wp.rotation == "cw") {
        // Clockwise = always negative angular.z
        // Force yaw_error negative so robot always turns CW
        // If yaw_error > 0 (CCW is shorter), override and go CW anyway
        double cw_error = (yaw_error > 0)
          ? (yaw_error - 2.0 * M_PI)   // convert CCW error → CW (full circle)
          : yaw_error;
        rot_cmd = std::clamp(2.0 * cw_error, -angular_speed_, -0.05);

        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
          "[WP%zu] ↻ CW  theta=%.1f°  remaining=%.1f°  cmd=%.3f",
          wp_index_, wp.theta * 180.0 / M_PI,
          cw_error * 180.0 / M_PI, rot_cmd);

      } else if (wp.rotation == "ccw") {
        // Anticlockwise = always positive angular.z
        // If yaw_error < 0 (CW is shorter), override and go CCW anyway
        double ccw_error = (yaw_error < 0)
          ? (yaw_error + 2.0 * M_PI)   // convert CW error → CCW (full circle)
          : yaw_error;
        rot_cmd = std::clamp(2.0 * ccw_error, 0.05, angular_speed_);

        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
          "[WP%zu] ↺ CCW  theta=%.1f°  remaining=%.1f°  cmd=%.3f",
          wp_index_, wp.theta * 180.0 / M_PI,
          ccw_error * 180.0 / M_PI, rot_cmd);

      } else {
        // "auto" = shortest path (standard behavior)
        rot_cmd = std::clamp(2.0 * yaw_error, -angular_speed_, angular_speed_);

        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
          "[WP%zu] ↔ AUTO  theta=%.1f°  remaining=%.1f°  cmd=%.3f",
          wp_index_, wp.theta * 180.0 / M_PI,
          yaw_error * 180.0 / M_PI, rot_cmd);
      }

      cmd.linear.x  = 0.0;
      cmd.angular.z = rot_cmd;
      cmd_vel_pub_->publish(cmd);
      break;
    }

    // ════════════════════════════════════════════════════════════
    // WAYPOINT_DONE
    //   waypoint type → arrives here directly (no stop, no rotate)
    //   station type  → arrives here AFTER ROTATE_TO_THETA
    //   If PLC enabled + station → trigger PLC and enter WAIT_PLC
    // ════════════════════════════════════════════════════════════
    case State::WAYPOINT_DONE:
    {
      // If this is a "station" type AND plc_enabled → notify PLC bridge
      if (plc_enabled_ && wp.type == "station") {
        std::string station_name = !wp.name.empty()
          ? wp.name
          : (wp_index_ < waypoint_mission_names_.size() ? waypoint_mission_names_[wp_index_] : ("station_" + std::to_string(wp_index_)));

        RCLCPP_INFO(get_logger(),
          "[WP%zu] 🏭 STATION [%s] plc_feedback=%s → sending command to PLC",
          wp_index_, station_name.c_str(),
          wp.plc_feedback.empty() ? "none" : wp.plc_feedback.c_str());

        // Publish "STATION_A:I0.0" so plc_bridge knows which input to wait for
        // Format: "name:wait_input"  or just "name" if no wait_input
        std_msgs::msg::String smsg;
        smsg.data = wp.plc_feedback.empty()
          ? station_name
          : station_name + ":" + wp.plc_feedback;
        station_pub_->publish(smsg);

        // Enter WAIT_PLC state — robot stays stopped until /plc_done
        plc_done_        = false;
        waiting_for_plc_ = true;
        state_           = State::WAIT_PLC;
        stopRobot();
        break;
      }

      // No PLC — advance to next waypoint immediately
      wp_index_++;
      if (wp_index_ < waypoints_.size()) {
        state_ = State::ROTATE_TO_HEADING;
        const Waypoint & nwp = waypoints_[wp_index_];
        RCLCPP_INFO(get_logger(), "─────────────────────────────────────────");
        RCLCPP_INFO(get_logger(),
          "→ WP[%zu/%zu] [%s/%s]  x=%.2f y=%.2f theta=%.1f°",
          wp_index_, waypoints_.size() - 1,
          nwp.type.c_str(), nwp.mission.c_str(),
          nwp.x, nwp.y, nwp.theta * 180.0 / M_PI);
      } else {
        state_ = State::CYCLE_DONE;
      }
      break;
    }

    // ════════════════════════════════════════════════════════════
    // WAIT_PLC — robot stopped at station, waiting for PLC trigger
    //
    // Checks TWO sources:
    //   1. /plc_done topic (String) — from plc_station_monitor
    //   2. /plc_trigger topic (Bool) — continuous PLC bit value
    // Either one being TRUE will advance the robot.
    // ════════════════════════════════════════════════════════════
    case State::WAIT_PLC:
    {
      stopRobot();

      // Check EITHER /plc_done message OR /plc_trigger bit value
      if (plc_done_ || plc_trigger_value_) {
        // PLC action complete — advance to next waypoint
        RCLCPP_INFO(get_logger(),
          "[WP%zu] ✅ PLC trigger detected (done=%s, trigger=%s) → advancing",
          wp_index_,
          plc_done_ ? "TRUE" : "FALSE",
          plc_trigger_value_ ? "TRUE" : "FALSE");

        plc_done_          = false;
        waiting_for_plc_   = false;
        plc_trigger_value_ = false;  // reset so next station waits fresh

        wp_index_++;
        if (wp_index_ < waypoints_.size()) {
          state_ = State::ROTATE_TO_HEADING;
          const Waypoint & nwp = waypoints_[wp_index_];
          RCLCPP_INFO(get_logger(), "─────────────────────────────────────────");
          RCLCPP_INFO(get_logger(),
            "→ WP[%zu/%zu] [%s/%s]  x=%.2f y=%.2f theta=%.1f°",
            wp_index_, waypoints_.size() - 1,
            nwp.type.c_str(), nwp.mission.c_str(),
            nwp.x, nwp.y, nwp.theta * 180.0 / M_PI);
        } else {
          state_ = State::CYCLE_DONE;
        }
      } else {
        // Still waiting — log every 2 seconds
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
          "[WP%zu] ⏳ Waiting for PLC trigger (DB bit = FALSE)...", wp_index_);
      }
      break;
    }

    case State::IDLE:
    case State::ALL_DONE:
    case State::CYCLE_DONE:
    default:
      break;
  }
}

}  // namespace amr_pure_navigation

// ─── Global node pointer for signal handler ───────────────────────
static std::shared_ptr<amr_pure_navigation::WaypointNavigator> g_node;

static void signalHandler(int /*sig*/)
{
  // Publish zero velocity immediately on Ctrl+C
  if (g_node) {
    try {
      auto pub = g_node->create_publisher<geometry_msgs::msg::Twist>(
        "/diff_drive_controller/cmd_vel", 1);
      geometry_msgs::msg::Twist stop;
      pub->publish(stop);
      RCLCPP_WARN(g_node->get_logger(), "🛑 Ctrl+C — Robot STOPPED.");
    } catch (...) {}
  }
  rclcpp::shutdown();
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  g_node = std::make_shared<amr_pure_navigation::WaypointNavigator>();

  // Register signal handlers so Ctrl+C stops the robot
  signal(SIGINT,  signalHandler);
  signal(SIGTERM, signalHandler);

  rclcpp::spin(g_node);

  // Extra safety stop after spin exits
  if (g_node) {
    try {
      auto pub = g_node->create_publisher<geometry_msgs::msg::Twist>(
        "/diff_drive_controller/cmd_vel", 1);
      geometry_msgs::msg::Twist stop;
      pub->publish(stop);
    } catch (...) {}
  }

  g_node.reset();
  rclcpp::shutdown();
  return 0;
}








// // ================================================================
// //  waypoint_navigator.cpp  v6  —  Forward + Reverse Mission
// //
// //  mission: forward → rotate to face goal → drive forward → rotate to theta
// //  mission: reverse → rotate BACK to goal → drive REVERSE → rotate to theta
// //
// //  WAYPOINTS YAML EXAMPLE:
// //    waypoints:
// //      - x: -10.0
// //        y:  10.0
// //        theta: 0.0
// //        type: station
// //        mission: reverse    ← robot drives backward to this point
// //
// //      - x:  20.0
// //        y:  10.0
// //        theta: 0.0
// //        type: waypoint
// //        mission: forward    ← robot drives forward to this point
// //
// //  CYCLE CONTROL via /set_cycles topic:
// //    ros2 topic pub --once /set_cycles std_msgs/msg/Int32 "data: 1"
// //    ros2 topic pub --once /set_cycles std_msgs/msg/Int32 "data: -1"  # forever
// //    ros2 topic pub --once /set_cycles std_msgs/msg/Int32 "data: 0"   # stop
// //
// //  CHECK STATUS:
// //    ros2 topic echo /nav_status
// // ================================================================

// #include "amr_pure_navigation/waypoint_navigator.hpp"

// namespace amr_pure_navigation
// {

// // ─── Constructor ──────────────────────────────────────────────────
// WaypointNavigator::WaypointNavigator(const rclcpp::NodeOptions & options)
// : Node("waypoint_navigator", options)
// {
//   declare_parameter<std::string>("waypoints_file",     "");
//   declare_parameter<std::string>("odom_topic",         "/odometry/filtered");
//   declare_parameter<std::string>("cmd_vel_topic",      "/diff_drive_controller/cmd_vel");
//   declare_parameter<std::string>("amcl_topic",         "/amcl_pose");
//   declare_parameter<double>     ("linear_speed",       0.20);
//   declare_parameter<double>     ("angular_speed",      0.40);
//   declare_parameter<double>     ("drive_correction",   0.15);
//   declare_parameter<double>     ("xy_tolerance",       0.10);
//   declare_parameter<double>     ("yaw_tolerance",      0.08);
//   declare_parameter<double>     ("heading_tolerance",  0.10);
//   declare_parameter<double>     ("obstacle_stop_dist", 0.40);
//   declare_parameter<bool>       ("stop_on_obstacle",   true);
//   declare_parameter<bool>       ("use_amcl",           true);
//   declare_parameter<bool>       ("plc_enabled",        false);

//   get_parameter("waypoints_file",    waypoints_file_);
//   get_parameter("odom_topic",        odom_topic_);
//   get_parameter("cmd_vel_topic",     cmd_vel_topic_);
//   get_parameter("amcl_topic",        amcl_topic_);
//   get_parameter("linear_speed",      linear_speed_);
//   get_parameter("angular_speed",     angular_speed_);
//   get_parameter("drive_correction",  drive_correction_);
//   get_parameter("xy_tolerance",      xy_tolerance_);
//   get_parameter("yaw_tolerance",     yaw_tolerance_);
//   get_parameter("heading_tolerance", heading_tolerance_);
//   get_parameter("obstacle_stop_dist",obstacle_stop_dist_);
//   get_parameter("stop_on_obstacle",  stop_on_obstacle_);
//   get_parameter("use_amcl",          use_amcl_);
//   get_parameter("plc_enabled",       plc_enabled_);

//   if (!loadWaypoints(waypoints_file_)) {
//     RCLCPP_FATAL(get_logger(), "Cannot load waypoints: %s", waypoints_file_.c_str());
//     init_failed_ = true;
//     return;
//   }

//   RCLCPP_INFO(get_logger(), "=============================================");
//   RCLCPP_INFO(get_logger(), "  AMR Navigation v6 - Forward + Reverse");
//   RCLCPP_INFO(get_logger(), "  Waypoints    : %zu",    waypoints_.size());
//   RCLCPP_INFO(get_logger(), "  Odom topic   : %s",     odom_topic_.c_str());
//   RCLCPP_INFO(get_logger(), "  Mode         : %s",     use_amcl_ ? "AMCL map frame" : "Odom only");
//   RCLCPP_INFO(get_logger(), "  Linear speed : %.2f m/s",   linear_speed_);
//   RCLCPP_INFO(get_logger(), "  Angular speed: %.2f rad/s", angular_speed_);
//   RCLCPP_INFO(get_logger(), "=============================================");
//   RCLCPP_INFO(get_logger(), "  Waiting for /set_cycles command...");
//   RCLCPP_INFO(get_logger(), "  ros2 topic pub --once /set_cycles std_msgs/msg/Int32 \"data: 1\"");
//   RCLCPP_INFO(get_logger(), "=============================================");

//   cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);
//   status_pub_   = create_publisher<std_msgs::msg::String>("/nav_status",       10);
//   station_pub_  = create_publisher<std_msgs::msg::String>("/station_reached", 10);

//   // Subscribe to /plc_done (plc_bridge tells us when action is complete)
//   plc_done_sub_ = create_subscription<std_msgs::msg::String>(
//     "/plc_done", 10,
//     std::bind(&WaypointNavigator::plcDoneCallback, this, std::placeholders::_1));

//   cycles_sub_ = create_subscription<std_msgs::msg::Int32>(
//     "/set_cycles", 10,
//     std::bind(&WaypointNavigator::cyclesCallback, this, std::placeholders::_1));

//   if (use_amcl_) {
//     amcl_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
//       amcl_topic_, 10,
//       std::bind(&WaypointNavigator::amclCallback, this, std::placeholders::_1));
//   }

//   odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
//     odom_topic_, 10,
//     std::bind(&WaypointNavigator::odomCallback, this, std::placeholders::_1));

//   scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
//     "/scan", 10,
//     std::bind(&WaypointNavigator::scanCallback, this, std::placeholders::_1));

//   timer_ = create_wall_timer(
//     std::chrono::milliseconds(50),
//     std::bind(&WaypointNavigator::controlLoop, this));
// }

// // ─── PLC Done Callback ───────────────────────────────────────────
// // Called when plc_bridge finishes station action → robot continues
// void WaypointNavigator::plcDoneCallback(const std_msgs::msg::String::SharedPtr msg)
// {
//   RCLCPP_INFO(get_logger(), "✅ /plc_done received for station: [%s] → resuming", msg->data.c_str());
//   plc_done_      = true;
//   waiting_for_plc_ = false;
// }

// // ─── Cycles Callback ──────────────────────────────────────────────
// void WaypointNavigator::cyclesCallback(const std_msgs::msg::Int32::SharedPtr msg)
// {
//   int requested = msg->data;

//   if (requested == 0) {
//     stopRobot();
//     state_ = State::ALL_DONE;
//     max_cycles_ = 0; loop_forever_ = false;
//     RCLCPP_WARN(get_logger(), "🛑 STOP command! Robot halted.");
//     publishStatus("STOPPED by user");
//     return;
//   }

//   if (requested == -1) {
//     loop_forever_ = true; max_cycles_ = -1;
//     cycles_done_ = 0; wp_index_ = 0;
//     if (pose_received_) {
//       state_ = State::ROTATE_TO_HEADING;
//       RCLCPP_INFO(get_logger(), "🔁 LOOP FOREVER started!");
//     }
//     publishStatus("RUNNING - loop forever");
//     return;
//   }

//   if (requested > 0) {
//     // ── Robot is currently running — add more cycles on top ───
//     if (state_ != State::IDLE && state_ != State::ALL_DONE) {
//       max_cycles_ = cycles_done_ + requested;
//       loop_forever_ = false;
//       RCLCPP_INFO(get_logger(), "🔄 %d more cycle(s) added. Total=%d", requested, max_cycles_);
//       char buf[64]; snprintf(buf, sizeof(buf), "RUNNING - %d more added", requested);
//       publishStatus(buf);
//       return;
//     }

//     // ── Fresh start (IDLE or ALL_DONE — both reset fully) ─────
//     loop_forever_ = false;
//     max_cycles_   = requested;
//     cycles_done_  = 0;       // ← CRITICAL: reset cycle counter
//     wp_index_     = 0;       // ← CRITICAL: reset waypoint index

//     if (pose_received_) {
//       state_ = State::ROTATE_TO_HEADING;
//       RCLCPP_INFO(get_logger(), "🚀 Starting %d cycle(s)! (fresh start)", requested);
//     } else {
//       state_ = State::IDLE;  // stay IDLE until pose arrives
//       RCLCPP_INFO(get_logger(), "✅ %d cycle(s) queued. Waiting for pose...", requested);
//     }
//     char buf[64]; snprintf(buf, sizeof(buf), "STARTING %d cycles", requested);
//     publishStatus(buf);
//   }
// }

// // ─── AMCL Callback ────────────────────────────────────────────────
// void WaypointNavigator::amclCallback(
//   const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
// {
//   if (init_failed_) return;
//   robot_x_   = msg->pose.pose.position.x;
//   robot_y_   = msg->pose.pose.position.y;
//   robot_yaw_ = quaternionToYaw(msg->pose.pose.orientation);

//   if (!pose_received_) {
//     pose_received_ = true;
//     RCLCPP_INFO(get_logger(), "✅ AMCL active! x=%.2f y=%.2f yaw=%.1f°",
//       robot_x_, robot_y_, robot_yaw_ * 180.0 / M_PI);
//     if (max_cycles_ != 0 && state_ == State::IDLE) {
//       state_ = State::ROTATE_TO_HEADING;
//       RCLCPP_INFO(get_logger(), "🚀 Pose received — starting!");
//     }
//   }
// }

// // ─── Odom Callback ────────────────────────────────────────────────
// void WaypointNavigator::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
// {
//   if (init_failed_) return;
//   if (use_amcl_ && pose_received_) return;

//   robot_x_   = msg->pose.pose.position.x;
//   robot_y_   = msg->pose.pose.position.y;
//   robot_yaw_ = quaternionToYaw(msg->pose.pose.orientation);

//   if (!pose_received_) {
//     pose_received_ = true;
//     RCLCPP_INFO(get_logger(), "✅ Odom active! x=%.2f y=%.2f", robot_x_, robot_y_);
//     if (max_cycles_ != 0 && state_ == State::IDLE)
//       state_ = State::ROTATE_TO_HEADING;
//   }
// }

// // ─── Scan Callback ────────────────────────────────────────────────
// void WaypointNavigator::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
// {
//   if (!stop_on_obstacle_) return;
//   obstacle_ahead_ = false;

//   // For REVERSE: check rear sector (±30° from back)
//   // For FORWARD: check front sector (±30° from front)
//   // We store both
//   obstacle_front_ = false;
//   obstacle_rear_  = false;

//   int n = static_cast<int>(msg->ranges.size());
//   for (int i = 0; i < n; ++i) {
//     float angle = msg->angle_min + i * msg->angle_increment;
//     float r     = msg->ranges[i];
//     if (!std::isfinite(r) || r < 0.01f || r >= static_cast<float>(obstacle_stop_dist_)) continue;

//     // Front sector ±30°
//     if (angle >= -0.524f && angle <= 0.524f)
//       obstacle_front_ = true;

//     // Rear sector ±30° (around ±PI)
//     if (std::abs(std::abs(angle) - M_PI) < 0.524f)
//       obstacle_rear_ = true;
//   }
// }

// // ─── Helpers ──────────────────────────────────────────────────────
// double WaypointNavigator::quaternionToYaw(const geometry_msgs::msg::Quaternion & q)
// {
//   return std::atan2(
//     2.0 * (q.w * q.z + q.x * q.y),
//     1.0 - 2.0 * (q.y * q.y + q.z * q.z));
// }

// double WaypointNavigator::normalizeAngle(double a)
// {
//   while (a >  M_PI) a -= 2.0 * M_PI;
//   while (a < -M_PI) a += 2.0 * M_PI;
//   return a;
// }

// void WaypointNavigator::stopRobot()
// {
//   cmd_vel_pub_->publish(geometry_msgs::msg::Twist());
// }

// void WaypointNavigator::publishStatus(const std::string & text)
// {
//   std_msgs::msg::String msg;
//   msg.data = text;
//   status_pub_->publish(msg);
// }

// bool WaypointNavigator::isOnlyRotation(const Waypoint & prev, const Waypoint & curr)
// {
//   return (std::abs(curr.x - prev.x) < xy_tolerance_ &&
//           std::abs(curr.y - prev.y) < xy_tolerance_);
// }

// const char * WaypointNavigator::stateStr()
// {
//   switch (state_) {
//     case State::IDLE:              return "IDLE";
//     case State::ROTATE_TO_HEADING: return "ROT_HEADING";
//     case State::DRIVE_STRAIGHT:    return "DRIVE_FWD";
//     case State::DRIVE_REVERSE:     return "DRIVE_REV";
//     case State::ROTATE_TO_THETA:   return "ROT_THETA";
//     case State::WAIT_PLC:          return "WAIT_PLC";
//     case State::WAYPOINT_DONE:     return "WP_DONE";
//     case State::CYCLE_DONE:        return "CYCLE_DONE";
//     case State::ALL_DONE:          return "ALL_DONE";
//     default:                       return "UNKNOWN";
//   }
// }

// // ─── Load Waypoints ───────────────────────────────────────────────
// bool WaypointNavigator::loadWaypoints(const std::string & path)
// {
//   if (path.empty()) { RCLCPP_ERROR(get_logger(), "waypoints_file is empty!"); return false; }
//   std::ifstream f(path);
//   if (!f.good()) { RCLCPP_ERROR(get_logger(), "Cannot open: %s", path.c_str()); return false; }
//   f.seekg(0, std::ios::end);
//   if (f.tellg() == 0) { RCLCPP_ERROR(get_logger(), "File empty: %s", path.c_str()); return false; }
//   f.close();

//   try {
//     YAML::Node root = YAML::LoadFile(path);
//     YAML::Node wps  = root["waypoints"];
//     if (!wps || !wps.IsSequence()) {
//       RCLCPP_ERROR(get_logger(), "No 'waypoints' list in YAML"); return false;
//     }
//     for (size_t i = 0; i < wps.size(); ++i) {
//       Waypoint w;
//       w.x       = wps[i]["x"].as<double>();
//       w.y       = wps[i]["y"].as<double>();
//       w.theta   = wps[i]["theta"].as<double>();
//       w.type     = wps[i]["type"]     ? wps[i]["type"].as<std::string>()     : "waypoint";
//       w.mission  = wps[i]["mission"]  ? wps[i]["mission"].as<std::string>()  : "forward";
//       w.rotation = wps[i]["rotation"] ? wps[i]["rotation"].as<std::string>() : "auto";
//       waypoints_.push_back(w);

//       RCLCPP_INFO(get_logger(),
//         "  WP[%02zu] %-10s %-8s rot=%-5s x=%.2f y=%.2f theta=%.1f°",
//         i, w.type.c_str(), w.mission.c_str(), w.rotation.c_str(),
//         w.x, w.y, w.theta * 180.0 / M_PI);
//     }
//   } catch (const std::exception & e) {
//     RCLCPP_ERROR(get_logger(), "YAML error: %s", e.what()); return false;
//   }
//   return !waypoints_.empty();
// }

// // ─── Main Control Loop  20 Hz ─────────────────────────────────────
// void WaypointNavigator::controlLoop()
// {
//   if (init_failed_ || !pose_received_) return;
//   if (state_ == State::IDLE)     { stopRobot(); return; }
//   if (state_ == State::ALL_DONE) {
//     stopRobot();
//     // Stay here — cyclesCallback will reset state when user sends new command
//     return;
//   }

//   if (wp_index_ >= waypoints_.size()) {
//     state_ = State::CYCLE_DONE; return;
//   }

//   // ── CYCLE DONE ───────────────────────────────────────────────
//   if (state_ == State::CYCLE_DONE) {
//     cycles_done_++;
//     if (loop_forever_ || cycles_done_ < max_cycles_) {
//       wp_index_ = 0;
//       state_    = State::ROTATE_TO_HEADING;
//       RCLCPP_INFO(get_logger(), loop_forever_ ?
//         "🔁 Cycle %d done — looping!" : "🔄 Cycle %d/%d done — next cycle!",
//         cycles_done_, max_cycles_);
//     } else {
//       state_ = State::ALL_DONE; stopRobot();
//       RCLCPP_INFO(get_logger(), "✅ ALL %d CYCLE(S) DONE!", cycles_done_);
//     }
//     return;
//   }

//   const Waypoint & wp = waypoints_[wp_index_];
//   const bool is_reverse = (wp.mission == "reverse");

//   double dx            = wp.x - robot_x_;
//   double dy            = wp.y - robot_y_;
//   double distance      = std::hypot(dx, dy);
//   double angle_to_goal = std::atan2(dy, dx);

//   // For reverse: robot needs its BACK to face goal
//   // So target heading = angle_to_goal + PI
//   double heading_target = is_reverse
//     ? normalizeAngle(angle_to_goal + M_PI)
//     : angle_to_goal;

//   double heading_error = normalizeAngle(heading_target - robot_yaw_);
//   double yaw_error     = normalizeAngle(wp.theta - robot_yaw_);

//   geometry_msgs::msg::Twist cmd;

//   // 1 second log
//   if (++log_counter_ >= 20) {
//     log_counter_ = 0;
//     RCLCPP_INFO(get_logger(),
//       "[%s | %s | WP%zu] pos(%.2f,%.2f,%.1f°)→(%.2f,%.2f,%.1f°) dist=%.2fm",
//       stateStr(), is_reverse ? "REV" : "FWD", wp_index_,
//       robot_x_, robot_y_, robot_yaw_ * 180.0 / M_PI,
//       wp.x, wp.y, wp.theta * 180.0 / M_PI, distance);
//   }

//   switch (state_)
//   {
//     // ════════════════════════════════════════════════════════════
//     // ROTATE_TO_HEADING
//     // Forward:  rotate until robot FACES the goal
//     // Reverse:  rotate until robot BACK is toward the goal
//     // ════════════════════════════════════════════════════════════
//     case State::ROTATE_TO_HEADING:
//     {
//       Waypoint prev = (wp_index_ > 0) ?
//         waypoints_[wp_index_ - 1] :
//         Waypoint{robot_x_, robot_y_, robot_yaw_};

//       if (isOnlyRotation(prev, wp)) {
//         state_ = State::ROTATE_TO_THETA;
//         RCLCPP_INFO(get_logger(), "[WP%zu] Pure rotation → %.1f°",
//           wp_index_, wp.theta * 180.0 / M_PI);
//         break;
//       }

//       if (distance < xy_tolerance_) {
//         stopRobot(); state_ = State::ROTATE_TO_THETA;
//         break;
//       }

//       if (std::abs(heading_error) < heading_tolerance_) {
//         stopRobot();
//         // Go to correct drive state based on mission
//         state_ = is_reverse ? State::DRIVE_REVERSE : State::DRIVE_STRAIGHT;
//         RCLCPP_INFO(get_logger(),
//           "[WP%zu] ✓ Heading aligned (%.1f°). %s → dist=%.2fm",
//           wp_index_, heading_error * 180.0 / M_PI,
//           is_reverse ? "REVERSING" : "DRIVING FORWARD", distance);
//         break;
//       }

//       // Rotate in place
//       cmd.linear.x  = 0.0;
//       cmd.angular.z = std::clamp(2.0 * heading_error, -angular_speed_, angular_speed_);
//       cmd_vel_pub_->publish(cmd);

//       RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
//         "[WP%zu] %s Rotating  err=%.1f°  cmd.z=%.3f",
//         wp_index_, is_reverse ? "⬅" : "➡",
//         heading_error * 180.0 / M_PI, cmd.angular.z);
//       break;
//     }

//     // ════════════════════════════════════════════════════════════
//     // DRIVE_STRAIGHT — Forward mission
//     // linear.x = +speed  (forward)
//     // angular.z = gentle correction
//     // ════════════════════════════════════════════════════════════
//     case State::DRIVE_STRAIGHT:
//     {
//       if (distance < xy_tolerance_) {
//         stopRobot(); state_ = State::ROTATE_TO_THETA;
//         RCLCPP_INFO(get_logger(), "[WP%zu] ✓ Forward goal reached!", wp_index_);
//         break;
//       }

//       if (obstacle_front_) {
//         stopRobot();
//         RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
//           "[WP%zu] ⚠ Obstacle FRONT! Waiting...", wp_index_);
//         break;
//       }

//       if (std::abs(heading_error) > 1.05) {
//         stopRobot(); state_ = State::ROTATE_TO_HEADING;
//         RCLCPP_WARN(get_logger(), "[WP%zu] Off course %.1f° — re-rotating",
//           wp_index_, heading_error * 180.0 / M_PI);
//         break;
//       }

//       double speed = linear_speed_;
//       if (distance < 0.30)
//         speed = std::max(0.05, linear_speed_ * (distance / 0.30));

//       cmd.linear.x  = speed;   // POSITIVE = forward
//       cmd.angular.z = std::clamp(0.5 * heading_error, -drive_correction_, drive_correction_);
//       cmd_vel_pub_->publish(cmd);

//       RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
//         "[WP%zu] ▲ FORWARD  dist=%.2fm  spd=%.2f  err=%.1f°",
//         wp_index_, distance, speed, heading_error * 180.0 / M_PI);
//       break;
//     }

//     // ════════════════════════════════════════════════════════════
//     // DRIVE_REVERSE — Reverse mission
//     //
//     // KEY FIX for straight-line reverse:
//     //   During reversing, we track angle_to_goal directly.
//     //   The robot's back needs to stay pointed at the goal.
//     //   back_error = angle_to_goal - (robot_yaw + PI)
//     //              = angle_to_goal - robot_yaw - PI
//     //   Correction = +back_error  (same sign as forward)
//     //   because diff drive angular.z turns left/right the same
//     //   regardless of whether robot moves forward or backward.
//     // ════════════════════════════════════════════════════════════
//     case State::DRIVE_REVERSE:
//     {
//       if (distance < xy_tolerance_) {
//         stopRobot(); state_ = State::ROTATE_TO_THETA;
//         RCLCPP_INFO(get_logger(), "[WP%zu] ✓ Reverse goal reached!", wp_index_);
//         break;
//       }

//       if (obstacle_rear_) {
//         stopRobot();
//         RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
//           "[WP%zu] ⚠ Obstacle REAR! Waiting...", wp_index_);
//         break;
//       }

//       // Compute how much robot's BACK has drifted from goal direction
//       // back_facing = robot_yaw + PI  (direction robot's back points)
//       // back_error  = angle_to_goal - back_facing
//       double back_facing = normalizeAngle(robot_yaw_ + M_PI);
//       double back_error  = normalizeAngle(angle_to_goal - back_facing);

//       // Re-rotate if back drifted too much from goal
//       if (std::abs(back_error) > 1.05) {
//         stopRobot(); state_ = State::ROTATE_TO_HEADING;
//         RCLCPP_WARN(get_logger(),
//           "[WP%zu] Reverse off course %.1f° — re-rotating",
//           wp_index_, back_error * 180.0 / M_PI);
//         break;
//       }

//       double speed = linear_speed_;
//       if (distance < 0.30)
//         speed = std::max(0.05, linear_speed_ * (distance / 0.30));

//       // linear.x  = NEGATIVE  (reverse direction)
//       // angular.z = +back_error correction  (same sign as forward!)
//       //             When back drifts left  → back_error > 0 → turn left → corrects
//       //             When back drifts right → back_error < 0 → turn right → corrects
//       cmd.linear.x  = -speed;
//       cmd.angular.z = std::clamp(0.5 * back_error, -drive_correction_, drive_correction_);
//       cmd_vel_pub_->publish(cmd);

//       RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
//         "[WP%zu] ▼ REVERSE  dist=%.2fm  spd=%.2f  back_err=%.1f°",
//         wp_index_, distance, speed, back_error * 180.0 / M_PI);
//       break;
//     }

//     // ════════════════════════════════════════════════════════════
//     // ROTATE_TO_THETA — rotate to final heading of waypoint
//     //
//     // rotation field in waypoints.yaml:
//     //   "auto" = shortest path (default)
//     //   "cw"   = clockwise       (angular.z always NEGATIVE)
//     //   "ccw"  = anticlockwise   (angular.z always POSITIVE)
//     // ════════════════════════════════════════════════════════════
//     case State::ROTATE_TO_THETA:
//     {
//       if (std::abs(yaw_error) < yaw_tolerance_) {
//         stopRobot(); state_ = State::WAYPOINT_DONE;
//         RCLCPP_INFO(get_logger(),
//           "[WP%zu] ✅ COMPLETE  x=%.2f y=%.2f theta=%.1f° [%s/%s/rot=%s]",
//           wp_index_, wp.x, wp.y, wp.theta * 180.0 / M_PI,
//           wp.type.c_str(), wp.mission.c_str(), wp.rotation.c_str());
//         break;
//       }

//       // ── Determine angular.z sign based on rotation field ─────
//       double rot_cmd = 0.0;

//       if (wp.rotation == "cw") {
//         // Clockwise = always negative angular.z
//         // Force yaw_error negative so robot always turns CW
//         // If yaw_error > 0 (CCW is shorter), override and go CW anyway
//         double cw_error = (yaw_error > 0)
//           ? (yaw_error - 2.0 * M_PI)   // convert CCW error → CW (full circle)
//           : yaw_error;
//         rot_cmd = std::clamp(2.0 * cw_error, -angular_speed_, -0.05);

//         RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
//           "[WP%zu] ↻ CW  theta=%.1f°  remaining=%.1f°  cmd=%.3f",
//           wp_index_, wp.theta * 180.0 / M_PI,
//           cw_error * 180.0 / M_PI, rot_cmd);

//       } else if (wp.rotation == "ccw") {
//         // Anticlockwise = always positive angular.z
//         // If yaw_error < 0 (CW is shorter), override and go CCW anyway
//         double ccw_error = (yaw_error < 0)
//           ? (yaw_error + 2.0 * M_PI)   // convert CW error → CCW (full circle)
//           : yaw_error;
//         rot_cmd = std::clamp(2.0 * ccw_error, 0.05, angular_speed_);

//         RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
//           "[WP%zu] ↺ CCW  theta=%.1f°  remaining=%.1f°  cmd=%.3f",
//           wp_index_, wp.theta * 180.0 / M_PI,
//           ccw_error * 180.0 / M_PI, rot_cmd);

//       } else {
//         // "auto" = shortest path (standard behavior)
//         rot_cmd = std::clamp(2.0 * yaw_error, -angular_speed_, angular_speed_);

//         RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
//           "[WP%zu] ↔ AUTO  theta=%.1f°  remaining=%.1f°  cmd=%.3f",
//           wp_index_, wp.theta * 180.0 / M_PI,
//           yaw_error * 180.0 / M_PI, rot_cmd);
//       }

//       cmd.linear.x  = 0.0;
//       cmd.angular.z = rot_cmd;
//       cmd_vel_pub_->publish(cmd);
//       break;
//     }

//     // ════════════════════════════════════════════════════════════
//     // WAYPOINT_DONE — check if station type, trigger PLC if needed
//     // ════════════════════════════════════════════════════════════
//     case State::WAYPOINT_DONE:
//     {
//       // If this is a "station" type AND plc_enabled → notify PLC bridge
//       if (plc_enabled_ && wp.type == "station") {
//         std::string station_name = wp.name.empty()
//           ? ("station_" + std::to_string(wp_index_))
//           : wp.name;

//         RCLCPP_INFO(get_logger(),
//           "[WP%zu] 🏭 STATION reached: [%s] → publishing /station_reached",
//           wp_index_, station_name.c_str());

//         std_msgs::msg::String smsg;
//         smsg.data = station_name;
//         station_pub_->publish(smsg);

//         // Enter WAIT_PLC state — robot stays stopped until /plc_done
//         plc_done_        = false;
//         waiting_for_plc_ = true;
//         state_           = State::WAIT_PLC;
//         stopRobot();
//         break;
//       }

//       // No PLC — advance to next waypoint immediately
//       wp_index_++;
//       if (wp_index_ < waypoints_.size()) {
//         state_ = State::ROTATE_TO_HEADING;
//         const Waypoint & nwp = waypoints_[wp_index_];
//         RCLCPP_INFO(get_logger(), "─────────────────────────────────────────");
//         RCLCPP_INFO(get_logger(),
//           "→ WP[%zu/%zu] [%s/%s]  x=%.2f y=%.2f theta=%.1f°",
//           wp_index_, waypoints_.size() - 1,
//           nwp.type.c_str(), nwp.mission.c_str(),
//           nwp.x, nwp.y, nwp.theta * 180.0 / M_PI);
//       } else {
//         state_ = State::CYCLE_DONE;
//       }
//       break;
//     }

//     // ════════════════════════════════════════════════════════════
//     // WAIT_PLC — robot stopped at station, waiting for /plc_done
//     // ════════════════════════════════════════════════════════════
//     case State::WAIT_PLC:
//     {
//       stopRobot();

//       if (plc_done_) {
//         // PLC action complete — advance to next waypoint
//         plc_done_        = false;
//         waiting_for_plc_ = false;

//         wp_index_++;
//         if (wp_index_ < waypoints_.size()) {
//           state_ = State::ROTATE_TO_HEADING;
//           const Waypoint & nwp = waypoints_[wp_index_];
//           RCLCPP_INFO(get_logger(), "─────────────────────────────────────────");
//           RCLCPP_INFO(get_logger(),
//             "→ WP[%zu/%zu] [%s/%s]  x=%.2f y=%.2f theta=%.1f°",
//             wp_index_, waypoints_.size() - 1,
//             nwp.type.c_str(), nwp.mission.c_str(),
//             nwp.x, nwp.y, nwp.theta * 180.0 / M_PI);
//         } else {
//           state_ = State::CYCLE_DONE;
//         }
//       } else {
//         // Still waiting — log every second
//         RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
//           "[WP%zu] ⏳ Waiting for PLC to complete station action...", wp_index_);
//       }
//       break;
//     }

//     case State::IDLE:
//     case State::ALL_DONE:
//     case State::CYCLE_DONE:
//     default:
//       break;
//   }
// }

// }  // namespace amr_pure_navigation

// // ─── Global node pointer for signal handler ───────────────────────
// static std::shared_ptr<amr_pure_navigation::WaypointNavigator> g_node;

// static void signalHandler(int /*sig*/)
// {
//   // Publish zero velocity immediately on Ctrl+C
//   if (g_node) {
//     try {
//       auto pub = g_node->create_publisher<geometry_msgs::msg::Twist>(
//         "/diff_drive_controller/cmd_vel", 1);
//       geometry_msgs::msg::Twist stop;
//       pub->publish(stop);
//       RCLCPP_WARN(g_node->get_logger(), "🛑 Ctrl+C — Robot STOPPED.");
//     } catch (...) {}
//   }
//   rclcpp::shutdown();
// }

// int main(int argc, char ** argv)
// {
//   rclcpp::init(argc, argv);
//   g_node = std::make_shared<amr_pure_navigation::WaypointNavigator>();

//   // Register signal handlers so Ctrl+C stops the robot
//   signal(SIGINT,  signalHandler);
//   signal(SIGTERM, signalHandler);

//   rclcpp::spin(g_node);

//   // Extra safety stop after spin exits
//   if (g_node) {
//     try {
//       auto pub = g_node->create_publisher<geometry_msgs::msg::Twist>(
//         "/diff_drive_controller/cmd_vel", 1);
//       geometry_msgs::msg::Twist stop;
//       pub->publish(stop);
//     } catch (...) {}
//   }

//   g_node.reset();
//   rclcpp::shutdown();
//   return 0;
// }




// // forword and reverse movement

// // ================================================================
// //  waypoint_navigator.cpp  v6  —  Forward + Reverse Mission
// //
// //  mission: forward → rotate to face goal → drive forward → rotate to theta
// //  mission: reverse → rotate BACK to goal → drive REVERSE → rotate to theta
// //
// //  WAYPOINTS YAML EXAMPLE:
// //    waypoints:
// //      - x: -10.0
// //        y:  10.0
// //        theta: 0.0
// //        type: station
// //        mission: reverse    ← robot drives backward to this point
// //
// //      - x:  20.0
// //        y:  10.0
// //        theta: 0.0
// //        type: waypoint
// //        mission: forward    ← robot drives forward to this point
// //
// //  CYCLE CONTROL via /set_cycles topic:
// //    ros2 topic pub --once /set_cycles std_msgs/msg/Int32 "data: 1"
// //    ros2 topic pub --once /set_cycles std_msgs/msg/Int32 "data: -1"  # forever
// //    ros2 topic pub --once /set_cycles std_msgs/msg/Int32 "data: 0"   # stop
// //
// //  CHECK STATUS:
// //    ros2 topic echo /nav_status
// // ================================================================

// #include "amr_pure_navigation/waypoint_navigator.hpp"

// namespace amr_pure_navigation
// {

// // ─── Constructor ──────────────────────────────────────────────────
// WaypointNavigator::WaypointNavigator(const rclcpp::NodeOptions & options)
// : Node("waypoint_navigator", options)
// {
//   declare_parameter<std::string>("waypoints_file",     "");
//   declare_parameter<std::string>("odom_topic",         "/odometry/filtered");
//   declare_parameter<std::string>("cmd_vel_topic",      "/diff_drive_controller/cmd_vel");
//   declare_parameter<std::string>("amcl_topic",         "/amcl_pose");
//   declare_parameter<double>     ("linear_speed",       0.20);
//   declare_parameter<double>     ("angular_speed",      0.40);
//   declare_parameter<double>     ("drive_correction",   0.15);
//   declare_parameter<double>     ("xy_tolerance",       0.10);
//   declare_parameter<double>     ("yaw_tolerance",      0.08);
//   declare_parameter<double>     ("heading_tolerance",  0.10);
//   declare_parameter<double>     ("obstacle_stop_dist", 0.40);
//   declare_parameter<bool>       ("stop_on_obstacle",   true);
//   declare_parameter<bool>       ("use_amcl",           true);

//   get_parameter("waypoints_file",    waypoints_file_);
//   get_parameter("odom_topic",        odom_topic_);
//   get_parameter("cmd_vel_topic",     cmd_vel_topic_);
//   get_parameter("amcl_topic",        amcl_topic_);
//   get_parameter("linear_speed",      linear_speed_);
//   get_parameter("angular_speed",     angular_speed_);
//   get_parameter("drive_correction",  drive_correction_);
//   get_parameter("xy_tolerance",      xy_tolerance_);
//   get_parameter("yaw_tolerance",     yaw_tolerance_);
//   get_parameter("heading_tolerance", heading_tolerance_);
//   get_parameter("obstacle_stop_dist",obstacle_stop_dist_);
//   get_parameter("stop_on_obstacle",  stop_on_obstacle_);
//   get_parameter("use_amcl",          use_amcl_);

//   if (!loadWaypoints(waypoints_file_)) {
//     RCLCPP_FATAL(get_logger(), "Cannot load waypoints: %s", waypoints_file_.c_str());
//     init_failed_ = true;
//     return;
//   }

//   RCLCPP_INFO(get_logger(), "=============================================");
//   RCLCPP_INFO(get_logger(), "  AMR Navigation v6 - Forward + Reverse");
//   RCLCPP_INFO(get_logger(), "  Waypoints    : %zu",    waypoints_.size());
//   RCLCPP_INFO(get_logger(), "  Odom topic   : %s",     odom_topic_.c_str());
//   RCLCPP_INFO(get_logger(), "  Mode         : %s",     use_amcl_ ? "AMCL map frame" : "Odom only");
//   RCLCPP_INFO(get_logger(), "  Linear speed : %.2f m/s",   linear_speed_);
//   RCLCPP_INFO(get_logger(), "  Angular speed: %.2f rad/s", angular_speed_);
//   RCLCPP_INFO(get_logger(), "=============================================");
//   RCLCPP_INFO(get_logger(), "  Waiting for /set_cycles command...");
//   RCLCPP_INFO(get_logger(), "  ros2 topic pub --once /set_cycles std_msgs/msg/Int32 \"data: 1\"");
//   RCLCPP_INFO(get_logger(), "=============================================");

//   cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);
//   status_pub_  = create_publisher<std_msgs::msg::String>("/nav_status", 10);

//   cycles_sub_ = create_subscription<std_msgs::msg::Int32>(
//     "/set_cycles", 10,
//     std::bind(&WaypointNavigator::cyclesCallback, this, std::placeholders::_1));

//   if (use_amcl_) {
//     amcl_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
//       amcl_topic_, 10,
//       std::bind(&WaypointNavigator::amclCallback, this, std::placeholders::_1));
//   }

//   odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
//     odom_topic_, 10,
//     std::bind(&WaypointNavigator::odomCallback, this, std::placeholders::_1));

//   scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
//     "/scan", 10,
//     std::bind(&WaypointNavigator::scanCallback, this, std::placeholders::_1));

//   timer_ = create_wall_timer(
//     std::chrono::milliseconds(50),
//     std::bind(&WaypointNavigator::controlLoop, this));
// }

// // ─── Cycles Callback ──────────────────────────────────────────────
// void WaypointNavigator::cyclesCallback(const std_msgs::msg::Int32::SharedPtr msg)
// {
//   int requested = msg->data;

//   if (requested == 0) {
//     stopRobot();
//     state_ = State::ALL_DONE;
//     max_cycles_ = 0; loop_forever_ = false;
//     RCLCPP_WARN(get_logger(), "🛑 STOP command! Robot halted.");
//     publishStatus("STOPPED by user");
//     return;
//   }

//   if (requested == -1) {
//     loop_forever_ = true; max_cycles_ = -1;
//     cycles_done_ = 0; wp_index_ = 0;
//     if (pose_received_) {
//       state_ = State::ROTATE_TO_HEADING;
//       RCLCPP_INFO(get_logger(), "🔁 LOOP FOREVER started!");
//     }
//     publishStatus("RUNNING - loop forever");
//     return;
//   }

//   if (requested > 0) {
//     // ── Robot is currently running — add more cycles on top ───
//     if (state_ != State::IDLE && state_ != State::ALL_DONE) {
//       max_cycles_ = cycles_done_ + requested;
//       loop_forever_ = false;
//       RCLCPP_INFO(get_logger(), "🔄 %d more cycle(s) added. Total=%d", requested, max_cycles_);
//       char buf[64]; snprintf(buf, sizeof(buf), "RUNNING - %d more added", requested);
//       publishStatus(buf);
//       return;
//     }

//     // ── Fresh start (IDLE or ALL_DONE — both reset fully) ─────
//     loop_forever_ = false;
//     max_cycles_   = requested;
//     cycles_done_  = 0;       // ← CRITICAL: reset cycle counter
//     wp_index_     = 0;       // ← CRITICAL: reset waypoint index

//     if (pose_received_) {
//       state_ = State::ROTATE_TO_HEADING;
//       RCLCPP_INFO(get_logger(), "🚀 Starting %d cycle(s)! (fresh start)", requested);
//     } else {
//       state_ = State::IDLE;  // stay IDLE until pose arrives
//       RCLCPP_INFO(get_logger(), "✅ %d cycle(s) queued. Waiting for pose...", requested);
//     }
//     char buf[64]; snprintf(buf, sizeof(buf), "STARTING %d cycles", requested);
//     publishStatus(buf);
//   }
// }

// // ─── AMCL Callback ────────────────────────────────────────────────
// void WaypointNavigator::amclCallback(
//   const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
// {
//   if (init_failed_) return;
//   robot_x_   = msg->pose.pose.position.x;
//   robot_y_   = msg->pose.pose.position.y;
//   robot_yaw_ = quaternionToYaw(msg->pose.pose.orientation);

//   if (!pose_received_) {
//     pose_received_ = true;
//     RCLCPP_INFO(get_logger(), "✅ AMCL active! x=%.2f y=%.2f yaw=%.1f°",
//       robot_x_, robot_y_, robot_yaw_ * 180.0 / M_PI);
//     if (max_cycles_ != 0 && state_ == State::IDLE) {
//       state_ = State::ROTATE_TO_HEADING;
//       RCLCPP_INFO(get_logger(), "🚀 Pose received — starting!");
//     }
//   }
// }

// // ─── Odom Callback ────────────────────────────────────────────────
// void WaypointNavigator::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
// {
//   if (init_failed_) return;
//   if (use_amcl_ && pose_received_) return;

//   robot_x_   = msg->pose.pose.position.x;
//   robot_y_   = msg->pose.pose.position.y;
//   robot_yaw_ = quaternionToYaw(msg->pose.pose.orientation);

//   if (!pose_received_) {
//     pose_received_ = true;
//     RCLCPP_INFO(get_logger(), "✅ Odom active! x=%.2f y=%.2f", robot_x_, robot_y_);
//     if (max_cycles_ != 0 && state_ == State::IDLE)
//       state_ = State::ROTATE_TO_HEADING;
//   }
// }

// // ─── Scan Callback ────────────────────────────────────────────────
// void WaypointNavigator::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
// {
//   if (!stop_on_obstacle_) return;
//   obstacle_ahead_ = false;

//   // For REVERSE: check rear sector (±30° from back)
//   // For FORWARD: check front sector (±30° from front)
//   // We store both
//   obstacle_front_ = false;
//   obstacle_rear_  = false;

//   int n = static_cast<int>(msg->ranges.size());
//   for (int i = 0; i < n; ++i) {
//     float angle = msg->angle_min + i * msg->angle_increment;
//     float r     = msg->ranges[i];
//     if (!std::isfinite(r) || r < 0.01f || r >= static_cast<float>(obstacle_stop_dist_)) continue;

//     // Front sector ±30°
//     if (angle >= -0.524f && angle <= 0.524f)
//       obstacle_front_ = true;

//     // Rear sector ±30° (around ±PI)
//     if (std::abs(std::abs(angle) - M_PI) < 0.524f)
//       obstacle_rear_ = true;
//   }
// }

// // ─── Helpers ──────────────────────────────────────────────────────
// double WaypointNavigator::quaternionToYaw(const geometry_msgs::msg::Quaternion & q)
// {
//   return std::atan2(
//     2.0 * (q.w * q.z + q.x * q.y),
//     1.0 - 2.0 * (q.y * q.y + q.z * q.z));
// }

// double WaypointNavigator::normalizeAngle(double a)
// {
//   while (a >  M_PI) a -= 2.0 * M_PI;
//   while (a < -M_PI) a += 2.0 * M_PI;
//   return a;
// }

// void WaypointNavigator::stopRobot()
// {
//   cmd_vel_pub_->publish(geometry_msgs::msg::Twist());
// }

// void WaypointNavigator::publishStatus(const std::string & text)
// {
//   std_msgs::msg::String msg;
//   msg.data = text;
//   status_pub_->publish(msg);
// }

// bool WaypointNavigator::isOnlyRotation(const Waypoint & prev, const Waypoint & curr)
// {
//   return (std::abs(curr.x - prev.x) < xy_tolerance_ &&
//           std::abs(curr.y - prev.y) < xy_tolerance_);
// }

// const char * WaypointNavigator::stateStr()
// {
//   switch (state_) {
//     case State::IDLE:              return "IDLE";
//     case State::ROTATE_TO_HEADING: return "ROT_HEADING";
//     case State::DRIVE_STRAIGHT:    return "DRIVE_FWD";
//     case State::DRIVE_REVERSE:     return "DRIVE_REV";
//     case State::ROTATE_TO_THETA:   return "ROT_THETA";
//     case State::WAYPOINT_DONE:     return "WP_DONE";
//     case State::CYCLE_DONE:        return "CYCLE_DONE";
//     case State::ALL_DONE:          return "ALL_DONE";
//     default:                       return "UNKNOWN";
//   }
// }

// // ─── Load Waypoints ───────────────────────────────────────────────
// bool WaypointNavigator::loadWaypoints(const std::string & path)
// {
//   if (path.empty()) { RCLCPP_ERROR(get_logger(), "waypoints_file is empty!"); return false; }
//   std::ifstream f(path);
//   if (!f.good()) { RCLCPP_ERROR(get_logger(), "Cannot open: %s", path.c_str()); return false; }
//   f.seekg(0, std::ios::end);
//   if (f.tellg() == 0) { RCLCPP_ERROR(get_logger(), "File empty: %s", path.c_str()); return false; }
//   f.close();

//   try {
//     YAML::Node root = YAML::LoadFile(path);
//     YAML::Node wps  = root["waypoints"];
//     if (!wps || !wps.IsSequence()) {
//       RCLCPP_ERROR(get_logger(), "No 'waypoints' list in YAML"); return false;
//     }
//     for (size_t i = 0; i < wps.size(); ++i) {
//       Waypoint w;
//       w.x       = wps[i]["x"].as<double>();
//       w.y       = wps[i]["y"].as<double>();
//       w.theta   = wps[i]["theta"].as<double>();
//       w.type    = wps[i]["type"]    ? wps[i]["type"].as<std::string>()    : "waypoint";
//       w.mission = wps[i]["mission"] ? wps[i]["mission"].as<std::string>() : "forward";
//       waypoints_.push_back(w);

//       RCLCPP_INFO(get_logger(),
//         "  WP[%02zu] %-10s %-8s x=%.2f y=%.2f theta=%.1f°",
//         i, w.type.c_str(), w.mission.c_str(),
//         w.x, w.y, w.theta * 180.0 / M_PI);
//     }
//   } catch (const std::exception & e) {
//     RCLCPP_ERROR(get_logger(), "YAML error: %s", e.what()); return false;
//   }
//   return !waypoints_.empty();
// }

// // ─── Main Control Loop  20 Hz ─────────────────────────────────────
// void WaypointNavigator::controlLoop()
// {
//   if (init_failed_ || !pose_received_) return;
//   if (state_ == State::IDLE)     { stopRobot(); return; }
//   if (state_ == State::ALL_DONE) {
//     stopRobot();
//     // Stay here — cyclesCallback will reset state when user sends new command
//     return;
//   }

//   if (wp_index_ >= waypoints_.size()) {
//     state_ = State::CYCLE_DONE; return;
//   }

//   // ── CYCLE DONE ───────────────────────────────────────────────
//   if (state_ == State::CYCLE_DONE) {
//     cycles_done_++;
//     if (loop_forever_ || cycles_done_ < max_cycles_) {
//       wp_index_ = 0;
//       state_    = State::ROTATE_TO_HEADING;
//       RCLCPP_INFO(get_logger(), loop_forever_ ?
//         "🔁 Cycle %d done — looping!" : "🔄 Cycle %d/%d done — next cycle!",
//         cycles_done_, max_cycles_);
//     } else {
//       state_ = State::ALL_DONE; stopRobot();
//       RCLCPP_INFO(get_logger(), "✅ ALL %d CYCLE(S) DONE!", cycles_done_);
//     }
//     return;
//   }

//   const Waypoint & wp = waypoints_[wp_index_];
//   const bool is_reverse = (wp.mission == "reverse");

//   double dx            = wp.x - robot_x_;
//   double dy            = wp.y - robot_y_;
//   double distance      = std::hypot(dx, dy);
//   double angle_to_goal = std::atan2(dy, dx);

//   // For reverse: robot needs its BACK to face goal
//   // So target heading = angle_to_goal + PI
//   double heading_target = is_reverse
//     ? normalizeAngle(angle_to_goal + M_PI)
//     : angle_to_goal;

//   double heading_error = normalizeAngle(heading_target - robot_yaw_);
//   double yaw_error     = normalizeAngle(wp.theta - robot_yaw_);

//   geometry_msgs::msg::Twist cmd;

//   // 1 second log
//   if (++log_counter_ >= 20) {
//     log_counter_ = 0;
//     RCLCPP_INFO(get_logger(),
//       "[%s | %s | WP%zu] pos(%.2f,%.2f,%.1f°)→(%.2f,%.2f,%.1f°) dist=%.2fm",
//       stateStr(), is_reverse ? "REV" : "FWD", wp_index_,
//       robot_x_, robot_y_, robot_yaw_ * 180.0 / M_PI,
//       wp.x, wp.y, wp.theta * 180.0 / M_PI, distance);
//   }

//   switch (state_)
//   {
//     // ════════════════════════════════════════════════════════════
//     // ROTATE_TO_HEADING
//     // Forward:  rotate until robot FACES the goal
//     // Reverse:  rotate until robot BACK is toward the goal
//     // ════════════════════════════════════════════════════════════
//     case State::ROTATE_TO_HEADING:
//     {
//       Waypoint prev = (wp_index_ > 0) ?
//         waypoints_[wp_index_ - 1] :
//         Waypoint{robot_x_, robot_y_, robot_yaw_};

//       if (isOnlyRotation(prev, wp)) {
//         state_ = State::ROTATE_TO_THETA;
//         RCLCPP_INFO(get_logger(), "[WP%zu] Pure rotation → %.1f°",
//           wp_index_, wp.theta * 180.0 / M_PI);
//         break;
//       }

//       if (distance < xy_tolerance_) {
//         stopRobot(); state_ = State::ROTATE_TO_THETA;
//         break;
//       }

//       if (std::abs(heading_error) < heading_tolerance_) {
//         stopRobot();
//         // Go to correct drive state based on mission
//         state_ = is_reverse ? State::DRIVE_REVERSE : State::DRIVE_STRAIGHT;
//         RCLCPP_INFO(get_logger(),
//           "[WP%zu] ✓ Heading aligned (%.1f°). %s → dist=%.2fm",
//           wp_index_, heading_error * 180.0 / M_PI,
//           is_reverse ? "REVERSING" : "DRIVING FORWARD", distance);
//         break;
//       }

//       // Rotate in place
//       cmd.linear.x  = 0.0;
//       cmd.angular.z = std::clamp(2.0 * heading_error, -angular_speed_, angular_speed_);
//       cmd_vel_pub_->publish(cmd);

//       RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
//         "[WP%zu] %s Rotating  err=%.1f°  cmd.z=%.3f",
//         wp_index_, is_reverse ? "⬅" : "➡",
//         heading_error * 180.0 / M_PI, cmd.angular.z);
//       break;
//     }

//     // ════════════════════════════════════════════════════════════
//     // DRIVE_STRAIGHT — Forward mission
//     // linear.x = +speed  (forward)
//     // angular.z = gentle correction
//     // ════════════════════════════════════════════════════════════
//     case State::DRIVE_STRAIGHT:
//     {
//       if (distance < xy_tolerance_) {
//         stopRobot(); state_ = State::ROTATE_TO_THETA;
//         RCLCPP_INFO(get_logger(), "[WP%zu] ✓ Forward goal reached!", wp_index_);
//         break;
//       }

//       if (obstacle_front_) {
//         stopRobot();
//         RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
//           "[WP%zu] ⚠ Obstacle FRONT! Waiting...", wp_index_);
//         break;
//       }

//       if (std::abs(heading_error) > 1.05) {
//         stopRobot(); state_ = State::ROTATE_TO_HEADING;
//         RCLCPP_WARN(get_logger(), "[WP%zu] Off course %.1f° — re-rotating",
//           wp_index_, heading_error * 180.0 / M_PI);
//         break;
//       }

//       double speed = linear_speed_;
//       if (distance < 0.30)
//         speed = std::max(0.05, linear_speed_ * (distance / 0.30));

//       cmd.linear.x  = speed;   // POSITIVE = forward
//       cmd.angular.z = std::clamp(0.5 * heading_error, -drive_correction_, drive_correction_);
//       cmd_vel_pub_->publish(cmd);

//       RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
//         "[WP%zu] ▲ FORWARD  dist=%.2fm  spd=%.2f  err=%.1f°",
//         wp_index_, distance, speed, heading_error * 180.0 / M_PI);
//       break;
//     }

//     // ════════════════════════════════════════════════════════════
//     // DRIVE_REVERSE — Reverse mission
//     //
//     // KEY FIX for straight-line reverse:
//     //   During reversing, we track angle_to_goal directly.
//     //   The robot's back needs to stay pointed at the goal.
//     //   back_error = angle_to_goal - (robot_yaw + PI)
//     //              = angle_to_goal - robot_yaw - PI
//     //   Correction = +back_error  (same sign as forward)
//     //   because diff drive angular.z turns left/right the same
//     //   regardless of whether robot moves forward or backward.
//     // ════════════════════════════════════════════════════════════
//     case State::DRIVE_REVERSE:
//     {
//       if (distance < xy_tolerance_) {
//         stopRobot(); state_ = State::ROTATE_TO_THETA;
//         RCLCPP_INFO(get_logger(), "[WP%zu] ✓ Reverse goal reached!", wp_index_);
//         break;
//       }

//       if (obstacle_rear_) {
//         stopRobot();
//         RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
//           "[WP%zu] ⚠ Obstacle REAR! Waiting...", wp_index_);
//         break;
//       }

//       // Compute how much robot's BACK has drifted from goal direction
//       // back_facing = robot_yaw + PI  (direction robot's back points)
//       // back_error  = angle_to_goal - back_facing
//       double back_facing = normalizeAngle(robot_yaw_ + M_PI);
//       double back_error  = normalizeAngle(angle_to_goal - back_facing);

//       // Re-rotate if back drifted too much from goal
//       if (std::abs(back_error) > 1.05) {
//         stopRobot(); state_ = State::ROTATE_TO_HEADING;
//         RCLCPP_WARN(get_logger(),
//           "[WP%zu] Reverse off course %.1f° — re-rotating",
//           wp_index_, back_error * 180.0 / M_PI);
//         break;
//       }

//       double speed = linear_speed_;
//       if (distance < 0.30)
//         speed = std::max(0.05, linear_speed_ * (distance / 0.30));

//       // linear.x  = NEGATIVE  (reverse direction)
//       // angular.z = +back_error correction  (same sign as forward!)
//       //             When back drifts left  → back_error > 0 → turn left → corrects
//       //             When back drifts right → back_error < 0 → turn right → corrects
//       cmd.linear.x  = -speed;
//       cmd.angular.z = std::clamp(0.5 * back_error, -drive_correction_, drive_correction_);
//       cmd_vel_pub_->publish(cmd);

//       RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
//         "[WP%zu] ▼ REVERSE  dist=%.2fm  spd=%.2f  back_err=%.1f°",
//         wp_index_, distance, speed, back_error * 180.0 / M_PI);
//       break;
//     }

//     // ════════════════════════════════════════════════════════════
//     // ROTATE_TO_THETA — rotate to final heading of waypoint
//     // ════════════════════════════════════════════════════════════
//     case State::ROTATE_TO_THETA:
//     {
//       if (std::abs(yaw_error) < yaw_tolerance_) {
//         stopRobot(); state_ = State::WAYPOINT_DONE;
//         RCLCPP_INFO(get_logger(),
//           "[WP%zu] ✅ COMPLETE  x=%.2f y=%.2f theta=%.1f° [%s/%s]",
//           wp_index_, wp.x, wp.y, wp.theta * 180.0 / M_PI,
//           wp.type.c_str(), wp.mission.c_str());
//         break;
//       }

//       cmd.linear.x  = 0.0;
//       cmd.angular.z = std::clamp(2.0 * yaw_error, -angular_speed_, angular_speed_);
//       cmd_vel_pub_->publish(cmd);

//       RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
//         "[WP%zu] Rotating to final theta=%.1f°  remaining=%.1f°",
//         wp_index_, wp.theta * 180.0 / M_PI, yaw_error * 180.0 / M_PI);
//       break;
//     }

//     // ════════════════════════════════════════════════════════════
//     // WAYPOINT_DONE — advance to next
//     // ════════════════════════════════════════════════════════════
//     case State::WAYPOINT_DONE:
//     {
//       wp_index_++;
//       if (wp_index_ < waypoints_.size()) {
//         state_ = State::ROTATE_TO_HEADING;
//         const Waypoint & nwp = waypoints_[wp_index_];
//         RCLCPP_INFO(get_logger(), "─────────────────────────────────────────");
//         RCLCPP_INFO(get_logger(),
//           "→ WP[%zu/%zu] [%s/%s]  x=%.2f y=%.2f theta=%.1f°",
//           wp_index_, waypoints_.size() - 1,
//           nwp.type.c_str(), nwp.mission.c_str(),
//           nwp.x, nwp.y, nwp.theta * 180.0 / M_PI);
//       } else {
//         state_ = State::CYCLE_DONE;
//       }
//       break;
//     }

//     case State::IDLE:
//     case State::ALL_DONE:
//     case State::CYCLE_DONE:
//     default:
//       break;
//   }
// }

// }  // namespace amr_pure_navigation

// int main(int argc, char ** argv)
// {
//   rclcpp::init(argc, argv);
//   auto node = std::make_shared<amr_pure_navigation::WaypointNavigator>();
//   rclcpp::spin(node);
//   rclcpp::shutdown();
//   return 0;
// }




//Perfect (some time rotact wrong direaction)

// ================================================================
//  waypoint_navigator.cpp  v5  —  Cycle Control via ROS2 Topic
//
//  HOW CYCLES WORK:
//    Robot starts in IDLE — waits for your command
//    You send a number to /set_cycles topic:
//      1  = run once
//      3  = run 3 times then stop
//      -1 = loop forever
//      0  = stop immediately
//
//  SEND COMMAND EXAMPLE:
//    ros2 topic pub --once /set_cycles std_msgs/msg/Int32 "data: 3"
//    ros2 topic pub --once /set_cycles std_msgs/msg/Int32 "data: -1"
//    ros2 topic pub --once /set_cycles std_msgs/msg/Int32 "data: 0"
//
//  CHECK STATUS:
//    ros2 topic echo /nav_status
//
//  TOPICS:
//    Subscribes: /amcl_pose                     (robot position on map)
//    Subscribes: /diff_drive_controller/odom    (fallback position)
//    Subscribes: /scan                          (obstacle detection)
//    Subscribes: /set_cycles  std_msgs/Int32    (your cycle command)
//    Publishes:  /diff_drive_controller/cmd_vel (move robot)
//    Publishes:  /nav_status  std_msgs/String   (current status)
// ================================================================

// #include "amr_pure_navigation/waypoint_navigator.hpp"

// namespace amr_pure_navigation
// {

// // ─── Constructor ──────────────────────────────────────────────────
// WaypointNavigator::WaypointNavigator(const rclcpp::NodeOptions & options)
// : Node("waypoint_navigator", options)
// {
//   // ── Parameters ────────────────────────────────────────────────
//   declare_parameter<std::string>("waypoints_file",     "");
//   declare_parameter<std::string>("odom_topic",         "/diff_drive_controller/odom");
//   declare_parameter<std::string>("cmd_vel_topic",      "/diff_drive_controller/cmd_vel");
//   declare_parameter<std::string>("amcl_topic",         "/amcl_pose");
//   declare_parameter<double>     ("linear_speed",       0.20);
//   declare_parameter<double>     ("angular_speed",      0.40);
//   declare_parameter<double>     ("drive_correction",   0.15);
//   declare_parameter<double>     ("xy_tolerance",       0.10);
//   declare_parameter<double>     ("yaw_tolerance",      0.08);
//   declare_parameter<double>     ("heading_tolerance",  0.10);
//   declare_parameter<double>     ("obstacle_stop_dist", 0.40);
//   declare_parameter<bool>       ("stop_on_obstacle",   true);
//   declare_parameter<bool>       ("use_amcl",           true);

//   get_parameter("waypoints_file",    waypoints_file_);
//   get_parameter("odom_topic",        odom_topic_);
//   get_parameter("cmd_vel_topic",     cmd_vel_topic_);
//   get_parameter("amcl_topic",        amcl_topic_);
//   get_parameter("linear_speed",      linear_speed_);
//   get_parameter("angular_speed",     angular_speed_);
//   get_parameter("drive_correction",  drive_correction_);
//   get_parameter("xy_tolerance",      xy_tolerance_);
//   get_parameter("yaw_tolerance",     yaw_tolerance_);
//   get_parameter("heading_tolerance", heading_tolerance_);
//   get_parameter("obstacle_stop_dist",obstacle_stop_dist_);
//   get_parameter("stop_on_obstacle",  stop_on_obstacle_);
//   get_parameter("use_amcl",          use_amcl_);

//   // ── Load waypoints ────────────────────────────────────────────
//   if (!loadWaypoints(waypoints_file_)) {
//     RCLCPP_FATAL(get_logger(), "Cannot load waypoints: %s", waypoints_file_.c_str());
//     init_failed_ = true;
//     return;
//   }

//   // ── Startup info ──────────────────────────────────────────────
//   RCLCPP_INFO(get_logger(), "=============================================");
//   RCLCPP_INFO(get_logger(), "  AMR Navigation v5 - Cycle Control");
//   RCLCPP_INFO(get_logger(), "  Waypoints    : %zu",    waypoints_.size());
//   RCLCPP_INFO(get_logger(), "  Mode         : %s",     use_amcl_ ? "AMCL map frame" : "Odom only");
//   RCLCPP_INFO(get_logger(), "  Linear speed : %.2f m/s",   linear_speed_);
//   RCLCPP_INFO(get_logger(), "  Angular speed: %.2f rad/s", angular_speed_);
//   RCLCPP_INFO(get_logger(), "=============================================");
//   RCLCPP_INFO(get_logger(), "  ⏳ WAITING FOR CYCLE COMMAND...");
//   RCLCPP_INFO(get_logger(), "");
//   RCLCPP_INFO(get_logger(), "  Send command to start:");
//   RCLCPP_INFO(get_logger(), "  ros2 topic pub --once /set_cycles std_msgs/msg/Int32 \"data: 1\"");
//   RCLCPP_INFO(get_logger(), "  ros2 topic pub --once /set_cycles std_msgs/msg/Int32 \"data: 3\"");
//   RCLCPP_INFO(get_logger(), "  ros2 topic pub --once /set_cycles std_msgs/msg/Int32 \"data: -1\"  # forever");
//   RCLCPP_INFO(get_logger(), "  ros2 topic pub --once /set_cycles std_msgs/msg/Int32 \"data: 0\"   # stop");
//   RCLCPP_INFO(get_logger(), "=============================================");

//   // ── Publishers ────────────────────────────────────────────────
//   cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);
//   status_pub_  = create_publisher<std_msgs::msg::String>("/nav_status", 10);

//   // ── Subscribers ───────────────────────────────────────────────
//   // Cycle command from user
//   cycles_sub_ = create_subscription<std_msgs::msg::Int32>(
//     "/set_cycles", 10,
//     std::bind(&WaypointNavigator::cyclesCallback, this, std::placeholders::_1));

//   // Position source
//   if (use_amcl_) {
//     amcl_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
//       amcl_topic_, 10,
//       std::bind(&WaypointNavigator::amclCallback, this, std::placeholders::_1));
//   }

//   odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
//     odom_topic_, 10,
//     std::bind(&WaypointNavigator::odomCallback, this, std::placeholders::_1));

//   scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
//     "/scan", 10,
//     std::bind(&WaypointNavigator::scanCallback, this, std::placeholders::_1));

//   // 20 Hz control loop
//   timer_ = create_wall_timer(
//     std::chrono::milliseconds(50),
//     std::bind(&WaypointNavigator::controlLoop, this));
// }

// // ─── Cycle Command Callback ───────────────────────────────────────
// // User sends: ros2 topic pub --once /set_cycles std_msgs/msg/Int32 "data: N"
// //   N > 0  : run N cycles then stop
// //   N = -1 : loop forever
// //   N = 0  : stop immediately
// void WaypointNavigator::cyclesCallback(const std_msgs::msg::Int32::SharedPtr msg)
// {
//   int requested = msg->data;

//   // ── STOP command ──────────────────────────────────────────────
//   if (requested == 0) {
//     stopRobot();
//     state_        = State::ALL_DONE;
//     max_cycles_   = 0;
//     loop_forever_ = false;
//     RCLCPP_WARN(get_logger(), "🛑 STOP command received! Robot stopped.");
//     publishStatus("STOPPED by user command");
//     return;
//   }

//   // ── LOOP FOREVER command ──────────────────────────────────────
//   if (requested == -1) {
//     loop_forever_ = true;
//     max_cycles_   = -1;
//     cycles_done_  = 0;
//     wp_index_     = 0;

//     if (pose_received_) {
//       state_ = State::ROTATE_TO_HEADING;
//       RCLCPP_INFO(get_logger(), "🔁 LOOP FOREVER started!");
//       publishStatus("RUNNING - loop forever");
//     } else {
//       RCLCPP_INFO(get_logger(), "🔁 Loop forever set. Waiting for pose...");
//       publishStatus("WAITING for pose - loop forever queued");
//     }
//     return;
//   }

//   // ── RUN N CYCLES command ──────────────────────────────────────
//   if (requested > 0) {
//     // If already running, update remaining cycles
//     if (state_ != State::IDLE && state_ != State::ALL_DONE) {
//       int remaining = requested;
//       max_cycles_   = cycles_done_ + remaining;
//       loop_forever_ = false;
//       RCLCPP_INFO(get_logger(),
//         "🔄 Cycles updated! Will complete %d more cycle(s) (total=%d)",
//         remaining, max_cycles_);
//       char buf[128];
//       snprintf(buf, sizeof(buf), "RUNNING - %d more cycles requested", remaining);
//       publishStatus(buf);
//       return;
//     }

//     // Fresh start
//     loop_forever_ = false;
//     max_cycles_   = requested;
//     cycles_done_  = 0;
//     wp_index_     = 0;

//     if (pose_received_) {
//       state_ = State::ROTATE_TO_HEADING;
//       RCLCPP_INFO(get_logger(),
//         "🚀 Starting %d cycle(s)!", requested);
//       char buf[128];
//       snprintf(buf, sizeof(buf), "RUNNING - %d cycle(s) requested", requested);
//       publishStatus(buf);
//     } else {
//       RCLCPP_INFO(get_logger(),
//         "✅ %d cycle(s) queued. Waiting for pose (set 2D Pose in RViz)...",
//         requested);
//       char buf[128];
//       snprintf(buf, sizeof(buf), "WAITING for pose - %d cycle(s) queued", requested);
//       publishStatus(buf);
//     }
//   }
// }

// // ─── AMCL Callback ───────────────────────────────────────────────
// void WaypointNavigator::amclCallback(
//   const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
// {
//   if (init_failed_) return;

//   robot_x_   = msg->pose.pose.position.x;
//   robot_y_   = msg->pose.pose.position.y;
//   robot_yaw_ = quaternionToYaw(msg->pose.pose.orientation);

//   if (!pose_received_) {
//     pose_received_ = true;
//     RCLCPP_INFO(get_logger(), "✅ AMCL active! pos: x=%.2f y=%.2f yaw=%.1f°",
//       robot_x_, robot_y_, robot_yaw_ * 180.0 / M_PI);

//     // If user already sent cycles command — start now
//     if (max_cycles_ != 0 && state_ == State::IDLE) {
//       state_ = State::ROTATE_TO_HEADING;
//       RCLCPP_INFO(get_logger(), "🚀 Pose received — starting queued cycles!");
//     }
//   }
// }

// // ─── Odom Callback ───────────────────────────────────────────────
// void WaypointNavigator::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
// {
//   if (init_failed_) return;
//   if (use_amcl_ && pose_received_) return;

//   robot_x_   = msg->pose.pose.position.x;
//   robot_y_   = msg->pose.pose.position.y;
//   robot_yaw_ = quaternionToYaw(msg->pose.pose.orientation);

//   if (!pose_received_) {
//     pose_received_ = true;
//     RCLCPP_INFO(get_logger(),
//       use_amcl_ ?
//       "⚠ Using ODOM fallback. Set 2D Pose Estimate in RViz for AMCL!" :
//       "✅ Odom active! x=%.2f y=%.2f — ready for cycle command.",
//       robot_x_, robot_y_);

//     if (max_cycles_ != 0 && state_ == State::IDLE) {
//       state_ = State::ROTATE_TO_HEADING;
//     }
//   }
// }

// // ─── Scan Callback ───────────────────────────────────────────────
// void WaypointNavigator::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
// {
//   if (!stop_on_obstacle_) return;
//   obstacle_ahead_ = false;
//   for (int i = 0; i < static_cast<int>(msg->ranges.size()); ++i) {
//     float angle = msg->angle_min + i * msg->angle_increment;
//     if (angle < -0.524f || angle > 0.524f) continue;
//     float r = msg->ranges[i];
//     if (std::isfinite(r) && r > 0.01f && r < static_cast<float>(obstacle_stop_dist_)) {
//       obstacle_ahead_ = true;
//       return;
//     }
//   }
// }

// // ─── Helpers ──────────────────────────────────────────────────────
// double WaypointNavigator::quaternionToYaw(const geometry_msgs::msg::Quaternion & q)
// {
//   return std::atan2(
//     2.0 * (q.w * q.z + q.x * q.y),
//     1.0 - 2.0 * (q.y * q.y + q.z * q.z));
// }

// double WaypointNavigator::normalizeAngle(double a)
// {
//   while (a >  M_PI) a -= 2.0 * M_PI;
//   while (a < -M_PI) a += 2.0 * M_PI;
//   return a;
// }

// void WaypointNavigator::stopRobot()
// {
//   cmd_vel_pub_->publish(geometry_msgs::msg::Twist());
// }

// void WaypointNavigator::publishStatus(const std::string & text)
// {
//   std_msgs::msg::String msg;
//   msg.data = text;
//   status_pub_->publish(msg);
// }

// bool WaypointNavigator::isOnlyRotation(const Waypoint & prev, const Waypoint & curr)
// {
//   return (std::abs(curr.x - prev.x) < xy_tolerance_ &&
//           std::abs(curr.y - prev.y) < xy_tolerance_);
// }

// const char * WaypointNavigator::stateStr()
// {
//   switch (state_) {
//     case State::IDLE:              return "IDLE";
//     case State::ROTATE_TO_HEADING: return "ROTATING_TO_GOAL";
//     case State::DRIVE_STRAIGHT:    return "DRIVING";
//     case State::ROTATE_TO_THETA:   return "ROTATING_TO_THETA";
//     case State::WAYPOINT_DONE:     return "WP_DONE";
//     case State::CYCLE_DONE:        return "CYCLE_DONE";
//     case State::ALL_DONE:          return "ALL_DONE";
//     default:                       return "UNKNOWN";
//   }
// }

// // ─── Load Waypoints ───────────────────────────────────────────────
// bool WaypointNavigator::loadWaypoints(const std::string & path)
// {
//   if (path.empty()) {
//     RCLCPP_ERROR(get_logger(), "waypoints_file param is empty!");
//     return false;
//   }
//   std::ifstream f(path);
//   if (!f.good()) {
//     RCLCPP_ERROR(get_logger(), "Cannot open: %s", path.c_str());
//     return false;
//   }
//   f.seekg(0, std::ios::end);
//   if (f.tellg() == 0) {
//     RCLCPP_ERROR(get_logger(), "File empty: %s", path.c_str());
//     return false;
//   }
//   f.close();

//   try {
//     YAML::Node root = YAML::LoadFile(path);
//     YAML::Node wps  = root["waypoints"];
//     if (!wps || !wps.IsSequence()) {
//       RCLCPP_ERROR(get_logger(), "No 'waypoints' list in YAML");
//       return false;
//     }
//     for (size_t i = 0; i < wps.size(); ++i) {
//       Waypoint w;
//       w.x     = wps[i]["x"].as<double>();
//       w.y     = wps[i]["y"].as<double>();
//       w.theta = wps[i]["theta"].as<double>();
//       w.type  = wps[i]["type"] ? wps[i]["type"].as<std::string>() : "waypoint";
//       waypoints_.push_back(w);
//       RCLCPP_INFO(get_logger(), "  WP[%02zu] %-10s x=%.2f y=%.2f theta=%.1f°",
//         i, w.type.c_str(), w.x, w.y, w.theta * 180.0 / M_PI);
//     }
//   } catch (const std::exception & e) {
//     RCLCPP_ERROR(get_logger(), "YAML error: %s", e.what());
//     return false;
//   }
//   return !waypoints_.empty();
// }

// // ─── Main Control Loop  20 Hz ─────────────────────────────────────
// void WaypointNavigator::controlLoop()
// {
//   if (init_failed_) return;

//   // ── IDLE — waiting for /set_cycles command ───────────────────
//   if (state_ == State::IDLE) {
//     stopRobot();
//     return;
//   }

//   // ── ALL DONE — all cycles finished ───────────────────────────
//   if (state_ == State::ALL_DONE) {
//     stopRobot();
//     RCLCPP_INFO_ONCE(get_logger(),
//       "✅ All %d cycle(s) completed! Robot stopped. Send /set_cycles to run again.",
//       cycles_done_);
//     return;
//   }

//   // ── Wait for pose ─────────────────────────────────────────────
//   if (!pose_received_) return;

//   // ── CYCLE DONE — check if more cycles needed ──────────────────
//   if (state_ == State::CYCLE_DONE) {
//     cycles_done_++;

//     if (loop_forever_) {
//       // Loop forever — reset and go again
//       wp_index_ = 0;
//       state_    = State::ROTATE_TO_HEADING;
//       RCLCPP_INFO(get_logger(),
//         "🔁 Cycle %d done. Looping forever — starting again!", cycles_done_);
//       char buf[64];
//       snprintf(buf, sizeof(buf), "RUNNING - cycle %d done, looping", cycles_done_);
//       publishStatus(buf);

//     } else if (cycles_done_ < max_cycles_) {
//       // More cycles remaining
//       int remaining = max_cycles_ - cycles_done_;
//       wp_index_ = 0;
//       state_    = State::ROTATE_TO_HEADING;
//       RCLCPP_INFO(get_logger(),
//         "🔄 Cycle %d/%d done. %d more remaining — starting next cycle!",
//         cycles_done_, max_cycles_, remaining);
//       char buf[128];
//       snprintf(buf, sizeof(buf),
//         "RUNNING - cycle %d/%d done", cycles_done_, max_cycles_);
//       publishStatus(buf);

//     } else {
//       // All cycles done
//       state_ = State::ALL_DONE;
//       stopRobot();
//       RCLCPP_INFO(get_logger(),
//         "✅ ALL %d CYCLE(S) COMPLETE! Robot stopped.", cycles_done_);
//       RCLCPP_INFO(get_logger(),
//         "   Send /set_cycles again to run more cycles.");
//       char buf[128];
//       snprintf(buf, sizeof(buf),
//         "DONE - %d cycle(s) completed. Send /set_cycles to run again.", cycles_done_);
//       publishStatus(buf);
//     }
//     return;
//   }

//   // ── Current target waypoint ───────────────────────────────────
//   const Waypoint & wp = waypoints_[wp_index_];

//   double dx            = wp.x - robot_x_;
//   double dy            = wp.y - robot_y_;
//   double distance      = std::hypot(dx, dy);
//   double angle_to_goal = std::atan2(dy, dx);
//   double heading_error = normalizeAngle(angle_to_goal - robot_yaw_);
//   double yaw_error     = normalizeAngle(wp.theta - robot_yaw_);

//   geometry_msgs::msg::Twist cmd;

//   // 1 second status log
//   if (++log_counter_ >= 20) {
//     log_counter_ = 0;
//     RCLCPP_INFO(get_logger(),
//       "[Cycle %d/%s | %s | WP%zu/%zu] pos(%.2f,%.2f,%.1f°)→(%.2f,%.2f,%.1f°) dist=%.2fm",
//       cycles_done_ + 1,
//       loop_forever_ ? "∞" : std::to_string(max_cycles_).c_str(),
//       stateStr(),
//       wp_index_, waypoints_.size() - 1,
//       robot_x_, robot_y_, robot_yaw_ * 180.0 / M_PI,
//       wp.x, wp.y, wp.theta * 180.0 / M_PI,
//       distance);
//   }

//   switch (state_)
//   {
//     // ── STEP 1: Rotate to face goal ───────────────────────────────
//     case State::ROTATE_TO_HEADING:
//     {
//       Waypoint prev = (wp_index_ > 0) ?
//         waypoints_[wp_index_ - 1] :
//         Waypoint{robot_x_, robot_y_, robot_yaw_};

//       if (isOnlyRotation(prev, wp)) {
//         state_ = State::ROTATE_TO_THETA;
//         RCLCPP_INFO(get_logger(), "[WP%zu] Pure rotation → %.1f°",
//           wp_index_, wp.theta * 180.0 / M_PI);
//         break;
//       }

//       if (distance < xy_tolerance_) {
//         stopRobot();
//         state_ = State::ROTATE_TO_THETA;
//         break;
//       }

//       if (std::abs(heading_error) < heading_tolerance_) {
//         stopRobot();
//         state_ = State::DRIVE_STRAIGHT;
//         RCLCPP_INFO(get_logger(), "[WP%zu] ✓ Heading OK → DRIVING dist=%.2fm",
//           wp_index_, distance);
//         break;
//       }

//       cmd.linear.x  = 0.0;
//       cmd.angular.z = std::clamp(2.0 * heading_error, -angular_speed_, angular_speed_);
//       cmd_vel_pub_->publish(cmd);
//       break;
//     }

//     // ── STEP 2: Drive straight ────────────────────────────────────
//     case State::DRIVE_STRAIGHT:
//     {
//       if (distance < xy_tolerance_) {
//         stopRobot();
//         state_ = State::ROTATE_TO_THETA;
//         RCLCPP_INFO(get_logger(), "[WP%zu] ✓ Reached. Rotating to theta=%.1f°",
//           wp_index_, wp.theta * 180.0 / M_PI);
//         break;
//       }

//       if (obstacle_ahead_) {
//         stopRobot();
//         RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
//           "[WP%zu] ⚠ Obstacle! Waiting...", wp_index_);
//         break;
//       }

//       if (std::abs(heading_error) > 1.05) {
//         stopRobot();
//         state_ = State::ROTATE_TO_HEADING;
//         break;
//       }

//       double speed = linear_speed_;
//       if (distance < 0.30) {
//         speed = std::max(0.05, linear_speed_ * (distance / 0.30));
//       }

//       cmd.linear.x  = speed;
//       cmd.angular.z = std::clamp(0.5 * heading_error, -drive_correction_, drive_correction_);
//       cmd_vel_pub_->publish(cmd);
//       break;
//     }

//     // ── STEP 3: Rotate to final theta ─────────────────────────────
//     case State::ROTATE_TO_THETA:
//     {
//       if (std::abs(yaw_error) < yaw_tolerance_) {
//         stopRobot();
//         state_ = State::WAYPOINT_DONE;
//         RCLCPP_INFO(get_logger(), "[WP%zu] ✅ Done  type=%s",
//           wp_index_, wp.type.c_str());
//         break;
//       }

//       cmd.linear.x  = 0.0;
//       cmd.angular.z = std::clamp(2.0 * yaw_error, -angular_speed_, angular_speed_);
//       cmd_vel_pub_->publish(cmd);
//       break;
//     }

//     // ── STEP 4: Next waypoint or end of cycle ─────────────────────
//     case State::WAYPOINT_DONE:
//     {
//       wp_index_++;

//       if (wp_index_ < waypoints_.size()) {
//         // More waypoints in this cycle
//         state_ = State::ROTATE_TO_HEADING;
//         RCLCPP_INFO(get_logger(), "→ WP[%zu/%zu]  x=%.2f y=%.2f theta=%.1f°",
//           wp_index_, waypoints_.size() - 1,
//           waypoints_[wp_index_].x,
//           waypoints_[wp_index_].y,
//           waypoints_[wp_index_].theta * 180.0 / M_PI);
//       } else {
//         // End of waypoint list = one full cycle complete
//         state_ = State::CYCLE_DONE;
//       }
//       break;
//     }

//     case State::IDLE:
//     case State::ALL_DONE:
//     case State::CYCLE_DONE:
//     default:
//       break;
//   }
// }

// }  // namespace amr_pure_navigation

// int main(int argc, char ** argv)
// {
//   rclcpp::init(argc, argv);
//   auto node = std::make_shared<amr_pure_navigation::WaypointNavigator>();
//   rclcpp::spin(node);
//   rclcpp::shutdown();
//   return 0;
// }


//currect code 

// // ================================================================
// //  waypoint_navigator.cpp  v4
// //
// //  HOW IT WORKS:
// //    1. AMCL tells us where robot is on the MAP (no drift)
// //    2. We calculate direction to next waypoint
// //    3. We send cmd_vel to move the robot
// //
// //  STRAIGHT LINE FIX:
// //    drive_correction = 0.15  (very gentle steering while driving)
// //    This keeps left/right RPM nearly equal = straight line
// //
// //  TOPICS:
// //    Reads  : /amcl_pose                      (robot position on map)
// //    Reads  : /diff_drive_controller/odom     (fallback if AMCL not ready)
// //    Reads  : /scan                           (obstacle detection)
// //    Writes : /diff_drive_controller/cmd_vel  (move robot)
// // ================================================================

// #include "amr_pure_navigation/waypoint_navigator.hpp"

// namespace amr_pure_navigation
// {

// // ─── Constructor ──────────────────────────────────────────────────
// WaypointNavigator::WaypointNavigator(const rclcpp::NodeOptions & options)
// : Node("waypoint_navigator", options)
// {
//   // ── Declare all parameters with default values ─────────────
//   declare_parameter<std::string>("waypoints_file",     "");
//   declare_parameter<std::string>("odom_topic",         "/diff_drive_controller/odom");
//   declare_parameter<std::string>("cmd_vel_topic",      "/diff_drive_controller/cmd_vel");
//   declare_parameter<std::string>("amcl_topic",         "/amcl_pose");
//   declare_parameter<double>     ("linear_speed",       0.20);
//   declare_parameter<double>     ("angular_speed",      0.40);
//   declare_parameter<double>     ("drive_correction",   0.15);
//   declare_parameter<double>     ("xy_tolerance",       0.10);
//   declare_parameter<double>     ("yaw_tolerance",      0.08);
//   declare_parameter<double>     ("heading_tolerance",  0.10);
//   declare_parameter<double>     ("obstacle_stop_dist", 0.40);
//   declare_parameter<bool>       ("loop_waypoints",     false);
//   declare_parameter<bool>       ("stop_on_obstacle",   true);
//   declare_parameter<bool>       ("use_amcl",           true);

//   // ── Read all parameters ────────────────────────────────────
//   get_parameter("waypoints_file",    waypoints_file_);
//   get_parameter("odom_topic",        odom_topic_);
//   get_parameter("cmd_vel_topic",     cmd_vel_topic_);
//   get_parameter("amcl_topic",        amcl_topic_);
//   get_parameter("linear_speed",      linear_speed_);
//   get_parameter("angular_speed",     angular_speed_);
//   get_parameter("drive_correction",  drive_correction_);
//   get_parameter("xy_tolerance",      xy_tolerance_);
//   get_parameter("yaw_tolerance",     yaw_tolerance_);
//   get_parameter("heading_tolerance", heading_tolerance_);
//   get_parameter("obstacle_stop_dist",obstacle_stop_dist_);
//   get_parameter("loop_waypoints",    loop_waypoints_);
//   get_parameter("stop_on_obstacle",  stop_on_obstacle_);
//   get_parameter("use_amcl",          use_amcl_);

//   // ── Load waypoints from YAML file ─────────────────────────
//   if (!loadWaypoints(waypoints_file_)) {
//     RCLCPP_FATAL(get_logger(), "Cannot load waypoints: %s", waypoints_file_.c_str());
//     init_failed_ = true;
//     return;
//   }

//   // ── Print startup info ────────────────────────────────────
//   RCLCPP_INFO(get_logger(), "=============================================");
//   RCLCPP_INFO(get_logger(), "  AMR Navigation v4 - Ready");
//   RCLCPP_INFO(get_logger(), "  Waypoints      : %zu",    waypoints_.size());
//   RCLCPP_INFO(get_logger(), "  Mode           : %s",     use_amcl_ ? "AMCL map frame" : "Odom only");
//   RCLCPP_INFO(get_logger(), "  Linear speed   : %.2f m/s",   linear_speed_);
//   RCLCPP_INFO(get_logger(), "  Angular speed  : %.2f rad/s", angular_speed_);
//   RCLCPP_INFO(get_logger(), "  Drive correct  : %.2f rad/s (keeps straight line)", drive_correction_);
//   RCLCPP_INFO(get_logger(), "  XY tolerance   : %.2f m",     xy_tolerance_);
//   RCLCPP_INFO(get_logger(), "  Yaw tolerance  : %.2f rad",   yaw_tolerance_);
//   RCLCPP_INFO(get_logger(), "=============================================");
//   if (use_amcl_) {
//     RCLCPP_INFO(get_logger(), "  Waiting for AMCL...");
//     RCLCPP_INFO(get_logger(), "  ➡ Open RViz → click '2D Pose Estimate'");
//     RCLCPP_INFO(get_logger(), "  ➡ Click on map where robot is located");
//   }
//   RCLCPP_INFO(get_logger(), "=============================================");

//   // ── Create publisher ──────────────────────────────────────
//   cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);

//   // ── Create subscribers ────────────────────────────────────
//   if (use_amcl_) {
//     amcl_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
//       amcl_topic_, 10,
//       std::bind(&WaypointNavigator::amclCallback, this, std::placeholders::_1));
//   }

//   odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
//     odom_topic_, 10,
//     std::bind(&WaypointNavigator::odomCallback, this, std::placeholders::_1));

//   scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
//     "/scan", 10,
//     std::bind(&WaypointNavigator::scanCallback, this, std::placeholders::_1));

//   // ── 20 Hz timer ───────────────────────────────────────────
//   timer_ = create_wall_timer(
//     std::chrono::milliseconds(50),
//     std::bind(&WaypointNavigator::controlLoop, this));
// }

// // ─── AMCL Callback ───────────────────────────────────────────────
// // Called every time AMCL publishes robot position on the map
// // This is the main position source — no drift!
// void WaypointNavigator::amclCallback(
//   const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
// {
//   if (init_failed_) return;

//   robot_x_   = msg->pose.pose.position.x;
//   robot_y_   = msg->pose.pose.position.y;
//   robot_yaw_ = quaternionToYaw(msg->pose.pose.orientation);

//   if (!pose_received_) {
//     pose_received_ = true;
//     state_ = State::ROTATE_TO_HEADING;
//     RCLCPP_INFO(get_logger(), "✅ AMCL active! Robot at x=%.2f y=%.2f yaw=%.1f°",
//       robot_x_, robot_y_, robot_yaw_ * 180.0 / M_PI);
//     RCLCPP_INFO(get_logger(), "   Target WP[0]: x=%.2f y=%.2f theta=%.1f°",
//       waypoints_[0].x, waypoints_[0].y, waypoints_[0].theta * 180.0 / M_PI);
//     RCLCPP_INFO(get_logger(), "   🚀 Starting navigation!");
//   }
// }

// // ─── Odom Callback ───────────────────────────────────────────────
// // Used as fallback before AMCL is ready
// // In use_amcl=false mode this is the only position source
// void WaypointNavigator::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
// {
//   if (init_failed_) return;
//   if (use_amcl_ && pose_received_) return;  // AMCL already active, ignore odom

//   robot_x_   = msg->pose.pose.position.x;
//   robot_y_   = msg->pose.pose.position.y;
//   robot_yaw_ = quaternionToYaw(msg->pose.pose.orientation);

//   if (!pose_received_) {
//     pose_received_ = true;
//     state_ = State::ROTATE_TO_HEADING;
//     if (use_amcl_) {
//       RCLCPP_WARN(get_logger(),
//         "⚠ AMCL not ready — using odom. Set 2D Pose Estimate in RViz!");
//     } else {
//       RCLCPP_INFO(get_logger(),
//         "✅ Odom active! Robot at x=%.2f y=%.2f — Starting!",
//         robot_x_, robot_y_);
//     }
//   }
// }

// // ─── Scan Callback ───────────────────────────────────────────────
// // Checks if obstacle is within stop distance in front of robot
// void WaypointNavigator::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
// {
//   if (!stop_on_obstacle_) return;
//   obstacle_ahead_ = false;
//   for (int i = 0; i < static_cast<int>(msg->ranges.size()); ++i) {
//     float angle = msg->angle_min + i * msg->angle_increment;
//     if (angle < -0.524f || angle > 0.524f) continue;  // only check ±30° front
//     float r = msg->ranges[i];
//     if (std::isfinite(r) && r > 0.01f && r < static_cast<float>(obstacle_stop_dist_)) {
//       obstacle_ahead_ = true;
//       return;
//     }
//   }
// }

// // ─── Helper: quaternion → yaw angle ──────────────────────────────
// double WaypointNavigator::quaternionToYaw(const geometry_msgs::msg::Quaternion & q)
// {
//   return std::atan2(
//     2.0 * (q.w * q.z + q.x * q.y),
//     1.0 - 2.0 * (q.y * q.y + q.z * q.z));
// }

// // ─── Helper: normalize angle to [-pi, pi] ────────────────────────
// double WaypointNavigator::normalizeAngle(double a)
// {
//   while (a >  M_PI) a -= 2.0 * M_PI;
//   while (a < -M_PI) a += 2.0 * M_PI;
//   return a;
// }

// // ─── Helper: stop robot ───────────────────────────────────────────
// void WaypointNavigator::stopRobot()
// {
//   cmd_vel_pub_->publish(geometry_msgs::msg::Twist());
// }

// // ─── Helper: is this waypoint same x,y as previous? ──────────────
// bool WaypointNavigator::isOnlyRotation(const Waypoint & prev, const Waypoint & curr)
// {
//   return (std::abs(curr.x - prev.x) < xy_tolerance_ &&
//           std::abs(curr.y - prev.y) < xy_tolerance_);
// }

// // ─── Helper: state name for logging ──────────────────────────────
// const char * WaypointNavigator::stateStr()
// {
//   switch (state_) {
//     case State::IDLE:              return "IDLE";
//     case State::ROTATE_TO_HEADING: return "ROTATING_TO_GOAL";
//     case State::DRIVE_STRAIGHT:    return "DRIVING";
//     case State::ROTATE_TO_THETA:   return "ROTATING_TO_THETA";
//     case State::WAYPOINT_DONE:     return "WP_DONE";
//     case State::ALL_DONE:          return "ALL_DONE";
//     default:                       return "UNKNOWN";
//   }
// }

// // ─── Load waypoints from YAML ─────────────────────────────────────
// bool WaypointNavigator::loadWaypoints(const std::string & path)
// {
//   if (path.empty()) {
//     RCLCPP_ERROR(get_logger(), "waypoints_file param is empty!");
//     return false;
//   }

//   std::ifstream f(path);
//   if (!f.good()) {
//     RCLCPP_ERROR(get_logger(), "Cannot open file: %s", path.c_str());
//     return false;
//   }
//   f.seekg(0, std::ios::end);
//   if (f.tellg() == 0) {
//     RCLCPP_ERROR(get_logger(), "File is empty: %s", path.c_str());
//     return false;
//   }
//   f.close();

//   try {
//     YAML::Node root = YAML::LoadFile(path);
//     YAML::Node wps  = root["waypoints"];
//     if (!wps || !wps.IsSequence()) {
//       RCLCPP_ERROR(get_logger(), "No 'waypoints' list in YAML");
//       return false;
//     }
//     for (size_t i = 0; i < wps.size(); ++i) {
//       Waypoint w;
//       w.x     = wps[i]["x"].as<double>();
//       w.y     = wps[i]["y"].as<double>();
//       w.theta = wps[i]["theta"].as<double>();
//       w.type  = wps[i]["type"] ? wps[i]["type"].as<std::string>() : "waypoint";
//       waypoints_.push_back(w);
//       RCLCPP_INFO(get_logger(), "  WP[%02zu] %-10s x=%.2f  y=%.2f  theta=%.1f°",
//         i, w.type.c_str(), w.x, w.y, w.theta * 180.0 / M_PI);
//     }
//   } catch (const std::exception & e) {
//     RCLCPP_ERROR(get_logger(), "YAML error: %s", e.what());
//     return false;
//   }
//   return !waypoints_.empty();
// }

// // ─── Main Control Loop  (called 20 times per second) ──────────────
// void WaypointNavigator::controlLoop()
// {
//   if (init_failed_ || !pose_received_) return;

//   // ── Finished all waypoints ───────────────────────────────────
//   if (state_ == State::ALL_DONE) {
//     stopRobot();
//     RCLCPP_INFO_ONCE(get_logger(), "✅ All waypoints done! Robot stopped.");
//     return;
//   }

//   // ── Safety check ─────────────────────────────────────────────
//   if (wp_index_ >= waypoints_.size()) {
//     if (loop_waypoints_) {
//       wp_index_ = 0;
//       state_    = State::ROTATE_TO_HEADING;
//       RCLCPP_INFO(get_logger(), "🔁 Looping back to start.");
//     } else {
//       state_ = State::ALL_DONE;
//     }
//     return;
//   }

//   // ── Current target waypoint ───────────────────────────────────
//   const Waypoint & wp = waypoints_[wp_index_];

//   // ── Calculate distance and angle errors ──────────────────────
//   double dx            = wp.x - robot_x_;
//   double dy            = wp.y - robot_y_;
//   double distance      = std::hypot(dx, dy);
//   double angle_to_goal = std::atan2(dy, dx);
//   double heading_error = normalizeAngle(angle_to_goal - robot_yaw_);
//   double yaw_error     = normalizeAngle(wp.theta - robot_yaw_);

//   geometry_msgs::msg::Twist cmd;

//   // ── Print status every 1 second ──────────────────────────────
//   if (++log_counter_ >= 20) {
//     log_counter_ = 0;
//     RCLCPP_INFO(get_logger(),
//       "[%s WP%zu/%zu] robot(%.2f, %.2f, %.1f°) → target(%.2f, %.2f, %.1f°) dist=%.2fm",
//       stateStr(), wp_index_, waypoints_.size() - 1,
//       robot_x_, robot_y_, robot_yaw_ * 180.0 / M_PI,
//       wp.x, wp.y, wp.theta * 180.0 / M_PI, distance);
//   }

//   switch (state_)
//   {
//     // ════════════════════════════════════════════════
//     // STEP 1: Rotate in place to face the goal x,y
//     //   left RPM  = +speed
//     //   right RPM = -speed  (opposite = rotate)
//     // ════════════════════════════════════════════════
//     case State::ROTATE_TO_HEADING:
//     {
//       Waypoint prev = (wp_index_ > 0) ?
//         waypoints_[wp_index_ - 1] :
//         Waypoint{robot_x_, robot_y_, robot_yaw_};

//       // Same x,y as previous — skip to final rotation
//       if (isOnlyRotation(prev, wp)) {
//         state_ = State::ROTATE_TO_THETA;
//         RCLCPP_INFO(get_logger(), "[WP%zu] Pure rotation → %.1f°",
//           wp_index_, wp.theta * 180.0 / M_PI);
//         break;
//       }

//       // Already at goal position
//       if (distance < xy_tolerance_) {
//         stopRobot();
//         state_ = State::ROTATE_TO_THETA;
//         RCLCPP_INFO(get_logger(), "[WP%zu] Already at position → rotating", wp_index_);
//         break;
//       }

//       // Heading is aligned — start driving
//       if (std::abs(heading_error) < heading_tolerance_) {
//         stopRobot();
//         state_ = State::DRIVE_STRAIGHT;
//         RCLCPP_INFO(get_logger(), "[WP%zu] ✓ Heading OK (%.1f°). DRIVING dist=%.2fm",
//           wp_index_, heading_error * 180.0 / M_PI, distance);
//         break;
//       }

//       // Rotate in place
//       cmd.linear.x  = 0.0;
//       cmd.angular.z = std::clamp(2.0 * heading_error, -angular_speed_, angular_speed_);
//       cmd_vel_pub_->publish(cmd);

//       RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
//         "[WP%zu] Rotating to face goal  err=%.1f°  cmd.z=%.3f",
//         wp_index_, heading_error * 180.0 / M_PI, cmd.angular.z);
//       break;
//     }

//     // ════════════════════════════════════════════════
//     // STEP 2: Drive straight to goal x,y
//     //   left RPM  = +speed
//     //   right RPM = +speed  (same = straight)
//     //
//     //   KEY FIX: angular.z is capped at drive_correction_
//     //   (0.15 rad/s) so RPM difference is tiny — robot
//     //   drives in a nearly perfect straight line
//     // ════════════════════════════════════════════════
//     case State::DRIVE_STRAIGHT:
//     {
//       // Reached goal
//       if (distance < xy_tolerance_) {
//         stopRobot();
//         state_ = State::ROTATE_TO_THETA;
//         RCLCPP_INFO(get_logger(), "[WP%zu] ✓ Reached goal (%.3fm). Rotating to theta",
//           wp_index_, distance);
//         break;
//       }

//       // Obstacle — stop and wait
//       if (obstacle_ahead_) {
//         stopRobot();
//         RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
//           "[WP%zu] ⚠ Obstacle! Waiting...", wp_index_);
//         break;
//       }

//       // Too far off course — re-rotate
//       if (std::abs(heading_error) > 1.05) {
//         stopRobot();
//         state_ = State::ROTATE_TO_HEADING;
//         RCLCPP_WARN(get_logger(), "[WP%zu] Off course %.1f° — re-rotating",
//           wp_index_, heading_error * 180.0 / M_PI);
//         break;
//       }

//       // Slow down near goal
//       double speed = linear_speed_;
//       if (distance < 0.30) {
//         speed = std::max(0.05, linear_speed_ * (distance / 0.30));
//       }

//       // Drive straight with GENTLE correction
//       // drive_correction_ = 0.15  →  tiny angular.z
//       // → left and right RPM stay nearly equal
//       // → robot drives in straight line
//       cmd.linear.x  = speed;
//       cmd.angular.z = std::clamp(
//         0.5 * heading_error,   // gain 0.5 (gentle)
//         -drive_correction_,    // max -0.15 rad/s
//          drive_correction_);   // max +0.15 rad/s
//       cmd_vel_pub_->publish(cmd);

//       RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
//         "[WP%zu] Driving  dist=%.2fm  speed=%.2f  heading_err=%.1f°",
//         wp_index_, distance, speed, heading_error * 180.0 / M_PI);
//       break;
//     }

//     // ════════════════════════════════════════════════
//     // STEP 3: Rotate in place to final theta
//     //   + error = anticlockwise (left turn)
//     //   - error = clockwise     (right turn)
//     // ════════════════════════════════════════════════
//     case State::ROTATE_TO_THETA:
//     {
//       if (std::abs(yaw_error) < yaw_tolerance_) {
//         stopRobot();
//         state_ = State::WAYPOINT_DONE;
//         RCLCPP_INFO(get_logger(),
//           "[WP%zu] ✅ DONE  x=%.2f y=%.2f theta=%.1f°  type=%s",
//           wp_index_, wp.x, wp.y,
//           wp.theta * 180.0 / M_PI, wp.type.c_str());
//         break;
//       }

//       cmd.linear.x  = 0.0;
//       cmd.angular.z = std::clamp(2.0 * yaw_error, -angular_speed_, angular_speed_);
//       cmd_vel_pub_->publish(cmd);

//       RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
//         "[WP%zu] Rotating to theta=%.1f°  remaining=%.1f°",
//         wp_index_, wp.theta * 180.0 / M_PI,
//         yaw_error * 180.0 / M_PI);
//       break;
//     }

//     // ════════════════════════════════════════════════
//     // STEP 4: Load next waypoint
//     // ════════════════════════════════════════════════
//     case State::WAYPOINT_DONE:
//     {
//       wp_index_++;
//       if (wp_index_ < waypoints_.size()) {
//         state_ = State::ROTATE_TO_HEADING;
//         RCLCPP_INFO(get_logger(), "─────────────────────────────────────");
//         RCLCPP_INFO(get_logger(), "→ Next WP[%zu/%zu]  x=%.2f y=%.2f theta=%.1f°",
//           wp_index_, waypoints_.size() - 1,
//           waypoints_[wp_index_].x,
//           waypoints_[wp_index_].y,
//           waypoints_[wp_index_].theta * 180.0 / M_PI);
//       }
//       break;
//     }

//     case State::IDLE:
//     case State::ALL_DONE:
//     default:
//       break;
//   }
// }

// }  // namespace amr_pure_navigation

// int main(int argc, char ** argv)
// {
//   rclcpp::init(argc, argv);
//   auto node = std::make_shared<amr_pure_navigation::WaypointNavigator>();
//   rclcpp::spin(node);
//   rclcpp::shutdown();
//   return 0;
// }








//linenot stright 


// // ================================================================
// //  waypoint_navigator.cpp  —  Map-based AMR Navigation
// //
// //  Position source : /amcl_pose  (map frame, no drift)
// //  Fallback        : /diff_drive_controller/odom (until AMCL ready)
// //  Output          : /diff_drive_controller/cmd_vel
// //
// //  Flow per waypoint:
// //    1. ROTATE_TO_HEADING — rotate to face goal x,y
// //    2. DRIVE_STRAIGHT    — drive to goal x,y
// //    3. ROTATE_TO_THETA   — rotate to final theta
// //    4. WAYPOINT_DONE     — next waypoint
// //
// //  Key difference from odom-only version:
// //    Robot position comes from AMCL (map frame) so:
// //    - No drift over long distances
// //    - Waypoints are real map coordinates
// //    - Works even if robot is moved/repositioned
// // ================================================================

// #include "amr_pure_navigation/waypoint_navigator.hpp"

// namespace amr_navigation
// {

// // ─── Constructor ──────────────────────────────────────────────────
// WaypointNavigator::WaypointNavigator(const rclcpp::NodeOptions & options)
// : Node("waypoint_navigator", options)
// {
//   // ── Parameters ────────────────────────────────────────────────
//   declare_parameter<std::string>("waypoints_file",     "");
//   declare_parameter<std::string>("cmd_vel_topic",      "/diff_drive_controller/cmd_vel");
//   declare_parameter<std::string>("amcl_topic",         "/amcl_pose");
//   declare_parameter<std::string>("odom_topic",         "/diff_drive_controller/odom");
//   declare_parameter<double>     ("linear_speed",       0.20);
//   declare_parameter<double>     ("angular_speed",      0.40);
//   declare_parameter<double>     ("xy_tolerance",       0.10);
//   declare_parameter<double>     ("yaw_tolerance",      0.08);
//   declare_parameter<double>     ("heading_tolerance",  0.10);
//   declare_parameter<double>     ("obstacle_stop_dist", 0.40);
//   declare_parameter<bool>       ("loop_waypoints",     false);
//   declare_parameter<bool>       ("stop_on_obstacle",   true);

//   get_parameter("waypoints_file",    waypoints_file_);
//   get_parameter("cmd_vel_topic",     cmd_vel_topic_);
//   get_parameter("amcl_topic",        amcl_topic_);
//   get_parameter("odom_topic",        odom_topic_);
//   get_parameter("linear_speed",      linear_speed_);
//   get_parameter("angular_speed",     angular_speed_);
//   get_parameter("xy_tolerance",      xy_tolerance_);
//   get_parameter("yaw_tolerance",     yaw_tolerance_);
//   get_parameter("heading_tolerance", heading_tolerance_);
//   get_parameter("obstacle_stop_dist",obstacle_stop_dist_);
//   get_parameter("loop_waypoints",    loop_waypoints_);
//   get_parameter("stop_on_obstacle",  stop_on_obstacle_);

//   // ── Load waypoints ────────────────────────────────────────────
//   if (!loadWaypoints(waypoints_file_)) {
//     RCLCPP_FATAL(get_logger(), "Failed to load waypoints: %s", waypoints_file_.c_str());
//     init_failed_ = true;
//     return;
//   }

//   RCLCPP_INFO(get_logger(), "==============================================");
//   RCLCPP_INFO(get_logger(), "  AMR Map-Based Navigation");
//   RCLCPP_INFO(get_logger(), "  Waypoints    : %zu",    waypoints_.size());
//   RCLCPP_INFO(get_logger(), "  AMCL topic   : %s",     amcl_topic_.c_str());
//   RCLCPP_INFO(get_logger(), "  Odom topic   : %s",     odom_topic_.c_str());
//   RCLCPP_INFO(get_logger(), "  Cmd_vel      : %s",     cmd_vel_topic_.c_str());
//   RCLCPP_INFO(get_logger(), "  Lin speed    : %.2f m/s",   linear_speed_);
//   RCLCPP_INFO(get_logger(), "  Ang speed    : %.2f rad/s",  angular_speed_);
//   RCLCPP_INFO(get_logger(), "  XY tolerance : %.3f m",     xy_tolerance_);
//   RCLCPP_INFO(get_logger(), "  Yaw tolerance: %.3f rad",   yaw_tolerance_);
//   RCLCPP_INFO(get_logger(), "==============================================");
//   RCLCPP_INFO(get_logger(), "  Waiting for AMCL pose on: %s", amcl_topic_.c_str());
//   RCLCPP_INFO(get_logger(), "  ⚠ Set initial pose in RViz first!");
//   RCLCPP_INFO(get_logger(), "==============================================");

//   // ── Publishers / Subscribers ──────────────────────────────────
//   cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);

//   // Primary: AMCL pose in MAP frame (no drift)
//   amcl_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
//     amcl_topic_, 10,
//     std::bind(&WaypointNavigator::amclCallback, this, std::placeholders::_1));

//   // Fallback: odom (used only until AMCL publishes first pose)
//   odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
//     odom_topic_, 10,
//     std::bind(&WaypointNavigator::odomCallback, this, std::placeholders::_1));

//   // Obstacle detection
//   scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
//     "/scan", 10,
//     std::bind(&WaypointNavigator::scanCallback, this, std::placeholders::_1));

//   // 20 Hz control loop
//   timer_ = create_wall_timer(
//     std::chrono::milliseconds(50),
//     std::bind(&WaypointNavigator::controlLoop, this));
// }

// // ─── AMCL Callback — PRIMARY pose source (map frame) ─────────────
// void WaypointNavigator::amclCallback(
//   const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
// {
//   if (init_failed_) return;

//   robot_x_ = msg->pose.pose.position.x;
//   robot_y_ = msg->pose.pose.position.y;

//   const auto & q = msg->pose.pose.orientation;
//   robot_yaw_ = std::atan2(
//     2.0 * (q.w * q.z + q.x * q.y),
//     1.0 - 2.0 * (q.y * q.y + q.z * q.z));

//   if (!pose_received_) {
//     pose_received_ = true;
//     state_ = State::ROTATE_TO_HEADING;
//     RCLCPP_INFO(get_logger(),
//       "✅ AMCL pose received! MAP position: x=%.3f y=%.3f yaw=%.3f",
//       robot_x_, robot_y_, robot_yaw_);
//     RCLCPP_INFO(get_logger(),
//       "   First target: x=%.3f y=%.3f theta=%.3f",
//       waypoints_[0].x, waypoints_[0].y, waypoints_[0].theta);
//     RCLCPP_INFO(get_logger(), "   🚀 Navigation starting!");
//   }
// }

// // ─── Odom Callback — FALLBACK only (used until AMCL ready) ────────
// void WaypointNavigator::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
// {
//   if (init_failed_ || pose_received_) return;  // ignore once AMCL is active

//   // Use odom only as fallback before AMCL initializes
//   robot_x_ = msg->pose.pose.position.x;
//   robot_y_ = msg->pose.pose.position.y;
//   const auto & q = msg->pose.pose.orientation;
//   robot_yaw_ = std::atan2(
//     2.0 * (q.w * q.z + q.x * q.y),
//     1.0 - 2.0 * (q.y * q.y + q.z * q.z));

//   RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
//     "⚠ Using ODOM (not map frame) — set initial pose in RViz to activate AMCL!");
// }

// // ─── Scan Callback — Obstacle Detection ──────────────────────────
// void WaypointNavigator::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
// {
//   if (!stop_on_obstacle_) return;
//   obstacle_ahead_ = false;
//   int n = static_cast<int>(msg->ranges.size());
//   for (int i = 0; i < n; ++i) {
//     float angle = msg->angle_min + i * msg->angle_increment;
//     if (angle < -0.524f || angle > 0.524f) continue;  // ±30° front
//     float r = msg->ranges[i];
//     if (std::isfinite(r) && r > 0.01f && r < static_cast<float>(obstacle_stop_dist_)) {
//       obstacle_ahead_ = true;
//       return;
//     }
//   }
// }

// // ─── Helpers ──────────────────────────────────────────────────────
// double WaypointNavigator::normalizeAngle(double a)
// {
//   while (a >  M_PI) a -= 2.0 * M_PI;
//   while (a < -M_PI) a += 2.0 * M_PI;
//   return a;
// }

// void WaypointNavigator::stopRobot()
// {
//   geometry_msgs::msg::Twist cmd;
//   cmd_vel_pub_->publish(cmd);
// }

// bool WaypointNavigator::isOnlyRotation(const Waypoint & prev, const Waypoint & curr)
// {
//   return (std::abs(curr.x - prev.x) < xy_tolerance_ &&
//           std::abs(curr.y - prev.y) < xy_tolerance_);
// }

// const char * WaypointNavigator::stateStr()
// {
//   switch (state_) {
//     case State::IDLE:              return "IDLE";
//     case State::ROTATE_TO_HEADING: return "ROT_HEADING";
//     case State::DRIVE_STRAIGHT:    return "DRIVE";
//     case State::ROTATE_TO_THETA:   return "ROT_THETA";
//     case State::WAYPOINT_DONE:     return "WP_DONE";
//     case State::ALL_DONE:          return "ALL_DONE";
//     default:                       return "UNKNOWN";
//   }
// }

// // ─── Load Waypoints ───────────────────────────────────────────────
// bool WaypointNavigator::loadWaypoints(const std::string & path)
// {
//   if (path.empty()) {
//     RCLCPP_ERROR(get_logger(), "waypoints_file param is empty!");
//     return false;
//   }
//   std::ifstream f(path);
//   if (!f.good()) {
//     RCLCPP_ERROR(get_logger(), "Cannot open: %s", path.c_str());
//     return false;
//   }
//   f.seekg(0, std::ios::end);
//   if (f.tellg() == 0) {
//     RCLCPP_ERROR(get_logger(), "File is empty: %s", path.c_str());
//     return false;
//   }
//   f.close();

//   try {
//     YAML::Node root = YAML::LoadFile(path);
//     YAML::Node wps  = root["waypoints"];
//     if (!wps || !wps.IsSequence()) {
//       RCLCPP_ERROR(get_logger(), "No 'waypoints' list found in YAML");
//       return false;
//     }
//     for (size_t i = 0; i < wps.size(); ++i) {
//       Waypoint w;
//       w.x     = wps[i]["x"].as<double>();
//       w.y     = wps[i]["y"].as<double>();
//       w.theta = wps[i]["theta"].as<double>();
//       w.type  = wps[i]["type"] ? wps[i]["type"].as<std::string>() : "waypoint";
//       waypoints_.push_back(w);
//       RCLCPP_INFO(get_logger(),
//         "  WP[%02zu] %-10s  x=%.3f  y=%.3f  theta=%.3f",
//         i, w.type.c_str(), w.x, w.y, w.theta);
//     }
//   } catch (const std::exception & e) {
//     RCLCPP_ERROR(get_logger(), "YAML parse error: %s", e.what());
//     return false;
//   }
//   return !waypoints_.empty();
// }

// // ─── Main Control Loop  20 Hz ─────────────────────────────────────
// void WaypointNavigator::controlLoop()
// {
//   if (init_failed_ || !pose_received_) return;

//   if (state_ == State::ALL_DONE) {
//     stopRobot();
//     RCLCPP_INFO_ONCE(get_logger(), "✅ All waypoints done! Robot stopped.");
//     return;
//   }

//   if (wp_index_ >= waypoints_.size()) {
//     state_ = loop_waypoints_ ? State::ROTATE_TO_HEADING : State::ALL_DONE;
//     if (loop_waypoints_) {
//       wp_index_ = 0;
//       RCLCPP_INFO(get_logger(), "🔁 Looping back to start.");
//     }
//     return;
//   }

//   const Waypoint & wp = waypoints_[wp_index_];

//   double dx            = wp.x - robot_x_;
//   double dy            = wp.y - robot_y_;
//   double distance      = std::hypot(dx, dy);
//   double angle_to_goal = std::atan2(dy, dx);
//   double heading_error = normalizeAngle(angle_to_goal - robot_yaw_);
//   double yaw_error     = normalizeAngle(wp.theta - robot_yaw_);

//   geometry_msgs::msg::Twist cmd;

//   // ── 1 second status log ───────────────────────────────────────
//   if (++log_counter_ >= 20) {
//     log_counter_ = 0;
//     RCLCPP_INFO(get_logger(),
//       "[%s WP%zu/%zu] pos(%.2f,%.2f,%.2f°) → (%.2f,%.2f,%.2f°) dist=%.2fm",
//       stateStr(), wp_index_, waypoints_.size() - 1,
//       robot_x_, robot_y_, robot_yaw_ * 180.0 / M_PI,
//       wp.x, wp.y, wp.theta * 180.0 / M_PI,
//       distance);
//   }

//   switch (state_)
//   {
//     // ── STATE 1: Rotate to face goal x,y ─────────────────────────
//     case State::ROTATE_TO_HEADING:
//     {
//       Waypoint prev = (wp_index_ > 0) ?
//         waypoints_[wp_index_ - 1] :
//         Waypoint{robot_x_, robot_y_, robot_yaw_};

//       // Same x,y as previous → pure rotation only
//       if (isOnlyRotation(prev, wp)) {
//         state_ = State::ROTATE_TO_THETA;
//         RCLCPP_INFO(get_logger(),
//           "[WP%zu] Same position → rotating to theta=%.2f°",
//           wp_index_, wp.theta * 180.0 / M_PI);
//         break;
//       }

//       // Already close enough
//       if (distance < xy_tolerance_) {
//         stopRobot();
//         state_ = State::ROTATE_TO_THETA;
//         RCLCPP_INFO(get_logger(),
//           "[WP%zu] Already at position → rotating to theta", wp_index_);
//         break;
//       }

//       // Heading aligned → start driving
//       if (std::abs(heading_error) < heading_tolerance_) {
//         stopRobot();
//         state_ = State::DRIVE_STRAIGHT;
//         RCLCPP_INFO(get_logger(),
//           "[WP%zu] ✓ Heading aligned (%.2f°). Driving → dist=%.3fm",
//           wp_index_, heading_error * 180.0 / M_PI, distance);
//         break;
//       }

//       // Rotate — positive = anticlockwise, negative = clockwise
//       cmd.linear.x  = 0.0;
//       cmd.angular.z = std::clamp(2.0 * heading_error, -angular_speed_, angular_speed_);
//       cmd_vel_pub_->publish(cmd);

//       RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
//         "[WP%zu] Rotating to face goal  err=%.2f°  cmd.z=%.3f",
//         wp_index_, heading_error * 180.0 / M_PI, cmd.angular.z);
//       break;
//     }

//     // ── STATE 2: Drive straight to x,y ───────────────────────────
//     case State::DRIVE_STRAIGHT:
//     {
//       // Reached goal position
//       if (distance < xy_tolerance_) {
//         stopRobot();
//         state_ = State::ROTATE_TO_THETA;
//         RCLCPP_INFO(get_logger(),
//           "[WP%zu] ✓ Position reached (%.3fm). Rotating to %.2f°",
//           wp_index_, distance, wp.theta * 180.0 / M_PI);
//         break;
//       }

//       // Obstacle
//       if (obstacle_ahead_) {
//         stopRobot();
//         RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
//           "[WP%zu] ⚠ Obstacle! Waiting...", wp_index_);
//         break;
//       }

//       // Very off course (> 60°) — stop and re-rotate
//       if (std::abs(heading_error) > 1.05) {
//         stopRobot();
//         state_ = State::ROTATE_TO_HEADING;
//         RCLCPP_WARN(get_logger(),
//           "[WP%zu] Off course by %.2f° — re-rotating",
//           wp_index_, heading_error * 180.0 / M_PI);
//         break;
//       }

//       // Speed ramp-down near goal
//       double speed = linear_speed_;
//       if (distance < 0.30) {
//         speed = std::max(0.05, linear_speed_ * (distance / 0.30));
//       }

//       // Drive with continuous heading correction
//       cmd.linear.x  = speed;
//       cmd.angular.z = std::clamp(2.0 * heading_error, -angular_speed_, angular_speed_);
//       cmd_vel_pub_->publish(cmd);

//       RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
//         "[WP%zu] Driving  dist=%.3fm  spd=%.2f  err=%.2f°",
//         wp_index_, distance, speed, heading_error * 180.0 / M_PI);
//       break;
//     }

//     // ── STATE 3: Rotate to final theta ────────────────────────────
//     case State::ROTATE_TO_THETA:
//     {
//       if (std::abs(yaw_error) < yaw_tolerance_) {
//         stopRobot();
//         state_ = State::WAYPOINT_DONE;
//         RCLCPP_INFO(get_logger(),
//           "[WP%zu] ✅ COMPLETE  x=%.3f y=%.3f theta=%.2f°  type=%s",
//           wp_index_, wp.x, wp.y, wp.theta * 180.0 / M_PI, wp.type.c_str());
//         break;
//       }

//       // Positive error = anticlockwise, negative = clockwise
//       cmd.linear.x  = 0.0;
//       cmd.angular.z = std::clamp(2.0 * yaw_error, -angular_speed_, angular_speed_);
//       cmd_vel_pub_->publish(cmd);

//       RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
//         "[WP%zu] Rotating to final theta  err=%.2f°  cmd.z=%.3f",
//         wp_index_, yaw_error * 180.0 / M_PI, cmd.angular.z);
//       break;
//     }

//     // ── STATE 4: Load next waypoint ───────────────────────────────
//     case State::WAYPOINT_DONE:
//     {
//       wp_index_++;
//       if (wp_index_ < waypoints_.size()) {
//         state_ = State::ROTATE_TO_HEADING;
//         RCLCPP_INFO(get_logger(), "─────────────────────────────────────────");
//         RCLCPP_INFO(get_logger(),
//           "→ Next WP[%zu/%zu]  x=%.3f  y=%.3f  theta=%.2f°  type=%s",
//           wp_index_, waypoints_.size() - 1,
//           waypoints_[wp_index_].x,
//           waypoints_[wp_index_].y,
//           waypoints_[wp_index_].theta * 180.0 / M_PI,
//           waypoints_[wp_index_].type.c_str());
//       }
//       break;
//     }

//     case State::IDLE:
//     case State::ALL_DONE:
//     default:
//       break;
//   }
// }

// }  // namespace amr_navigation

// // ── main ──────────────────────────────────────────────────────────
// int main(int argc, char ** argv)
// {
//   rclcpp::init(argc, argv);
//   auto node = std::make_shared<amr_navigation::WaypointNavigator>();
//   rclcpp::spin(node);
//   rclcpp::shutdown();
//   return 0;
// }

