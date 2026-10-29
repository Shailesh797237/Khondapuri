#pragma once

#include <string>
#include <vector>
#include <deque>
#include <map>
#include <cmath>
#include <algorithm>
#include <fstream>
#include <filesystem>
#include <sstream>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#include <yaml-cpp/yaml.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace amr_pure_navigation
{

struct Waypoint {
  double      x        {0.0};
  double      y        {0.0};
  double      theta    {0.0};
  std::string type       {"waypoint"};
  std::string mission    {"forward"};
  std::string name       {""};        // station name for PLC
  std::string rotation   {"auto"};    // "cw" "ccw" "auto"
  std::string plc_feedback {""};      // I0.x IPC reads to know PLC operation is done
  double      speed      {0.0};       // per-waypoint speed override (0 = use default linear_speed)
};

enum class State {
  IDLE,
  ROTATE_TO_HEADING,
  DRIVE_STRAIGHT,
  DRIVE_REVERSE,
  ROTATE_TO_THETA,
  WAIT_PLC,          // ← NEW: robot stopped, waiting for /plc_done
  WAYPOINT_DONE,
  CYCLE_DONE,
  ALL_DONE
};

class WaypointNavigator : public rclcpp::Node
{
public:
  explicit WaypointNavigator(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void amclCallback    (const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
  void odomCallback    (const nav_msgs::msg::Odometry::SharedPtr msg);
  void scanCallback    (const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void cyclesCallback  (const std_msgs::msg::Int32::SharedPtr msg);
  void plcDoneCallback    (const std_msgs::msg::String::SharedPtr msg);
  void plcTriggerCallback (const std_msgs::msg::Bool::SharedPtr msg);
  void lidarObstacleCallback(const std_msgs::msg::Bool::SharedPtr msg);
  void emergencyStopCallback(const std_msgs::msg::Bool::SharedPtr msg);
  void loadMissionCallback(const std_msgs::msg::String::SharedPtr msg);
  void queueMissionsCallback(const std_msgs::msg::String::SharedPtr msg);
  void controlLoop        ();
  void updateSpeedRamp    ();
  void updateEmergencyRamp();

  bool        loadWaypoints    (const std::string & path);
  bool        loadMissionsFolder(const std::string & folder_path);
  bool        selectMission    (const std::string & mission_name);
  void        publishMissionList();
  void        stopRobot       ();
  void        publishStatus   (const std::string & msg);
  double      normalizeAngle  (double a);
  double      quaternionToYaw (const geometry_msgs::msg::Quaternion & q);
  bool        isOnlyRotation  (const Waypoint & prev, const Waypoint & curr);
  const char* stateStr        ();
  int         getTagIndex     (const std::string & name);
  std::string getTagName      (int index);

  // ── ROS interfaces ────────────────────────────────────────────
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr                        cmd_vel_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr                            status_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr                            station_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr                            mission_list_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr                       odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr                   scan_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr                          cycles_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr                         plc_done_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr                            plc_trigger_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr                            lidar_obstacle_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr                            emergency_stop_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr                          load_mission_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr                          queue_missions_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr                             queue_status_pub_;
  rclcpp::TimerBase::SharedPtr                                                   timer_;

  // ── TF2 for map→base_link lookup ───────────────────────────────
  std::shared_ptr<tf2_ros::Buffer>            tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // ── Robot pose ────────────────────────────────────────────────
  double robot_x_          {0.0};
  double robot_y_          {0.0};
  double robot_yaw_        {0.0};
  bool   pose_received_    {false};
  bool   init_failed_      {false};

  // ── Obstacle detection ────────────────────────────────────────
  bool   obstacle_ahead_   {false};
  bool   obstacle_front_   {false};
  bool   obstacle_rear_    {false};

  // ── PLC bridge ────────────────────────────────────────────────
  bool   plc_enabled_        {false};  // enable PLC integration
  bool   waiting_for_plc_    {false};  // robot paused at station
  bool   plc_done_           {false};  // /plc_done received
  bool   plc_trigger_value_  {false};  // live PLC bit from /plc_trigger

  // ── Lidar obstacle speed ramp ─────────────────────────────────
  //  DB1.DBX0.0 → /lidar_obstacle → ramp speed down/up
  bool   lidar_obstacle_active_  {false};   // current lidar obstacle state
  double speed_ramp_factor_      {1.0};     // 1.0 = full speed, 0.0 = stopped
  double ramp_down_duration_     {0.7};     // seconds to ramp from full → 0 (FAST for safety)
  double ramp_up_duration_       {2.0};     // seconds to ramp from 0 → full
  rclcpp::Time ramp_start_time_;            // when ramp started
  bool   ramp_timer_initialized_ {false};   // track if ramp_start_time_ is set

  // ── Emergency stop ramp ────────────────────────────────────────
  //  DB1.DBX0.5 → /emergency_stop → immediate stop in 0.2s
  bool   emergency_active_          {false};   // current emergency state
  double emergency_ramp_factor_     {1.0};     // 1.0 = normal, 0.0 = stopped
  double emergency_ramp_down_       {0.2};     // seconds to stop (VERY FAST)
  double emergency_ramp_up_         {1.0};     // seconds to resume after release
  rclcpp::Time emergency_ramp_start_;
  bool   emergency_ramp_initialized_ {false};

  // ── Navigation state ──────────────────────────────────────────
  State                 state_       {State::IDLE};
  std::vector<Waypoint> waypoints_;
  size_t                wp_index_    {0};
  int                   log_counter_ {0};

  // ── Mission selection ─────────────────────────────────────────
  std::map<std::string, std::vector<Waypoint>> missions_map_;  // all missions from folder
  std::string current_mission_name_;                            // currently active mission
  bool missions_loaded_ {false};                                // true if missions folder loaded OK

  // ── Dynamic mission queue ─────────────────────────────────────
  //  User sends: "tag3,tag7,tag9,tag1,home" or "tag1 to tag8" on /queue_missions
  //  Robot chains through them automatically as a unified path.
  bool                     queue_active_ {false};
  size_t                   queue_total_  {0};
  std::vector<std::string> waypoint_mission_names_;
  std::string              last_tracked_mission_;

  // ── Cycle control ─────────────────────────────────────────────
  int  max_cycles_     {0};
  int  cycles_done_    {0};
  bool loop_forever_   {false};

  // ── Parameters ────────────────────────────────────────────────
  std::string waypoints_file_;
  std::string missions_folder_;
  std::string odom_topic_         {"/odometry/filtered"};
  std::string cmd_vel_topic_      {"/diff_drive_controller/cmd_vel"};
  std::string amcl_topic_         {"/amcl_pose"};
  double      linear_speed_       {0.20};
  double      angular_speed_      {0.40};
  double      drive_correction_   {0.15};
  double      xy_tolerance_       {0.10};
  double      lookahead_radius_   {0.50};   // smooth curve-through radius for waypoints
  double      yaw_tolerance_      {0.08};
  double      heading_tolerance_  {0.10};
  double      obstacle_stop_dist_ {0.40};
  bool        stop_on_obstacle_   {true};
  bool        use_amcl_           {true};
  std::vector<std::string> ordered_tags_;
};

}  // namespace amr_pure_navigation





// // trail
// #pragma once

// #include <string>
// #include <vector>
// #include <cmath>
// #include <algorithm>
// #include <fstream>

// #include "rclcpp/rclcpp.hpp"
// #include "geometry_msgs/msg/twist.hpp"
// #include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
// #include "geometry_msgs/msg/quaternion.hpp"
// #include "nav_msgs/msg/odometry.hpp"
// #include "sensor_msgs/msg/laser_scan.hpp"
// #include "std_msgs/msg/int32.hpp"
// #include "std_msgs/msg/string.hpp"
// #include <yaml-cpp/yaml.h>

// namespace amr_pure_navigation
// {

// struct Waypoint {
//   double      x        {0.0};
//   double      y        {0.0};
//   double      theta    {0.0};
//   std::string type     {"waypoint"};
//   std::string mission  {"forward"};
//   std::string name     {""};        // used as station name for PLC
//   std::string rotation {"auto"};    // "cw"=clockwise, "ccw"=anticlockwise, "auto"=shortest
// };

// enum class State {
//   IDLE,
//   ROTATE_TO_HEADING,
//   DRIVE_STRAIGHT,
//   DRIVE_REVERSE,
//   ROTATE_TO_THETA,
//   WAIT_PLC,          // ← NEW: robot stopped, waiting for /plc_done
//   WAYPOINT_DONE,
//   CYCLE_DONE,
//   ALL_DONE
// };

// class WaypointNavigator : public rclcpp::Node
// {
// public:
//   explicit WaypointNavigator(
//     const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

// private:
//   void amclCallback    (const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
//   void odomCallback    (const nav_msgs::msg::Odometry::SharedPtr msg);
//   void scanCallback    (const sensor_msgs::msg::LaserScan::SharedPtr msg);
//   void cyclesCallback  (const std_msgs::msg::Int32::SharedPtr msg);
//   void plcDoneCallback (const std_msgs::msg::String::SharedPtr msg);
//   void controlLoop     ();

//   bool        loadWaypoints   (const std::string & path);
//   void        stopRobot       ();
//   void        publishStatus   (const std::string & msg);
//   double      normalizeAngle  (double a);
//   double      quaternionToYaw (const geometry_msgs::msg::Quaternion & q);
//   bool        isOnlyRotation  (const Waypoint & prev, const Waypoint & curr);
//   const char* stateStr        ();

//   // ── ROS interfaces ────────────────────────────────────────────
//   rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr                        cmd_vel_pub_;
//   rclcpp::Publisher<std_msgs::msg::String>::SharedPtr                            status_pub_;
//   rclcpp::Publisher<std_msgs::msg::String>::SharedPtr                            station_pub_;
//   rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_sub_;
//   rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr                       odom_sub_;
//   rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr                   scan_sub_;
//   rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr                          cycles_sub_;
//   rclcpp::Subscription<std_msgs::msg::String>::SharedPtr                         plc_done_sub_;
//   rclcpp::TimerBase::SharedPtr                                                   timer_;

//   // ── Robot pose ────────────────────────────────────────────────
//   double robot_x_          {0.0};
//   double robot_y_          {0.0};
//   double robot_yaw_        {0.0};
//   bool   pose_received_    {false};
//   bool   init_failed_      {false};

//   // ── Obstacle detection ────────────────────────────────────────
//   bool   obstacle_ahead_   {false};
//   bool   obstacle_front_   {false};
//   bool   obstacle_rear_    {false};

//   // ── PLC bridge ────────────────────────────────────────────────
//   bool   plc_enabled_      {false};  // enable PLC integration
//   bool   waiting_for_plc_  {false};  // robot paused at station
//   bool   plc_done_         {false};  // /plc_done received

//   // ── Navigation state ──────────────────────────────────────────
//   State                 state_       {State::IDLE};
//   std::vector<Waypoint> waypoints_;
//   size_t                wp_index_    {0};
//   int                   log_counter_ {0};

//   // ── Cycle control ─────────────────────────────────────────────
//   int  max_cycles_     {0};
//   int  cycles_done_    {0};
//   bool loop_forever_   {false};

//   // ── Parameters ────────────────────────────────────────────────
//   std::string waypoints_file_;
//   std::string odom_topic_         {"/odometry/filtered"};
//   std::string cmd_vel_topic_      {"/diff_drive_controller/cmd_vel"};
//   std::string amcl_topic_         {"/amcl_pose"};
//   double      linear_speed_       {0.20};
//   double      angular_speed_      {0.40};
//   double      drive_correction_   {0.15};
//   double      xy_tolerance_       {0.10};
//   double      yaw_tolerance_      {0.08};
//   double      heading_tolerance_  {0.10};
//   double      obstacle_stop_dist_ {0.40};
//   bool        stop_on_obstacle_   {true};
//   bool        use_amcl_           {true};
// };

// }  // namespace amr_pure_navigation





// // forword and reverse movement
// #pragma once

// #include <string>
// #include <vector>
// #include <cmath>
// #include <algorithm>
// #include <fstream>

// #include "rclcpp/rclcpp.hpp"
// #include "geometry_msgs/msg/twist.hpp"
// #include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
// #include "geometry_msgs/msg/quaternion.hpp"
// #include "nav_msgs/msg/odometry.hpp"
// #include "sensor_msgs/msg/laser_scan.hpp"
// #include "std_msgs/msg/int32.hpp"
// #include "std_msgs/msg/string.hpp"
// #include <yaml-cpp/yaml.h>

// namespace amr_pure_navigation
// {

// struct Waypoint {
//   double      x       {0.0};
//   double      y       {0.0};
//   double      theta   {0.0};
//   std::string type    {"waypoint"};
//   std::string mission {"forward"};   // "forward" or "reverse"
// };

// enum class State {
//   IDLE,
//   ROTATE_TO_HEADING,   // rotate to face goal (fwd) or back-to-goal (rev)
//   DRIVE_STRAIGHT,      // drive forward  linear.x = +speed
//   DRIVE_REVERSE,       // drive backward linear.x = -speed
//   ROTATE_TO_THETA,     // rotate to final theta of waypoint
//   WAYPOINT_DONE,
//   CYCLE_DONE,
//   ALL_DONE
// };

// class WaypointNavigator : public rclcpp::Node
// {
// public:
//   explicit WaypointNavigator(
//     const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

// private:
//   void amclCallback   (const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
//   void odomCallback   (const nav_msgs::msg::Odometry::SharedPtr msg);
//   void scanCallback   (const sensor_msgs::msg::LaserScan::SharedPtr msg);
//   void cyclesCallback (const std_msgs::msg::Int32::SharedPtr msg);
//   void controlLoop    ();

//   bool        loadWaypoints   (const std::string & path);
//   void        stopRobot       ();
//   void        publishStatus   (const std::string & msg);
//   double      normalizeAngle  (double a);
//   double      quaternionToYaw (const geometry_msgs::msg::Quaternion & q);
//   bool        isOnlyRotation  (const Waypoint & prev, const Waypoint & curr);
//   const char* stateStr        ();

//   // ── ROS interfaces ───────────────────────────────────────────
//   rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr                        cmd_vel_pub_;
//   rclcpp::Publisher<std_msgs::msg::String>::SharedPtr                            status_pub_;
//   rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_sub_;
//   rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr                       odom_sub_;
//   rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr                   scan_sub_;
//   rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr                          cycles_sub_;
//   rclcpp::TimerBase::SharedPtr                                                   timer_;

//   // ── Robot pose ───────────────────────────────────────────────
//   double robot_x_         {0.0};
//   double robot_y_         {0.0};
//   double robot_yaw_       {0.0};
//   bool   pose_received_   {false};
//   bool   init_failed_     {false};

//   // ── Obstacle detection (front + rear separate) ───────────────
//   bool   obstacle_ahead_  {false};   // legacy
//   bool   obstacle_front_  {false};   // blocks DRIVE_STRAIGHT
//   bool   obstacle_rear_   {false};   // blocks DRIVE_REVERSE

//   // ── Navigation state ─────────────────────────────────────────
//   State                 state_       {State::IDLE};
//   std::vector<Waypoint> waypoints_;
//   size_t                wp_index_    {0};
//   int                   log_counter_ {0};

//   // ── Cycle control ────────────────────────────────────────────
//   int  max_cycles_    {0};
//   int  cycles_done_   {0};
//   bool loop_forever_  {false};

//   // ── Parameters ───────────────────────────────────────────────
//   std::string waypoints_file_;
//   std::string odom_topic_         {"/odometry/filtered"};
//   std::string cmd_vel_topic_      {"/diff_drive_controller/cmd_vel"};
//   std::string amcl_topic_         {"/amcl_pose"};
//   double      linear_speed_       {0.20};
//   double      angular_speed_      {0.40};
//   double      drive_correction_   {0.15};
//   double      xy_tolerance_       {0.10};
//   double      yaw_tolerance_      {0.08};
//   double      heading_tolerance_  {0.10};
//   double      obstacle_stop_dist_ {0.40};
//   bool        stop_on_obstacle_   {true};
//   bool        use_amcl_           {true};
// };

// }  // namespace amr_pure_navigation



// //perfect (sometime rotate wrong direaction)

// #pragma once

// #include <string>
// #include <vector>
// #include <cmath>
// #include <algorithm>
// #include <fstream>

// #include "rclcpp/rclcpp.hpp"
// #include "geometry_msgs/msg/twist.hpp"
// #include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
// #include "geometry_msgs/msg/quaternion.hpp"
// #include "nav_msgs/msg/odometry.hpp"
// #include "sensor_msgs/msg/laser_scan.hpp"
// #include "std_msgs/msg/int32.hpp"       // for /set_cycles topic
// #include "std_msgs/msg/string.hpp"      // for /nav_status topic
// #include <yaml-cpp/yaml.h>

// namespace amr_pure_navigation
// {

// struct Waypoint {
//   double      x     {0.0};
//   double      y     {0.0};
//   double      theta {0.0};
//   std::string type  {"waypoint"};
// };

// enum class State {
//   IDLE,               // waiting for cycles command
//   ROTATE_TO_HEADING,
//   DRIVE_STRAIGHT,
//   ROTATE_TO_THETA,
//   WAYPOINT_DONE,
//   CYCLE_DONE,         // one full cycle complete — check if more needed
//   ALL_DONE            // all cycles complete
// };

// class WaypointNavigator : public rclcpp::Node
// {
// public:
//   explicit WaypointNavigator(
//     const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

// private:
//   // Callbacks
//   void amclCallback   (const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
//   void odomCallback   (const nav_msgs::msg::Odometry::SharedPtr msg);
//   void scanCallback   (const sensor_msgs::msg::LaserScan::SharedPtr msg);
//   void cyclesCallback (const std_msgs::msg::Int32::SharedPtr msg);  // /set_cycles
//   void controlLoop    ();

//   // Helpers
//   bool        loadWaypoints   (const std::string & path);
//   void        stopRobot       ();
//   void        publishStatus   (const std::string & msg);
//   double      normalizeAngle  (double a);
//   double      quaternionToYaw (const geometry_msgs::msg::Quaternion & q);
//   bool        isOnlyRotation  (const Waypoint & prev, const Waypoint & curr);
//   const char* stateStr        ();

//   // ── ROS interfaces ──────────────────────────────────────────────
//   rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr                        cmd_vel_pub_;
//   rclcpp::Publisher<std_msgs::msg::String>::SharedPtr                            status_pub_;
//   rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_sub_;
//   rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr                       odom_sub_;
//   rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr                   scan_sub_;
//   rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr                          cycles_sub_;
//   rclcpp::TimerBase::SharedPtr                                                   timer_;

//   // ── Robot pose ──────────────────────────────────────────────────
//   double robot_x_         {0.0};
//   double robot_y_         {0.0};
//   double robot_yaw_       {0.0};
//   bool   pose_received_   {false};
//   bool   obstacle_ahead_  {false};
//   bool   init_failed_     {false};

//   // ── Navigation state ────────────────────────────────────────────
//   State                 state_       {State::IDLE};
//   std::vector<Waypoint> waypoints_;
//   size_t                wp_index_    {0};
//   int                   log_counter_ {0};

//   // ── Cycle control ────────────────────────────────────────────────
//   int  max_cycles_    {0};     // 0 = waiting for user command
//   int  cycles_done_   {0};     // how many cycles completed
//   bool loop_forever_  {false}; // true when user sends -1

//   // ── Parameters ──────────────────────────────────────────────────
//   std::string waypoints_file_;
//   std::string odom_topic_         {"/diff_drive_controller/odom"};
//   std::string cmd_vel_topic_      {"/cmd_vel"};
//   std::string amcl_topic_         {"/amcl_pose"};
//   double      linear_speed_       {0.20};
//   double      angular_speed_      {0.40};
//   double      drive_correction_   {0.15};
//   double      xy_tolerance_       {0.10};
//   double      yaw_tolerance_      {0.08};
//   double      heading_tolerance_  {0.10};
//   double      obstacle_stop_dist_ {0.40};
//   bool        stop_on_obstacle_   {true};
//   bool        use_amcl_           {true};
// };

// }  // namespace amr_pure_navigation




//currect code 
// #pragma once

// #include <string>
// #include <vector>
// #include <cmath>
// #include <algorithm>
// #include <fstream>

// #include "rclcpp/rclcpp.hpp"
// #include "geometry_msgs/msg/twist.hpp"
// #include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
// #include "geometry_msgs/msg/quaternion.hpp"
// #include "nav_msgs/msg/odometry.hpp"
// #include "sensor_msgs/msg/laser_scan.hpp"
// #include <yaml-cpp/yaml.h>

// namespace amr_pure_navigation
// {

// struct Waypoint {
//   double      x     {0.0};
//   double      y     {0.0};
//   double      theta {0.0};
//   std::string type  {"waypoint"};
// };

// enum class State {
//   IDLE,
//   ROTATE_TO_HEADING,
//   DRIVE_STRAIGHT,
//   ROTATE_TO_THETA,
//   WAYPOINT_DONE,
//   ALL_DONE
// };

// class WaypointNavigator : public rclcpp::Node
// {
// public:
//   explicit WaypointNavigator(
//     const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

// private:
//   void amclCallback (const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
//   void odomCallback (const nav_msgs::msg::Odometry::SharedPtr msg);
//   void scanCallback (const sensor_msgs::msg::LaserScan::SharedPtr msg);
//   void controlLoop  ();

//   bool        loadWaypoints    (const std::string & path);
//   void        stopRobot        ();
//   double      normalizeAngle   (double a);
//   double      quaternionToYaw  (const geometry_msgs::msg::Quaternion & q);
//   bool        isOnlyRotation   (const Waypoint & prev, const Waypoint & curr);
//   const char* stateStr         ();

//   // ROS
//   rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr                        cmd_vel_pub_;
//   rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_sub_;
//   rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr                       odom_sub_;
//   rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr                   scan_sub_;
//   rclcpp::TimerBase::SharedPtr                                                   timer_;

//   // Robot state
//   double robot_x_         {0.0};
//   double robot_y_         {0.0};
//   double robot_yaw_       {0.0};
//   bool   pose_received_   {false};
//   bool   obstacle_ahead_  {false};
//   bool   init_failed_     {false};

//   // Navigation
//   State                 state_       {State::IDLE};
//   std::vector<Waypoint> waypoints_;
//   size_t                wp_index_    {0};
//   int                   log_counter_ {0};

//   // Parameters
//   std::string waypoints_file_;
//   std::string odom_topic_         {"/diff_drive_controller/odom"};
//   std::string cmd_vel_topic_      {"/diff_drive_controller/cmd_vel"};
//   std::string amcl_topic_         {"/amcl_pose"};
//   double      linear_speed_       {0.20};
//   double      angular_speed_      {0.40};
//   double      drive_correction_   {0.15};  // max angular.z while driving straight
//   double      xy_tolerance_       {0.10};
//   double      yaw_tolerance_      {0.08};
//   double      heading_tolerance_  {0.10};
//   double      obstacle_stop_dist_ {0.40};
//   bool        loop_waypoints_     {false};
//   bool        stop_on_obstacle_   {true};
//   bool        use_amcl_           {true};
// };

// }  // namespace amr_pure_navigation




// #pragma once

// // ================================================================
// //  waypoint_navigator.hpp
// //  Map-based AMR navigation using AMCL + Map Server
// //  Position source: /amcl_pose  (map frame — no drift!)
// //  Output:          /diff_drive_controller/cmd_vel
// // ================================================================

// #include <string>
// #include <vector>
// #include <cmath>
// #include <algorithm>
// #include <fstream>

// #include "rclcpp/rclcpp.hpp"
// #include "geometry_msgs/msg/twist.hpp"
// #include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
// #include "nav_msgs/msg/odometry.hpp"
// #include "sensor_msgs/msg/laser_scan.hpp"
// #include <yaml-cpp/yaml.h>

// namespace amr_navigation
// {

// // ── Waypoint in MAP frame ─────────────────────────────────────────
// struct Waypoint {
//   double      x     {0.0};
//   double      y     {0.0};
//   double      theta {0.0};   // radians
//   std::string type  {"waypoint"};
// };

// // ── State Machine ─────────────────────────────────────────────────
// enum class State {
//   IDLE,
//   ROTATE_TO_HEADING,   // rotate to face next x,y
//   DRIVE_STRAIGHT,      // drive to x,y
//   ROTATE_TO_THETA,     // rotate to final theta
//   WAYPOINT_DONE,       // load next waypoint
//   ALL_DONE             // finished
// };

// // ── Node ──────────────────────────────────────────────────────────
// class WaypointNavigator : public rclcpp::Node
// {
// public:
//   explicit WaypointNavigator(
//     const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

// private:
//   // Callbacks
//   void amclCallback (const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
//   void odomCallback (const nav_msgs::msg::Odometry::SharedPtr msg);
//   void scanCallback (const sensor_msgs::msg::LaserScan::SharedPtr msg);
//   void controlLoop  ();

//   // Helpers
//   bool        loadWaypoints  (const std::string & path);
//   void        stopRobot      ();
//   double      normalizeAngle (double a);
//   bool        isOnlyRotation (const Waypoint & prev, const Waypoint & curr);
//   const char* stateStr       ();

//   // ── ROS interfaces ────────────────────────────────────────────────
//   rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr                        cmd_vel_pub_;
//   rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_sub_;
//   rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr                       odom_sub_;
//   rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr                   scan_sub_;
//   rclcpp::TimerBase::SharedPtr                                                   timer_;

//   // ── Robot pose (MAP frame from AMCL) ──────────────────────────────
//   double robot_x_          {0.0};
//   double robot_y_          {0.0};
//   double robot_yaw_        {0.0};
//   bool   pose_received_    {false};   // true once AMCL publishes
//   bool   obstacle_ahead_   {false};
//   bool   init_failed_      {false};

//   // ── Navigation state ──────────────────────────────────────────────
//   State                 state_       {State::IDLE};
//   std::vector<Waypoint> waypoints_;
//   size_t                wp_index_    {0};
//   int                   log_counter_ {0};

//   // ── Parameters ────────────────────────────────────────────────────
//   std::string waypoints_file_;
//   std::string cmd_vel_topic_       {"/diff_drive_controller/cmd_vel"};
//   std::string amcl_topic_          {"/amcl_pose"};
//   std::string odom_topic_          {"/diff_drive_controller/odom"};
//   double      linear_speed_        {0.20};
//   double      angular_speed_       {0.40};
//   double      xy_tolerance_        {0.10};
//   double      yaw_tolerance_       {0.08};
//   double      heading_tolerance_   {0.10};
//   double      obstacle_stop_dist_  {0.40};
//   bool        loop_waypoints_      {false};
//   bool        stop_on_obstacle_    {true};
// };

// }  // namespace amr_navigation
