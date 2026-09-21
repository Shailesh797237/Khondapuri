// ================================================================
//  plc_lidar_monitor.cpp
//
//  PLC Lidar Obstacle Monitor — reads front lidar obstacle bit
//  from PLC and publishes to /lidar_obstacle topic.
//
//  ALWAYS:
//    - Connects to PLC at startup
//    - Polls DB1.DBX0.0 (configurable) at poll_rate Hz
//    - Publishes bit value on /lidar_obstacle (Bool) every poll cycle
//      → waypoint_navigator subscribes and ramps speed down/up
//
//  Parameters:
//    plc_ip      : PLC IP address        (default: 192.168.1.21)
//    rack        : PLC rack              (default: 0)
//    slot        : PLC slot              (default: 1)
//    plc_db      : DB number to read     (default: 1)
//    plc_byte    : byte offset in DB     (default: 0)
//    plc_bit     : bit index (0-7)       (default: 0)  → DB1.DBX0.0
//    poll_rate   : polling rate Hz        (default: 10.0)
//
//  Topics published:
//    /lidar_obstacle       (std_msgs/Bool)   — continuous, every poll
//    /lidar_monitor_status (std_msgs/String) — status messages
//
//  Usage:
//    ros2 run amr_pure_navigation plc_lidar_monitor \
//      --ros-args -p plc_ip:=192.168.1.21 -p plc_db:=1 \
//      -p plc_byte:=0 -p plc_bit:=0
// ================================================================

#include "amr_pure_navigation/plc_lidar_monitor.hpp"
#include <chrono>

using namespace std::chrono_literals;

namespace amr_pure_navigation
{

// ─── Constructor ──────────────────────────────────────────────────
PlcLidarMonitor::PlcLidarMonitor(const rclcpp::NodeOptions & options)
: Node("plc_lidar_monitor", options)
{
  declare_parameter<std::string>("plc_ip",    "192.168.1.21");
  declare_parameter<int>        ("rack",      0);
  declare_parameter<int>        ("slot",      1);
  declare_parameter<int>        ("plc_db",    1);
  declare_parameter<int>        ("plc_byte",  0);
  declare_parameter<int>        ("plc_bit",   0);
  declare_parameter<double>     ("poll_rate", 10.0);

  get_parameter("plc_ip",    plc_ip_);
  get_parameter("rack",      rack_);
  get_parameter("slot",      slot_);
  get_parameter("plc_db",    plc_db_);
  get_parameter("plc_byte",  plc_byte_);
  get_parameter("plc_bit",   plc_bit_);
  get_parameter("poll_rate", poll_rate_hz_);

  RCLCPP_INFO(get_logger(), "=============================================");
  RCLCPP_INFO(get_logger(), "  PLC Lidar Obstacle Monitor");
  RCLCPP_INFO(get_logger(), "  PLC        : %s  rack=%d slot=%d",
    plc_ip_.c_str(), rack_, slot_);
  RCLCPP_INFO(get_logger(), "  Monitor    : DB%d.DBX%d.%d",
    plc_db_, plc_byte_, plc_bit_);
  RCLCPP_INFO(get_logger(), "  Poll rate  : %.1f Hz", poll_rate_hz_);
  RCLCPP_INFO(get_logger(), "=============================================");
  RCLCPP_INFO(get_logger(), "  Topics:");
  RCLCPP_INFO(get_logger(), "    /lidar_obstacle        — continuous Bool");
  RCLCPP_INFO(get_logger(), "    /lidar_monitor_status  — status messages");
  RCLCPP_INFO(get_logger(), "=============================================");

  // Publishers
  lidar_pub_  = create_publisher<std_msgs::msg::Bool>  ("/lidar_obstacle",        10);
  status_pub_ = create_publisher<std_msgs::msg::String>("/lidar_monitor_status",  10);

  // Polling timer — runs ALWAYS at poll_rate_hz_
  int period_ms = static_cast<int>(1000.0 / poll_rate_hz_);
  poll_timer_ = create_wall_timer(
    std::chrono::milliseconds(period_ms),
    std::bind(&PlcLidarMonitor::pollLoop, this));

  // Create snap7 client and connect
  client_ = Cli_Create();
  if (client_ == 0) {
    RCLCPP_FATAL(get_logger(), "Failed to create snap7 client!");
    return;
  }
  connectPLC();
}

// ─── Destructor ───────────────────────────────────────────────────
PlcLidarMonitor::~PlcLidarMonitor()
{
  disconnectPLC();
  if (client_ != 0) {
    Cli_Destroy(&client_);
    client_ = 0;
  }
}

// ─── Connect to PLC ──────────────────────────────────────────────
bool PlcLidarMonitor::connectPLC()
{
  std::lock_guard<std::mutex> lock(s7_mutex_);
  if (client_ == 0) return false;
  if (connected_) {
    Cli_Disconnect(client_);
    connected_ = false;
  }
  int rc = Cli_ConnectTo(client_, plc_ip_.c_str(), rack_, slot_);
  if (rc != 0) {
    RCLCPP_ERROR(get_logger(), "PLC connect failed: %s", errorText(rc).c_str());
    return false;
  }
  connected_ = true;
  RCLCPP_INFO(get_logger(), "✅ Connected to PLC at %s", plc_ip_.c_str());

  std_msgs::msg::String status;
  status.data = "CONNECTED";
  status_pub_->publish(status);

  return true;
}

void PlcLidarMonitor::disconnectPLC()
{
  std::lock_guard<std::mutex> lock(s7_mutex_);
  if (client_ != 0 && connected_) {
    Cli_Disconnect(client_);
    connected_ = false;
  }
}

bool PlcLidarMonitor::isConnected()
{
  if (client_ == 0 || !connected_) return false;
  int state = 0;
  return (Cli_GetConnected(client_, &state) == 0 && state != 0);
}

// ─── Read a single bit from a DB ─────────────────────────────────
bool PlcLidarMonitor::readBit(int db, int byte_offset, int bit_index, bool & value)
{
  if (!connected_ || client_ == 0) return false;
  uint8_t buf[1] = {0};
  {
    std::lock_guard<std::mutex> lock(s7_mutex_);
    int rc = Cli_DBRead(client_, db, byte_offset, 1, buf);
    if (rc != 0) {
      RCLCPP_ERROR(get_logger(), "Read DB%d.DBX%d.%d FAILED: %s",
        db, byte_offset, bit_index, errorText(rc).c_str());
      connected_ = false;
      return false;
    }
  }
  value = ((buf[0] >> bit_index) & 0x01) != 0;
  return true;
}

// ─── Poll Loop — runs ALWAYS at poll_rate_hz_ ────────────────────
//
// Reads DB1.DBX0.0 and publishes the value on /lidar_obstacle (Bool).
// waypoint_navigator subscribes and uses this to ramp speed down/up.
//
void PlcLidarMonitor::pollLoop()
{
  // Try reconnect if disconnected
  if (!isConnected()) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
      "PLC disconnected — attempting reconnect...");
    connectPLC();

    // Publish FALSE while disconnected
    std_msgs::msg::Bool msg;
    msg.data = false;
    lidar_pub_->publish(msg);
    return;
  }

  // Read the bit and publish on /lidar_obstacle
  bool bit_value = false;
  if (!readBit(plc_db_, plc_byte_, plc_bit_, bit_value)) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
      "⚠ Read DB%d.DBX%d.%d failed — will retry",
      plc_db_, plc_byte_, plc_bit_);

    // Publish FALSE on read failure
    std_msgs::msg::Bool msg;
    msg.data = false;
    lidar_pub_->publish(msg);
    return;
  }

  // PLC lidar logic is INVERTED:
  //   PLC bit TRUE  = path CLEAR (no obstacle)
  //   PLC bit FALSE = OBSTACLE DETECTED
  // We invert so /lidar_obstacle topic: TRUE = obstacle, FALSE = clear
  bool obstacle = !bit_value;

  std_msgs::msg::Bool msg;
  msg.data = obstacle;
  lidar_pub_->publish(msg);

  // Log on change of state
  if (bit_value != last_bit_value_) {
    RCLCPP_INFO(get_logger(),
      "🔄 DB%d.DBX%d.%d changed: %s → %s  [LIDAR %s]",
      plc_db_, plc_byte_, plc_bit_,
      last_bit_value_ ? "TRUE" : "FALSE",
      bit_value ? "TRUE" : "FALSE",
      obstacle ? "OBSTACLE DETECTED" : "CLEAR");
    last_bit_value_ = bit_value;

    std_msgs::msg::String status;
    status.data = obstacle ? "OBSTACLE_DETECTED" : "OBSTACLE_CLEARED";
    status_pub_->publish(status);
  }
}

// ─── Error text helper ───────────────────────────────────────────
std::string PlcLidarMonitor::errorText(int code)
{
  char buf[256] = {0};
  Cli_ErrorText(code, buf, sizeof(buf));
  return std::string(buf);
}

}  // namespace amr_pure_navigation

// ─── main ─────────────────────────────────────────────────────────
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<amr_pure_navigation::PlcLidarMonitor>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
