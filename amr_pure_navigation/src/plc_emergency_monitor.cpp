// ================================================================
//  plc_emergency_monitor.cpp
//
//  PLC Emergency Button Monitor — reads emergency button bit
//  from PLC and publishes to /emergency_stop topic.
//
//  ALWAYS:
//    - Connects to PLC at startup
//    - Polls DB1.DBX0.5 (configurable) at poll_rate Hz
//    - Publishes bit value on /emergency_stop (Bool) every poll cycle
//      → waypoint_navigator subscribes and stops immediately
//
//  PLC logic (INVERTED):
//    PLC bit TRUE  = Emergency NOT pressed (normal)
//    PLC bit FALSE = Emergency IS pressed (STOP!)
//
//  Published /emergency_stop:
//    TRUE  = Emergency ACTIVE → robot stops in 0.2s
//    FALSE = Normal operation
//
//  Parameters:
//    plc_ip      : PLC IP address        (default: 192.168.1.21)
//    rack        : PLC rack              (default: 0)
//    slot        : PLC slot              (default: 1)
//    plc_db      : DB number to read     (default: 1)
//    plc_byte    : byte offset in DB     (default: 0)
//    plc_bit     : bit index (0-7)       (default: 5)  → DB1.DBX0.5
//    poll_rate   : polling rate Hz        (default: 10.0)
//
//  Topics published:
//    /emergency_stop          (std_msgs/Bool)   — continuous, every poll
//    /emergency_monitor_status (std_msgs/String) — status messages
//
//  Usage:
//    ros2 run amr_pure_navigation plc_emergency_monitor \
//      --ros-args -p plc_ip:=192.168.1.21 -p plc_db:=1 \
//      -p plc_byte:=0 -p plc_bit:=5
// ================================================================

#include "amr_pure_navigation/plc_emergency_monitor.hpp"
#include <chrono>

using namespace std::chrono_literals;

namespace amr_pure_navigation
{

// ─── Constructor ──────────────────────────────────────────────────
PlcEmergencyMonitor::PlcEmergencyMonitor(const rclcpp::NodeOptions & options)
: Node("plc_emergency_monitor", options)
{
  declare_parameter<std::string>("plc_ip",    "192.168.1.21");
  declare_parameter<int>        ("rack",      0);
  declare_parameter<int>        ("slot",      1);
  declare_parameter<int>        ("plc_db",    1);
  declare_parameter<int>        ("plc_byte",  0);
  declare_parameter<int>        ("plc_bit",   5);
  declare_parameter<double>     ("poll_rate", 10.0);

  get_parameter("plc_ip",    plc_ip_);
  get_parameter("rack",      rack_);
  get_parameter("slot",      slot_);
  get_parameter("plc_db",    plc_db_);
  get_parameter("plc_byte",  plc_byte_);
  get_parameter("plc_bit",   plc_bit_);
  get_parameter("poll_rate", poll_rate_hz_);

  RCLCPP_INFO(get_logger(), "=============================================");
  RCLCPP_INFO(get_logger(), "  PLC Emergency Button Monitor");
  RCLCPP_INFO(get_logger(), "  PLC        : %s  rack=%d slot=%d",
    plc_ip_.c_str(), rack_, slot_);
  RCLCPP_INFO(get_logger(), "  Monitor    : DB%d.DBX%d.%d",
    plc_db_, plc_byte_, plc_bit_);
  RCLCPP_INFO(get_logger(), "  Poll rate  : %.1f Hz", poll_rate_hz_);
  RCLCPP_INFO(get_logger(), "=============================================");
  RCLCPP_INFO(get_logger(), "  Topics:");
  RCLCPP_INFO(get_logger(), "    /emergency_stop           — continuous Bool");
  RCLCPP_INFO(get_logger(), "    /emergency_monitor_status — status messages");
  RCLCPP_INFO(get_logger(), "  Logic (INVERTED):");
  RCLCPP_INFO(get_logger(), "    PLC bit TRUE  = Normal (button NOT pressed)");
  RCLCPP_INFO(get_logger(), "    PLC bit FALSE = EMERGENCY (button pressed)");
  RCLCPP_INFO(get_logger(), "=============================================");

  // Publishers
  emergency_pub_ = create_publisher<std_msgs::msg::Bool>  ("/emergency_stop",           10);
  status_pub_    = create_publisher<std_msgs::msg::String>("/emergency_monitor_status",  10);

  // Polling timer — runs ALWAYS at poll_rate_hz_
  int period_ms = static_cast<int>(1000.0 / poll_rate_hz_);
  poll_timer_ = create_wall_timer(
    std::chrono::milliseconds(period_ms),
    std::bind(&PlcEmergencyMonitor::pollLoop, this));

  // Create snap7 client and connect
  client_ = Cli_Create();
  if (client_ == 0) {
    RCLCPP_FATAL(get_logger(), "Failed to create snap7 client!");
    return;
  }
  connectPLC();
}

// ─── Destructor ───────────────────────────────────────────────────
PlcEmergencyMonitor::~PlcEmergencyMonitor()
{
  disconnectPLC();
  if (client_ != 0) {
    Cli_Destroy(&client_);
    client_ = 0;
  }
}

// ─── Connect to PLC ──────────────────────────────────────────────
bool PlcEmergencyMonitor::connectPLC()
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

void PlcEmergencyMonitor::disconnectPLC()
{
  std::lock_guard<std::mutex> lock(s7_mutex_);
  if (client_ != 0 && connected_) {
    Cli_Disconnect(client_);
    connected_ = false;
  }
}

bool PlcEmergencyMonitor::isConnected()
{
  if (client_ == 0 || !connected_) return false;
  int state = 0;
  return (Cli_GetConnected(client_, &state) == 0 && state != 0);
}

// ─── Read a single bit from a DB ─────────────────────────────────
bool PlcEmergencyMonitor::readBit(int db, int byte_offset, int bit_index, bool & value)
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
// Reads DB1.DBX0.5 and publishes on /emergency_stop (Bool).
// waypoint_navigator subscribes → emergency stop in 0.2 sec.
//
void PlcEmergencyMonitor::pollLoop()
{
  // Try reconnect if disconnected
  if (!isConnected()) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
      "PLC disconnected — attempting reconnect...");
    connectPLC();

    // Publish TRUE (emergency) while disconnected for safety
    std_msgs::msg::Bool msg;
    msg.data = true;
    emergency_pub_->publish(msg);
    return;
  }

  // Read the bit and publish on /emergency_stop
  bool bit_value = false;
  if (!readBit(plc_db_, plc_byte_, plc_bit_, bit_value)) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
      "⚠ Read DB%d.DBX%d.%d failed — will retry",
      plc_db_, plc_byte_, plc_bit_);

    // Publish TRUE (emergency) on read failure for safety
    std_msgs::msg::Bool msg;
    msg.data = true;
    emergency_pub_->publish(msg);
    return;
  }

  // PLC emergency logic is INVERTED:
  //   PLC bit TRUE  = Emergency NOT pressed (normal)
  //   PLC bit FALSE = Emergency IS pressed (STOP!)
  // We invert so /emergency_stop topic: TRUE = emergency active, FALSE = normal
  bool emergency = !bit_value;

  std_msgs::msg::Bool msg;
  msg.data = emergency;
  emergency_pub_->publish(msg);

  // Log on change of state
  if (bit_value != last_bit_value_) {
    RCLCPP_INFO(get_logger(),
      "🔄 DB%d.DBX%d.%d changed: %s → %s  [EMERGENCY %s]",
      plc_db_, plc_byte_, plc_bit_,
      last_bit_value_ ? "TRUE" : "FALSE",
      bit_value ? "TRUE" : "FALSE",
      emergency ? "⛔ BUTTON PRESSED" : "✅ RELEASED");
    last_bit_value_ = bit_value;

    std_msgs::msg::String status;
    status.data = emergency ? "EMERGENCY_ACTIVE" : "EMERGENCY_RELEASED";
    status_pub_->publish(status);
  }
}

// ─── Error text helper ───────────────────────────────────────────
std::string PlcEmergencyMonitor::errorText(int code)
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
  auto node = std::make_shared<amr_pure_navigation::PlcEmergencyMonitor>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
