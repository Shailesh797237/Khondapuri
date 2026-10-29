// ================================================================
//  plc_station_monitor.cpp
//
//  PLC monitor — CONTINUOUS bit publishing + station trigger.
//
//  ALWAYS:
//    - Connects to PLC at startup
//    - Polls DB1.DBX0.4 (configurable) at poll_rate Hz
//    - Publishes bit value on /plc_trigger (Bool) every poll cycle
//      → any node can subscribe and see live PLC state
//
//  STATION FLOW:
//    1. waypoint_navigator reaches a station → publishes /station_reached
//    2. This node enters "monitoring" mode for that station
//    3. When /plc_trigger bit = TRUE → publishes /plc_done → robot continues
//
//  Parameters:
//    plc_ip      : PLC IP address        (default: 192.168.1.21)
//    rack        : PLC rack              (default: 0)
//    slot        : PLC slot              (default: 1)
//    plc_db      : DB number to monitor  (default: 1)
//    plc_byte    : byte offset in DB     (default: 0)
//    plc_bit     : bit index (0-7)       (default: 4)
//    poll_rate   : polling rate Hz        (default: 10.0)
//
//  Topics published:
//    /plc_trigger  (std_msgs/Bool)   — continuous, every poll cycle
//    /plc_done     (std_msgs/String) — once per station, when bit=TRUE
//    /plc_status   (std_msgs/String) — status messages
//
//  Usage:
//    ros2 run amr_pure_navigation plc_station_monitor \
//      --ros-args -p plc_ip:=192.168.1.21 -p plc_db:=1 \
//      -p plc_byte:=0 -p plc_bit:=4
// ================================================================

#include "amr_pure_navigation/plc_station_monitor.hpp"
#include <chrono>

using namespace std::chrono_literals;

namespace amr_pure_navigation
{

// ─── Constructor ──────────────────────────────────────────────────
PlcStationMonitor::PlcStationMonitor(const rclcpp::NodeOptions & options)
: Node("plc_station_monitor", options)
{
  declare_parameter<std::string>("plc_ip",    "192.168.1.21");
  declare_parameter<int>        ("rack",      0);
  declare_parameter<int>        ("slot",      1);
  declare_parameter<int>        ("plc_db",    1);
  declare_parameter<int>        ("plc_byte",  0);
  declare_parameter<int>        ("plc_bit",   4);
  declare_parameter<double>     ("poll_rate", 10.0);

  get_parameter("plc_ip",    plc_ip_);
  get_parameter("rack",      rack_);
  get_parameter("slot",      slot_);
  get_parameter("plc_db",    plc_db_);
  get_parameter("plc_byte",  plc_byte_);
  get_parameter("plc_bit",   plc_bit_);
  get_parameter("poll_rate", poll_rate_hz_);

  RCLCPP_INFO(get_logger(), "=============================================");
  RCLCPP_INFO(get_logger(), "  PLC Station Monitor — CONTINUOUS mode");
  RCLCPP_INFO(get_logger(), "  PLC        : %s  rack=%d slot=%d",
    plc_ip_.c_str(), rack_, slot_);
  RCLCPP_INFO(get_logger(), "  Monitor    : DB%d.DBX%d.%d",
    plc_db_, plc_byte_, plc_bit_);
  RCLCPP_INFO(get_logger(), "  Poll rate  : %.1f Hz", poll_rate_hz_);
  RCLCPP_INFO(get_logger(), "=============================================");
  RCLCPP_INFO(get_logger(), "  Topics:");
  RCLCPP_INFO(get_logger(), "    /plc_trigger  — continuous Bool (every poll)");
  RCLCPP_INFO(get_logger(), "    /plc_done     — when station bit = TRUE");
  RCLCPP_INFO(get_logger(), "    /plc_status   — status messages");
  RCLCPP_INFO(get_logger(), "  Flow:");
  RCLCPP_INFO(get_logger(), "    /station_reached → wait for bit=TRUE → /plc_done");
  RCLCPP_INFO(get_logger(), "=============================================");

  // Publishers
  done_pub_    = create_publisher<std_msgs::msg::String>("/plc_done",    10);
  status_pub_  = create_publisher<std_msgs::msg::String>("/plc_status",  10);
  trigger_pub_ = create_publisher<std_msgs::msg::Bool>  ("/plc_trigger", 10);

  // Subscribe to station_reached from waypoint_navigator
  station_sub_ = create_subscription<std_msgs::msg::String>(
    "/station_reached", 10,
    std::bind(&PlcStationMonitor::stationCallback, this, std::placeholders::_1));

  // Polling timer — runs ALWAYS, not just when monitoring
  int period_ms = static_cast<int>(1000.0 / poll_rate_hz_);
  poll_timer_ = create_wall_timer(
    std::chrono::milliseconds(period_ms),
    std::bind(&PlcStationMonitor::pollLoop, this));

  // Create snap7 client and connect
  client_ = Cli_Create();
  if (client_ == 0) {
    RCLCPP_FATAL(get_logger(), "Failed to create snap7 client!");
    return;
  }
  connectPLC();
}

// ─── Destructor ───────────────────────────────────────────────────
PlcStationMonitor::~PlcStationMonitor()
{
  disconnectPLC();
  if (client_ != 0) {
    Cli_Destroy(&client_);
    client_ = 0;
  }
}

// ─── Connect to PLC ──────────────────────────────────────────────
bool PlcStationMonitor::connectPLC()
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
  return true;
}

void PlcStationMonitor::disconnectPLC()
{
  std::lock_guard<std::mutex> lock(s7_mutex_);
  if (client_ != 0 && connected_) {
    Cli_Disconnect(client_);
    connected_ = false;
  }
}

bool PlcStationMonitor::isConnected()
{
  if (client_ == 0 || !connected_) return false;
  int state = 0;
  return (Cli_GetConnected(client_, &state) == 0 && state != 0);
}

// ─── Read a single bit from a DB ─────────────────────────────────
bool PlcStationMonitor::readBit(int db, int byte_offset, int bit_index, bool & value)
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

// ─── Station Callback ────────────────────────────────────────────
// Called when waypoint_navigator reaches a station and publishes
// the station name on /station_reached
void PlcStationMonitor::stationCallback(const std_msgs::msg::String::SharedPtr msg)
{
  // Parse station name and feedback input (format: "STATION_A:I0.4" or "STATION_A")
  std::string raw = msg->data;
  std::string name = raw;
  std::string fb = "";
  auto colon = raw.find(':');
  if (colon != std::string::npos) {
    name = raw.substr(0, colon);
    fb   = raw.substr(colon + 1);
  }

  // Parse feedback input if provided, e.g. "I0.4" -> byte 0, bit 4
  if (!fb.empty() && (fb[0] == 'I' || fb[0] == 'i')) {
    auto dot = fb.find('.');
    if (dot != std::string::npos) {
      try {
        int byte_num = std::stoi(fb.substr(1, dot - 1));
        int bit_num  = std::stoi(fb.substr(dot + 1));
        if (bit_num >= 0 && bit_num <= 7 && byte_num >= 0) {
          plc_byte_ = byte_num;
          plc_bit_  = bit_num;
          RCLCPP_INFO(get_logger(),
            "🎯 Configured monitor from plc_feedback '%s' → DB%d.DBX%d.%d",
            fb.c_str(), plc_db_, plc_byte_, plc_bit_);
        }
      } catch (const std::exception & e) {
        RCLCPP_WARN(get_logger(),
          "Failed to parse plc_feedback '%s': %s (using default DB%d.DBX%d.%d)",
          fb.c_str(), e.what(), plc_db_, plc_byte_, plc_bit_);
      }
    }
  }

  RCLCPP_INFO(get_logger(),
    "📍 Station [%s] reached (feedback=%s) — waiting for DB%d.DBX%d.%d = TRUE to publish /plc_done",
    name.c_str(), fb.empty() ? "default" : fb.c_str(), plc_db_, plc_byte_, plc_bit_);

  current_station_ = name;
  monitoring_      = true;

  std_msgs::msg::String status;
  status.data = "MONITORING:" + name + ":DB" + std::to_string(plc_db_)
              + ".DBX" + std::to_string(plc_byte_) + "." + std::to_string(plc_bit_);
  status_pub_->publish(status);
}

// ─── Poll Loop — runs ALWAYS at poll_rate_hz_ ────────────────────
//
// ALWAYS:
//   Reads DB1.DBX0.4 and publishes the value on /plc_trigger (Bool)
//   This allows any node to subscribe and see the live PLC state.
//
// WHEN MONITORING (station reached):
//   Also checks if bit = TRUE → publish /plc_done → stop monitoring.
//
void PlcStationMonitor::pollLoop()
{
  // Try reconnect if disconnected
  if (!isConnected()) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
      "PLC disconnected — attempting reconnect...");
    connectPLC();

    // Publish FALSE while disconnected so subscribers know
    std_msgs::msg::Bool trig;
    trig.data = false;
    trigger_pub_->publish(trig);
    return;
  }

  // ── ALWAYS: Read the bit and publish on /plc_trigger ──────────
  bool bit_value = false;
  if (!readBit(plc_db_, plc_byte_, plc_bit_, bit_value)) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
      "⚠ Read DB%d.DBX%d.%d failed — will retry",
      plc_db_, plc_byte_, plc_bit_);

    // Publish FALSE on read failure
    std_msgs::msg::Bool trig;
    trig.data = false;
    trigger_pub_->publish(trig);
    return;
  }

  // Publish the bit value continuously on /plc_trigger
  std_msgs::msg::Bool trig;
  trig.data = bit_value;
  trigger_pub_->publish(trig);

  // Log on change of state
  if (bit_value != last_bit_value_) {
    RCLCPP_INFO(get_logger(),
      "🔄 DB%d.DBX%d.%d changed: %s → %s",
      plc_db_, plc_byte_, plc_bit_,
      last_bit_value_ ? "TRUE" : "FALSE",
      bit_value ? "TRUE" : "FALSE");
    last_bit_value_ = bit_value;
  }

  // ── STATION MONITORING: check if bit=TRUE to send /plc_done ───
  if (monitoring_) {
    if (bit_value) {
      // BIT IS TRUE → PLC says "go ahead"
      RCLCPP_INFO(get_logger(),
        "✅ [%s] DB%d.DBX%d.%d = TRUE → publishing /plc_done",
        current_station_.c_str(), plc_db_, plc_byte_, plc_bit_);

      std_msgs::msg::String done;
      done.data = current_station_;
      done_pub_->publish(done);

      monitoring_ = false;
      current_station_ = "";

      std_msgs::msg::String status;
      status.data = "DONE";
      status_pub_->publish(status);
    } else {
      // Still waiting at station
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
        "⏳ [%s] DB%d.DBX%d.%d = FALSE — waiting for trigger...",
        current_station_.c_str(), plc_db_, plc_byte_, plc_bit_);
    }
  }
}

// ─── Error text helper ───────────────────────────────────────────
std::string PlcStationMonitor::errorText(int code)
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
  auto node = std::make_shared<amr_pure_navigation::PlcStationMonitor>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
