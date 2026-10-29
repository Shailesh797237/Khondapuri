#pragma once
// ================================================================
//  plc_lidar_monitor.hpp
//
//  PLC Lidar Obstacle Monitor — CONTINUOUS bit publishing.
//
//  Reads DB1.DBX1.5 (front lidar obstacle signal from PLC) at
//  poll_rate Hz and publishes the value on /lidar_obstacle (Bool).
//
//  When the front lidar detects an obstacle, the PLC sets DB1.DBX1.5
//  to TRUE. The waypoint_navigator subscribes and ramps speed down.
//
//  Parameters:
//    plc_ip      : PLC IP address        (default: 192.168.1.21)
//    rack        : PLC rack              (default: 0)
//    slot        : PLC slot              (default: 1)
//    plc_db      : DB number to read     (default: 1)
//    plc_byte    : byte offset in DB     (default: 1)
//    plc_bit     : bit index (0-7)       (default: 5)  → DB1.DBX1.5
//    poll_rate   : polling rate Hz        (default: 10.0)
// ================================================================

#include <string>
#include <mutex>
#include <atomic>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"

#ifdef __cplusplus
extern "C" {
#endif

#include "snap7.h"

#ifdef __cplusplus
}
#endif

namespace amr_pure_navigation
{

class PlcLidarMonitor : public rclcpp::Node
{
public:
  explicit PlcLidarMonitor(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~PlcLidarMonitor();

private:
  void pollLoop();

  bool connectPLC();
  void disconnectPLC();
  bool isConnected();
  bool readBit(int db, int byte_offset, int bit_index, bool & value);
  std::string errorText(int code);

  // ROS interfaces
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr     lidar_pub_;    // /lidar_obstacle
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr   status_pub_;   // /lidar_monitor_status
  rclcpp::TimerBase::SharedPtr                         poll_timer_;

  // snap7
  S7Object          client_    {0};
  std::mutex        s7_mutex_;
  std::atomic<bool> connected_ {false};

  // Last known bit value (for change-of-state logging)
  bool        last_bit_value_   {false};

  // PLC address to monitor  (DB1.DBX1.5)
  std::string plc_ip_       {"192.168.1.21"};
  int         rack_         {0};
  int         slot_         {1};
  int         plc_db_       {1};     // DB number
  int         plc_byte_     {1};     // byte offset
  int         plc_bit_      {5};     // bit index (1.5 = byte 1, bit 5)
  double      poll_rate_hz_ {10.0};  // how often to check (10 Hz)
};

}  // namespace amr_pure_navigation
