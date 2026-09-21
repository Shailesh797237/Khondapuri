#pragma once
// ================================================================
//  plc_emergency_monitor.hpp
//
//  PLC Emergency Button Monitor — CONTINUOUS bit publishing.
//
//  Reads DB1.DBX0.5 (emergency button) from PLC at poll_rate Hz
//  and publishes the value on /emergency_stop (Bool).
//
//  PLC logic (INVERTED):
//    DB1.DBX0.5 = TRUE  → Emergency NOT pressed (normal operation)
//    DB1.DBX0.5 = FALSE → Emergency IS pressed (STOP!)
//
//  Published topic /emergency_stop:
//    TRUE  = Emergency active → robot must stop
//    FALSE = Normal operation → robot runs normally
//
//  Parameters:
//    plc_ip      : PLC IP address        (default: 192.168.1.21)
//    rack        : PLC rack              (default: 0)
//    slot        : PLC slot              (default: 1)
//    plc_db      : DB number to read     (default: 1)
//    plc_byte    : byte offset in DB     (default: 0)
//    plc_bit     : bit index (0-7)       (default: 5)  → DB1.DBX0.5
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

class PlcEmergencyMonitor : public rclcpp::Node
{
public:
  explicit PlcEmergencyMonitor(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~PlcEmergencyMonitor();

private:
  void pollLoop();

  bool connectPLC();
  void disconnectPLC();
  bool isConnected();
  bool readBit(int db, int byte_offset, int bit_index, bool & value);
  std::string errorText(int code);

  // ROS interfaces
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr     emergency_pub_;   // /emergency_stop
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr   status_pub_;      // /emergency_monitor_status
  rclcpp::TimerBase::SharedPtr                         poll_timer_;

  // snap7
  S7Object          client_    {0};
  std::mutex        s7_mutex_;
  std::atomic<bool> connected_ {false};

  // Last known bit value (for change-of-state logging)
  bool        last_bit_value_   {false};

  // PLC address to monitor  (DB1.DBX0.5)
  std::string plc_ip_       {"192.168.1.21"};
  int         rack_         {0};
  int         slot_         {1};
  int         plc_db_       {1};     // DB number
  int         plc_byte_     {0};     // byte offset
  int         plc_bit_      {5};     // bit index (0.5 = byte 0, bit 5)
  double      poll_rate_hz_ {10.0};  // how often to check (10 Hz)
};

}  // namespace amr_pure_navigation
