#pragma once
// ================================================================
//  plc_station_monitor.hpp
//
//  PLC monitor with CONTINUOUS bit publishing.
//
//  1) ALWAYS polls DB1.DBX0.4 (configurable) at poll_rate Hz
//  2) Publishes the bit value CONTINUOUSLY on /plc_trigger (Bool)
//     — any node can subscribe and see the live PLC state
//  3) When robot reaches a station → /station_reached triggers
//     monitoring mode: waits for bit=TRUE, then publishes /plc_done
//     so waypoint_navigator can advance
//
//  Parameters:
//    plc_ip      : PLC IP address        (default: 192.168.1.21)
//    rack        : PLC rack              (default: 0)
//    slot        : PLC slot              (default: 1)
//    plc_db      : DB number to monitor  (default: 1)
//    plc_byte    : byte offset in DB     (default: 0)
//    plc_bit     : bit index (0-7)       (default: 4)  → DB1.DBX0.4
//    poll_rate   : polling rate Hz        (default: 10.0)
// ================================================================

#include <string>
#include <mutex>
#include <atomic>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"

#ifdef __cplusplus
extern "C" {
#endif

#include "snap7.h"

#ifdef __cplusplus
}
#endif

namespace amr_pure_navigation
{

class PlcStationMonitor : public rclcpp::Node
{
public:
  explicit PlcStationMonitor(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~PlcStationMonitor();

private:
  void stationCallback(const std_msgs::msg::String::SharedPtr msg);
  void pollLoop();

  bool connectPLC();
  void disconnectPLC();
  bool isConnected();
  bool readBit(int db, int byte_offset, int bit_index, bool & value);
  std::string errorText(int code);

  // ROS interfaces
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr station_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr    done_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr    status_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr      trigger_pub_;   // continuous PLC bit
  rclcpp::TimerBase::SharedPtr                          poll_timer_;

  // snap7
  S7Object          client_    {0};
  std::mutex        s7_mutex_;
  std::atomic<bool> connected_ {false};

  // Monitoring state
  bool        monitoring_       {false};    // true = we are at a station, polling PLC
  std::string current_station_  {""};       // name of current station

  // Last known bit value (for change-of-state logging)
  bool        last_bit_value_   {false};

  // PLC address to monitor  (DB1.DBX0.4)
  std::string plc_ip_       {"192.168.1.21"};
  int         rack_         {0};
  int         slot_         {1};
  int         plc_db_       {1};     // DB number
  int         plc_byte_     {0};     // byte offset
  int         plc_bit_      {4};     // bit index (0.4 = byte 0, bit 4)
  double      poll_rate_hz_ {10.0};  // how often to check (10 Hz)
  double      reconnect_s_  {5.0};   // reconnect interval
};

}  // namespace amr_pure_navigation
