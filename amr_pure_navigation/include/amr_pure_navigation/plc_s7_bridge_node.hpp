#pragma once
// ================================================================
//  plc_s7_bridge_node.hpp  v4
//  All build errors fixed:
//    - S7Object = unsigned long → init with 0 not nullptr
//    - extern "C" include on separate line
// ================================================================

#include <string>
#include <map>
#include <vector>
#include <mutex>
#include <atomic>
#include <fstream>
#include <sstream>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <yaml-cpp/yaml.h>

// ── snap7 C header — must be on its own line inside extern "C" ───
#ifdef __cplusplus
extern "C" {
#endif

#include "snap7.h"

#ifdef __cplusplus
}
#endif

namespace plc_s7_bridge
{

struct StationAction {
  std::string station_name;
  std::string input_i      {""};
  bool        wait_confirm {false};
};

class PlcS7BridgeNode : public rclcpp::Node
{
public:
  explicit PlcS7BridgeNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~PlcS7BridgeNode();

private:
  void stationCallback (const std_msgs::msg::String::SharedPtr msg);
  void manualCallback  (const std_msgs::msg::String::SharedPtr msg);
  void diagnosticLoop  ();

  bool connectPLC      ();
  void disconnectPLC   ();
  bool isConnected     ();

  bool writeCommandWord   (int value);
  bool readCommandWord    (int & value);
  bool readFeedbackBit    (int bit_index, bool & value);
  bool readAllFeedbackBits(uint8_t & byte_val);

  void handleStation   (const StationAction & action);
  bool loadConfig      (const std::string & path);
  void publishStatus   (const std::string & text);
  std::string errorText(int code);

  // ROS
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr station_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr manual_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr    done_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr    status_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr    io_pub_;
  rclcpp::TimerBase::SharedPtr                           diag_timer_;

  // ── FIX: S7Object is unsigned long — use 0 not nullptr ────────
  S7Object          client_    {0};
  std::mutex        s7_mutex_;
  std::atomic<bool> connected_ {false};

  // Maps
  std::map<std::string, StationAction> station_map_;
  std::map<std::string, int>           command_map_;
  std::map<std::string, int>           input_map_;

  // Hardcoded DB layout
  int    db_number_         {46};
  int    command_offset_    {0};
  int    feedback_offset_   {0};
  double confirm_timeout_s_ {30.0};
  double write_zero_sec_    {5.0};
  double reconnect_sec_     {5.0};

  // Connection params
  std::string plc_ip_           {"192.168.1.21"};
  int         rack_             {0};
  int         slot_             {1};
  std::string station_map_file_ {""};
};

}  // namespace plc_s7_bridge






// #pragma once
// // ================================================================
// //  plc_s7_bridge_node.hpp  v4
// //  All build errors fixed:
// //    - S7Object = unsigned long → init with 0 not nullptr
// //    - extern "C" include on separate line
// // ================================================================

// #include <string>
// #include <map>
// #include <vector>
// #include <mutex>
// #include <atomic>
// #include <fstream>
// #include <sstream>

// #include "rclcpp/rclcpp.hpp"
// #include "std_msgs/msg/string.hpp"
// #include <yaml-cpp/yaml.h>

// // ── snap7 C header — must be on its own line inside extern "C" ───
// #ifdef __cplusplus
// extern "C" {
// #endif

// #include "snap7.h"

// #ifdef __cplusplus
// }
// #endif

// namespace plc_s7_bridge
// {

// struct StationAction {
//   std::string station_name;
//   std::string input_i      {""};
//   bool        wait_confirm {false};
// };

// class PlcS7BridgeNode : public rclcpp::Node
// {
// public:
//   explicit PlcS7BridgeNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
//   ~PlcS7BridgeNode();

// private:
//   void stationCallback (const std_msgs::msg::String::SharedPtr msg);
//   void manualCallback  (const std_msgs::msg::String::SharedPtr msg);
//   void diagnosticLoop  ();

//   bool connectPLC      ();
//   void disconnectPLC   ();
//   bool isConnected     ();

//   bool writeCommandWord   (int value);
//   bool readCommandWord    (int & value);
//   bool readFeedbackBit    (int bit_index, bool & value);
//   bool readAllFeedbackBits(uint8_t & byte_val);

//   void handleStation   (const StationAction & action);
//   bool loadConfig      (const std::string & path);
//   void publishStatus   (const std::string & text);
//   std::string errorText(int code);

//   // ROS
//   rclcpp::Subscription<std_msgs::msg::String>::SharedPtr station_sub_;
//   rclcpp::Subscription<std_msgs::msg::String>::SharedPtr manual_sub_;
//   rclcpp::Publisher<std_msgs::msg::String>::SharedPtr    done_pub_;
//   rclcpp::Publisher<std_msgs::msg::String>::SharedPtr    status_pub_;
//   rclcpp::Publisher<std_msgs::msg::String>::SharedPtr    io_pub_;
//   rclcpp::TimerBase::SharedPtr                           diag_timer_;

//   // ── FIX: S7Object is unsigned long — use 0 not nullptr ────────
//   S7Object          client_    {0};
//   std::mutex        s7_mutex_;
//   std::atomic<bool> connected_ {false};

//   // Maps
//   std::map<std::string, StationAction> station_map_;
//   std::map<std::string, int>           command_map_;
//   std::map<std::string, int>           input_map_;

//   // Hardcoded DB layout
//   int    db_number_         {21};
//   int    command_offset_    {4};
//   int    feedback_offset_   {6};
//   double confirm_timeout_s_ {30.0};
//   double write_zero_sec_    {5.0};
//   double reconnect_sec_     {5.0};

//   // Connection params
//   std::string plc_ip_           {"192.168.1.51"};
//   int         rack_             {0};
//   int         slot_             {1};
//   std::string station_map_file_ {""};
// };

// }  // namespace plc_s7_bridge
