#ifndef MOBILE_BASE_HARDWARE_HPP
#define MOBILE_BASE_HARDWARE_HPP

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/handle.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "rclcpp/rclcpp.hpp"

#include "my_robot_hardware/moons_hardware.hpp"

#include <std_msgs/msg/string.hpp>

// ===== snap7 for PLC S7 communication (STO double confirmation) =====
#ifdef __cplusplus
extern "C" {
#endif
#include "snap7.h"
#ifdef __cplusplus
}
#endif
// ====================================================================

#include <vector>
#include <memory>
#include <string>
#include <atomic>

namespace mobile_base_hardware
{

// =========================================================
//  Diagnostic publisher node  (NO timer, NO background thread)
//  Only holds the publisher — read() calls publishDiagnostics()
// =========================================================
class MoonsMotorDiagNode : public rclcpp::Node
{
public:
  MoonsMotorDiagNode();

  /// Called from read() — reads registers AND publishes in the
  /// same thread as the control loop (no bus collision).
  void publishDiagnostics(
    std::shared_ptr<MoonsMotorEnable> driver,
    int left_slave,
    int right_slave);

private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
};

// =========================================================

class MobileBaseHardwareInterface : public hardware_interface::SystemInterface
{
public:
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  int radPerSecToRpm(double rad_per_sec);

  std::shared_ptr<MoonsMotorEnable> driver_;

  std::string port_;
  int baudrate_;
  int left_slave_;
  int right_slave_;
  int max_rpm_;

  double pulses_per_revolution_;

  double left_pos_;
  double right_pos_;
  double left_vel_;
  double right_vel_;
  double left_cmd_;
  double right_cmd_;
  double gear_ratio_ = 10.0; // if 1:10 gearbox

  int32_t prev_left_encoder_;
  int32_t prev_right_encoder_;

  // ===== Diagnostics (runs in read(), no separate thread) =====
  std::shared_ptr<MoonsMotorDiagNode> diag_node_;
  int diag_cycle_counter_ = 0;
  static constexpr int DIAG_PUBLISH_EVERY_N_CYCLES = 66;  // ~every 2s at 33Hz

  // ===== Error monitoring (40001) — check every 5th cycle =====
  int error_check_counter_ = 0;
  static constexpr int ERROR_CHECK_EVERY_N = 5;  // every 5th cycle (~150ms at 33Hz)
  bool prev_error_was_2_ = false;
  bool motors_disabled_ = false;  // true during STO + 2s cooldown — write() sends zero

  // ===== Throttle STO confirmation polling (~2s interval at 33 Hz) =====
  int sto_poll_counter_ = 0;
  static constexpr int STO_POLL_EVERY_N = 33;  // 33 cycles × ~30ms ≈ 1 seconds

  // ===== Skip redundant speed writes =====
  int prev_final_left_rpm_ = 0;
  int prev_final_right_rpm_ = 0;

  // ===== PLC S7 — STO double confirmation =====
  //  Read  DB1.DBX1.5 → STO trigger confirmation from PLC
  //  Write DB3.DBX1.0 → one-shot trigger to PLC (acknowledge)
  //  Reset DB1.DBX1.5 → clear confirmation after processing
  S7Object  plc_client_     {0};
  std::mutex plc_mutex_;
  std::atomic<bool> plc_connected_ {false};

  std::string plc_ip_   {"192.168.1.21"};
  int plc_rack_         {0};
  int plc_slot_         {1};

  bool connectPLC();
  void disconnectPLC();
  bool plcReadBit (int db, int byte_offset, int bit_index, bool & value);
  bool plcWriteBit(int db, int byte_offset, int bit_index, bool value);
  // =============================================
};

}  

#endif

// #ifndef MOBILE_BASE_HARDWARE_HPP
// #define MOBILE_BASE_HARDWARE_HPP

// #include <cstdint>
// #include <memory>
// #include <string>
// #include <vector>

// #include "hardware_interface/system_interface.hpp"
// #include "hardware_interface/types/hardware_interface_return_values.hpp"
// #include "hardware_interface/handle.hpp"
// #include "rclcpp/rclcpp.hpp"
// #include "rclcpp_lifecycle/state.hpp"

// #include "my_robot_hardware/moons_hardware.hpp"

// namespace mobile_base_hardware
// {

// class MobileBaseHardwareInterface : public hardware_interface::SystemInterface
// {
// public:
//   hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
//   hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
//   hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
//   hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

//   std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
//   std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

//   hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
//   hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

// private:
//   std::shared_ptr<MoonsMotorEnable> driver_;

//   int left_slave;
//   int right_slave;
//   std::int32_t baudrate;
//   std::string port_;

//   double left_pos_;
//   double right_pos_;
//   double left_vel_;
//   double right_vel_;

//   double left_cmd_;
//   double right_cmd_;

//   int max_rpm_;

//   int32_t prev_left_encoder_;
//   int32_t prev_right_encoder_;
//   bool first_read_;

//   double rpmToRadPerSec(std::int32_t rpm);
//   int radPerSecToRpm(double rad_per_sec);
// };

// }

// #endif


