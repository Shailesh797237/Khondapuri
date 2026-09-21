/// try this 

#include "my_robot_hardware/mobile_base_hardware.hpp"

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

#include <std_msgs/msg/string.hpp>

#include <unistd.h>
#include <cmath>
#include <algorithm>
#include <sstream>

namespace mobile_base_hardware
{

// ===========================================================
//  MoonsMotorDiagNode — publish-only, NO timer, NO thread
// ===========================================================

MoonsMotorDiagNode::MoonsMotorDiagNode()
: Node("moons_motor_diagnostics")
{
  pub_ = this->create_publisher<std_msgs::msg::String>(
    "/moons_motor_diagnostics", 10);

  RCLCPP_INFO(this->get_logger(),
    "Moons motor diagnostics publisher created on /moons_motor_diagnostics");
}

void MoonsMotorDiagNode::publishDiagnostics(
  std::shared_ptr<MoonsMotorEnable> driver,
  int left_slave,
  int right_slave)
{
  if (!driver) return;

  // ---- Read diagnostic registers for LEFT motor ----
  // No extra delays needed — readRegister16/readError32 have internal turnaround
  uint32_t left_err40001      = driver->readError32(left_slave);               // registers 40001+40002
  int16_t  left_actual_vel    = driver->readRegister16(left_slave, 40017);     // Actual Velocity
  int16_t  left_target_vel    = driver->readRegister16(left_slave, 40018);     // Target Velocity
  int16_t  left_drive_temp    = driver->readRegister16(left_slave, 40019);     // Drive Temp
  int16_t  left_current       = driver->readRegister16(left_slave, 40031);     // Current
  int16_t  left_motor_voltage = driver->readRegister16(left_slave, 40022);     // Motor Voltage
  int16_t  left_motor_temp    = driver->readRegister16(left_slave, 40020);     // Motor Temp

  // ---- Read diagnostic registers for RIGHT motor ----
  uint32_t right_err40001      = driver->readError32(right_slave);              // registers 40001+40002
  int16_t  right_actual_vel    = driver->readRegister16(right_slave, 40017);    // Actual Velocity
  int16_t  right_target_vel    = driver->readRegister16(right_slave, 40018);    // Target Velocity
  int16_t  right_drive_temp    = driver->readRegister16(right_slave, 40019);    // Drive Temp
  int16_t  right_current       = driver->readRegister16(right_slave, 40031);    // Current
  int16_t  right_motor_voltage = driver->readRegister16(right_slave, 40022);    // Motor Voltage
  int16_t  right_motor_temp    = driver->readRegister16(right_slave, 40020);    // Motor Temp

  // ---- Build compact single-line message (avoids ros2 topic echo truncation) ----
  std::ostringstream oss;
  oss << "L_Error=0x" << std::hex << std::uppercase << left_err40001 << std::dec
      << ",actual_vel=" << left_actual_vel
      << ",target_vel=" << left_target_vel
      << ",drive_temp=" << left_drive_temp
      << ",current=" << left_current
      << ",motor_voltage=" << left_motor_voltage
      << ",motor_temp=" << left_motor_temp
      << " | R_Error=0x" << std::hex << std::uppercase << right_err40001 << std::dec
      << ",actual_vel=" << right_actual_vel
      << ",target_vel=" << right_target_vel
      << ",drive_temp=" << right_drive_temp
      << ",current=" << right_current
      << ",motor_voltage=" << right_motor_voltage
      << ",motor_temp=" << right_motor_temp;

  auto msg = std_msgs::msg::String();
  msg.data = oss.str();
  pub_->publish(msg);
}

// ===========================================================

hardware_interface::CallbackReturn
MobileBaseHardwareInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  info_ = info;

  if (info_.hardware_parameters.find("pulses_per_revolution") ==
      info_.hardware_parameters.end())
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("MobileBaseHardwareInterface"),
      "pulses_per_revolution parameter NOT set!");
    return hardware_interface::CallbackReturn::ERROR;
  }

  pulses_per_revolution_ =
    std::stod(info_.hardware_parameters.at("pulses_per_revolution"));

  port_      = "/dev/ttyUSB0";
  baudrate_  = 115200;
  left_slave_  = 1;
  right_slave_ = 2;
  max_rpm_ = 24000;

  left_pos_ = right_pos_ = 0.0;
  left_vel_ = right_vel_ = 0.0;
  left_cmd_ = right_cmd_ = 0.0;

  prev_left_encoder_ = 0;
  prev_right_encoder_ = 0;
 
  driver_ = std::make_shared<MoonsMotorEnable>(
    port_, baudrate_,
    left_slave_, right_slave_,
    40125, 40343, 40011);

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
MobileBaseHardwareInterface::on_configure(
  const rclcpp_lifecycle::State &)
{
  if (!driver_ || !driver_->connect())
    return hardware_interface::CallbackReturn::ERROR;

  // ===== CONNECT TO PLC (snap7) FOR STO DOUBLE CONFIRMATION =====
  plc_client_ = Cli_Create();
  if (plc_client_ == 0) {
    RCLCPP_ERROR(rclcpp::get_logger("MobileBaseHardwareInterface"),
      "Failed to create snap7 client!");
  } else {
    connectPLC();
  }
  // ===============================================================

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
MobileBaseHardwareInterface::on_activate(
  const rclcpp_lifecycle::State &)
{
  driver_->enableMotors();
  usleep(500000);
  driver_->enableJogMode();
  usleep(200000);

  prev_left_encoder_  = driver_->readEncoder(left_slave_);
  prev_right_encoder_ = driver_->readEncoder(right_slave_);

  left_pos_ = right_pos_ = 0.0;

  // ===== START DIAGNOSTICS PUBLISHER (no thread — runs in read()) =====
  diag_node_ = std::make_shared<MoonsMotorDiagNode>();
  diag_cycle_counter_ = 0;

  RCLCPP_INFO(rclcpp::get_logger("MobileBaseHardwareInterface"),
    "Motor diagnostics publisher started (topic: /moons_motor_diagnostics)");
  // ====================================================================

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
MobileBaseHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  // ===== STOP DIAGNOSTICS PUBLISHER =====
  diag_node_.reset();

  RCLCPP_INFO(rclcpp::get_logger("MobileBaseHardwareInterface"),
    "Motor diagnostics publisher stopped.");
  // ======================================

  // ===== DISCONNECT PLC =====
  disconnectPLC();
  // ==========================

  driver_->disableMotors();
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
MobileBaseHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  // state_interfaces.emplace_back(
  //   "base_left_wheel_joint",
  //   hardware_interface::HW_IF_POSITION,
  //   &left_pos_);

  // state_interfaces.emplace_back(
  //   "base_left_wheel_joint",
  //   hardware_interface::HW_IF_VELOCITY,
  //   &left_vel_);

  // state_interfaces.emplace_back(
  //   "base_right_wheel_joint",
  //   hardware_interface::HW_IF_POSITION,
  //   &right_pos_);

  // state_interfaces.emplace_back(
  //   "base_right_wheel_joint",
  //   hardware_interface::HW_IF_VELOCITY,
  //   &right_vel_);

  // state_interfaces.emplace_back(
  //   "base_left_wheel_joint",
  //   hardware_interface::HW_IF_POSITION,
  //   &left_pos_);
  state_interfaces.emplace_back(
    "leftwheel",
    hardware_interface::HW_IF_POSITION,
    &left_pos_);

  state_interfaces.emplace_back(
    "leftwheel",
    hardware_interface::HW_IF_VELOCITY,
    &left_vel_);

  state_interfaces.emplace_back(
    "rightwheel",
    hardware_interface::HW_IF_POSITION,
    &right_pos_);

  state_interfaces.emplace_back(
    "rightwheel",
    hardware_interface::HW_IF_VELOCITY,
    &right_vel_);  

  // state_interfaces.emplace_back(
  //   "rightwheel",
  //   hardware_interface::HW_IF_POSITION
  //   &right_vel_); 
  
  // state_interfaces.emplace_back(
  //   "rightwheel",
  //   hardware_interface::HW_IF_POSITION,
  //   &right_vel_);   


  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
MobileBaseHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  // command_interfaces.emplace_back(
  //   "base_left_wheel_joint",
  //   hardware_interface::HW_IF_VELOCITY,
  //   &left_cmd_);

  // command_interfaces.emplace_back(
  //   "base_right_wheel_joint",
  //   hardware_interface::HW_IF_VELOCITY,
  //   &right_cmd_);

   command_interfaces.emplace_back(
    "leftwheel",
    hardware_interface::HW_IF_VELOCITY,
    &left_cmd_);

  command_interfaces.emplace_back(
    "rightwheel",
    hardware_interface::HW_IF_VELOCITY,
    &right_cmd_);  

  return command_interfaces;
}

hardware_interface::return_type
MobileBaseHardwareInterface::read(
  const rclcpp::Time &, const rclcpp::Duration & period)
{
  // ===== PERIODIC DIAGNOSTICS — runs ALWAYS (even during error/STO) every ~2s =====
  diag_cycle_counter_++;
  if (diag_node_ && diag_cycle_counter_ >= DIAG_PUBLISH_EVERY_N_CYCLES)
  {
    diag_cycle_counter_ = 0;
    diag_node_->publishDiagnostics(driver_, left_slave_, right_slave_);
  }

  // ===== FAST ENCODER READ — 2 registers per slave, every cycle =====
  int32_t left_enc  = driver_->readEncoder(left_slave_);
  int32_t right_enc = driver_->readEncoder(right_slave_);

  // ===== ERROR CHECK (registers 40001+40002, 32-bit) — every 5th cycle =====
  // Separate small reads are much faster than the 12-register bulk read
  bool check_error = (++error_check_counter_ >= ERROR_CHECK_EVERY_N);
  if (check_error) error_check_counter_ = 0;

  // Also always check errors if we're already in error state
  if (check_error || motors_disabled_)
  {
    uint32_t left_error  = driver_->readError32(left_slave_);
    uint32_t right_error = driver_->readError32(right_slave_);

    if (left_error != 0 || right_error != 0)
    {
      RCLCPP_WARN(rclcpp::get_logger("MobileBaseHardwareInterface"),
                  "Error detected! Left=0x%08X Right=0x%08X",
                  left_error, right_error);

      left_vel_  = 0.0;
      right_vel_ = 0.0;
      motors_disabled_ = true;

      // Extract register 40001 (upper 16 bits) for STO error code check
      uint16_t left_err_40001  = static_cast<uint16_t>(left_error >> 16);
      uint16_t right_err_40001 = static_cast<uint16_t>(right_error >> 16);
      if (left_err_40001 == 2 || right_err_40001 == 2)
      {
        prev_error_was_2_ = true;
      }

      return hardware_interface::return_type::OK;
    }

    // ===== ERROR CLEARED: 2→0 transition (DOUBLE CONFIRMATION via PLC) =====
    //
    // Flow:
    //   1. Register 40001 went from 2 → 0
    //   2. Read PLC DB1.DBX1.5 (STO confirmation bit from PLC)
    //   3. If DB1.DBX1.5 is SET:
    //      a. Zero speed + 2s wait
    //      b. Enable motors 3x (double-confirmed)
    //      c. Send one-shot trigger to PLC DB3.DBX1.0
    //      d. Reset PLC DB1.DBX1.5 back to 0
    //      e. Jog mode + encoder reset + resume
    //   4. If DB1.DBX1.5 is NOT set:
    //      - STO not confirmed by PLC, skip recovery (stay disabled)
    //
    if (prev_error_was_2_)
    {
      // ---- Throttle PLC polling: only check every STO_POLL_EVERY_N cycles ----
      // At ~33 Hz, 66 cycles ≈ 2 seconds between PLC reads
      sto_poll_counter_++;
      if (sto_poll_counter_ < STO_POLL_EVERY_N) {
        return hardware_interface::return_type::OK;  // skip this cycle
      }
      sto_poll_counter_ = 0;

      RCLCPP_INFO(rclcpp::get_logger("MobileBaseHardwareInterface"),
                  "Error cleared (40001 = 0). Checking PLC DB1.DBX1.5 for STO confirmation...");

      // ---- Step 0: Read PLC DB1.DBX1.5 for double confirmation ----
      bool sto_confirmed = false;
      if (!plcReadBit(1, 1, 5, sto_confirmed)) {
        RCLCPP_WARN(rclcpp::get_logger("MobileBaseHardwareInterface"),
                    "Failed to read PLC DB1.DBX1.5 — skipping recovery (will retry next poll)");
        return hardware_interface::return_type::OK;
      }

      if (!sto_confirmed) {
        RCLCPP_WARN(rclcpp::get_logger("MobileBaseHardwareInterface"),
                    "PLC DB1.DBX1.5 is NOT set — STO not confirmed by PLC. Waiting...");
        return hardware_interface::return_type::OK;
      }

      RCLCPP_INFO(rclcpp::get_logger("MobileBaseHardwareInterface"),
                  "✅ PLC DB1.DBX1.5 is SET — STO confirmed! Proceeding with motor recovery...");

      // ---- Step 1: Enable motors (single call) ----
      RCLCPP_INFO(rclcpp::get_logger("MobileBaseHardwareInterface"),
                  "Sending motor enable on register 40125 (PLC confirmed)...");
      driver_->enableMotors();
      RCLCPP_INFO(rclcpp::get_logger("MobileBaseHardwareInterface"),
                  "Motor enable sent");

      // ---- Step 4: One-shot pulse to PLC DB3.DBX1.0 (SET → wait → CLEAR) ----
      RCLCPP_INFO(rclcpp::get_logger("MobileBaseHardwareInterface"),
                  "Sending one-shot pulse to PLC DB3.DBX1.0...");
      if (plcWriteBit(3, 1, 0, true)) {
        RCLCPP_INFO(rclcpp::get_logger("MobileBaseHardwareInterface"),
                    "✅ PLC DB3.DBX1.0 set HIGH — waiting 200ms before clearing...");
        usleep(200000);  // 200ms pulse width
        if (plcWriteBit(3, 1, 0, false)) {
          RCLCPP_INFO(rclcpp::get_logger("MobileBaseHardwareInterface"),
                      "✅ PLC DB3.DBX1.0 cleared LOW — one-shot pulse complete");
        } else {
          RCLCPP_ERROR(rclcpp::get_logger("MobileBaseHardwareInterface"),
                      "❌ Failed to clear PLC DB3.DBX1.0 back to LOW!");
        }
      } else {
        RCLCPP_ERROR(rclcpp::get_logger("MobileBaseHardwareInterface"),
                    "❌ Failed to write PLC DB3.DBX1.0 trigger!");
      }

      // ---- Step 5: Reset PLC DB1.DBX1.5 (clear STO confirmation) ----
      RCLCPP_INFO(rclcpp::get_logger("MobileBaseHardwareInterface"),
                  "Resetting PLC DB1.DBX1.5 (clearing STO confirmation)...");
      if (plcWriteBit(1, 1, 5, false)) {
        RCLCPP_INFO(rclcpp::get_logger("MobileBaseHardwareInterface"),
                    "✅ PLC DB1.DBX1.5 reset to 0");
      } else {
        RCLCPP_ERROR(rclcpp::get_logger("MobileBaseHardwareInterface"),
                    "❌ Failed to reset PLC DB1.DBX1.5!");
      }

      // ---- Step 6: Jog mode ----
      usleep(100000);
      driver_->enableJogMode();
      usleep(50000);

      // ---- Step 6.5: Zero speed + 1s cooldown ----
      RCLCPP_INFO(rclcpp::get_logger("MobileBaseHardwareInterface"),
                  "Initializing jog mode at zero speed. Waiting 1 second cooldown...");
      driver_->setJogSpeed(0, 0);
      usleep(1000000);

      // ---- Step 7: Reset encoder baseline ----
      prev_left_encoder_  = driver_->readEncoder(left_slave_);
      prev_right_encoder_ = driver_->readEncoder(right_slave_);

      RCLCPP_INFO(rclcpp::get_logger("MobileBaseHardwareInterface"),
                  "Motor re-enabled (PLC confirmed). Encoder baseline reset: L=%d R=%d",
                  prev_left_encoder_, prev_right_encoder_);

      // ---- Step 8: Resume ----
      prev_final_left_rpm_  = 0;
      prev_final_right_rpm_ = 0;
      motors_disabled_ = false;
      prev_error_was_2_ = false;
      sto_poll_counter_ = 0;

      return hardware_interface::return_type::OK;
    }
  }

  // ===== COMPUTE ODOMETRY (normal path) =====
  int32_t delta_left  = left_enc  - prev_left_encoder_;
  int32_t delta_right = right_enc - prev_right_encoder_;

  prev_left_encoder_  = left_enc;
  prev_right_encoder_ = right_enc;

  double rad_per_pulse = 2.0 * M_PI / pulses_per_revolution_;

  double delta_left_rad  = delta_left  * rad_per_pulse;
  double delta_right_rad = delta_right * rad_per_pulse;

  left_pos_  += delta_left_rad;
  right_pos_ += delta_right_rad;

  double dt = period.seconds();
  if (dt > 0.0001)
  {
    left_vel_  = delta_left_rad  / dt;
    right_vel_ = delta_right_rad / dt;
  }
  else
  {
    left_vel_ = right_vel_ = 0.0;
  }

  // (Diagnostics now run at the top of read(), before any early returns)

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type
MobileBaseHardwareInterface::write(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  // ===== FORCE ZERO SPEED during STO / recovery =====
  // When motors_disabled_ is true, ensure drive is at zero speed and don't write repeatedly
  if (motors_disabled_)
  {
    if (prev_final_left_rpm_ != 0 || prev_final_right_rpm_ != 0)
    {
      driver_->setJogSpeed(0, 0);
      prev_final_left_rpm_  = 0;
      prev_final_right_rpm_ = 0;
    }
    return hardware_interface::return_type::OK;
  }

  int left_rpm  = radPerSecToRpm(left_cmd_);
  int right_rpm = radPerSecToRpm(right_cmd_);

  // Clamp to max RPM
  left_rpm  = std::clamp(left_rpm,  -max_rpm_, max_rpm_);
  right_rpm = std::clamp(right_rpm, -max_rpm_, max_rpm_);

  // ---- NEW CONVERSION ----
  double left_rps  = static_cast<double>(left_rpm) / 60.0;
  double right_rps = static_cast<double>(right_rpm) / 60.0;

  int final_left_rpm  = static_cast<int>(std::round(left_rps * 240.0));
  int final_right_rpm = static_cast<int>(std::round(right_rps * 240.0));
  // ------------------------

  // ===== SKIP REDUNDANT WRITES — saves 2 Modbus transactions (~30ms) =====
  // Moons controller holds the last jog speed, no need to resend if unchanged
  if (final_left_rpm != prev_final_left_rpm_ || final_right_rpm != prev_final_right_rpm_)
  {
    if (!driver_->setJogSpeed(final_left_rpm, final_right_rpm)) {
      RCLCPP_ERROR(rclcpp::get_logger("MobileBaseHardwareInterface"),
                   "setJogSpeed FAILED! Command not acknowledged by motor (L=%d R=%d)",
                   final_left_rpm, final_right_rpm);
    }
    prev_final_left_rpm_  = final_left_rpm;
    prev_final_right_rpm_ = final_right_rpm;
  }

  return hardware_interface::return_type::OK;
}

int MobileBaseHardwareInterface::radPerSecToRpm(double rad_per_sec)
{
  double motor_rad_per_sec = rad_per_sec * gear_ratio_;
  return static_cast<int>(motor_rad_per_sec * 60.0 / (2.0 * M_PI));
}

// ===================================================================
//  PLC S7 helpers — STO double confirmation
// ===================================================================

bool MobileBaseHardwareInterface::connectPLC()
{
  std::lock_guard<std::mutex> lock(plc_mutex_);
  if (plc_client_ == 0) return false;
  if (plc_connected_) {
    Cli_Disconnect(plc_client_);
    plc_connected_ = false;
  }
  int rc = Cli_ConnectTo(plc_client_, plc_ip_.c_str(), plc_rack_, plc_slot_);
  if (rc != 0) {
    char buf[256] = {0};
    Cli_ErrorText(rc, buf, sizeof(buf));
    RCLCPP_ERROR(rclcpp::get_logger("MobileBaseHardwareInterface"),
      "PLC S7 connect FAILED: %s", buf);
    return false;
  }
  plc_connected_ = true;
  RCLCPP_INFO(rclcpp::get_logger("MobileBaseHardwareInterface"),
    "✅ Connected to PLC S7 at %s (STO double confirmation)", plc_ip_.c_str());
  return true;
}

void MobileBaseHardwareInterface::disconnectPLC()
{
  std::lock_guard<std::mutex> lock(plc_mutex_);
  if (plc_client_ != 0 && plc_connected_) {
    Cli_Disconnect(plc_client_);
    plc_connected_ = false;
    RCLCPP_INFO(rclcpp::get_logger("MobileBaseHardwareInterface"),
      "PLC S7 disconnected.");
  }
  if (plc_client_ != 0) {
    Cli_Destroy(&plc_client_);
    plc_client_ = 0;
  }
}

bool MobileBaseHardwareInterface::plcReadBit(int db, int byte_offset, int bit_index, bool & value)
{
  if (!plc_connected_ || plc_client_ == 0) {
    // Try reconnect
    RCLCPP_WARN(rclcpp::get_logger("MobileBaseHardwareInterface"),
      "PLC not connected — attempting reconnect for read DB%d.DBX%d.%d...",
      db, byte_offset, bit_index);
    if (!connectPLC()) return false;
  }

  uint8_t buf[1] = {0};
  {
    std::lock_guard<std::mutex> lock(plc_mutex_);
    int rc = Cli_DBRead(plc_client_, db, byte_offset, 1, buf);
    if (rc != 0) {
      char errbuf[256] = {0};
      Cli_ErrorText(rc, errbuf, sizeof(errbuf));
      RCLCPP_ERROR(rclcpp::get_logger("MobileBaseHardwareInterface"),
        "PLC read DB%d.DBX%d.%d FAILED: %s", db, byte_offset, bit_index, errbuf);
      plc_connected_ = false;
      return false;
    }
  }
  value = ((buf[0] >> bit_index) & 0x01) != 0;
  return true;
}

bool MobileBaseHardwareInterface::plcWriteBit(int db, int byte_offset, int bit_index, bool value)
{
  if (!plc_connected_ || plc_client_ == 0) {
    RCLCPP_WARN(rclcpp::get_logger("MobileBaseHardwareInterface"),
      "PLC not connected — attempting reconnect for write DB%d.DBX%d.%d...",
      db, byte_offset, bit_index);
    if (!connectPLC()) return false;
  }

  // Read-modify-write: read the byte, set/clear the bit, write back
  uint8_t buf[1] = {0};
  {
    std::lock_guard<std::mutex> lock(plc_mutex_);

    // Read current byte
    int rc = Cli_DBRead(plc_client_, db, byte_offset, 1, buf);
    if (rc != 0) {
      char errbuf[256] = {0};
      Cli_ErrorText(rc, errbuf, sizeof(errbuf));
      RCLCPP_ERROR(rclcpp::get_logger("MobileBaseHardwareInterface"),
        "PLC read (for write) DB%d byte %d FAILED: %s", db, byte_offset, errbuf);
      plc_connected_ = false;
      return false;
    }

    // Modify the bit
    if (value)
      buf[0] |= (1 << bit_index);    // set bit
    else
      buf[0] &= ~(1 << bit_index);   // clear bit

    // Write back
    rc = Cli_DBWrite(plc_client_, db, byte_offset, 1, buf);
    if (rc != 0) {
      char errbuf[256] = {0};
      Cli_ErrorText(rc, errbuf, sizeof(errbuf));
      RCLCPP_ERROR(rclcpp::get_logger("MobileBaseHardwareInterface"),
        "PLC write DB%d.DBX%d.%d FAILED: %s", db, byte_offset, bit_index, errbuf);
      plc_connected_ = false;
      return false;
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("MobileBaseHardwareInterface"),
    "PLC write DB%d.DBX%d.%d = %s OK",
    db, byte_offset, bit_index, value ? "TRUE" : "FALSE");
  return true;
}

}  

PLUGINLIB_EXPORT_CLASS(
  mobile_base_hardware::MobileBaseHardwareInterface,
  hardware_interface::SystemInterface)





// #include "my_robot_hardware/mobile_base_hardware.hpp"

// #include "hardware_interface/types/hardware_interface_type_values.hpp"
// #include "pluginlib/class_list_macros.hpp"
// #include "rclcpp/rclcpp.hpp"

// #include <unistd.h>
// #include <cmath>
// #include <algorithm>

// namespace mobile_base_hardware
// {

// hardware_interface::CallbackReturn
// MobileBaseHardwareInterface::on_init(
//   const hardware_interface::HardwareInfo & info)
// {
//   if (hardware_interface::SystemInterface::on_init(info) !=
//       hardware_interface::CallbackReturn::SUCCESS)
//   {
//     return hardware_interface::CallbackReturn::ERROR;
//   }

//   info_ = info;

//   if (info_.hardware_parameters.find("pulses_per_revolution") ==
//       info_.hardware_parameters.end())
//   {
//     RCLCPP_ERROR(
//       rclcpp::get_logger("MobileBaseHardwareInterface"),
//       "pulses_per_revolution parameter NOT set!");
//     return hardware_interface::CallbackReturn::ERROR;
//   }

//   pulses_per_revolution_ =
//     std::stod(info_.hardware_parameters.at("pulses_per_revolution"));

//   port_      = "/dev/ttyUSB0";
//   baudrate_  = 9600;
//   left_slave_  = 1;
//   right_slave_ = 2;
//   max_rpm_ = 24000;

//   left_pos_ = right_pos_ = 0.0;
//   left_vel_ = right_vel_ = 0.0;
//   left_cmd_ = right_cmd_ = 0.0;

//   prev_left_encoder_ = 0;
//   prev_right_encoder_ = 0;

//   driver_ = std::make_shared<MoonsMotorEnable>(
//     port_, baudrate_,
//     left_slave_, right_slave_,
//     40125, 40343, 40011);

//   // ✅ ADDED: cmd_vel subscription
//   cmd_vel_sub_ = get_node()->create_subscription<geometry_msgs::msg::Twist>(
//     "/diff_drive_controller/cmd_vel", 10,
//     std::bind(&MobileBaseHardwareInterface::cmdVelCallback, this, std::placeholders::_1));

//   return hardware_interface::CallbackReturn::SUCCESS;
// }

// hardware_interface::CallbackReturn
// MobileBaseHardwareInterface::on_configure(
//   const rclcpp_lifecycle::State &)
// {
//   if (!driver_ || !driver_->connect())
//     return hardware_interface::CallbackReturn::ERROR;

//   return hardware_interface::CallbackReturn::SUCCESS;
// }

// hardware_interface::CallbackReturn
// MobileBaseHardwareInterface::on_activate(
//   const rclcpp_lifecycle::State &)
// {
//   driver_->enableMotors();
//   usleep(500000);
//   driver_->enableJogMode();
//   usleep(200000);

//   prev_left_encoder_  = driver_->readEncoder(left_slave_);
//   prev_right_encoder_ = driver_->readEncoder(right_slave_);

//   left_pos_ = right_pos_ = 0.0;

//   return hardware_interface::CallbackReturn::SUCCESS;
// }

// hardware_interface::CallbackReturn
// MobileBaseHardwareInterface::on_deactivate(
//   const rclcpp_lifecycle::State &)
// {
//   driver_->disableMotors();
//   return hardware_interface::CallbackReturn::SUCCESS;
// }

// std::vector<hardware_interface::StateInterface>
// MobileBaseHardwareInterface::export_state_interfaces()
// {
//   std::vector<hardware_interface::StateInterface> state_interfaces;

//   state_interfaces.emplace_back(
//     "base_left_wheel_joint",
//     hardware_interface::HW_IF_POSITION,
//     &left_pos_);

//   state_interfaces.emplace_back(
//     "base_left_wheel_joint",
//     hardware_interface::HW_IF_VELOCITY,
//     &left_vel_);

//   state_interfaces.emplace_back(
//     "base_right_wheel_joint",
//     hardware_interface::HW_IF_POSITION,
//     &right_pos_);

//   state_interfaces.emplace_back(
//     "base_right_wheel_joint",
//     hardware_interface::HW_IF_VELOCITY,
//     &right_vel_);

//   return state_interfaces;
// }

// std::vector<hardware_interface::CommandInterface>
// MobileBaseHardwareInterface::export_command_interfaces()
// {
//   std::vector<hardware_interface::CommandInterface> command_interfaces;

//   command_interfaces.emplace_back(
//     "base_left_wheel_joint",
//     hardware_interface::HW_IF_VELOCITY,
//     &left_cmd_);

//   command_interfaces.emplace_back(
//     "base_right_wheel_joint",
//     hardware_interface::HW_IF_VELOCITY,
//     &right_cmd_);

//   return command_interfaces;
// }

// // ✅ ADDED: cmd_vel callback
// void MobileBaseHardwareInterface::cmdVelCallback(
//   const geometry_msgs::msg::Twist::SharedPtr msg)
// {
//   desired_linear_ = msg->linear.x;
//   desired_angular_ = msg->angular.z;
// }

// bool MobileBaseHardwareInterface::isValidEncoder(int32_t value)
// {
//   // Example validity check
//   return (value > 0 && value < 1000000000);
// }

// int32_t MobileBaseHardwareInterface::normalizeDelta(int32_t delta)
// {
//   const int32_t MAX = 2147483647;
//   const int32_t MIN = -2147483648;

//   if (delta > MAX/2)  delta -= MAX;
//   if (delta < MIN/2)  delta += MAX;
//   return delta;
// }

// hardware_interface::return_type
// MobileBaseHardwareInterface::read(
//   const rclcpp::Time &, const rclcpp::Duration & period)
// {
//   int32_t left_enc  = driver_->readEncoder(left_slave_);
//   int32_t right_enc = driver_->readEncoder(right_slave_);

//   int32_t delta_left = 0;
//   int32_t delta_right = 0;

//   // ✅ Fix: ignore invalid encoder values
//   if (isValidEncoder(left_enc))
//   {
//     delta_left = left_enc - prev_left_encoder_;
//     prev_left_encoder_ = left_enc;
//   }

//   if (isValidEncoder(right_enc))
//   {
//     delta_right = right_enc - prev_right_encoder_;
//     prev_right_encoder_ = right_enc;
//   }

//   // ✅ Fix: handle encoder overflow
//   delta_left  = normalizeDelta(delta_left);
//   delta_right = normalizeDelta(delta_right);

//   double rad_per_pulse = 2.0 * M_PI / pulses_per_revolution_;

//   // ✅ Fix: invert right wheel sign (because encoder sign is reversed)
//   double delta_left_rad  = delta_left  * rad_per_pulse;
//   double delta_right_rad = -delta_right * rad_per_pulse;

//   left_pos_  += delta_left_rad;
//   right_pos_ += delta_right_rad;

//   double dt = period.seconds();
//   if (dt > 0.0001)
//   {
//     left_vel_  = delta_left_rad  / dt;
//     right_vel_ = delta_right_rad / dt;
//   }
//   else
//   {
//     left_vel_ = right_vel_ = 0.0;
//   }

//   RCLCPP_INFO(rclcpp::get_logger("MobileBaseHardwareInterface"),
//     "Encoders L=%d R=%d | ΔL=%d ΔR=%d | PosL=%.3f PosR=%.3f",
//     left_enc, right_enc, delta_left, delta_right,
//     left_pos_, right_pos_);

//   return hardware_interface::return_type::OK;
// }

// hardware_interface::return_type
// MobileBaseHardwareInterface::write(
//   const rclcpp::Time &, const rclcpp::Duration &)
// {
//   // ✅ ADDED: convert cmd_vel -> wheel velocity
//   double v = desired_linear_;
//   double w = desired_angular_;

//   double v_left  = v - (wheel_separation_ / 2.0) * w;
//   double v_right = v + (wheel_separation_ / 2.0) * w;

//   left_cmd_  = v_left  / wheel_radius_;
//   right_cmd_ = v_right / wheel_radius_;

//   int left_rpm  = radPerSecToRpm(left_cmd_);
//   int right_rpm = radPerSecToRpm(right_cmd_);

//   left_rpm  = std::clamp(left_rpm,  -max_rpm_, max_rpm_);
//   right_rpm = std::clamp(right_rpm, -max_rpm_, max_rpm_);

//   driver_->setJogSpeed(left_rpm, right_rpm);

//   return hardware_interface::return_type::OK;
// }

// int MobileBaseHardwareInterface::radPerSecToRpm(double rad_per_sec)
// {
//   return static_cast<int>(rad_per_sec * 60.0 / (2.0 * M_PI));
// }

// }  

// PLUGINLIB_EXPORT_CLASS(
//   mobile_base_hardware::MobileBaseHardwareInterface,
//   hardware_interface::SystemInterface)





// #include "my_robot_hardware/mobile_base_hardware.hpp"

// #include "hardware_interface/types/hardware_interface_type_values.hpp"
// #include "pluginlib/class_list_macros.hpp"
// #include "rclcpp/rclcpp.hpp"

// #include <unistd.h>
// #include <cmath>
// #include <algorithm>

// namespace mobile_base_hardware
// {

// hardware_interface::CallbackReturn
// MobileBaseHardwareInterface::on_init(
//   const hardware_interface::HardwareInfo & info)
// {
//   if (hardware_interface::SystemInterface::on_init(info) !=
//       hardware_interface::CallbackReturn::SUCCESS)
//   {
//     return hardware_interface::CallbackReturn::ERROR;
//   }

//   info_ = info;

//   if (info_.hardware_parameters.find("pulses_per_revolution") ==
//       info_.hardware_parameters.end())
//   {
//     RCLCPP_ERROR(
//       rclcpp::get_logger("MobileBaseHardwareInterface"),
//       "pulses_per_revolution parameter NOT set!");
//     return hardware_interface::CallbackReturn::ERROR;
//   }

//   pulses_per_revolution_ =
//     std::stod(info_.hardware_parameters.at("pulses_per_revolution"));

//   port_      = "/dev/ttyUSB0";
//   baudrate_  = 9600;
//   left_slave_  = 1;
//   right_slave_ = 2;
//   max_rpm_ = 24000;

//   left_pos_ = right_pos_ = 0.0;
//   left_vel_ = right_vel_ = 0.0;
//   left_cmd_ = right_cmd_ = 0.0;

//   prev_left_encoder_ = 0;
//   prev_right_encoder_ = 0;

//   driver_ = std::make_shared<MoonsMotorEnable>(
//     port_, baudrate_,
//     left_slave_, right_slave_,
//     40125, 40343, 40011);

//   return hardware_interface::CallbackReturn::SUCCESS;
// }

// hardware_interface::CallbackReturn
// MobileBaseHardwareInterface::on_configure(
//   const rclcpp_lifecycle::State &)
// {
//   if (!driver_ || !driver_->connect())
//     return hardware_interface::CallbackReturn::ERROR;

//   return hardware_interface::CallbackReturn::SUCCESS;
// }

// hardware_interface::CallbackReturn
// MobileBaseHardwareInterface::on_activate(
//   const rclcpp_lifecycle::State &)
// {
//   driver_->enableMotors();
//   usleep(500000);
//   driver_->enableJogMode();
//   usleep(200000);

//   prev_left_encoder_  = driver_->readEncoder(left_slave_);
//   prev_right_encoder_ = driver_->readEncoder(right_slave_);

//   left_pos_ = right_pos_ = 0.0;

//   return hardware_interface::CallbackReturn::SUCCESS;
// }

// hardware_interface::CallbackReturn
// MobileBaseHardwareInterface::on_deactivate(
//   const rclcpp_lifecycle::State &)
// {
//   driver_->disableMotors();
//   return hardware_interface::CallbackReturn::SUCCESS;
// }

// std::vector<hardware_interface::StateInterface>
// MobileBaseHardwareInterface::export_state_interfaces()
// {
//   std::vector<hardware_interface::StateInterface> state_interfaces;

//   state_interfaces.emplace_back(
//     "base_left_wheel_joint",
//     hardware_interface::HW_IF_POSITION,
//     &left_pos_);

//   state_interfaces.emplace_back(
//     "base_left_wheel_joint",
//     hardware_interface::HW_IF_VELOCITY,
//     &left_vel_);

//   state_interfaces.emplace_back(
//     "base_right_wheel_joint",
//     hardware_interface::HW_IF_POSITION,
//     &right_pos_);

//   state_interfaces.emplace_back(
//     "base_right_wheel_joint",
//     hardware_interface::HW_IF_VELOCITY,
//     &right_vel_);

//   return state_interfaces;
// }

// std::vector<hardware_interface::CommandInterface>
// MobileBaseHardwareInterface::export_command_interfaces()
// {
//   std::vector<hardware_interface::CommandInterface> command_interfaces;

//   command_interfaces.emplace_back(
//     "base_left_wheel_joint",
//     hardware_interface::HW_IF_VELOCITY,
//     &left_cmd_);

//   command_interfaces.emplace_back(
//     "base_right_wheel_joint",
//     hardware_interface::HW_IF_VELOCITY,
//     &right_cmd_);

//   return command_interfaces;
// }

// hardware_interface::return_type
// MobileBaseHardwareInterface::read(
//   const rclcpp::Time &, const rclcpp::Duration & period)
// {
//   int32_t left_enc  = driver_->readEncoder(left_slave_);
//   int32_t right_enc = driver_->readEncoder(right_slave_);

//   int32_t delta_left  = left_enc  - prev_left_encoder_;
//   int32_t delta_right = right_enc - prev_right_encoder_;

//   prev_left_encoder_  = left_enc;
//   prev_right_encoder_ = right_enc;

//   double rad_per_pulse = 2.0 * M_PI / pulses_per_revolution_;

//   double delta_left_rad  = delta_left  * rad_per_pulse;
//   double delta_right_rad = delta_right * rad_per_pulse;

//   left_pos_  += delta_left_rad;
//   right_pos_ += delta_right_rad;

//   double dt = period.seconds();
//   if (dt > 0.0001)
//   {
//     left_vel_  = delta_left_rad  / dt;
//     right_vel_ = delta_right_rad / dt;
//   }
//   else
//   {
//     left_vel_ = right_vel_ = 0.0;
//   }

//   auto logger = rclcpp::get_logger("MobileBaseHardwareInterface");
//   RCLCPP_INFO(logger,
//     "Encoders: left=%d right=%d | delta: left=%d right=%d | vel: left=%.5f right=%.5f",
//     left_enc, right_enc, delta_left, delta_right, left_vel_, right_vel_);

//   return hardware_interface::return_type::OK;
// }

// hardware_interface::return_type
// MobileBaseHardwareInterface::write(
//   const rclcpp::Time &, const rclcpp::Duration &)
// {
//   int left_rpm  = radPerSecToRpm(left_cmd_);
//   int right_rpm = radPerSecToRpm(right_cmd_);

//   left_rpm  = std::clamp(left_rpm,  -max_rpm_, max_rpm_);
//   right_rpm = std::clamp(right_rpm, -max_rpm_, max_rpm_);

//   driver_->setJogSpeed(left_rpm, right_rpm);

//   return hardware_interface::return_type::OK;
// }

// int MobileBaseHardwareInterface::radPerSecToRpm(double rad_per_sec)
// {
//   return static_cast<int>(rad_per_sec * 60.0 / (2.0 * M_PI));
// }

// }  

// PLUGINLIB_EXPORT_CLASS(
//   mobile_base_hardware::MobileBaseHardwareInterface,
//   hardware_interface::SystemInterface)



// #include "my_robot_hardware/mobile_base_hardware.hpp"

// #include "hardware_interface/types/hardware_interface_type_values.hpp"
// #include "pluginlib/class_list_macros.hpp"
// #include "rclcpp/rclcpp.hpp"

// #include <unistd.h>
// #include <cmath>

// namespace mobile_base_hardware
// {

// hardware_interface::CallbackReturn
// MobileBaseHardwareInterface::on_init(
//   const hardware_interface::HardwareInfo & info)
// {
//   if (hardware_interface::SystemInterface::on_init(info) !=
//       hardware_interface::CallbackReturn::SUCCESS)
//   {
//     return hardware_interface::CallbackReturn::ERROR;
//   }

//   info_ = info;

//   port_        = "/dev/ttyUSB0";
//   baudrate     = 9600;
//   left_slave   = 1;
//   right_slave  = 2;

//   max_rpm_     = 24000;

//   left_pos_  = 0.0;
//   right_pos_ = 0.0;
//   left_vel_  = 0.0;
//   right_vel_ = 0.0;
//   left_cmd_  = 0.0;
//   right_cmd_ = 0.0;

//   prev_left_encoder_  = 0;
//   prev_right_encoder_ = 0;

//   driver_ = std::make_shared<MoonsMotorEnable>(
//     port_,
//     baudrate,
//     left_slave,
//     right_slave,
//     40125,
//     40343,
//     40011
//   );

//   return hardware_interface::CallbackReturn::SUCCESS;
// }

// hardware_interface::CallbackReturn
// MobileBaseHardwareInterface::on_configure(
//   const rclcpp_lifecycle::State &)
// {
//   auto logger = rclcpp::get_logger("MobileBaseHardwareInterface");

//   if (!driver_ || !driver_->connect())
//   {
//     RCLCPP_ERROR(logger, "Failed to connect to motor driver");
//     return hardware_interface::CallbackReturn::ERROR;
//   }

//   return hardware_interface::CallbackReturn::SUCCESS;
// }

// hardware_interface::CallbackReturn
// MobileBaseHardwareInterface::on_activate(
//   const rclcpp_lifecycle::State &)
// {
//   if (!driver_) {
//     return hardware_interface::CallbackReturn::ERROR;
//   }

//   driver_->enableMotors();
//   usleep(500000);
//   driver_->enableJogMode();
//   usleep(200000);

//   // 🔥 IMPORTANT: Reset encoder baseline
//   prev_left_encoder_  = driver_->readEncoder(left_slave);
//   prev_right_encoder_ = driver_->readEncoder(right_slave);

//   left_pos_  = 0.0;
//   right_pos_ = 0.0;

//   return hardware_interface::CallbackReturn::SUCCESS;
// }

// hardware_interface::CallbackReturn
// MobileBaseHardwareInterface::on_deactivate(
//   const rclcpp_lifecycle::State &)
// {
//   if (driver_) {
//     driver_->disableMotors();
//   }

//   return hardware_interface::CallbackReturn::SUCCESS;
// }

// std::vector<hardware_interface::StateInterface>
// MobileBaseHardwareInterface::export_state_interfaces()
// {
//   std::vector<hardware_interface::StateInterface> state_interfaces;

//   state_interfaces.emplace_back(
//     "base_left_wheel_joint",
//     hardware_interface::HW_IF_POSITION,
//     &left_pos_);

//   state_interfaces.emplace_back(
//     "base_left_wheel_joint",
//     hardware_interface::HW_IF_VELOCITY,
//     &left_vel_);

//   state_interfaces.emplace_back(
//     "base_right_wheel_joint",
//     hardware_interface::HW_IF_POSITION,
//     &right_pos_);

//   state_interfaces.emplace_back(
//     "base_right_wheel_joint",
//     hardware_interface::HW_IF_VELOCITY,
//     &right_vel_);

//   return state_interfaces;
// }

// std::vector<hardware_interface::CommandInterface>
// MobileBaseHardwareInterface::export_command_interfaces()
// {
//   std::vector<hardware_interface::CommandInterface> command_interfaces;

//   command_interfaces.emplace_back(
//     "base_left_wheel_joint",
//     hardware_interface::HW_IF_VELOCITY,
//     &left_cmd_);

//   command_interfaces.emplace_back(
//     "base_right_wheel_joint",
//     hardware_interface::HW_IF_VELOCITY,
//     &right_cmd_);

//   return command_interfaces;
// }

// hardware_interface::return_type
// MobileBaseHardwareInterface::read(
//   const rclcpp::Time &, const rclcpp::Duration & period)
// {
//   if (!driver_) {
//     return hardware_interface::return_type::ERROR;
//   }

//   int32_t left_enc  = driver_->readEncoder(left_slave);
//   int32_t right_enc = driver_->readEncoder(right_slave);

//   int32_t delta_left  = left_enc  - prev_left_encoder_;
//   int32_t delta_right = right_enc - prev_right_encoder_;

//   // Mirror correction
//   delta_right = -delta_right;

//   // 🔥 Remove 1–2 pulse noise
//   if (std::abs(delta_left) < 2)  delta_left = 0;
//   if (std::abs(delta_right) < 2) delta_right = 0;

//   prev_left_encoder_  = left_enc;
//   prev_right_encoder_ = right_enc;

//   constexpr double RAD_PER_PULSE = 2.0 * M_PI / 100000.0;

//   double delta_left_rad  = delta_left  * RAD_PER_PULSE;
//   double delta_right_rad = delta_right * RAD_PER_PULSE;

//   left_pos_  += delta_left_rad;
//   right_pos_ += delta_right_rad;

//   double dt = period.seconds();

//   if (dt > 0.0001)
//   {
//     left_vel_  = delta_left_rad  / dt;
//     right_vel_ = delta_right_rad / dt;
//   }
//   else
//   {
//     left_vel_  = 0.0;
//     right_vel_ = 0.0;
//   }

//   auto logger = rclcpp::get_logger("MobileBaseHardwareInterface");
//   RCLCPP_INFO(logger,
//     "Encoders: left=%d right=%d | delta: left=%d right=%d | vel: left=%.5f right=%.5f",
//     left_enc, right_enc, delta_left, delta_right, left_vel_, right_vel_);

//   return hardware_interface::return_type::OK;
// }

// hardware_interface::return_type
// MobileBaseHardwareInterface::write(
//   const rclcpp::Time &, const rclcpp::Duration &)
// {
//   if (!driver_) {
//     return hardware_interface::return_type::ERROR;
//   }

//   double right_cmd_inverted = -right_cmd_;

//   int left_rpm  = radPerSecToRpm(left_cmd_);
//   int right_rpm = radPerSecToRpm(right_cmd_inverted);

//   if (left_rpm > max_rpm_) left_rpm = max_rpm_;
//   if (left_rpm < -max_rpm_) left_rpm = -max_rpm_;

//   if (right_rpm > max_rpm_) right_rpm = max_rpm_;
//   if (right_rpm < -max_rpm_) right_rpm = -max_rpm_;

//   driver_->setJogSpeed(left_rpm, right_rpm);

//   return hardware_interface::return_type::OK;
// }

// double MobileBaseHardwareInterface::rpmToRadPerSec(int32_t rpm)
// {
//   return static_cast<double>(rpm) * 2.0 * M_PI / 60.0;
// }

// int MobileBaseHardwareInterface::radPerSecToRpm(double rad_per_sec)
// {
//   return static_cast<int>(rad_per_sec * 60.0 / (2.0 * M_PI));
// }

// }  // namespace mobile_base_hardware

// PLUGINLIB_EXPORT_CLASS(
//   mobile_base_hardware::MobileBaseHardwareInterface,
//   hardware_interface::SystemInterface
// )


