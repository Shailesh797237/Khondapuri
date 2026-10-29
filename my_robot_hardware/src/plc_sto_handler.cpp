// ============================================================================
//  PlcStoHandler — PLC STO (Safe Torque Off) monitor & reset implementation
//
//  Bit addresses are configured via StoConfig struct.
//  Default mapping:
//    MONITOR (read):   DB1.DBX3.5
//    WRITE   (pulse):  DB3.DBX1.4
//    RESET:            DB1.DBX3.5 (same as monitor — cleared after processing)
// ============================================================================

#include "my_robot_hardware/plc_sto_handler.hpp"
#include "rclcpp/rclcpp.hpp"

#include <unistd.h>   // usleep

static const char * LOG_TAG = "PlcStoHandler";

// ── Constructor / Destructor ─────────────────────────────────────────────────

PlcStoHandler::PlcStoHandler(const StoConfig & config)
  : config_(config)
{
  plc_client_ = Cli_Create();
  if (plc_client_ == 0) {
    RCLCPP_ERROR(rclcpp::get_logger(LOG_TAG),
      "Failed to create snap7 client!");
  }
}

PlcStoHandler::~PlcStoHandler()
{
  disconnect();
}

// ── Connection ───────────────────────────────────────────────────────────────

bool PlcStoHandler::connect()
{
  std::lock_guard<std::mutex> lock(plc_mutex_);
  if (plc_client_ == 0) return false;

  if (plc_connected_) {
    Cli_Disconnect(plc_client_);
    plc_connected_ = false;
  }

  int rc = Cli_ConnectTo(plc_client_,
                         config_.plc_ip.c_str(),
                         config_.plc_rack,
                         config_.plc_slot);
  if (rc != 0) {
    char buf[256] = {0};
    Cli_ErrorText(rc, buf, sizeof(buf));
    RCLCPP_ERROR(rclcpp::get_logger(LOG_TAG),
      "PLC S7 connect FAILED: %s", buf);
    return false;
  }

  plc_connected_ = true;
  RCLCPP_INFO(rclcpp::get_logger(LOG_TAG),
    "✅ Connected to PLC S7 at %s (rack=%d slot=%d)",
    config_.plc_ip.c_str(), config_.plc_rack, config_.plc_slot);
  RCLCPP_INFO(rclcpp::get_logger(LOG_TAG),
    "   STO Monitor bit: DB%d.DBX%d.%d",
    config_.monitor_db, config_.monitor_byte_offset, config_.monitor_bit_index);
  RCLCPP_INFO(rclcpp::get_logger(LOG_TAG),
    "   STO Write bit:   DB%d.DBX%d.%d",
    config_.write_db, config_.write_byte_offset, config_.write_bit_index);
  return true;
}

void PlcStoHandler::disconnect()
{
  std::lock_guard<std::mutex> lock(plc_mutex_);
  if (plc_client_ != 0 && plc_connected_) {
    Cli_Disconnect(plc_client_);
    plc_connected_ = false;
    RCLCPP_INFO(rclcpp::get_logger(LOG_TAG), "PLC S7 disconnected.");
  }
  if (plc_client_ != 0) {
    Cli_Destroy(&plc_client_);
    plc_client_ = 0;
  }
}

bool PlcStoHandler::isConnected() const
{
  return plc_connected_;
}

// ── STO Monitor ──────────────────────────────────────────────────────────────

bool PlcStoHandler::readStoConfirmation(bool & confirmed)
{
  return plcReadBit(config_.monitor_db,
                    config_.monitor_byte_offset,
                    config_.monitor_bit_index,
                    confirmed);
}

// ── STO Acknowledge + Reset ──────────────────────────────────────────────────

bool PlcStoHandler::acknowledgeAndReset()
{
  // ---- Step 1: One-shot pulse to WRITE bit (DB3.DBX1.4) ----
  RCLCPP_INFO(rclcpp::get_logger(LOG_TAG),
    "Sending one-shot pulse to DB%d.DBX%d.%d...",
    config_.write_db, config_.write_byte_offset, config_.write_bit_index);

  if (!plcWriteBit(config_.write_db,
                   config_.write_byte_offset,
                   config_.write_bit_index, true))
  {
    RCLCPP_ERROR(rclcpp::get_logger(LOG_TAG),
      "❌ Failed to set DB%d.DBX%d.%d HIGH!",
      config_.write_db, config_.write_byte_offset, config_.write_bit_index);
    return false;
  }

  RCLCPP_INFO(rclcpp::get_logger(LOG_TAG),
    "✅ DB%d.DBX%d.%d set HIGH — waiting %d µs before clearing...",
    config_.write_db, config_.write_byte_offset, config_.write_bit_index,
    config_.pulse_width_us);
  usleep(config_.pulse_width_us);

  if (!plcWriteBit(config_.write_db,
                   config_.write_byte_offset,
                   config_.write_bit_index, false))
  {
    RCLCPP_ERROR(rclcpp::get_logger(LOG_TAG),
      "❌ Failed to clear DB%d.DBX%d.%d back to LOW!",
      config_.write_db, config_.write_byte_offset, config_.write_bit_index);
    return false;
  }

  RCLCPP_INFO(rclcpp::get_logger(LOG_TAG),
    "✅ DB%d.DBX%d.%d cleared LOW — one-shot pulse complete",
    config_.write_db, config_.write_byte_offset, config_.write_bit_index);

  // ---- Step 2: Reset MONITOR bit back to FALSE (DB1.DBX3.5) ----
  RCLCPP_INFO(rclcpp::get_logger(LOG_TAG),
    "Resetting monitor bit DB%d.DBX%d.%d to FALSE...",
    config_.monitor_db, config_.monitor_byte_offset, config_.monitor_bit_index);

  if (!plcWriteBit(config_.monitor_db,
                   config_.monitor_byte_offset,
                   config_.monitor_bit_index, false))
  {
    RCLCPP_ERROR(rclcpp::get_logger(LOG_TAG),
      "❌ Failed to reset DB%d.DBX%d.%d!",
      config_.monitor_db, config_.monitor_byte_offset, config_.monitor_bit_index);
    return false;
  }

  RCLCPP_INFO(rclcpp::get_logger(LOG_TAG),
    "✅ Monitor bit DB%d.DBX%d.%d reset to FALSE",
    config_.monitor_db, config_.monitor_byte_offset, config_.monitor_bit_index);

  return true;
}

// ── Low-level PLC bit helpers ────────────────────────────────────────────────

bool PlcStoHandler::plcReadBit(int db, int byte_offset, int bit_index, bool & value)
{
  if (!plc_connected_ || plc_client_ == 0) {
    RCLCPP_WARN(rclcpp::get_logger(LOG_TAG),
      "PLC not connected — attempting reconnect for read DB%d.DBX%d.%d...",
      db, byte_offset, bit_index);
    if (!connect()) return false;
  }

  uint8_t buf[1] = {0};
  {
    std::lock_guard<std::mutex> lock(plc_mutex_);
    int rc = Cli_DBRead(plc_client_, db, byte_offset, 1, buf);
    if (rc != 0) {
      char errbuf[256] = {0};
      Cli_ErrorText(rc, errbuf, sizeof(errbuf));
      RCLCPP_ERROR(rclcpp::get_logger(LOG_TAG),
        "PLC read DB%d.DBX%d.%d FAILED: %s", db, byte_offset, bit_index, errbuf);
      plc_connected_ = false;
      return false;
    }
  }
  value = ((buf[0] >> bit_index) & 0x01) != 0;
  return true;
}

bool PlcStoHandler::plcWriteBit(int db, int byte_offset, int bit_index, bool value)
{
  if (!plc_connected_ || plc_client_ == 0) {
    RCLCPP_WARN(rclcpp::get_logger(LOG_TAG),
      "PLC not connected — attempting reconnect for write DB%d.DBX%d.%d...",
      db, byte_offset, bit_index);
    if (!connect()) return false;
  }

  uint8_t buf[1] = {0};
  {
    std::lock_guard<std::mutex> lock(plc_mutex_);

    // Read current byte
    int rc = Cli_DBRead(plc_client_, db, byte_offset, 1, buf);
    if (rc != 0) {
      char errbuf[256] = {0};
      Cli_ErrorText(rc, errbuf, sizeof(errbuf));
      RCLCPP_ERROR(rclcpp::get_logger(LOG_TAG),
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
      RCLCPP_ERROR(rclcpp::get_logger(LOG_TAG),
        "PLC write DB%d.DBX%d.%d FAILED: %s", db, byte_offset, bit_index, errbuf);
      plc_connected_ = false;
      return false;
    }
  }

  RCLCPP_INFO(rclcpp::get_logger(LOG_TAG),
    "PLC write DB%d.DBX%d.%d = %s OK",
    db, byte_offset, bit_index, value ? "TRUE" : "FALSE");
  return true;
}
