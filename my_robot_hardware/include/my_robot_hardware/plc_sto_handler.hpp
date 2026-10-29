#ifndef PLC_STO_HANDLER_HPP
#define PLC_STO_HANDLER_HPP

// ============================================================================
//  PlcStoHandler — Separate PLC STO (Safe Torque Off) monitor & reset module
//
//  This class is intentionally decoupled from the main hardware interface
//  so that PLC bit addresses and recovery logic can be changed independently.
//
//  Current PLC bit mapping (change these in the constructor or config):
//    MONITOR (read):  DB1.DBX3.5  — PLC sets TRUE when STO is cleared
//    WRITE   (pulse): DB3.DBX1.4  — IPC sends one-shot acknowledge to PLC
//    RESET:           DB1.DBX3.5  — IPC clears back to FALSE after recovery
// ============================================================================

// snap7 for PLC S7 communication
#include "snap7.h"

#include <string>
#include <mutex>
#include <atomic>

// ===== PLC bit address configuration =====
// Change these values to update STO addresses without touching any other code.
struct StoConfig
{
  // PLC connection
  std::string plc_ip   = "192.168.1.21";
  int         plc_rack = 0;
  int         plc_slot = 1;

  // STO MONITOR bit — PLC → IPC (read)
  //   PLC sets this TRUE when STO event is cleared
  int monitor_db          = 1;    // DB number
  int monitor_byte_offset = 3;    // byte offset   → DB1.DBX3.5
  int monitor_bit_index   = 5;    // bit index

  // STO WRITE bit — IPC → PLC (one-shot pulse)
  //   IPC sets HIGH, waits pulse_width_us, sets LOW
  int write_db          = 3;      // DB number
  int write_byte_offset = 1;      // byte offset   → DB3.DBX1.4
  int write_bit_index   = 4;      // bit index

  // Pulse width for the one-shot acknowledge (microseconds)
  int pulse_width_us = 200000;    // 200ms
};

class PlcStoHandler
{
public:
  // ── Constructor ──────────────────────────────────────────────────
  explicit PlcStoHandler(const StoConfig & config);
  ~PlcStoHandler();

  // ── Connection ───────────────────────────────────────────────────
  bool connect();
  void disconnect();
  bool isConnected() const;

  // ── STO Monitor ──────────────────────────────────────────────────
  //  Reads the monitor bit (DB1.DBX3.5 by default).
  //  Returns true if the read succeeded.
  //  Sets `confirmed` to true if PLC has confirmed the STO is cleared.
  bool readStoConfirmation(bool & confirmed);

  // ── STO Acknowledge + Reset ──────────────────────────────────────
  //  1. Sends one-shot pulse to the WRITE bit (DB3.DBX1.4)
  //  2. Resets the MONITOR bit back to FALSE (DB1.DBX3.5)
  //  Returns true if both operations succeeded.
  bool acknowledgeAndReset();

  // ── Get current config (for logging / debugging) ─────────────────
  const StoConfig & getConfig() const { return config_; }

private:
  StoConfig config_;

  S7Object   plc_client_{0};
  std::mutex plc_mutex_;
  std::atomic<bool> plc_connected_{false};

  // Low-level helpers
  bool plcReadBit (int db, int byte_offset, int bit_index, bool & value);
  bool plcWriteBit(int db, int byte_offset, int bit_index, bool value);
};

#endif  // PLC_STO_HANDLER_HPP
