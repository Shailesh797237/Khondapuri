#ifndef MOONS_HARDWARE_HPP
#define MOONS_HARDWARE_HPP

#include <modbus/modbus.h>
#include <string>
#include <mutex>
#include <unistd.h>
#include <cstdint>

class MoonsMotorEnable
{
public:
  MoonsMotorEnable(const std::string& port,
                   int baudrate,
                   int left_slave,
                   int right_slave,
                   int enable_register,
                   int speed_register,
                   int encoder_register)
  : port_(port),
    baudrate_(baudrate),
    left_slave_(left_slave),
    right_slave_(right_slave),
    enable_reg_(enable_register - 40001),
    speed_reg_(speed_register - 40001),
    encoder_reg_(encoder_register - 40001),

    // ===== ADDED FOR ERROR MONITORING =====
    error_reg_(0)   // 40001 → offset 0
    // ======================================
  {}

  ~MoonsMotorEnable()
  {
    if (ctx_)
    {
      modbus_close(ctx_);
      modbus_free(ctx_);
    }
  }

  bool connect()
  {
    ctx_ = modbus_new_rtu(port_.c_str(), baudrate_, 'N', 8, 1);
    if (!ctx_) return false;

    // 200ms response timeout (was 1s — too long for failed reads)
    modbus_set_response_timeout(ctx_, 0, 200000);

    if (modbus_connect(ctx_) == -1)
      return false;

    return true;
  }

  bool enableMotors()  { return writeEnable(159, "enableMotors"); }
  bool enableJogMode() { return writeEnable(150, "enableJogMode"); }
  bool disableMotors() { return writeEnable(0,   "disableMotors"); }

  bool setJogSpeed(int32_t left_speed, int32_t right_speed)
  {
    bool left_ok  = writeSpeed32(left_slave_, left_speed);
    bool right_ok = writeSpeed32(right_slave_, right_speed);

    if (!left_ok) {
      fprintf(stderr, "[MoonsMotor] ERROR: Failed to write speed to LEFT motor (slave %d, speed %d)\n",
              left_slave_, left_speed);
    }
    if (!right_ok) {
      fprintf(stderr, "[MoonsMotor] ERROR: Failed to write speed to RIGHT motor (slave %d, speed %d)\n",
              right_slave_, right_speed);
    }
    return left_ok && right_ok;
  }

  int32_t readEncoder(int slave)
  {
    uint16_t regs[2];

    std::lock_guard<std::mutex> lock(mutex_);
    modbus_set_slave(ctx_, slave);
    usleep(1000);  // 1ms turnaround (Modbus RTU 3.5 char @ 115200 ≈ 0.3ms)

    if (modbus_read_registers(ctx_, encoder_reg_, 2, regs) != 2)
      return 0;

    return (int32_t)((regs[0] << 16) | regs[1]);
  }

  // ===== ADDED FOR ERROR MONITORING =====
  uint16_t readError(int slave)
  {
    uint16_t value = 0;

    std::lock_guard<std::mutex> lock(mutex_);
    modbus_set_slave(ctx_, slave);
    usleep(1000);  // 1ms turnaround (Modbus RTU 3.5 char @ 115200 ≈ 0.3ms)

    if (modbus_read_registers(ctx_, error_reg_, 1, &value) != 1)
      return 0;

    return value;
  }

  // ===== 32-BIT ERROR READ: registers 40001 + 40002 =====
  uint32_t readError32(int slave)
  {
    uint16_t regs[2] = {0, 0};

    std::lock_guard<std::mutex> lock(mutex_);
    modbus_set_slave(ctx_, slave);
    usleep(1000);  // 1ms turnaround

    if (modbus_read_registers(ctx_, error_reg_, 2, regs) != 2)
      return 0;

    return (static_cast<uint32_t>(regs[0]) << 16) | static_cast<uint32_t>(regs[1]);
  }
  // ======================================

  // ===== BULK READ: error (reg 0) + encoder (reg 10-11) in ONE transaction =====
  // Reads registers 0..11 in a single Modbus call → 1 transaction instead of 2
  struct EncoderAndError {
    int32_t  encoder;
    uint16_t error;
    bool     ok;
  };

  EncoderAndError readEncoderAndError(int slave)
  {
    EncoderAndError result = {0, 0, false};
    // We need registers 0 (error) and encoder_reg_ .. encoder_reg_+1 (encoder)
    // Read from 0 to encoder_reg_+2 in one shot
    int count = encoder_reg_ + 2;  // e.g. 10 + 2 = 12 registers
    uint16_t regs[16] = {0};       // enough for up to 16 registers

    std::lock_guard<std::mutex> lock(mutex_);
    modbus_set_slave(ctx_, slave);
    usleep(1000);

    if (modbus_read_registers(ctx_, 0, count, regs) != count)
      return result;

    result.error   = regs[0];
    result.encoder = (int32_t)((regs[encoder_reg_] << 16) | regs[encoder_reg_ + 1]);
    result.ok      = true;
    return result;
  }
  // =============================================================================

  // ===== GENERIC SINGLE-REGISTER READ (16-bit) =====
  // Pass the Moons register address (e.g. 40011) directly.
  // Returns the raw int16_t (SHORT) value; 0 on failure.
  int16_t readRegister16(int slave, int moons_register)
  {
    uint16_t value = 0;
    int modbus_addr = moons_register - 40001;  // convert to 0-based

    std::lock_guard<std::mutex> lock(mutex_);
    modbus_set_slave(ctx_, slave);
    usleep(1000);  // 1ms turnaround (Modbus RTU 3.5 char @ 115200 ≈ 0.3ms)

    if (modbus_read_registers(ctx_, modbus_addr, 1, &value) != 1)
      return 0;

    return static_cast<int16_t>(value);
  }
  // ==================================================

private:
  bool writeEnable(int value, const char* cmd_name)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    bool success = true;

    // ---- LEFT motor ----
    modbus_set_slave(ctx_, left_slave_);
    int rc_left = modbus_write_register(ctx_, enable_reg_, value);
    if (rc_left == -1) {
      fprintf(stderr, "[MoonsMotor] ERROR: %s FAILED for LEFT motor (slave %d, reg %d, val %d): %s\n",
              cmd_name, left_slave_, enable_reg_ + 40001, value, modbus_strerror(errno));
      success = false;
    }
    usleep(100000);

    // ---- RIGHT motor ----
    modbus_set_slave(ctx_, right_slave_);
    int rc_right = modbus_write_register(ctx_, enable_reg_, value);
    if (rc_right == -1) {
      fprintf(stderr, "[MoonsMotor] ERROR: %s FAILED for RIGHT motor (slave %d, reg %d, val %d): %s\n",
              cmd_name, right_slave_, enable_reg_ + 40001, value, modbus_strerror(errno));
      success = false;
    }
    usleep(100000);

    // ---- Read-back verification: read the enable register to confirm ----
    if (success) {
      uint16_t left_readback = 0, right_readback = 0;

      modbus_set_slave(ctx_, left_slave_);
      usleep(1000);
      if (modbus_read_registers(ctx_, enable_reg_, 1, &left_readback) == 1) {
        if (left_readback != static_cast<uint16_t>(value)) {
          fprintf(stderr, "[MoonsMotor] WARNING: %s LEFT motor readback mismatch! Wrote %d, read back %u\n",
                  cmd_name, value, left_readback);
        }
      }

      modbus_set_slave(ctx_, right_slave_);
      usleep(1000);
      if (modbus_read_registers(ctx_, enable_reg_, 1, &right_readback) == 1) {
        if (right_readback != static_cast<uint16_t>(value)) {
          fprintf(stderr, "[MoonsMotor] WARNING: %s RIGHT motor readback mismatch! Wrote %d, read back %u\n",
                  cmd_name, value, right_readback);
        }
      }
    }

    return success;
  }

  bool writeSpeed32(int slave, int32_t speed)
  {
    uint16_t regs[2];
    regs[0] = (speed >> 16) & 0xFFFF;
    regs[1] = speed & 0xFFFF;

    modbus_set_slave(ctx_, slave);
    int rc = modbus_write_registers(ctx_, speed_reg_, 2, regs);
    if (rc == -1) {
      fprintf(stderr, "[MoonsMotor] ERROR: writeSpeed32 FAILED for slave %d (speed %d): %s\n",
              slave, speed, modbus_strerror(errno));
      return false;
    }
    return true;
  }

  std::string port_;
  int baudrate_;
  int left_slave_;
  int right_slave_;
  int enable_reg_;
  int speed_reg_;
  int encoder_reg_;

  // ===== ADDED FOR ERROR MONITORING =====
  int error_reg_;
  // ======================================

  modbus_t* ctx_{nullptr};
  std::mutex mutex_;
};

#endif



// #ifndef MOONS_HARDWARE_HPP
// #define MOONS_HARDWARE_HPP

// #include <modbus/modbus.h>
// #include <string>
// #include <mutex>
// #include <unistd.h>
// #include <cstdint>

// class MoonsMotorEnable
// {
// public:
//   MoonsMotorEnable(const std::string& port,
//                    int baudrate,
//                    int left_slave,
//                    int right_slave,
//                    int enable_register,
//                    int speed_register,
//                    int encoder_register)
//   : port_(port),
//     baudrate_(baudrate),
//     left_slave_(left_slave),
//     right_slave_(right_slave),
//     enable_reg_(enable_register - 40001),
//     speed_reg_(speed_register - 40001),
//     encoder_reg_(encoder_register - 40001)
//   {}

//   ~MoonsMotorEnable()
//   {
//     if (ctx_)
//     {
//       modbus_close(ctx_);
//       modbus_free(ctx_);
//     }
//   }

//   bool connect()
//   {
//     ctx_ = modbus_new_rtu(port_.c_str(), baudrate_, 'N', 8, 1);
//     if (!ctx_) return false;

//     modbus_set_response_timeout(ctx_, 1, 0);

//     if (modbus_connect(ctx_) == -1)
//       return false;

//     return true;
//   }

//   bool enableMotors()  { return writeEnable(159); }
//   bool enableJogMode() { return writeEnable(150); }
//   bool disableMotors() { return writeEnable(0);   }

//   bool setJogSpeed(int32_t left_speed, int32_t right_speed)
//   {
//     writeSpeed32(left_slave_, left_speed);
//     usleep(100000);
//     writeSpeed32(right_slave_, right_speed);
//     return true;
//   }

//   int32_t readEncoder(int slave)
//   {
//     uint16_t regs[2];

//     std::lock_guard<std::mutex> lock(mutex_);
//     modbus_set_slave(ctx_, slave);

//     if (modbus_read_registers(ctx_, encoder_reg_, 2, regs) != 2)
//       return 0;

//     return (int32_t)((regs[0] << 16) | regs[1]);
//   }

// private:
//   bool writeEnable(int value)
//   {
//     std::lock_guard<std::mutex> lock(mutex_);

//     modbus_set_slave(ctx_, left_slave_);
//     modbus_write_register(ctx_, enable_reg_, value);
//     usleep(100000);

//     modbus_set_slave(ctx_, right_slave_);
//     modbus_write_register(ctx_, enable_reg_, value);
//     usleep(100000);

//     return true;
//   }

//   bool writeSpeed32(int slave, int32_t speed)
//   {
//     uint16_t regs[2];
//     regs[0] = (speed >> 16) & 0xFFFF;
//     regs[1] = speed & 0xFFFF;

//     modbus_set_slave(ctx_, slave);
//     return modbus_write_registers(ctx_, speed_reg_, 2, regs) != -1;
//   }

//   std::string port_;
//   int baudrate_;
//   int left_slave_;
//   int right_slave_;
//   int enable_reg_;
//   int speed_reg_;
//   int encoder_reg_;

//   modbus_t* ctx_{nullptr};
//   std::mutex mutex_;
// };

// #endif
