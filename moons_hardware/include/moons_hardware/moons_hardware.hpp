#ifndef MOONS_MOTOR_ENABLE_HPP
#define MOONS_MOTOR_ENABLE_HPP

#include <modbus/modbus.h>
#include <string>
#include <mutex>
#include <iostream>
#include <errno.h>
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
                   int speed_register)
  : port_(port),
    baudrate_(baudrate),
    left_slave_(left_slave),
    right_slave_(right_slave),
    enable_reg_(enable_register - 40001),
    speed_reg_(speed_register - 40001)
  {}

  ~MoonsMotorEnable()
  {
    if (ctx_)
    {
      modbus_close(ctx_);
      modbus_free(ctx_);
    }
  }

  /* ---------- CONNECT ---------- */
  bool connect()
  {
    ctx_ = modbus_new_rtu(port_.c_str(), baudrate_, 'N', 8, 1);
    if (!ctx_)
    {
      std::cerr << "❌ Failed to create Modbus context\n";
      return false;
    }

    modbus_set_response_timeout(ctx_, 1, 0);

    if (modbus_connect(ctx_) == -1)
    {
      std::cerr << "❌ Modbus connect failed: "
                << modbus_strerror(errno) << "\n";
      return false;
    }

    std::cout << "✅ Modbus connected\n";
    return true;
  }

  /* ---------- SERVO ENABLE ---------- */
  bool enableMotors()
  {
    return writeBothMotors(159, "SERVO ENABLE");
  }

  /* ---------- JOG MODE ENABLE ---------- */
  bool enableJogMode()
  {
    return writeBothMotors(150, "JOG MODE ENABLE");
  }

  /* ---------- SERVO DISABLE ---------- */
  bool disableMotors()
  {
    return writeBothMotors(0, "SERVO DISABLE");
  }

  /* ---------- SET JOG SPEED (32-bit) ---------- */
  bool setJogSpeed(int left_speed, int right_speed)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return writeSpeed32(left_slave_, left_speed) &&
           writeSpeed32(right_slave_, right_speed);
  }

  /* ---------- GET VELOCITY (32-bit) ---------- */
  bool getVelocity(int slave, int &velocity)
  {
    if (!ctx_) return false;

    uint16_t regs[2];

    std::lock_guard<std::mutex> lock(mutex_);
    modbus_set_slave(ctx_, slave);

    if (modbus_read_registers(ctx_, speed_reg_, 2, regs) != 2)
    {
      std::cerr << "❌ Velocity read failed: "
                << modbus_strerror(errno) << "\n";
      return false;
    }

    // Combine HIGH + LOW → signed 32-bit
    velocity = (int32_t)((regs[0] << 16) | regs[1]);
    return true;
  }

private:
  /* ---------- ENABLE / MODE WRITE ---------- */
  bool writeBothMotors(int value, const std::string& action)
  {
    if (!ctx_) return false;

    std::lock_guard<std::mutex> lock(mutex_);

    modbus_set_slave(ctx_, left_slave_);
    if (modbus_write_register(ctx_, enable_reg_, value) == -1)
    {
      printError("Left motor " + action);
      return false;
    }

    usleep(500000);

    modbus_set_slave(ctx_, right_slave_);
    if (modbus_write_register(ctx_, enable_reg_, value) == -1)
    {
      printError("Right motor " + action);
      return false;
    }

    std::cout << "⚡ " << action
              << " (40125 = " << value << ")\n";
    return true;
  }

  /* ---------- WRITE 32-BIT SPEED ---------- */
  bool writeSpeed32(int slave, int speed)
  {
    uint16_t regs[2];
    regs[0] = (speed >> 16) & 0xFFFF;
    regs[1] = speed & 0xFFFF;

    modbus_set_slave(ctx_, slave);

    if (modbus_write_registers(ctx_, speed_reg_, 2, regs) == -1)
    {
      printError("Speed write");
      return false;
    }
    return true;
  }

  void printError(const std::string& where)
  {
    std::cerr << "❌ " << where
              << " failed: " << modbus_strerror(errno) << "\n";
  }

  /* ---------- MEMBERS ---------- */
  std::string port_;
  int baudrate_;
  int left_slave_;
  int right_slave_;
  int enable_reg_;
  int speed_reg_;

  modbus_t* ctx_{nullptr};
  std::mutex mutex_;
};

#endif // MOONS_MOTOR_ENABLE_HPP



// #ifndef MOONS_MOTOR_ENABLE_HPP
// #define MOONS_MOTOR_ENABLE_HPP

// #include <modbus/modbus.h>
// #include <string>
// #include <mutex>
// #include <iostream>
// #include <errno.h>
// #include <unistd.h>

// class MoonsMotorEnable
// {
// public:
//   MoonsMotorEnable(const std::string& port,
//                    int baudrate,
//                    int left_slave,
//                    int right_slave,
//                    int enable_register,
//                    int speed_register)
//   : port_(port),
//     baudrate_(baudrate),
//     left_slave_(left_slave),
//     right_slave_(right_slave),
//     enable_reg_(enable_register - 40001),
//     speed_reg_(speed_register - 40001)
//   {}

//   ~MoonsMotorEnable()
//   {
//     if (ctx_)
//     {
//       modbus_close(ctx_);
//       modbus_free(ctx_);
//     }
//   }

//   /* ---------- CONNECT ---------- */
//   bool connect()
//   {
//     ctx_ = modbus_new_rtu(port_.c_str(), baudrate_, 'N', 8, 1);
//     if (!ctx_)
//     {
//       std::cerr << "❌ Failed to create Modbus context\n";
//       return false;
//     }

//     modbus_set_response_timeout(ctx_, 1, 0);

//     if (modbus_connect(ctx_) == -1)
//     {
//       std::cerr << "❌ Modbus connect failed: "
//                 << modbus_strerror(errno) << "\n";
//       return false;
//     }

//     std::cout << "✅ Modbus connected\n";
//     return true;
//   }

//   /* ---------- SERVO ENABLE ---------- */
//   bool enableMotors()
//   {
//     return writeBothMotors(enable_reg_, 159, "SERVO ENABLE");
//   }

//   /* ---------- JOG MODE ENABLE ---------- */
//   bool enableJogMode()
//   {
//     return writeBothMotors(enable_reg_, 150, "JOG MODE ENABLE");
//   }

//   /* ---------- SERVO DISABLE ---------- */
//   bool disableMotors()
//   {
//     return writeBothMotors(enable_reg_, 0, "SERVO DISABLE");
//   }

//   /* ---------- SET JOG SPEED (32-bit) ---------- */
//   bool setJogSpeed(int left_speed, int right_speed)
//   {
//     if (!ctx_) return false;

//     std::lock_guard<std::mutex> lock(mutex_);

//     // LEFT MOTOR
//     if (!writeSpeed32(left_slave_, left_speed))
//       return false;

//     // RIGHT MOTOR
//     if (!writeSpeed32(right_slave_, right_speed))
//       return false;

//     std::cout << "⚡ Jog Speed Set | L=" << left_speed
//               << " R=" << right_speed << "\n";
//     return true;
//   }

// private:
//   /* ---------- COMMON ENABLE WRITE ---------- */
//   bool writeBothMotors(int reg, int value, const std::string& action)
//   {
//     if (!ctx_) return false;

//     std::lock_guard<std::mutex> lock(mutex_);

//     // LEFT MOTOR
//     modbus_set_slave(ctx_, left_slave_);
//     if (modbus_write_register(ctx_, reg, value) == -1)
//     {
//       printError("Left motor " + action);
//       return false;
//     }

//     usleep(500000);

//     // RIGHT MOTOR
//     modbus_set_slave(ctx_, right_slave_);
//     if (modbus_write_register(ctx_, reg, value) == -1)
//     {
//       printError("Right motor " + action);
//       return false;
//     }

//     std::cout << "⚡ " << action
//               << " (40125 = " << value << ")\n";
//     return true;
//   }

//   /* ---------- WRITE 32-BIT SPEED ---------- */
//   bool writeSpeed32(int slave, int speed)
//   {
//     uint16_t regs[2];
//     regs[0] = (speed >> 16) & 0xFFFF; // High word
//     regs[1] = speed & 0xFFFF;         // Low word

//     modbus_set_slave(ctx_, slave);

//     if (modbus_write_registers(ctx_, speed_reg_, 2, regs) == -1)
//     {
//       printError("Speed write");
//       return false;
//     }

//     return true;
//   }

//   void printError(const std::string& where)
//   {
//     std::cerr << "❌ " << where
//               << " failed: " << modbus_strerror(errno) << "\n";
//   }

//   /* ---------- MEMBERS ---------- */
//   std::string port_;
//   int baudrate_;
//   int left_slave_;
//   int right_slave_;
//   int enable_reg_;
//   int speed_reg_;

//   modbus_t* ctx_{nullptr};
//   std::mutex mutex_;
// };

// #endif // MOONS_MOTOR_ENABLE_HPP





// #ifndef MOONS_MOTOR_ENABLE_HPP
// #define MOONS_MOTOR_ENABLE_HPP

// #include <modbus/modbus.h>
// #include <string>
// #include <mutex>
// #include <iostream>
// #include <errno.h>
// #include <unistd.h>

// class MoonsMotorEnable
// {
// public:
//   MoonsMotorEnable(const std::string& port,
//                    int baudrate,
//                    int left_slave,
//                    int right_slave,
//                    int enable_register)
//   : port_(port),
//     baudrate_(baudrate),
//     left_slave_(left_slave),
//     right_slave_(right_slave),
//     enable_reg_(enable_register - 40001)
//   {}

//   ~MoonsMotorEnable()
//   {
//     if (ctx_)
//     {
//       modbus_close(ctx_);
//       modbus_free(ctx_);
//     }
//   }

//   /* ---------- CONNECT ---------- */
//   bool connect()
//   {
//     ctx_ = modbus_new_rtu(port_.c_str(), baudrate_, 'N', 8, 1);
//     if (!ctx_)
//     {
//       std::cerr << "❌ Failed to create Modbus context\n";
//       return false;
//     }

//     modbus_set_response_timeout(ctx_, 1, 0);

//     if (modbus_connect(ctx_) == -1)
//     {
//       std::cerr << "❌ Modbus connect failed: "
//                 << modbus_strerror(errno) << "\n";
//       return false;
//     }

//     std::cout << "✅ Modbus connected\n";
//     return true;
//   }

//   /* ---------- SERVO ENABLE ---------- */
//   bool enableMotors()
//   {
//     return writeBothMotors(159, "SERVO ENABLE");
//   }

//   /* ---------- JOG MODE ENABLE ---------- */
//   bool enableJogMode()
//   {
//     return writeBothMotors(150, "JOG MODE ENABLE");
//   }

//   /* ---------- SERVO DISABLE ---------- */
//   bool disableMotors()
//   {
//     return writeBothMotors(0, "SERVO DISABLE");
//   }

// private:
//   bool writeBothMotors(int value, const std::string& action)
//   {
//     if (!ctx_) return false;

//     std::lock_guard<std::mutex> lock(mutex_);

//     // LEFT MOTOR
//     modbus_set_slave(ctx_, left_slave_);
//     if (modbus_write_register(ctx_, enable_reg_, value) == -1)
//     {
//       printError("Left motor " + action);
//       return false;
//     }

//     usleep(500000); // required delay

//     // RIGHT MOTOR
//     modbus_set_slave(ctx_, right_slave_);
//     if (modbus_write_register(ctx_, enable_reg_, value) == -1)
//     {
//       printError("Right motor " + action);
//       return false;
//     }

//     std::cout << "⚡ " << action
//               << " (40125 = " << value << ")\n";
//     return true;
//   }

//   void printError(const std::string& where)
//   {
//     std::cerr << "❌ " << where
//               << " failed: " << modbus_strerror(errno) << "\n";
//   }

//   /* ---------- MEMBERS ---------- */
//   std::string port_;
//   int baudrate_;
//   int left_slave_;
//   int right_slave_;
//   int enable_reg_;

//   modbus_t* ctx_{nullptr};
//   std::mutex mutex_;
// };

// #endif // MOONS_MOTOR_ENABLE_HPP



