#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped, Twist
from tf2_ros import TransformBroadcaster
from pymodbus.client.sync import ModbusSerialClient
from pymodbus.exceptions import ModbusIOException

import math
import time
import struct


# ---------------------------
# Helper: Write signed 32-bit
# ---------------------------
def write_32bit_signed(client, register_address, value, unit=1):
    packed_value = struct.pack(">i", value)
    reg1, reg2 = struct.unpack(">HH", packed_value)
    return client.write_registers(register_address, [reg1, reg2], unit=unit)


# ================================================================
#                   MAIN MOTOR + ODOM NODE
# ================================================================
class DualMotorWithOdometry(Node):
    def __init__(self):
        super().__init__("dual_motor_with_odom")

        # ------------------------------------
        # MOTOR DRIVER SETTINGS (AC520/MBDV-520)
        # ------------------------------------
        self.left_motor_slave = 2
        self.right_motor_slave = 1

                # Motor direction multipliers
        # Adjust based on physical mounting
        self.LEFT_DIR = 1     # try 1 or -1
        self.RIGHT_DIR = -1  # opposite of left
        
        # Encoder direction (SEPARATE)
        self.LEFT_ENC_DIR = -1    # <-- this is the fix
        self.RIGHT_ENC_DIR = 1  


        # Enable servo ON register
        self.enable_register = 40125
        self.enable_reg_addr = self.enable_register - 40001




        # Speed command register (0.1 RPM)
        self.speed_register = 40343
        self.speed_reg_addr = self.speed_register - 40001
                # ------------------------------------
        self.command_register = 40001
        self.command_reg_addr = self.command_register - 40001

        self.last_command_value = None

        # Monitor driver state every 0.5 sec
        self.create_timer(0.1, self.monitor_driver_state)

        # -------- SAFE SPEED LIMITS --------
        self.max_driver_speed = 2000     # = 200 RPM

        self.left_target = 0
        self.right_target = 0

        # ------------------------------------
        # ROBOT PHYSICAL PARAMETERS
        # ------------------------------------
        self.wheel_radius = 0.075        # 75 mm
        self.wheel_separation = 0.477    # center-to-center wheel distance

        # ------------------------------------
        # ENCODER PARAMETERS
        # ------------------------------------
        self.pulses_per_rev = 100000
        self.encoder_register = 40011
        self.enc_reg_addr = self.encoder_register - 40001
        self.encoder_count = 2

        # ------------------------------------
        # ODOMETRY INITIAL VALUES
        # ------------------------------------
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.left_last_pulses = 0
        self.right_last_pulses = 0
        self.last_time = time.time()

        # ------------------------------------
        # MODBUS CONNECT
        # ------------------------------------
        self.client = ModbusSerialClient(
            method="rtu",
            port="/dev/ttyUSB0",
            baudrate=9600,
            stopbits=1,
            bytesize=8,
            parity="N",
            timeout=1
        )

        if self.client.connect():
            self.get_logger().info("✅ Modbus connected.")
        else:
            self.get_logger().error("❌ Modbus connection failed!")

        # Enable both motors
        self.enable_motor(self.left_motor_slave)
        self.enable_motor(self.right_motor_slave)

        # ------------------------------------
        # ROS SETUP
        # ------------------------------------
        self.create_subscription(Twist, "/cmd_vel", self.cmd_vel_callback, 10)
        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_timer(0.05, self.update_motor_speeds)
        self.create_timer(0.05, self.update_odometry)

        self.get_logger().info("🚀 AC520 Dual Drive + Odometry Node Started!")


    # ================================================================
    #                           MOTOR ENABLE
    # ================================================================
    def enable_motor(self, slave_id):
        try:
            # 1. Servo ON
            res1 = self.client.write_register(self.enable_reg_addr, 159, unit=slave_id)

            # 2. Release brake / prepare speed mode
            res2 = self.client.write_register(self.enable_reg_addr, 150, unit=slave_id)

            if res1.isError() or res2.isError():
                self.get_logger().error(f"❌ Failed to fully enable motor {slave_id}")
            else:
                self.get_logger().info(f"🟢 Motor {slave_id} FULLY ENABLED (159 → 150)")
        except Exception as e:
            self.get_logger().error(f"❌ Enable Error: {e}")



    def monitor_driver_state(self):
        try:
            result = self.client.read_holding_registers(
                address=self.command_reg_addr,
                count=1,
                unit=self.left_motor_slave   # monitor ONLY ONE slave
            )

            if result is None or result.isError():
                self.get_logger().warn("⚠️ Failed to read command register 40001")
                return

            value = result.registers[0]

            # PRINT every value we receive
            # self.get_logger().info(f"📟 Driver register 40001 value = {value}")

            # Detect 2 → 0 transition
            if self.last_command_value == 2 and value == 0:
                self.get_logger().info(
                    "🔁 Detected 2 → 0 transition, enabling BOTH motors"
                )

                self.enable_motor(self.left_motor_slave)
                self.enable_motor(self.right_motor_slave)

            self.last_command_value = value

        except Exception as e:
            self.get_logger().error(f"❌ Driver monitor error: {e}")



    # ================================================================
    #                 CMD_VEL → MOTOR SPEED MAPPING
    # ================================================================
    def cmd_vel_callback(self, msg):
        linear = -msg.linear.x
        angular = -msg.angular.z

        # --- Compute wheel velocity (m/s) ---
        v_left = linear - angular * (self.wheel_separation / 2)
        v_right = linear + angular * (self.wheel_separation / 2)

        # --- Convert to wheel RPM ---
        rpm_left = (v_left / (2 * math.pi * self.wheel_radius)) * 60
        rpm_right = (v_right / (2 * math.pi * self.wheel_radius)) * 60


        # --- Convert RPM → driver units (0.1 RPM) ---
        left_cmd = int(rpm_left * 10)
        right_cmd = int(rpm_right * 10)

        # --- Clamp to safe limits ---
        left_cmd = max(-self.max_driver_speed, min(self.max_driver_speed, left_cmd))
        right_cmd = max(-self.max_driver_speed, min(self.max_driver_speed, right_cmd))

        # Save target for ramping
        self.left_target = left_cmd
        self.right_target = right_cmd


    # ================================================================
    #                       RAMPING (SMOOTH SPEED)
    # ================================================================

    # ================================================================
    #             WRITE SPEED COMMAND TO MOTOR DRIVERS
    # ================================================================
    def update_motor_speeds(self):
        self.send_speed(
            self.left_motor_slave,
            self.LEFT_DIR * self.left_target
        )
        self.send_speed(
            self.right_motor_slave,
            self.RIGHT_DIR * self.right_target
        )


    def send_speed(self, slave, speed):
        try:
            res = write_32bit_signed(self.client, self.speed_reg_addr, speed, unit=slave)
            if res is None or res.isError():
                self.get_logger().error(f"❌ Speed write failed for motor {slave}")
        except Exception as e:
            self.get_logger().error(f"❌ Speed write error: {e}")


    # ================================================================
    #                         ENCODER READING
    # ================================================================
    def read_encoder(self, slave_id):
        try:
            result = self.client.read_holding_registers(
                address=self.enc_reg_addr,
                count=self.encoder_count,
                unit=slave_id
            )
            if result is None or result.isError():
                return 0

            hi = result.registers[0]
            lo = result.registers[1]

            val = (hi << 16) | lo
            if val & 0x80000000:
                val -= 0x100000000
            return val

        except Exception:
            return 0


    # ================================================================
    #                         QUATERNION
    # ================================================================
    def yaw_to_quaternion(self, yaw):
        q = Quaternion()
        q.w = math.cos(yaw / 2)
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(yaw / 2)
        return q


    # ================================================================
    #                         ODOMETRY UPDATE
    # ================================================================
    def update_odometry(self):
            left_p = self.LEFT_ENC_DIR * self.read_encoder(self.left_motor_slave)
            right_p = self.RIGHT_ENC_DIR * self.read_encoder(self.right_motor_slave)

            now_time = time.time()
            dt = now_time - self.last_time
            if dt <= 0: return

            dL = left_p - self.left_last_pulses
            dR = right_p - self.right_last_pulses
            self.left_last_pulses, self.right_last_pulses, self.last_time = left_p, right_p, now_time

            if abs(dL) < 10: dL = 0
            if abs(dR) < 10: dR = 0

            distL = (dL / self.pulses_per_rev) * 2 * math.pi * self.wheel_radius
            distR = (dR / self.pulses_per_rev) * 2 * math.pi * self.wheel_radius

            v = (distL + distR) / 2.0 / dt
            w = (distR - distL) / self.wheel_separation / dt

            self.x += v * math.cos(self.theta) * dt
            self.y += v * math.sin(self.theta) * dt
            self.theta += w * dt

            current_time = self.get_clock().now().to_msg()
            q = self.yaw_to_quaternion(self.theta)

            # 1. Publish Odometry Message
            odom = Odometry()
            odom.header.stamp = current_time
            odom.header.frame_id = "odom"
            odom.child_frame_id = "base_footprint"
            odom.pose.pose.position.x = self.x
            odom.pose.pose.position.y = self.y
            odom.pose.pose.orientation = q
            odom.twist.twist.linear.x = v
            odom.twist.twist.angular.z = w
            self.odom_pub.publish(odom)

            # 2. --- ADDED: Broadcast TF Transform (odom -> base_link) ---
            t = TransformStamped()
            t.header.stamp = current_time
            t.header.frame_id = "odom"
            t.child_frame_id = "base_footprint"
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.translation.z = 0.0
            t.transform.rotation = q
            self.tf_broadcaster.sendTransform(t)
# ================================================================
#                            MAIN
# ================================================================
def main(args=None):
    rclpy.init(args=args)
    node = DualMotorWithOdometry()
    rclpy.spin(node)
    node.client.close()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()







