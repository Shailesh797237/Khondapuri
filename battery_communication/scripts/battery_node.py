#!/usr/bin/env python3

import time
import serial
import json
import os

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool, String


def make_packet(can_id, data=bytes([0x5A])):
    packet = bytearray(20)
    packet[0] = 0xAA
    packet[1] = 0x55
    packet[2] = 0x01
    packet[3] = 0x01
    packet[4] = 0x01
    packet[5] = can_id & 0xFF
    packet[6] = (can_id >> 8) & 0xFF
    packet[7] = (can_id >> 16) & 0xFF
    packet[8] = (can_id >> 24) & 0xFF
    packet[9] = len(data)

    for i, b in enumerate(data):
        packet[10 + i] = b

    packet[19] = sum(packet[2:19]) & 0xFF
    return bytes(packet)


def read_packet(ser, timeout=0.2):
    buffer = bytearray()
    start_time = time.time()

    while time.time() - start_time < timeout:
        if ser.in_waiting > 0:
            buffer.extend(ser.read(ser.in_waiting))

            while len(buffer) >= 2:
                if buffer[0] == 0xAA and buffer[1] == 0x55:
                    break
                buffer.pop(0)

            if len(buffer) >= 20:
                packet = bytes(buffer[:20])
                del buffer[:20]
                checksum = sum(packet[2:19]) & 0xFF
                if checksum == packet[19]:
                    return packet

        time.sleep(0.001)

    return None


def crc16(data):
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc & 0xFFFF


class UnionBatteryNode(Node):
    def __init__(self):
        super().__init__("battery_communication_node")

        # Declare ROS 2 Parameters
        self.declare_parameter("port", "/dev/battery_can")
        self.declare_parameter("fallback_port", "/dev/ttyUSB2")
        self.declare_parameter("baudrate", 2000000)
        self.declare_parameter("poll_time_sec", 1.0)
        self.declare_parameter("topic_prefix", "/battery")

        self.port_name = self.get_parameter("port").value
        self.fallback_port = self.get_parameter("fallback_port").value
        self.baudrate = self.get_parameter("baudrate").value
        self.poll_time = self.get_parameter("poll_time_sec").value
        self.prefix = self.get_parameter("topic_prefix").value.rstrip("/")

        # Publishers
        self.rsoc_pub = self.create_publisher(Float32, f"{self.prefix}/soc", 10)
        self.pack_voltage_pub = self.create_publisher(Float32, f"{self.prefix}/pack_voltage", 10)
        self.current_pub = self.create_publisher(Float32, f"{self.prefix}/current", 10)
        self.charging_pub = self.create_publisher(Bool, f"{self.prefix}/charging", 10)
        self.discharging_pub = self.create_publisher(Bool, f"{self.prefix}/discharging", 10)
        self.cell_voltages_pub = self.create_publisher(String, f"{self.prefix}/cell_voltages", 10)
        self.temperature_pub = self.create_publisher(String, f"{self.prefix}/temperature", 10)
        self.charged_allowed_pub = self.create_publisher(Bool, f"{self.prefix}/charged_allowed", 10)
        self.faults_json_pub = self.create_publisher(String, f"{self.prefix}/faults_json", 10)
        self.connected_pub = self.create_publisher(Bool, f"{self.prefix}/connected", 10)

        self.ser = None
        self.last_reconnect_time = 0.0

        self.get_logger().info(
            f"Initializing Battery Communication Node... Target Port: '{self.port_name}', "
            f"Baud: {self.baudrate}, Poll: {self.poll_time}s"
        )

        # Connect to serial port
        self.open_serial()

        # Polling timer
        self.timer = self.create_timer(self.poll_time, self.timer_callback)

    def find_serial_port(self):
        if os.path.exists(self.port_name):
            return self.port_name
        for p in ["/dev/battery_portal", "/dev/battery_can", "/dev/battery"]:
            if os.path.exists(p):
                return p
        if os.path.exists(self.fallback_port):
            return self.fallback_port
        return self.port_name

    def open_serial(self):
        target_port = self.find_serial_port()

        try:
            self.ser = serial.Serial(
                port=target_port,
                baudrate=self.baudrate,
                timeout=0.01
            )
            self.get_logger().info(f"Connected to BMS via USB-CAN on '{target_port}' at {self.baudrate} baud")
            return True
        except Exception as e:
            self.ser = None
            self.get_logger().warn(f"Failed to open port '{target_port}': {e}. Will retry...")
            return False

    def ensure_connected(self):
        if self.ser is not None and self.ser.is_open:
            return True

        now = time.time()
        if now - self.last_reconnect_time < 2.0:
            return False
        self.last_reconnect_time = now
        return self.open_serial()

    def query_frame(self, can_id):
        if self.ser is None or not self.ser.is_open:
            return None

        try:
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
            self.ser.write(make_packet(can_id))
            self.ser.flush()

            pkt = read_packet(self.ser, timeout=0.2)
            if not pkt:
                return None

            dlc = pkt[9]
            can_data = pkt[10:10 + dlc]

            if len(can_data) >= 8:
                rx_crc = (can_data[6] << 8) | can_data[7]
                calc_crc = crc16(can_data[:6])
                if rx_crc != calc_crc:
                    self.get_logger().warn(f"CRC check failed for CAN ID 0x{can_id:03X}")
                    return None

            return can_data
        except Exception as e:
            self.get_logger().error(f"CAN query error for ID 0x{can_id:03X}: {e}")
            if self.ser:
                try:
                    self.ser.close()
                except Exception:
                    pass
                self.ser = None
            return None

    def decode_faults(self, protection_flags):
        faults = {
            "cell_overvoltage": bool(protection_flags & (1 << 0)),
            "cell_undervoltage": bool(protection_flags & (1 << 1)),
            "pack_overvoltage": bool(protection_flags & (1 << 2)),
            "pack_undervoltage": bool(protection_flags & (1 << 3)),
            "charge_overtemperature": bool(protection_flags & (1 << 4)),
            "charge_low_temperature": bool(protection_flags & (1 << 5)),
            "discharge_overtemperature": bool(protection_flags & (1 << 6)),
            "discharge_low_temperature": bool(protection_flags & (1 << 7)),
            "charge_overcurrent": bool(protection_flags & (1 << 8)),
            "discharge_overcurrent": bool(protection_flags & (1 << 9)),
            "short_circuit": bool(protection_flags & (1 << 10)),
            "front_detection_ic_error": bool(protection_flags & (1 << 11)),
            "software_lock_mos": bool(protection_flags & (1 << 12))
        }
        active_faults = [name for name, active in faults.items() if active]
        return faults, active_faults

    def timer_callback(self):
        connected = self.ensure_connected()
        self.connected_pub.publish(Bool(data=connected))

        if not connected:
            return

        try:
            battery_voltage = 0.0
            battery_current = 0.0
            rsoc = 0.0
            protection_flags = 0
            chg_mos = False
            dis_mos = False
            total_cells = 0
            total_ntc = 0
            cell_voltages = []
            temperatures = []

            # 0x100: Pack Voltage and Current
            d100 = self.query_frame(0x100)
            if d100 and len(d100) >= 6:
                battery_voltage = ((d100[0] << 8) | d100[1]) * 0.01
                curr_raw = (d100[2] << 8) | d100[3]
                if curr_raw & 0x8000:
                    curr_raw -= 0x10000
                battery_current = curr_raw * 0.01

                self.pack_voltage_pub.publish(Float32(data=float(battery_voltage)))
                self.current_pub.publish(Float32(data=float(battery_current)))

            # 0x101: State of Charge (RSOC)
            d101 = self.query_frame(0x101)
            if d101 and len(d101) >= 6:
                rsoc = (d101[4] << 8) | d101[5]
                self.rsoc_pub.publish(Float32(data=float(rsoc)))

            # 0x102: Protection Flags / Faults
            d102 = self.query_frame(0x102)
            if d102 and len(d102) >= 6:
                protection_flags = (d102[4] << 8) | d102[5]

            faults, active_faults = self.decode_faults(protection_flags)
            fault_json = {
                "fault": len(active_faults) > 0,
                "protection_flags": protection_flags,
                "protection_flags_hex": f"0x{protection_flags:04X}",
                "active_faults": active_faults,
                "faults": faults
            }
            fault_msg = String()
            fault_msg.data = json.dumps(fault_json, separators=(",", ":"))
            self.faults_json_pub.publish(fault_msg)

            # 0x103: MOS Status
            d103 = self.query_frame(0x103)
            if d103 and len(d103) >= 2:
                mos_status = (d103[0] << 8) | d103[1]
                chg_mos = bool(mos_status & 0x01)
                dis_mos = bool(mos_status & 0x02)
                self.charging_pub.publish(Bool(data=chg_mos))
                self.discharging_pub.publish(Bool(data=dis_mos))

            charge_fault = (
                faults["cell_overvoltage"]
                or faults["pack_overvoltage"]
                or faults["charge_overtemperature"]
                or faults["charge_low_temperature"]
                or faults["charge_overcurrent"]
                or faults["software_lock_mos"]
            )
            charged_allowed = chg_mos and not charge_fault
            self.charged_allowed_pub.publish(Bool(data=charged_allowed))

            # 0x104: Total Cells & NTC Count
            d104 = self.query_frame(0x104)
            if d104 and len(d104) >= 2:
                total_cells = d104[0]
                total_ntc = d104[1]

            # 0x107..0x110: Cell Voltages
            cell_can_ids = [0x107, 0x108, 0x109, 0x10A, 0x10B, 0x10C, 0x10D, 0x10E, 0x10F, 0x110]
            for cell_can_id in cell_can_ids:
                d_cell = self.query_frame(cell_can_id)
                if d_cell is None or len(d_cell) < 6:
                    continue
                for i in range(3):
                    raw_cell = (d_cell[i * 2] << 8) | d_cell[i * 2 + 1]
                    cell_voltages.append(raw_cell / 1000.0)

            if total_cells > 0:
                cell_voltages = cell_voltages[:total_cells]

            if cell_voltages:
                cell_text = ", ".join(f"Cell{i + 1}={v:.3f}V" for i, v in enumerate(cell_voltages))
                cell_msg = String()
                cell_msg.data = cell_text
                self.cell_voltages_pub.publish(cell_msg)

            # 0x105..0x106: Temperatures
            for t_id in [0x105, 0x106]:
                d_temp = self.query_frame(t_id)
                if d_temp and len(d_temp) >= 6:
                    for i in range(3):
                        raw_t = (d_temp[i * 2] << 8) | d_temp[i * 2 + 1]
                        if raw_t > 0:
                            celsius = (raw_t - 2731) / 10.0
                            temperatures.append(celsius)

            if total_ntc > 0:
                temperatures = temperatures[:total_ntc]

            if temperatures:
                temp_text = ", ".join(f"NTC{i + 1}={t:.1f}C" for i, t in enumerate(temperatures))
                temp_msg = String()
                temp_msg.data = temp_text
                self.temperature_pub.publish(temp_msg)

            temp_display = f"{temperatures[0]:.1f}C" if temperatures else "N/A"

            self.get_logger().info(
                f"SOC: {rsoc:.0f}% | V: {battery_voltage:.2f}V | I: {battery_current:.2f}A | "
                f"Temp: {temp_display} | Cells: {len(cell_voltages)} | CHG: {chg_mos} | "
                f"DIS: {dis_mos} | Faults: {len(active_faults)}"
            )

        except Exception as e:
            self.get_logger().error(f"Error reading BMS data: {e}")

    def destroy_node(self):
        if self.ser is not None and self.ser.is_open:
            self.ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = UnionBatteryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
