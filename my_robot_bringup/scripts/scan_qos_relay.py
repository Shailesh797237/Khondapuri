#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class ScanQosRelay(Node):
    def __init__(self):
        super().__init__('scan_qos_relay')
        
        # Subscribe BEST_EFFORT (matches pointcloud_to_laserscan output)
        sub_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Publish RELIABLE (matches slam_toolbox input requirement)
        pub_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        self.pub = self.create_publisher(LaserScan, '/scan', pub_qos)
        self.sub = self.create_subscription(LaserScan, '/scan_raw', self.callback, sub_qos)
        self.get_logger().info("✅ Scan QoS Relay Node started (/scan_raw [BEST_EFFORT] -> /scan [RELIABLE])")

    def callback(self, msg):
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ScanQosRelay()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
