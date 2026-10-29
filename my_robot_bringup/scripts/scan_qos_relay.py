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
        
        # Publish RELIABLE with depth=1 and VOLATILE to satisfy all subscribers (slam_toolbox, amcl, nav2) without buffering stale scans
        pub_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        self.pub = self.create_publisher(LaserScan, '/scan', pub_qos)
        self.sub = self.create_subscription(LaserScan, '/scan_raw', self.callback, sub_qos)
        self.get_logger().info("✅ Scan QoS Relay Node started (/scan_raw [BEST_EFFORT] -> /scan [RELIABLE depth=1])")

    def callback(self, msg):
        msg.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ScanQosRelay()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
