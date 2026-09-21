#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import os
import glob
import yaml

class MissionBroadcaster(Node):
    def __init__(self):
        super().__init__('mission_broadcaster')
        
        # Correct path for the robot (taikisha)
        self.config_dir = "/home/taikisha/ekf_ws/src/amr_pure_navigation/config"
        
        # Create publisher with Transient Local QoS (latching)
        # so that late-joining UI subscribers get the latest list immediately.
        qos_profile = rclpy.qos.QoSProfile(
            depth=1,
            durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL
        )
        
        self.publisher_ = self.create_publisher(String, '/available_missions', qos_profile)
        
        # Publish every 5 seconds
        self.timer = self.create_timer(5.0, self.publish_missions)
        
        self.get_logger().info(f"Mission Broadcaster started. Watching: {self.config_dir}")
        self.publish_missions() # Initial publish

    def publish_missions(self):
        missions = []
        yaml_files = glob.glob(os.path.join(self.config_dir, "*.yaml"))

        for file_path in yaml_files:
            filename = os.path.basename(file_path)
            try:
                with open(file_path, 'r') as f:
                    data = yaml.safe_load(f) or {}
            except Exception:
                data = {}

            # Guess name and waypoints count
            name = filename.replace('.yaml', '').replace('_', ' ').title()
            waypoints_count = 0
            if isinstance(data, dict):
                for key, val in data.items():
                    if isinstance(val, list):
                        waypoints_count = max(waypoints_count, len(val))
                    elif isinstance(val, dict):
                        waypoints_count = max(waypoints_count, len(val.keys()))

            missions.append({
                "name": name,
                "file": filename,
                "waypoints": waypoints_count,
                "type": "forward"
            })

        msg = String()
        msg.data = json.dumps(missions)
        self.publisher_.publish(msg)
        self.get_logger().debug(f"Published {len(missions)} missions")

def main(args=None):
    rclpy.init(args=args)
    node = MissionBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
