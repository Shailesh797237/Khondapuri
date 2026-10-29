#!/usr/bin/env python3
import os
import glob
import json
import yaml
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import String


class MissionPublisher(Node):
    def __init__(self):
        super().__init__('mission_publisher')

        # Declare parameter with default path to turn_ws missions directory
        self.declare_parameter(
            'mission_path',
            '/home/taikisha/turn_ws/install/share/amr_pure_navigation/config/missions'
        )
        self.mission_path = self.get_parameter('mission_path').get_parameter_value().string_value

        # Transient Local QoS (latched topic: new subscribers receive latest list immediately)
        qos_profile = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE
        )

        # Global topic matching web_ui and ROS 2 ecosystem
        self.publisher_ = self.create_publisher(String, '/available_missions', qos_profile)

        # Timer to check and publish every 5.0 seconds
        self.timer = self.create_timer(5.0, self.publish_missions)
        self.last_published_data = None

        self.get_logger().info(f'Mission Publisher started. Watching: {self.mission_path}')
        self.publish_missions()

    def publish_missions(self):
        if not os.path.exists(self.mission_path):
            self.get_logger().error(f'Path does not exist: {self.mission_path}')
            return

        try:
            # Collect only .yaml and .yml files
            yaml_files = sorted(
                glob.glob(os.path.join(self.mission_path, '*.yaml')) +
                glob.glob(os.path.join(self.mission_path, '*.yml'))
            )

            missions = []
            for file_path in yaml_files:
                filename = os.path.basename(file_path)
                stem_name = os.path.splitext(filename)[0]

                # Count waypoints inside file
                waypoints_count = 0
                try:
                    with open(file_path, 'r') as f:
                        data = yaml.safe_load(f) or {}
                        if isinstance(data, dict):
                            for key, val in data.items():
                                if isinstance(val, list):
                                    waypoints_count = max(waypoints_count, len(val))
                                elif isinstance(val, dict):
                                    waypoints_count = max(waypoints_count, len(val.keys()))
                except Exception:
                    pass

                missions.append({
                    "name": stem_name.replace('_', ' ').title(),
                    "file": filename,
                    "mission_id": stem_name,  # Clean ID for /load_mission
                    "waypoints": waypoints_count,
                    "type": "forward"
                })

            json_data = json.dumps(missions)

            # Only log and publish when mission files change or on startup
            if json_data != self.last_published_data:
                msg = String()
                msg.data = json_data
                self.publisher_.publish(msg)
                self.last_published_data = json_data
                self.get_logger().info(f'Published {len(missions)} available missions.')

        except Exception as e:
            self.get_logger().error(f'Failed to read missions: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    node = MissionPublisher()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
