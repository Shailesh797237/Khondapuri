#!/usr/bin/env python3
# ================================================================
#  pose_persistence.py
#
#  Saves the robot's AMCL pose to a file periodically (every 5s).
#  On startup, restores the last known pose by publishing to
#  /initialpose — so AMCL initializes at the saved position.
#
#  This eliminates manual re-localization after every robot restart!
#
#  FLOW:
#    1. On startup: read saved pose from YAML → publish /initialpose
#    2. During operation: subscribe /amcl_pose → save to YAML every 5s
#    3. On shutdown: final save of last pose
#    4. Next startup: automatically restore → no manual localization!
#
#  FIRST-TIME USE:
#    - If no saved pose file exists, the user must localize manually
#      once via RViz "2D Pose Estimate". After that, it's automatic.
#
#  USAGE:
#    ros2 run amr_pure_navigation pose_persistence
#    OR included in navigation.launch.py (automatic)
#
#  PARAMETERS:
#    pose_file     : path to save/load pose (default: ~/.ros/last_robot_pose.yaml)
#    save_interval : seconds between saves (default: 5.0)
#    max_pose_age  : max age in hours before pose is considered stale (default: 720 = 30 days)
#    restore_delay : seconds to wait after startup before publishing initial pose (default: 10.0)
# ================================================================

import os
import time
import math
import yaml

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import String


class PosePersistence(Node):
    def __init__(self):
        super().__init__('pose_persistence')

        # ── Parameters ────────────────────────────────────────────
        self.declare_parameter('pose_file',
            '/home/taikisha/turn_ws/src/amr_pure_navigation/config/last_robot_pose.yaml')
        self.declare_parameter('save_interval', 5.0)       # save every 5 seconds
        self.declare_parameter('max_pose_age', 720.0)       # hours (30 days)
        self.declare_parameter('restore_delay', 10.0)       # seconds after start

        self.pose_file     = self.get_parameter('pose_file').get_parameter_value().string_value
        self.save_interval = self.get_parameter('save_interval').get_parameter_value().double_value
        self.max_pose_age  = self.get_parameter('max_pose_age').get_parameter_value().double_value
        self.restore_delay = self.get_parameter('restore_delay').get_parameter_value().double_value

        # ── State ─────────────────────────────────────────────────
        self.last_pose = None           # latest PoseWithCovarianceStamped
        self.pose_received = False      # have we received any AMCL pose?
        self.pose_restored = False      # have we published the saved pose?
        self.last_save_time = 0.0       # last time we saved to file

        # ── Ensure directory exists ───────────────────────────────
        pose_dir = os.path.dirname(self.pose_file)
        if pose_dir and not os.path.exists(pose_dir):
            os.makedirs(pose_dir, exist_ok=True)
            self.get_logger().info(f'Created directory: {pose_dir}')

        # ── Publisher: /initialpose (same type AMCL expects) ──────
        # Use transient_local durability so AMCL gets the message
        # even if it subscribes slightly after we publish
        qos_initial = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            '/initialpose',
            qos_initial
        )

        # ── Status publisher ──────────────────────────────────────
        self.status_pub = self.create_publisher(String, '/pose_persistence_status', 10)

        # ── Subscriber: /amcl_pose ────────────────────────────────
        self.amcl_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.amcl_callback,
            10
        )

        # ── Timer: periodic save ──────────────────────────────────
        self.save_timer = self.create_timer(self.save_interval, self.save_timer_callback)

        # ── Timer: delayed restore (one-shot after restore_delay) ─
        self.restore_timer = self.create_timer(self.restore_delay, self.restore_pose_callback)

        self.get_logger().info('═══════════════════════════════════════════')
        self.get_logger().info('  📍 Pose Persistence Node Started')
        self.get_logger().info(f'  Pose file    : {self.pose_file}')
        self.get_logger().info(f'  Save interval: {self.save_interval}s')
        self.get_logger().info(f'  Max pose age : {self.max_pose_age}h')
        self.get_logger().info(f'  Restore delay: {self.restore_delay}s')

        if os.path.exists(self.pose_file):
            self.get_logger().info(f'  ✅ Saved pose file FOUND — will restore in {self.restore_delay}s')
        else:
            self.get_logger().warn('  ⚠ No saved pose file — manual localization needed first time')

        self.get_logger().info('═══════════════════════════════════════════')

    # ── AMCL Pose Callback ────────────────────────────────────────
    def amcl_callback(self, msg):
        """Store the latest AMCL pose for periodic saving."""
        self.last_pose = msg
        if not self.pose_received:
            self.pose_received = True
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            yaw = self._quaternion_to_yaw(msg.pose.pose.orientation)
            self.get_logger().info(
                f'✅ First AMCL pose received: x={x:.3f} y={y:.3f} yaw={math.degrees(yaw):.1f}°')

    # ── Periodic Save Timer ───────────────────────────────────────
    def save_timer_callback(self):
        """Save the current pose to file periodically."""
        if self.last_pose is None:
            return  # no pose to save yet

        now = time.time()
        # Throttle saves to avoid excessive disk writes
        if (now - self.last_save_time) < (self.save_interval - 0.5):
            return

        self._save_pose_to_file(self.last_pose)
        self.last_save_time = now

    # ── Restore Pose (delayed one-shot) ───────────────────────────
    def restore_pose_callback(self):
        """
        Called once after restore_delay seconds.
        Reads the saved pose file and publishes to /initialpose.
        Then cancels itself (one-shot behavior).
        """
        # Cancel the timer so it doesn't fire again
        self.restore_timer.cancel()

        if self.pose_restored:
            return

        if self.pose_received:
            # AMCL already has a pose (maybe user already localized manually)
            self.get_logger().info(
                '📍 AMCL already has a pose — skipping restore from file')
            self.pose_restored = True
            self._publish_status('AMCL already localized — pose restore skipped')
            return

        # Try to load saved pose
        pose_data = self._load_pose_from_file()
        if pose_data is None:
            self.get_logger().warn(
                '⚠ No saved pose available — please localize manually via RViz "2D Pose Estimate"')
            self._publish_status('NO_SAVED_POSE — manual localization required')
            return

        # Check pose age
        saved_time = pose_data.get('timestamp', 0)
        age_hours = (time.time() - saved_time) / 3600.0

        if age_hours > self.max_pose_age:
            self.get_logger().warn(
                f'⚠ Saved pose is {age_hours:.1f} hours old (max={self.max_pose_age}h) — '
                f'ignoring stale pose. Please localize manually.')
            self._publish_status(f'STALE_POSE ({age_hours:.0f}h old) — manual localization required')
            return

        # Build the /initialpose message
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'

        msg.pose.pose.position.x = float(pose_data['x'])
        msg.pose.pose.position.y = float(pose_data['y'])
        msg.pose.pose.position.z = float(pose_data.get('z', 0.0))

        msg.pose.pose.orientation.x = float(pose_data.get('qx', 0.0))
        msg.pose.pose.orientation.y = float(pose_data.get('qy', 0.0))
        msg.pose.pose.orientation.z = float(pose_data['qz'])
        msg.pose.pose.orientation.w = float(pose_data['qw'])

        # Set moderate covariance — AMCL will refine with laser scans
        # Small enough to localize quickly, large enough for minor drift
        cov = [0.0] * 36
        cov[0]  = 0.25  # x variance (0.5m std dev)
        cov[7]  = 0.25  # y variance (0.5m std dev)
        cov[35] = 0.07  # yaw variance (~15° std dev)
        msg.pose.covariance = cov

        # Publish multiple times to ensure AMCL receives it
        for i in range(3):
            msg.header.stamp = self.get_clock().now().to_msg()
            self.initial_pose_pub.publish(msg)
            time.sleep(0.5)

        yaw = self._quaternion_to_yaw(msg.pose.pose.orientation)
        self.pose_restored = True

        self.get_logger().info('═══════════════════════════════════════════')
        self.get_logger().info('  🎯 POSE RESTORED FROM FILE!')
        self.get_logger().info(f'  x   = {pose_data["x"]:.3f}')
        self.get_logger().info(f'  y   = {pose_data["y"]:.3f}')
        self.get_logger().info(f'  yaw = {math.degrees(yaw):.1f}°')
        self.get_logger().info(f'  Age = {age_hours:.1f} hours')
        self.get_logger().info('  AMCL will refine position using laser scans...')
        self.get_logger().info('═══════════════════════════════════════════')
        self._publish_status(
            f'POSE_RESTORED x={pose_data["x"]:.2f} y={pose_data["y"]:.2f} '
            f'yaw={math.degrees(yaw):.1f}° age={age_hours:.1f}h')

    # ── Save Pose to YAML File ────────────────────────────────────
    def _save_pose_to_file(self, msg):
        """Save the current AMCL pose to a YAML file."""
        try:
            pose_data = {
                'timestamp': time.time(),
                'x':  float(msg.pose.pose.position.x),
                'y':  float(msg.pose.pose.position.y),
                'z':  float(msg.pose.pose.position.z),
                'qx': float(msg.pose.pose.orientation.x),
                'qy': float(msg.pose.pose.orientation.y),
                'qz': float(msg.pose.pose.orientation.z),
                'qw': float(msg.pose.pose.orientation.w),
            }

            # Write to temp file first, then rename (atomic write)
            tmp_file = self.pose_file + '.tmp'
            with open(tmp_file, 'w') as f:
                f.write('# Auto-saved robot pose — DO NOT EDIT\n')
                f.write(f'# Saved at: {time.strftime("%Y-%m-%d %H:%M:%S")}\n')
                yaml.dump(pose_data, f, default_flow_style=False)

            os.replace(tmp_file, self.pose_file)

            yaw = self._quaternion_to_yaw(msg.pose.pose.orientation)
            self.get_logger().debug(
                f'💾 Pose saved: x={pose_data["x"]:.3f} y={pose_data["y"]:.3f} '
                f'yaw={math.degrees(yaw):.1f}°')

        except Exception as e:
            self.get_logger().error(f'Failed to save pose: {e}')

    # ── Load Pose from YAML File ──────────────────────────────────
    def _load_pose_from_file(self):
        """Load the saved pose from the YAML file. Returns dict or None."""
        if not os.path.exists(self.pose_file):
            return None

        try:
            with open(self.pose_file, 'r') as f:
                data = yaml.safe_load(f)

            if not data or not isinstance(data, dict):
                return None

            # Handle nested format: pose.position & pose.orientation
            if 'pose' in data and isinstance(data['pose'], dict):
                pos = data['pose'].get('position', {})
                ori = data['pose'].get('orientation', {})
                return {
                    'x': float(pos.get('x', 0.0)),
                    'y': float(pos.get('y', 0.0)),
                    'z': float(pos.get('z', 0.0)),
                    'qx': float(ori.get('x', 0.0)),
                    'qy': float(ori.get('y', 0.0)),
                    'qz': float(ori.get('z', 0.0)),
                    'qw': float(ori.get('w', 1.0)),
                    'timestamp': float(data.get('timestamp', time.time()))
                }

            # Handle flat format: x, y, qz, qw
            required = ['x', 'y', 'qz', 'qw']
            for key in required:
                if key not in data:
                    self.get_logger().error(f'Saved pose missing field: {key}')
                    return None

            return data

        except Exception as e:
            self.get_logger().error(f'Failed to load pose file: {e}')
            return None

    # ── Helpers ───────────────────────────────────────────────────
    def _quaternion_to_yaw(self, q):
        """Extract yaw from quaternion."""
        return math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )

    def _publish_status(self, text):
        """Publish status message."""
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)

    def destroy_node(self):
        """On shutdown: save the final pose."""
        if self.last_pose is not None:
            self.get_logger().info('💾 Saving final pose before shutdown...')
            self._save_pose_to_file(self.last_pose)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PosePersistence()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
