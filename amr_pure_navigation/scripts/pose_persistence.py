#!/usr/bin/env python3
# ================================================================
#  pose_persistence.py  —  Full-Map Scan-Match Relocalizer
#
#  1. Restores last saved AMCL pose on startup.
#  2. Validates restored pose by matching LiDAR scan to map.
#  3. If mismatch detected (robot was moved), searches the ENTIRE MAP
#     to find the true robot location using 2-phase grid search:
#       Phase 1: Coarse search across full map (0.5m steps, 30° angles)
#       Phase 2: Fine search around best hit (0.1m steps, 5° angles)
#  4. Saves pose to YAML every 5s during normal operation.
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
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from std_srvs.srv import Empty


class PosePersistence(Node):
    def __init__(self):
        super().__init__('pose_persistence')

        # ── Parameters ────────────────────────────────────────────
        self.declare_parameter('pose_file',
            '/home/taikisha/turn_ws/src/amr_pure_navigation/config/last_robot_pose.yaml')
        self.declare_parameter('save_interval', 5.0)
        self.declare_parameter('max_pose_age', 720.0)
        self.declare_parameter('restore_delay', 3.0)
        self.declare_parameter('match_threshold', 0.45)

        self.pose_file       = self.get_parameter('pose_file').get_parameter_value().string_value
        self.save_interval   = self.get_parameter('save_interval').get_parameter_value().double_value
        self.max_pose_age    = self.get_parameter('max_pose_age').get_parameter_value().double_value
        self.restore_delay   = self.get_parameter('restore_delay').get_parameter_value().double_value
        self.match_threshold = self.get_parameter('match_threshold').get_parameter_value().double_value

        # ── State ─────────────────────────────────────────────────
        self.last_pose = None
        self.latest_map = None
        self.latest_scan = None
        self.pose_received = False
        self.pose_restored = False
        self.last_save_time = 0.0
        self.relocalizing = False
        self._verified = False
        self._verify_attempts = 0

        # ── Ensure directory exists ───────────────────────────────
        pose_dir = os.path.dirname(self.pose_file)
        if pose_dir and not os.path.exists(pose_dir):
            os.makedirs(pose_dir, exist_ok=True)

        # ── Publisher: /initialpose ────────────────────────────────
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

        self.status_pub = self.create_publisher(String, '/pose_persistence_status', 10)
        self.global_loc_client = self.create_client(Empty, '/reinitialize_global_localization')

        # ── Subscribers ───────────────────────────────────────────
        self.amcl_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.amcl_callback,
            10
        )

        # /map uses transient_local (latched from map_server)
        qos_map = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            qos_map
        )

        # /scan uses best_effort (sensor driver default)
        qos_scan = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos_scan
        )

        # ── Timers ────────────────────────────────────────────────
        self.save_timer = self.create_timer(self.save_interval, self.save_timer_callback)
        self.restore_timer = self.create_timer(self.restore_delay, self.restore_pose_callback)

        self.get_logger().info('═══════════════════════════════════════════')
        self.get_logger().info('  📍 Pose Persistence & Full-Map Relocalizer')
        self.get_logger().info(f'  Pose file       : {self.pose_file}')
        self.get_logger().info(f'  Match Threshold : {self.match_threshold * 100.0:.0f}%')
        self.get_logger().info(f'  Restore delay   : {self.restore_delay}s')
        self.get_logger().info('═══════════════════════════════════════════')

    # ── Callbacks ─────────────────────────────────────────────────

    def amcl_callback(self, msg):
        self.last_pose = msg
        if not self.pose_received:
            self.pose_received = True
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            yaw = self._quaternion_to_yaw(msg.pose.pose.orientation)
            self.get_logger().info(f'✅ AMCL Pose active: x={x:.3f} y={y:.3f} yaw={math.degrees(yaw):.1f}°')

    def map_callback(self, msg):
        if self.latest_map is None:
            self.get_logger().info(f'📌 Map received: {msg.info.width}x{msg.info.height} @ {msg.info.resolution:.3f}m/pix')
        self.latest_map = msg

    def scan_callback(self, msg):
        self.latest_scan = msg

    def save_timer_callback(self):
        if self.last_pose is None or self.relocalizing:
            return
        now = time.time()
        if (now - self.last_save_time) < (self.save_interval - 0.5):
            return
        self._save_pose_to_file(self.last_pose)
        self.last_save_time = now

    # ── Restore & Verify ──────────────────────────────────────────

    def restore_pose_callback(self):
        self.restore_timer.cancel()
        if self.pose_restored:
            return

        pose_data = self._load_pose_from_file()
        if pose_data is None:
            self.get_logger().warn('⚠ No saved pose — manual localization required')
            return

        saved_time = pose_data.get('timestamp', 0)
        age_hours = (time.time() - saved_time) / 3600.0

        if age_hours > self.max_pose_age:
            self.get_logger().warn(f'⚠ Saved pose is stale ({age_hours:.1f}h old)')
            return

        self._publish_initial_pose(pose_data['x'], pose_data['y'], pose_data['qz'], pose_data['qw'])
        self.pose_restored = True
        self.get_logger().info(f'🎯 Restored saved pose: x={pose_data["x"]:.3f}, y={pose_data["y"]:.3f}')
        self._publish_status('POSE_RESTORED')

        # [TEMPORARY] Verification and Phase 1, 2, 3 relocalization commented out
        # self._verify_timer = self.create_timer(2.0, self.verify_and_relocalize_callback)

    def verify_and_relocalize_callback(self):
        # [TEMPORARY] Scan-matching verification is disabled — using saved last pose only
        return

        # Wait for data
        if self.latest_map is None or self.latest_scan is None:
            missing = []
            if self.latest_map is None:
                missing.append('/map')
            if self.latest_scan is None:
                missing.append('/scan')
            self.get_logger().warn(
                f'⏳ Attempt {self._verify_attempts}/15: Waiting for {", ".join(missing)}...')
            if self._verify_attempts >= 15:
                self.get_logger().error('❌ Map/Scan not available after 30s. Skipping verification.')
                self._verified = True
                self._verify_timer.cancel()
            return

        # Data ready — run verification
        self._verified = True
        self._verify_timer.cancel()

        if self.last_pose is None:
            self.get_logger().warn('⚠ No AMCL pose yet — cannot verify.')
            return

        px = self.last_pose.pose.pose.position.x
        py = self.last_pose.pose.pose.position.y
        yaw = self._quaternion_to_yaw(self.last_pose.pose.pose.orientation)
        score = self.compute_scan_match_score(px, py, yaw)

        if score is None:
            self.get_logger().warn('⚠ Score computation failed.')
            return

        self.get_logger().info(f'📊 Scan-to-Map Match: {score * 100.0:.1f}% (need {self.match_threshold * 100.0:.0f}%)')

        if score >= self.match_threshold:
            self.get_logger().info(f'✅ Pose VALIDATED! Scan alignment is good ({score * 100.0:.1f}%).')
            self._publish_status('POSE_VALIDATED')
        else:
            self.get_logger().warn(f'⚠️ MISMATCHED! ({score * 100.0:.1f}%) Robot was physically moved!')
            self.get_logger().info('🔍 Searching ENTIRE MAP for true robot location...')
            self._publish_status('DELOCALIZED — FULL MAP SEARCH STARTING')
            self.perform_full_map_relocalization()

    # ── Scan-to-Map Matching ──────────────────────────────────────

    def compute_scan_match_score(self, px, py, yaw):
        if self.latest_map is None or self.latest_scan is None:
            return None

        map_info = self.latest_map.info
        width = map_info.width
        height = map_info.height
        res = map_info.resolution
        ox = map_info.origin.position.x
        oy = map_info.origin.position.y
        data = self.latest_map.data

        scan = self.latest_scan
        valid_beams = 0
        matched_beams = 0

        angle = scan.angle_min
        stride = 2  # denser sampling for better accuracy

        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)

        # Tolerance in cells — ±2 cells (~6cm at 0.03m/cell)
        tol = 2

        for i in range(0, len(scan.ranges), stride):
            r = scan.ranges[i]
            if math.isfinite(r) and scan.range_min <= r <= scan.range_max:
                lx = r * math.cos(angle)
                ly = r * math.sin(angle)

                mx = px + cos_yaw * lx - sin_yaw * ly
                my = py + sin_yaw * lx + cos_yaw * ly

                gx = int((mx - ox) / res)
                gy = int((my - oy) / res)

                # Count ALL valid beams including off-map (off-map = miss)
                valid_beams += 1

                if 0 <= gx < width and 0 <= gy < height:
                    matched = False
                    # ±2-cell tolerance (~6cm at 0.03m resolution)
                    for dx in range(-tol, tol + 1):
                        for dy in range(-tol, tol + 1):
                            cx, cy = gx + dx, gy + dy
                            if 0 <= cx < width and 0 <= cy < height:
                                if data[cy * width + cx] >= 50:
                                    matched = True
                                    break
                        if matched:
                            break

                    if matched:
                        matched_beams += 1
                # else: off-map = miss (valid but not matched)

            angle += scan.angle_increment * stride

        if valid_beams < 20:
            return 0.0
        return float(matched_beams) / float(valid_beams)

    # ── Full Map 2-Phase Relocalization ───────────────────────────

    def perform_full_map_relocalization(self):
        # [TEMPORARY] Full-map relocalization (Phase 1, 2, 3) is commented out
        self.get_logger().info('ℹ️ Full-map relocalization (Phase 1, 2, 3) is temporarily disabled.')
        self.relocalizing = False
        return

        map_info = self.latest_map.info
        width = map_info.width
        height = map_info.height
        res = map_info.resolution
        ox = map_info.origin.position.x
        oy = map_info.origin.position.y
        data = self.latest_map.data

        # Calculate map bounds in meters
        map_x_min = ox
        map_x_max = ox + width * res
        map_y_min = oy
        map_y_max = oy + height * res

        self.get_logger().info(f'📐 Map bounds: x=[{map_x_min:.1f}, {map_x_max:.1f}] y=[{map_y_min:.1f}, {map_y_max:.1f}]')

        # ════════════════════════════════════════════════════════
        # PHASE 1: Coarse search across ENTIRE map
        #   Step: 0.5m spatial, 30° angular
        # ════════════════════════════════════════════════════════
        coarse_step = 0.5
        coarse_headings = [math.radians(deg) for deg in range(0, 360, 30)]

        best_score = -1.0
        best_x, best_y, best_yaw = 0.0, 0.0, 0.0
        candidates_checked = 0

        # Build list of free-space positions (skip walls and unknown)
        free_positions = []
        cell_step = int(coarse_step / res)
        if cell_step < 1:
            cell_step = 1

        for gx in range(0, width, cell_step):
            for gy in range(0, height, cell_step):
                cell_val = data[gy * width + gx]
                if cell_val == 0:  # free space only
                    wx = ox + gx * res
                    wy = oy + gy * res
                    free_positions.append((wx, wy))

        self.get_logger().info(f'🔎 PHASE 1: Coarse search — {len(free_positions)} free positions × {len(coarse_headings)} angles = {len(free_positions) * len(coarse_headings)} candidates')

        for (wx, wy) in free_positions:
            for yaw in coarse_headings:
                score = self.compute_scan_match_score(wx, wy, yaw)
                candidates_checked += 1
                if score is not None and score > best_score:
                    best_score = score
                    best_x, best_y, best_yaw = wx, wy, yaw

        self.get_logger().info(f'🎯 Phase 1 best: x={best_x:.2f}, y={best_y:.2f}, yaw={math.degrees(best_yaw):.1f}° (Score: {best_score*100.0:.1f}%)')

        # ════════════════════════════════════════════════════════
        # PHASE 2: Fine search around best Phase 1 result
        #   Step: 0.1m spatial, 5° angular, ±0.5m range
        # ════════════════════════════════════════════════════════
        if best_score > 0.10:
            fine_step = 0.10
            fine_range = 0.6
            fine_headings = [best_yaw + math.radians(d) for d in range(-30, 31, 5)]

            fine_x = [best_x + i * fine_step for i in range(-int(fine_range/fine_step), int(fine_range/fine_step) + 1)]
            fine_y = [best_y + i * fine_step for i in range(-int(fine_range/fine_step), int(fine_range/fine_step) + 1)]

            self.get_logger().info(f'🔬 PHASE 2: Fine search — {len(fine_x)}×{len(fine_y)}×{len(fine_headings)} = {len(fine_x)*len(fine_y)*len(fine_headings)} candidates around best hit')

            for x in fine_x:
                for y in fine_y:
                    for yaw in fine_headings:
                        score = self.compute_scan_match_score(x, y, yaw)
                        if score is not None and score > best_score:
                            best_score = score
                            best_x, best_y, best_yaw = x, y, yaw

            self.get_logger().info(f'🎯 Phase 2 best: x={best_x:.3f}, y={best_y:.3f}, yaw={math.degrees(best_yaw):.1f}° (Score: {best_score*100.0:.1f}%)')

        # ════════════════════════════════════════════════════════
        # PHASE 3: Ultra-fine refinement around Phase 2 result
        #   Step: 0.03m spatial (1 cell), 2° angular, ±0.15m
        # ════════════════════════════════════════════════════════
        if best_score > 0.15:
            uf_step = 0.03
            uf_range = 0.15
            uf_headings = [best_yaw + math.radians(d) for d in range(-10, 11, 2)]

            uf_x = [best_x + i * uf_step for i in range(-int(uf_range/uf_step), int(uf_range/uf_step) + 1)]
            uf_y = [best_y + i * uf_step for i in range(-int(uf_range/uf_step), int(uf_range/uf_step) + 1)]

            self.get_logger().info(f'🔬 PHASE 3: Ultra-fine — {len(uf_x)}×{len(uf_y)}×{len(uf_headings)} = {len(uf_x)*len(uf_y)*len(uf_headings)} candidates')

            for x in uf_x:
                for y in uf_y:
                    for yaw in uf_headings:
                        score = self.compute_scan_match_score(x, y, yaw)
                        if score is not None and score > best_score:
                            best_score = score
                            best_x, best_y, best_yaw = x, y, yaw

            self.get_logger().info(f'🎯 Phase 3 best: x={best_x:.3f}, y={best_y:.3f}, yaw={math.degrees(best_yaw):.1f}° (Score: {best_score*100.0:.1f}%)')

        # ════════════════════════════════════════════════════════
        # Apply result — grid search uses lower threshold (30%)
        # since scan-match can't achieve scores as high as when
        # the robot is already near the correct pose
        # ════════════════════════════════════════════════════════
        grid_accept_threshold = self.match_threshold * 0.65  # ~30%
        if best_score >= grid_accept_threshold:
            qz = math.sin(best_yaw / 2.0)
            qw = math.cos(best_yaw / 2.0)
            self._publish_initial_pose(best_x, best_y, qz, qw)
            self.get_logger().info('═══════════════════════════════════════════')
            self.get_logger().info(f'  ✅ ROBOT FOUND! Pose updated to AMCL')
            self.get_logger().info(f'  x   = {best_x:.3f}')
            self.get_logger().info(f'  y   = {best_y:.3f}')
            self.get_logger().info(f'  yaw = {math.degrees(best_yaw):.1f}°')
            self.get_logger().info(f'  Score = {best_score*100.0:.1f}%')
            self.get_logger().info('═══════════════════════════════════════════')
            self._publish_status(f'RELOCALIZED x={best_x:.2f} y={best_y:.2f} score={best_score*100.0:.0f}%')
        else:
            self.get_logger().warn(f'⚠ Full map search best score only {best_score*100.0:.1f}% (need {grid_accept_threshold*100:.0f}%). Falling back to AMCL 15k global...')
            if self.global_loc_client.wait_for_service(timeout_sec=1.0):
                self.global_loc_client.call_async(Empty.Request())
                self.get_logger().info('⚡ AMCL 15k particle global relocalization triggered!')
            self._publish_status('GLOBAL_RELOCALIZATION_TRIGGERED')

        self.relocalizing = False

    # ── Publish /initialpose ──────────────────────────────────────

    def _publish_initial_pose(self, x, y, qz, qw):
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = float(qz)
        msg.pose.pose.orientation.w = float(qw)

        cov = [0.0] * 36
        cov[0]  = 0.15   # tight x covariance
        cov[7]  = 0.15   # tight y covariance
        cov[35] = 0.04   # tight yaw covariance
        msg.pose.covariance = cov

        for _ in range(3):
            msg.header.stamp = self.get_clock().now().to_msg()
            self.initial_pose_pub.publish(msg)
            time.sleep(0.3)

    # ── File I/O ──────────────────────────────────────────────────

    def _save_pose_to_file(self, msg):
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
            tmp_file = self.pose_file + '.tmp'
            with open(tmp_file, 'w') as f:
                f.write('# Auto-saved robot pose\n')
                yaml.dump(pose_data, f, default_flow_style=False)
            os.replace(tmp_file, self.pose_file)
        except Exception as e:
            self.get_logger().error(f'Failed to save pose: {e}')

    def _load_pose_from_file(self):
        if not os.path.exists(self.pose_file):
            return None
        try:
            with open(self.pose_file, 'r') as f:
                data = yaml.safe_load(f)
            if not data or not isinstance(data, dict):
                return None
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
            required = ['x', 'y', 'qz', 'qw']
            for k in required:
                if k not in data:
                    return None
            return data
        except Exception as e:
            self.get_logger().error(f'Failed to load pose file: {e}')
            return None

    # ── Helpers ───────────────────────────────────────────────────

    def _quaternion_to_yaw(self, q):
        return math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )

    def _publish_status(self, text):
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)

    def destroy_node(self):
        if self.last_pose is not None and not self.relocalizing:
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
