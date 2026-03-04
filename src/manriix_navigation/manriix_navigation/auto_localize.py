#!/usr/bin/env python3
"""
Smart Localization Node for Commercial Deployment

The core problem: AMCL global localization often converges on the WRONG position,
especially in symmetric or repetitive environments. Low covariance does NOT mean
correct localization. This node addresses that.

Approach:
  Instead of blindly trusting AMCL covariance, we VERIFY localization by checking
  how well the live LIDAR scan matches the map at the estimated pose. This catches
  the common failure mode where AMCL is confident but wrong.

Modes:
  manual:   Wait for operator to set pose via RViz. Verify with scan matching.
            Warn if scan doesn't match. (Recommended for testing)

  fixed:    Use known dock/start pose. Verify with scan matching.
            (Recommended for known start positions)

  home:     Load saved home position from home_position.yaml (set by home_manager).
            Verify with scan matching. Falls back to manual if wrong.
            (Recommended for commercial deployment)

  verify:   Accept any initial pose source (RViz, external, fixed).
            Continuously verify localization quality using scan matching.
            If quality drops below threshold, warn operator.
            (Recommended for commercial deployment with continuous monitoring)

Topics published:
  /localization/status      - "initializing", "waiting_for_pose", "verifying",
                              "localized", "poor_match", "failed"
  /localization/quality     - Float32: scan match score 0.0 (bad) to 1.0 (perfect)

The node does NOT use global_localization (particle spread) because that is the
primary source of wrong-place localization in real deployments.

Usage:
  # Operator sets pose in RViz (safest)
  ros2 launch manriix_navigation waypoint_nav_with_routes.launch.py \
    localize_mode:=manual use_rviz:=true

  # Known dock position with verification
  ros2 launch manriix_navigation waypoint_nav_with_routes.launch.py \
    localize_mode:=fixed initial_x:=1.165 initial_y:=-0.88 initial_yaw:=1.54

  # Continuous quality monitoring
  ros2 launch manriix_navigation waypoint_nav_with_routes.launch.py \
    localize_mode:=verify initial_x:=1.165 initial_y:=-0.88 initial_yaw:=1.54
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String, Float32

import math
import time
import os
import yaml
import numpy as np


class SmartLocalize(Node):
    def __init__(self):
        super().__init__('auto_localize')

        # ==================== PARAMETERS ====================
        self.declare_parameter('mode', 'manual')            # manual, fixed, home, verify
        self.declare_parameter('initial_x', 0.0)            # For fixed/verify mode
        self.declare_parameter('initial_y', 0.0)
        self.declare_parameter('initial_yaw', 0.0)
        self.declare_parameter('startup_delay', 5.0)         # Wait for AMCL to start
        self.declare_parameter('scan_match_threshold', 0.30)  # Min match score to accept
        self.declare_parameter('verify_interval', 3.0)        # How often to re-verify (s)
        self.declare_parameter('scan_match_range', 10.0)      # Max LIDAR range to consider (m)
        self.declare_parameter('map_resolution_cells', 3)     # Cell tolerance for matching
        self.declare_parameter('home_position_file', '')      # Path to home_position.yaml

        self.mode = self.get_parameter('mode').value
        self.initial_x = self.get_parameter('initial_x').value
        self.initial_y = self.get_parameter('initial_y').value
        self.initial_yaw = self.get_parameter('initial_yaw').value
        self.startup_delay = self.get_parameter('startup_delay').value
        self.match_threshold = self.get_parameter('scan_match_threshold').value
        self.verify_interval = self.get_parameter('verify_interval').value
        self.scan_match_range = self.get_parameter('scan_match_range').value
        self.map_cell_tolerance = self.get_parameter('map_resolution_cells').value
        self.home_position_file = self.get_parameter('home_position_file').value

        # Default home file path if not specified
        if not self.home_position_file:
            self.home_position_file = os.path.expanduser(
                '~/manriix2_ws/src/manriix_navigation/config/home_position.yaml')

        # ==================== STATE ====================
        self.is_localized = False
        self.match_score = 0.0
        self.latest_pose = None           # Latest AMCL pose
        self.latest_scan = None           # Latest LIDAR scan
        self.map_data = None              # Occupancy grid
        self.map_info = None              # Map metadata
        self.map_np = None                # Map as numpy array
        self.pose_received = False        # Whether any pose has been set
        self.initial_pose_published = False
        self.consecutive_good_matches = 0
        self.consecutive_bad_matches = 0

        # ==================== LAST KNOWN POSE SAVING ====================
        self.last_known_pose_file = os.path.expanduser(
            '~/manriix2_ws/src/manriix_navigation/config/last_known_pose.yaml')
        self.pose_save_interval = 5.0  # Save every 5 seconds

        # ==================== PUBLISHERS ====================
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1
        )
        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', reliable_qos)
        self.status_pub = self.create_publisher(
            String, '/localization/status', reliable_qos)
        self.quality_pub = self.create_publisher(
            Float32, '/localization/quality', 10)

        # ==================== SUBSCRIBERS ====================

        # AMCL pose (tells us where AMCL thinks the robot is)
        self.amcl_sub = self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose',
            self.amcl_pose_callback, 10)

        # LIDAR scan (for verification)
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)

        # Map (for scan matching)
        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1
        )
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, map_qos)

        # ==================== STARTUP ====================
        self.get_logger().info("=" * 60)
        self.get_logger().info(f"Smart Localization Node — mode: {self.mode}")
        self.get_logger().info(f"  Scan match threshold: {self.match_threshold}")
        self.get_logger().info("=" * 60)

        self._publish_status('initializing')

        # Start after delay
        self.create_timer(self.startup_delay, self._start_once)
        self._started = False

    # ==================== SUBSCRIBER CALLBACKS ====================

    def map_callback(self, msg: OccupancyGrid):
        """Store map for scan matching"""
        self.map_info = msg.info
        self.map_data = msg.data

        # Convert to numpy array for fast lookup
        width = msg.info.width
        height = msg.info.height
        self.map_np = np.array(msg.data, dtype=np.int8).reshape((height, width))

        self.get_logger().info(
            f"Map received: {width}x{height}, "
            f"resolution={msg.info.resolution:.3f}m/cell")

    def scan_callback(self, msg: LaserScan):
        """Store latest LIDAR scan"""
        self.latest_scan = msg

    def amcl_pose_callback(self, msg: PoseWithCovarianceStamped):
        """Monitor AMCL pose updates"""
        self.latest_pose = msg
        self.pose_received = True

        # Log covariance for debugging
        cov = msg.pose.covariance
        cov_x = cov[0]
        cov_y = cov[7]
        cov_yaw = cov[35]

        pose = msg.pose.pose
        x = pose.position.x
        y = pose.position.y
        yaw = 2.0 * math.atan2(pose.orientation.z, pose.orientation.w)

        self.get_logger().debug(
            f"AMCL pose: ({x:.2f}, {y:.2f}, {yaw:.2f}) "
            f"cov: x={cov_x:.4f}, y={cov_y:.4f}, yaw={cov_yaw:.4f}")

    # ==================== STARTUP LOGIC ====================

    def _start_once(self):
        """Run once after startup delay"""
        if self._started:
            return
        self._started = True

        if self.mode == 'fixed':
            self._start_fixed()
        elif self.mode == 'home':
            self._start_home()
        elif self.mode == 'last_known':
            self._start_last_known()
        elif self.mode == 'verify':
            self._start_verify()
        elif self.mode == 'manual':
            self._start_manual()
        else:
            self.get_logger().error(f"Unknown mode: {self.mode}. Using 'manual'.")
            self._start_manual()

    def _start_fixed(self):
        """Fixed mode: publish known pose, then verify with scan matching"""
        self.get_logger().info(
            f"[FIXED] Setting initial pose: "
            f"({self.initial_x:.3f}, {self.initial_y:.3f}, "
            f"yaw={self.initial_yaw:.2f})")

        self._publish_initial_pose(
            self.initial_x, self.initial_y, self.initial_yaw,
            cov_xy=0.25, cov_yaw=0.07)

        self._publish_status('verifying')

        # Wait a moment for AMCL to process, then verify
        self.create_timer(2.0, self._verify_after_initial_pose_once)
        self._verify_initial_called = False

    def _start_home(self):
        """Home mode: load saved home pose from YAML, publish, verify"""
        self.get_logger().info(
            f"[HOME] Loading home position from: {self.home_position_file}")

        if not os.path.exists(self.home_position_file):
            self.get_logger().error("=" * 60)
            self.get_logger().error(
                "[HOME] No home_position.yaml found!")
            self.get_logger().error(
                "  You need to calibrate the home position first:")
            self.get_logger().error(
                "  1. Launch with localize_mode:=manual use_rviz:=true")
            self.get_logger().error(
                "  2. Set pose in RViz, drive robot to home spot")
            self.get_logger().error(
                "  3. Run: ros2 topic pub /home/calibrate std_msgs/String "
                "\"{data: save}\" -1")
            self.get_logger().error("=" * 60)
            self.get_logger().info("[HOME] Falling back to manual mode...")
            self._start_manual()
            return

        try:
            with open(self.home_position_file, 'r') as f:
                data = yaml.safe_load(f)

            hp = data.get('home_position', {})
            home_x = hp.get('x', 0.0)
            home_y = hp.get('y', 0.0)
            home_yaw = hp.get('yaw', 0.0)
            map_name = hp.get('map_name', 'unknown')
            calibrated_at = hp.get('calibrated_at', 'unknown')

            self.get_logger().info("=" * 60)
            self.get_logger().info("[HOME] Home position loaded!")
            self.get_logger().info(
                f"  Position: ({home_x:.3f}, {home_y:.3f}, yaw={home_yaw:.2f})")
            self.get_logger().info(f"  Map: {map_name}")
            self.get_logger().info(f"  Calibrated: {calibrated_at}")
            self.get_logger().info("=" * 60)

            # Store for use by fixed verification
            self.initial_x = home_x
            self.initial_y = home_y
            self.initial_yaw = home_yaw

            # Publish to AMCL
            self._publish_initial_pose(
                home_x, home_y, home_yaw,
                cov_xy=0.25, cov_yaw=0.07)

            self._publish_status('verifying')

            # Verify with scan matching (reuse fixed mode's verification)
            self.create_timer(2.0, self._verify_after_initial_pose_once)
            self._verify_initial_called = False

        except Exception as e:
            self.get_logger().error(f"[HOME] Failed to load home file: {e}")
            self.get_logger().info("[HOME] Falling back to manual mode...")
            self._start_manual()

    def _start_last_known(self):
        """Last known mode: load last saved pose from file, verify with scan matching"""
        self.get_logger().info(
            f"[LAST_KNOWN] Loading last known pose from: {self.last_known_pose_file}")

        if not os.path.exists(self.last_known_pose_file):
            self.get_logger().warn("=" * 60)
            self.get_logger().warn(
                "[LAST_KNOWN] No last_known_pose.yaml found!")
            self.get_logger().warn(
                "  Robot has never saved a pose. Trying home mode...")
            self.get_logger().warn("=" * 60)
            self._start_home()
            return

        try:
            with open(self.last_known_pose_file, 'r') as f:
                data = yaml.safe_load(f)

            lp = data.get('last_known_pose', {})
            lk_x = lp.get('x', 0.0)
            lk_y = lp.get('y', 0.0)
            lk_yaw = lp.get('yaw', 0.0)
            lk_time = lp.get('timestamp', 0)

            age_minutes = (time.time() - lk_time) / 60.0 if lk_time > 0 else -1

            self.get_logger().info("=" * 60)
            self.get_logger().info("[LAST_KNOWN] Last known pose loaded!")
            self.get_logger().info(
                f"  Position: ({lk_x:.3f}, {lk_y:.3f}, yaw={lk_yaw:.2f})")
            self.get_logger().info(
                f"  Age: {age_minutes:.1f} minutes ago")
            self.get_logger().info("=" * 60)

            # Warn if pose is very old (robot may have been moved)
            if age_minutes > 60:
                self.get_logger().warn(
                    f"[LAST_KNOWN] Pose is {age_minutes:.0f} minutes old! "
                    "Robot may have been moved. Verify carefully.")

            # Store for verification
            self.initial_x = lk_x
            self.initial_y = lk_y
            self.initial_yaw = lk_yaw

            # Publish to AMCL with larger covariance (less certain)
            self._publish_initial_pose(
                lk_x, lk_y, lk_yaw,
                cov_xy=0.5, cov_yaw=0.15)

            self._publish_status('verifying')

            # Verify with scan matching
            self.create_timer(2.0, self._verify_after_initial_pose_once)
            self._verify_initial_called = False

        except Exception as e:
            self.get_logger().error(f"[LAST_KNOWN] Failed to load pose: {e}")
            self.get_logger().info("[LAST_KNOWN] Falling back to home mode...")
            self._start_home()

    def _start_verify(self):
        """Verify mode: publish initial pose if provided, then continuously verify"""
        if self.initial_x != 0.0 or self.initial_y != 0.0:
            self.get_logger().info(
                f"[VERIFY] Setting initial pose estimate: "
                f"({self.initial_x:.3f}, {self.initial_y:.3f}, "
                f"yaw={self.initial_yaw:.2f})")
            self._publish_initial_pose(
                self.initial_x, self.initial_y, self.initial_yaw,
                cov_xy=0.5, cov_yaw=0.1)
        else:
            self.get_logger().info(
                "[VERIFY] No initial pose provided. "
                "Waiting for pose from RViz or external source...")

        self._publish_status('verifying')

        # Start continuous verification
        self.create_timer(self.verify_interval, self._continuous_verify)

    def _start_manual(self):
        """Manual mode: wait for operator to set pose in RViz"""
        self.get_logger().info("=" * 60)
        self.get_logger().info(
            "[MANUAL] Waiting for operator to set pose in RViz")
        self.get_logger().info(
            "  1. Open RViz")
        self.get_logger().info(
            "  2. Click '2D Pose Estimate' in toolbar")
        self.get_logger().info(
            "  3. Click and drag on map to set robot position and heading")
        self.get_logger().info("=" * 60)

        self._publish_status('waiting_for_pose')

        # Monitor for pose + verify when received
        self.create_timer(1.0, self._wait_for_manual_pose)

    # ==================== POSE WAITING & VERIFICATION ====================

    def _wait_for_manual_pose(self):
        """Check if operator has set a pose in RViz"""
        if not self.pose_received:
            return

        # Pose received — verify it
        self.get_logger().info("[MANUAL] Pose received from RViz! Verifying...")
        self._publish_status('verifying')

        # Do scan match verification
        score = self._compute_scan_match_score()
        if score is None:
            self.get_logger().warn(
                "[MANUAL] Cannot verify yet (waiting for map/scan data). "
                "Accepting pose on trust.")
            self._accept_localization("manual (unverified)")
            return

        self.match_score = score
        self._publish_quality(score)

        if score >= self.match_threshold:
            self._accept_localization(f"manual (verified, score={score:.2f})")
        else:
            self.get_logger().warn("=" * 60)
            self.get_logger().warn(
                f"[MANUAL] POOR SCAN MATCH! Score: {score:.2f} "
                f"(threshold: {self.match_threshold})")
            self.get_logger().warn(
                "  The LIDAR scan does NOT match the map well at this position.")
            self.get_logger().warn(
                "  This usually means the pose is WRONG.")
            self.get_logger().warn(
                "  → Try setting the pose again in RViz")
            self.get_logger().warn("=" * 60)
            self._publish_status('poor_match')
            # Reset and wait for another pose
            self.pose_received = False

    def _verify_after_initial_pose_once(self):
        """One-shot verification after fixed initial pose"""
        if self._verify_initial_called:
            return
        self._verify_initial_called = True

        score = self._compute_scan_match_score()
        if score is None:
            self.get_logger().warn(
                "[FIXED] Cannot verify (no map/scan data yet). "
                "Accepting fixed pose on trust.")
            self._accept_localization("fixed (unverified)")
            return

        self.match_score = score
        self._publish_quality(score)

        if score >= self.match_threshold:
            self._accept_localization(f"fixed (verified, score={score:.2f})")
        else:
            self.get_logger().error("=" * 60)
            self.get_logger().error(
                f"[FIXED] SCAN MATCH FAILED! Score: {score:.2f} "
                f"(threshold: {self.match_threshold})")
            self.get_logger().error(
                "  The fixed pose does NOT match the LIDAR data!")
            self.get_logger().error(
                "  The robot is probably NOT at the expected position.")
            self.get_logger().error(
                "  → Set pose manually in RViz (2D Pose Estimate)")
            self.get_logger().error(
                "  → Or physically move robot to the expected dock position")
            self.get_logger().error("=" * 60)
            self._publish_status('poor_match')

            # Fall back to manual — wait for RViz pose
            self.get_logger().info("[FIXED] Falling back to manual mode...")
            self.pose_received = False
            self.create_timer(1.0, self._wait_for_manual_pose)

    def _continuous_verify(self):
        """Continuously verify localization quality (verify mode)"""
        if not self.pose_received:
            return

        score = self._compute_scan_match_score()
        if score is None:
            return

        self.match_score = score
        self._publish_quality(score)

        if score >= self.match_threshold:
            self.consecutive_good_matches += 1
            self.consecutive_bad_matches = 0

            if not self.is_localized and self.consecutive_good_matches >= 2:
                self._accept_localization(f"verified (score={score:.2f})")

        else:
            self.consecutive_bad_matches += 1
            self.consecutive_good_matches = 0

            if self.consecutive_bad_matches >= 3:
                self.get_logger().warn(
                    f"[VERIFY] Localization quality degraded! "
                    f"Score: {score:.2f} ({self.consecutive_bad_matches} consecutive)")
                if self.is_localized:
                    self.is_localized = False
                    self._publish_status('poor_match')
                    self.get_logger().error(
                        "[VERIFY] LOCALIZATION LOST — robot may have been "
                        "bumped or kidnapped. Re-set pose in RViz!")

    # ==================== SCAN MATCHING ENGINE ====================

    def _compute_scan_match_score(self):
        """
        Compare live LIDAR scan against the map at the current AMCL pose.

        Projects each LIDAR ray endpoint into map coordinates and checks
        if the map cell is occupied (wall). A well-localized robot will
        have most ray endpoints hitting occupied cells.

        Returns:
            float: Match score 0.0 (no match) to 1.0 (perfect match),
                   or None if data not available
        """
        if self.latest_pose is None or self.latest_scan is None or self.map_np is None:
            return None

        pose = self.latest_pose.pose.pose
        robot_x = pose.position.x
        robot_y = pose.position.y
        robot_yaw = 2.0 * math.atan2(pose.orientation.z, pose.orientation.w)

        scan = self.latest_scan
        resolution = self.map_info.resolution
        origin_x = self.map_info.origin.position.x
        origin_y = self.map_info.origin.position.y
        map_h, map_w = self.map_np.shape

        hits = 0
        total = 0
        tolerance = self.map_cell_tolerance  # Check nearby cells too

        for i, r in enumerate(scan.ranges):
            # Skip invalid readings
            if r < scan.range_min or r > min(scan.range_max, self.scan_match_range):
                continue
            if math.isinf(r) or math.isnan(r):
                continue

            # Only use a subset of rays for performance (every 5th)
            if i % 5 != 0:
                continue

            # Compute ray endpoint in map frame
            angle = scan.angle_min + i * scan.angle_increment + robot_yaw
            end_x = robot_x + r * math.cos(angle)
            end_y = robot_y + r * math.sin(angle)

            # Convert to map cell coordinates
            cell_x = int((end_x - origin_x) / resolution)
            cell_y = int((end_y - origin_y) / resolution)

            # Check if endpoint hits an occupied cell (with tolerance)
            found_wall = False
            for dx in range(-tolerance, tolerance + 1):
                if found_wall:
                    break
                for dy in range(-tolerance, tolerance + 1):
                    cx = cell_x + dx
                    cy = cell_y + dy
                    if 0 <= cx < map_w and 0 <= cy < map_h:
                        if self.map_np[cy, cx] >= 65:  # Occupied (threshold)
                            found_wall = True
                            break

            if found_wall:
                hits += 1
            total += 1

        if total == 0:
            return None

        score = hits / total

        self.get_logger().info(
            f"[SCAN MATCH] Score: {score:.2f} "
            f"({hits}/{total} rays hit walls) "
            f"threshold={self.match_threshold}")

        return score

    # ==================== HELPERS ====================

    def _accept_localization(self, method: str):
        """Mark localization as accepted"""
        self.is_localized = True
        # Start saving pose periodically for crash recovery
        if not hasattr(self, '_pose_save_timer_started'):
            self._pose_save_timer_started = True
            self.create_timer(self.pose_save_interval, self._save_last_known_pose)        

        pose = self.latest_pose.pose.pose if self.latest_pose else None
        if pose:
            x = pose.position.x
            y = pose.position.y
            yaw = 2.0 * math.atan2(pose.orientation.z, pose.orientation.w)
            self.get_logger().info("=" * 60)
            self.get_logger().info(
                f"LOCALIZED via {method}")
            self.get_logger().info(
                f"  Position: ({x:.3f}, {y:.3f}, yaw={yaw:.2f})")
            self.get_logger().info("=" * 60)
        else:
            self.get_logger().info(f"LOCALIZED via {method}")

        self._publish_status('localized')

    def _save_last_known_pose(self):
        """Periodically save current pose to file for crash recovery"""
        if not self.is_localized or self.latest_pose is None:
            return

        pose = self.latest_pose.pose.pose
        x = pose.position.x
        y = pose.position.y
        yaw = 2.0 * math.atan2(pose.orientation.z, pose.orientation.w)

        data = {
            'last_known_pose': {
                'x': float(x),
                'y': float(y),
                'z': 0.0,
                'yaw': float(yaw),
                'timestamp': time.time(),
                'frame_id': 'map'
            }
        }

        try:
            with open(self.last_known_pose_file, 'w') as f:
                yaml.dump(data, f, default_flow_style=False)
        except Exception as e:
            self.get_logger().warn(f"Failed to save last known pose: {e}")

    def _publish_initial_pose(self, x, y, yaw, cov_xy=0.25, cov_yaw=0.07):
        """Publish initial pose estimate to AMCL"""
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.z = math.sin(float(yaw) / 2.0)
        msg.pose.pose.orientation.w = math.cos(float(yaw) / 2.0)

        msg.pose.covariance = [0.0] * 36
        msg.pose.covariance[0] = float(cov_xy)
        msg.pose.covariance[7] = float(cov_xy)
        msg.pose.covariance[35] = float(cov_yaw)

        self.initial_pose_pub.publish(msg)
        self.get_logger().info(
            f"Published initial pose: ({x:.3f}, {y:.3f}, yaw={yaw:.2f})")

    def _publish_status(self, status: str):
        """Publish localization status"""
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)

    def _publish_quality(self, score: float):
        """Publish localization quality score"""
        msg = Float32()
        msg.data = score
        self.quality_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SmartLocalize()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()