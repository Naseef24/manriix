#!/usr/bin/env python3
"""
Home Position Manager Node

Manages a persistent home position for the robot at each venue.
The home position is saved to a YAML file and used by auto_localize.py
on startup to set the robot's initial pose.

Calibration workflow:
  1. Launch with localize_mode:=manual use_rviz:=true
  2. Set pose in RViz (2D Pose Estimate), verify robot is localized
  3. Drive/place robot at the desired home spot
  4. Save: ros2 topic pub /home/calibrate std_msgs/String "{data: save}" -1
  5. Mark the floor with tape at the robot's wheels
  6. From now on, always place robot at that tape mark before powering on
  7. Launch with default localize_mode:=home

Return to home:
  ros2 topic pub /home/return std_msgs/String "{data: go}" -1

Topics:
  /home/calibrate (sub)  - "save" to save current pose as home
  /home/return (sub)     - "go" to navigate back to home position
  /home/status (pub)     - Current home position status
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.callback_groups import ReentrantCallbackGroup

from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import String
from nav2_msgs.action import NavigateToPose

import os
import math
import yaml
from datetime import datetime


class HomePositionManager(Node):
    def __init__(self):
        super().__init__('home_position_manager')

        # Parameters
        self.declare_parameter('map_name', 'office1')
        self.declare_parameter('home_position_file', '')

        self.map_name = self.get_parameter('map_name').value

        # Home position file path
        home_file = self.get_parameter('home_position_file').value
        if not home_file:
            home_file = os.path.expanduser(
                '~/manriix2_ws/src/manriix_navigation/config/home_position.yaml')
        self.home_position_file = home_file

        # State
        self.current_pose = None
        self.home_pose = None  # Loaded from file
        self.returning_home = False
        self.cb_group = ReentrantCallbackGroup()

        # QoS
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1
        )

        # Publishers
        self.status_pub = self.create_publisher(
            String, '/home/status', reliable_qos)

        # Subscribers
        self.amcl_sub = self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose',
            self.amcl_pose_callback, 10)

        self.calibrate_sub = self.create_subscription(
            String, '/home/calibrate', self.calibrate_callback, 10,
            callback_group=self.cb_group)

        self.return_sub = self.create_subscription(
            String, '/home/return', self.return_callback, 10,
            callback_group=self.cb_group)

        # Action client for return-to-home
        self.nav_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose',
            callback_group=self.cb_group)

        # Load existing home position if available
        self._load_home_position()

        self.get_logger().info("=" * 60)
        self.get_logger().info("Home Position Manager started")
        self.get_logger().info(f"  Map: {self.map_name}")
        self.get_logger().info(f"  File: {self.home_position_file}")
        if self.home_pose:
            self.get_logger().info(
                f"  Home: ({self.home_pose['x']:.3f}, "
                f"{self.home_pose['y']:.3f}, "
                f"yaw={self.home_pose['yaw']:.2f})")
        else:
            self.get_logger().info("  Home: NOT CALIBRATED")
            self.get_logger().info(
                "  Calibrate: ros2 topic pub /home/calibrate "
                "std_msgs/String \"{data: save}\" -1")
        self.get_logger().info("=" * 60)

    # ==================== CALLBACKS ====================

    def amcl_pose_callback(self, msg: PoseWithCovarianceStamped):
        """Track current robot pose from AMCL"""
        pose = msg.pose.pose
        self.current_pose = {
            'x': pose.position.x,
            'y': pose.position.y,
            'yaw': 2.0 * math.atan2(
                pose.orientation.z, pose.orientation.w)
        }

    def calibrate_callback(self, msg: String):
        """Handle calibration commands"""
        cmd = msg.data.strip().lower()

        if cmd == 'save':
            self._save_home_position()
        elif cmd == 'show':
            self._show_home_position()
        elif cmd == 'clear':
            self._clear_home_position()
        else:
            self.get_logger().warn(
                f"Unknown calibrate command: '{cmd}'. "
                f"Use: save, show, clear")

    def return_callback(self, msg: String):
        """Handle return-to-home commands"""
        cmd = msg.data.strip().lower()

        if cmd == 'go':
            self._return_to_home()
        elif cmd == 'cancel':
            self._cancel_return()
        else:
            self.get_logger().warn(
                f"Unknown return command: '{cmd}'. Use: go, cancel")

    # ==================== SAVE HOME POSITION ====================

    def _save_home_position(self):
        """Save current AMCL pose as home position"""
        if self.current_pose is None:
            self.get_logger().error(
                "[HOME] Cannot save — no AMCL pose received yet!")
            self.get_logger().error(
                "  Make sure robot is localized first (set pose in RViz)")
            self._publish_status('error: no pose')
            return

        x = self.current_pose['x']
        y = self.current_pose['y']
        yaw = self.current_pose['yaw']

        # Create YAML data
        home_data = {
            'home_position': {
                'x': round(float(x), 4),
                'y': round(float(y), 4),
                'yaw': round(float(yaw), 4),
                'map_name': self.map_name,
                'calibrated_at': datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
                'note': 'Mark floor with tape at robot wheels for consistent placement'
            }
        }

        # Ensure config directory exists
        config_dir = os.path.dirname(self.home_position_file)
        os.makedirs(config_dir, exist_ok=True)

        # Save to YAML
        try:
            with open(self.home_position_file, 'w') as f:
                yaml.dump(home_data, f, default_flow_style=False, sort_keys=False)

            self.home_pose = home_data['home_position']

            self.get_logger().info("=" * 60)
            self.get_logger().info("[HOME] Home position SAVED!")
            self.get_logger().info(
                f"  Position: ({x:.3f}, {y:.3f}, yaw={yaw:.2f})")
            self.get_logger().info(f"  Map: {self.map_name}")
            self.get_logger().info(f"  File: {self.home_position_file}")
            self.get_logger().info("")
            self.get_logger().info(
                "  IMPORTANT: Mark the floor with tape at the robot's wheels!")
            self.get_logger().info(
                "  Always place the robot at this exact spot before powering on.")
            self.get_logger().info("=" * 60)

            self._publish_status('saved')

        except Exception as e:
            self.get_logger().error(f"[HOME] Failed to save: {e}")
            self._publish_status(f'error: {e}')

    # ==================== LOAD HOME POSITION ====================

    def _load_home_position(self):
        """Load home position from YAML file"""
        if not os.path.exists(self.home_position_file):
            self.home_pose = None
            return

        try:
            with open(self.home_position_file, 'r') as f:
                data = yaml.safe_load(f)

            hp = data.get('home_position', {})
            self.home_pose = {
                'x': hp.get('x', 0.0),
                'y': hp.get('y', 0.0),
                'yaw': hp.get('yaw', 0.0),
                'map_name': hp.get('map_name', 'unknown'),
                'calibrated_at': hp.get('calibrated_at', 'unknown'),
            }

            self.get_logger().info(
                f"[HOME] Loaded home position: "
                f"({self.home_pose['x']:.3f}, {self.home_pose['y']:.3f}, "
                f"yaw={self.home_pose['yaw']:.2f}) "
                f"map={self.home_pose['map_name']}")

        except Exception as e:
            self.get_logger().error(f"[HOME] Failed to load home file: {e}")
            self.home_pose = None

    # ==================== SHOW / CLEAR ====================

    def _show_home_position(self):
        """Display current home position"""
        if self.home_pose:
            self.get_logger().info("=" * 60)
            self.get_logger().info("[HOME] Current home position:")
            self.get_logger().info(
                f"  Position: ({self.home_pose['x']:.3f}, "
                f"{self.home_pose['y']:.3f}, "
                f"yaw={self.home_pose['yaw']:.2f})")
            self.get_logger().info(f"  Map: {self.home_pose.get('map_name', 'unknown')}")
            self.get_logger().info(
                f"  Calibrated: {self.home_pose.get('calibrated_at', 'unknown')}")
            self.get_logger().info(f"  File: {self.home_position_file}")
            self.get_logger().info("=" * 60)
        else:
            self.get_logger().info("[HOME] No home position saved.")

        if self.current_pose:
            self.get_logger().info(
                f"[HOME] Current robot pose: "
                f"({self.current_pose['x']:.3f}, "
                f"{self.current_pose['y']:.3f}, "
                f"yaw={self.current_pose['yaw']:.2f})")

    def _clear_home_position(self):
        """Remove saved home position"""
        if os.path.exists(self.home_position_file):
            os.remove(self.home_position_file)
            self.get_logger().info("[HOME] Home position file DELETED")
        self.home_pose = None
        self._publish_status('cleared')

    # ==================== RETURN TO HOME ====================

    def _return_to_home(self):
        """Navigate robot back to home position"""
        if self.home_pose is None:
            self.get_logger().error(
                "[HOME] Cannot return — no home position saved!")
            self._publish_status('error: no home')
            return

        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(
                "[HOME] NavigateToPose action server not available!")
            self._publish_status('error: no nav server')
            return

        x = self.home_pose['x']
        y = self.home_pose['y']
        yaw = self.home_pose['yaw']

        self.get_logger().info(
            f"[HOME] Returning to home: ({x:.3f}, {y:.3f}, yaw={yaw:.2f})")

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = float(x)
        goal_msg.pose.pose.position.y = float(y)
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation.z = math.sin(float(yaw) / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(float(yaw) / 2.0)

        self.returning_home = True
        self._publish_status('returning')

        send_future = self.nav_client.send_goal_async(goal_msg)
        send_future.add_done_callback(self._return_goal_response)

    def _return_goal_response(self, future):
        """Handle return-to-home goal acceptance"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("[HOME] Return goal rejected!")
            self.returning_home = False
            self._publish_status('error: rejected')
            return

        self.get_logger().info("[HOME] Navigating to home position...")
        self._return_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._return_result)

    def _return_result(self, future):
        """Handle return-to-home completion"""
        result = future.result()
        self.returning_home = False

        if result.status == 4:  # SUCCEEDED
            self.get_logger().info("=" * 60)
            self.get_logger().info("[HOME] Robot arrived at home position!")
            self.get_logger().info("=" * 60)
            self._publish_status('at_home')
        elif result.status == 5:  # CANCELED
            self.get_logger().info("[HOME] Return to home canceled")
            self._publish_status('canceled')
        else:
            self.get_logger().warn(
                f"[HOME] Return to home failed (status={result.status})")
            self._publish_status('error: nav failed')

    def _cancel_return(self):
        """Cancel return-to-home navigation"""
        if self.returning_home and hasattr(self, '_return_goal_handle'):
            self._return_goal_handle.cancel_goal_async()
            self.get_logger().info("[HOME] Canceling return to home")
        else:
            self.get_logger().info("[HOME] Not currently returning home")

    # ==================== HELPERS ====================

    def _publish_status(self, status: str):
        """Publish home position status"""
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = HomePositionManager()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()