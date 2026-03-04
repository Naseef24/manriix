#!/usr/bin/env python3
"""
Waypoint Navigator Node — Hybrid Route + Freespace Navigation

Navigation Architecture:
  All navigation goes through NavigateToPose action (BT Navigator).
  The behavior tree handles ComputeRoute/ComputePathToPose + SmoothPath + FollowPath + recovery.

  Two BT XMLs are used:
    - Route BT:     ComputeRoute → SmoothPath → FollowPath (graph-constrained)
    - Freespace BT: ComputePathToPose → FollowPath (free planning)

  First-mile strategy:
    - First waypoint (or when robot is far from graph): Freespace BT
    - Subsequent waypoints: Route BT
    - On route failure: Fallback to Freespace BT

Waypoints can be specified as:
  - Poses:    [{"x": 1.0, "y": 2.0, "theta": 0.0, "name": "WP1"}]
  - Node IDs: [{"node_id": 0}, {"node_id": 5}, {"node_id": 9}]
    (node IDs are resolved to (x,y) from the route graph)

Wait modes at each waypoint:
  - photo_command: Wait for /photo/command "stop" signal
  - time_based:   Wait fixed duration, publishes yes/no to /photo/command
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.callback_groups import ReentrantCallbackGroup

from geometry_msgs.msg import PoseStamped, PointStamped
from std_msgs.msg import String, Int32, Float32
from nav2_msgs.action import NavigateToPose
from visualization_msgs.msg import MarkerArray

import json
import math
import time
import threading
from enum import Enum
from typing import List, Optional, Dict
from dataclasses import dataclass, field


class WaypointState(Enum):
    IDLE = "idle"
    NAVIGATING = "navigating"
    WAITING = "waiting"
    PAUSED = "paused"
    STOPPED = "stopped"
    COMPLETED = "completed"


class WaitMode(Enum):
    PHOTO_COMMAND = "photo_command"
    TIME_BASED = "time_based"


@dataclass
class Waypoint:
    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0
    name: str = ""
    node_id: Optional[int] = None  # Graph node ID (for route mode)


class WaypointNavigator(Node):
    def __init__(self):
        super().__init__('waypoint_navigator')

        self.get_logger().info("=" * 60)
        self.get_logger().info("Waypoint Navigator — Hybrid Route + Freespace")
        self.get_logger().info("=" * 60)

        # Declare parameters
        self.declare_parameter('wait_mode', 'time_based')
        self.declare_parameter('wait_time', 300.0)
        self.declare_parameter('loop_continuous', True)
        self.declare_parameter('route_graph', '')

        # BT XML paths for dynamic switching
        self.declare_parameter('route_bt_xml', '')       # Route BT (ComputeRoute based)
        self.declare_parameter('freespace_bt_xml', '')   # Freespace BT (ComputePathToPose)

        # First-mile: distance threshold to decide if robot is "on graph"
        self.declare_parameter('on_graph_distance', 1.5)  # meters

        # Get parameters
        self.wait_mode = WaitMode(self.get_parameter('wait_mode').value)
        self.wait_time = self.get_parameter('wait_time').value
        self.loop_continuous = self.get_parameter('loop_continuous').value
        self.route_graph = self.get_parameter('route_graph').value
        # Load GeoJSON to get original node IDs
        self.geojson_node_positions: Dict[int, tuple] = {}
        self._load_geojson_nodes()
        self.route_bt_xml = self.get_parameter('route_bt_xml').value
        self.freespace_bt_xml = self.get_parameter('freespace_bt_xml').value
        self.on_graph_distance = self.get_parameter('on_graph_distance').value

        # State
        self.state = WaypointState.IDLE
        self.waypoints: List[Waypoint] = []
        self.current_waypoint_index = 0
        self.loop_count = 0
        self.photo_stop_received = False
        self.navigation_goal_handle = None
        self.is_on_graph = False  # Track if robot reached the graph
        self.route_failure_count = 0  # Track consecutive route failures
        self.max_route_failures = 2  # Fallback to freespace after N failures

        # Graph node positions (populated from /route_graph markers)
        self.graph_node_positions: Dict[int, tuple] = {}
        # Mapping: GeoJSON node ID → marker ID (built when /route_graph received)
        self.geojson_to_marker_id: Dict[int, int] = {}

        # Thread safety
        self.state_lock = threading.Lock()
        self.cb_group = ReentrantCallbackGroup()

        # QoS profiles
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )

        # Publishers
        self.status_pub = self.create_publisher(String, '/waypoint/status', reliable_qos)
        self.current_wp_pub = self.create_publisher(Int32, '/waypoint/current', reliable_qos)
        self.progress_pub = self.create_publisher(Float32, '/waypoint/progress', reliable_qos)
        self.photo_command_pub = self.create_publisher(String, '/photo/command', reliable_qos)

        # Subscribers
        self.waypoint_list_sub = self.create_subscription(
            String, '/waypoint/list', self.waypoint_list_callback, reliable_qos,
            callback_group=self.cb_group
        )
        self.control_sub = self.create_subscription(
            String, '/waypoint/control', self.control_callback, reliable_qos,
            callback_group=self.cb_group
        )
        self.config_sub = self.create_subscription(
            String, '/waypoint/config', self.config_callback, reliable_qos,
            callback_group=self.cb_group
        )
        self.photo_command_sub = self.create_subscription(
            String, '/photo/command', self.photo_command_callback, reliable_qos,
            callback_group=self.cb_group
        )

        # RViz clicked point subscriber (add waypoints by clicking)
        self.clicked_point_sub = self.create_subscription(
            PointStamped, "/clicked_point", self.clicked_point_callback, 10,
            callback_group=self.cb_group
        )

        # Subscribe to route graph markers to extract node positions
        self.route_graph_sub = self.create_subscription(
            MarkerArray, '/route_graph', self.route_graph_callback,
            QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                depth=1
            ),
            callback_group=self.cb_group
        )

        # Action Client — NavigateToPose only (BT handles everything)
        self.nav_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose',
            callback_group=self.cb_group
        )

        # Status timer
        self.status_timer = self.create_timer(1.0, self.publish_status)

        self.get_logger().info(f"Wait mode: {self.wait_mode.value}")
        self.get_logger().info(f"Wait time: {self.wait_time}s")
        self.get_logger().info(f"Loop continuous: {self.loop_continuous}")
        self.get_logger().info(f"On-graph distance: {self.on_graph_distance}m")
        if self.route_bt_xml:
            self.get_logger().info(f"Route BT: {self.route_bt_xml}")
        if self.freespace_bt_xml:
            self.get_logger().info(f"Freespace BT: {self.freespace_bt_xml}")
        if self.route_graph:
            self.get_logger().info(f"Route graph: {self.route_graph}")
        self.get_logger().info("Waypoint Navigator ready — waiting for waypoints")

    # ==================== ROUTE GRAPH EXTRACTION ====================

    def _load_geojson_nodes(self):
        """Load GeoJSON file to extract original node IDs and positions."""
        if not self.route_graph:
            self.get_logger().warn("No route_graph parameter set, cannot load GeoJSON")
            return
        try:
            with open(self.route_graph, 'r') as f:
                data = json.load(f)
            for feature in data.get('features', []):
                if feature['geometry']['type'] == 'Point':
                    node_id = feature['properties']['id']
                    coords = feature['geometry']['coordinates']
                    self.geojson_node_positions[node_id] = (coords[0], coords[1])
            self.get_logger().info(
                f"Loaded {len(self.geojson_node_positions)} nodes from GeoJSON: "
                f"{sorted(self.geojson_node_positions.keys())}")
        except Exception as e:
            self.get_logger().error(f"Failed to load GeoJSON: {e}")

    def route_graph_callback(self, msg: MarkerArray):
        """Extract graph node positions from /route_graph MarkerArray.

        The route_server publishes markers where SPHERE markers represent
        graph nodes. marker.id is the route server's internal ID (not the
        GeoJSON 'id' property). We match by coordinates to build a mapping.
        """
        count = 0
        for marker in msg.markers:
            # SPHERE (type=2) markers are nodes; LINE_LIST (type=5) are edges
            if marker.type == 2:  # visualization_msgs::Marker::SPHERE
                node_id = marker.id
                x = marker.pose.position.x
                y = marker.pose.position.y
                self.graph_node_positions[node_id] = (x, y)
                count += 1

        if count > 0:
            self.get_logger().info(
                f"Route graph: extracted {len(self.graph_node_positions)} node positions "
                f"(marker IDs: {sorted(self.graph_node_positions.keys())})")

        # Build GeoJSON ID → marker ID mapping by matching coordinates
        if count > 0 and self.geojson_node_positions and not self.geojson_to_marker_id:
            for geojson_id, (gx, gy) in self.geojson_node_positions.items():
                best_marker_id = None
                best_dist = float('inf')
                for marker_id, (mx, my) in self.graph_node_positions.items():
                    dist = math.sqrt((gx - mx)**2 + (gy - my)**2)
                    if dist < best_dist:
                        best_dist = dist
                        best_marker_id = marker_id
                if best_marker_id is not None and best_dist < 0.1:
                    self.geojson_to_marker_id[geojson_id] = best_marker_id
            if self.geojson_to_marker_id:
                self.get_logger().info(
                    f"GeoJSON to Marker ID mapping: {self.geojson_to_marker_id}")

    def resolve_node_to_pose(self, node_id: int) -> Optional[tuple]:
        """Resolve a graph node ID (GeoJSON or marker) to (x, y) position.

        Lookup order:
          1. Direct marker ID lookup (if user passes marker IDs)
          2. GeoJSON ID -> marker ID mapping (coordinate-matched)
          3. GeoJSON positions directly (fallback)
        """
        # First try GeoJSON ID -> marker ID mapping (most common case)
        if node_id in self.geojson_to_marker_id:
            marker_id = self.geojson_to_marker_id[node_id]
            if marker_id in self.graph_node_positions:
                return self.graph_node_positions[marker_id]
        # Fallback: try GeoJSON positions directly
        if node_id in self.geojson_node_positions:
            return self.geojson_node_positions[node_id]
        # Last resort: direct marker ID lookup
        if node_id in self.graph_node_positions:
            return self.graph_node_positions[node_id]
        self.get_logger().warn(
            f"Node ID {node_id} not found. "
            f"Marker IDs: {sorted(self.graph_node_positions.keys())}, "
            f"GeoJSON IDs: {sorted(self.geojson_node_positions.keys())}")
        return None

    # ==================== PUBLISHERS ====================

    def publish_photo_command(self, command: str):
        """Publish command to /photo/command topic"""
        msg = String()
        msg.data = command
        self.photo_command_pub.publish(msg)
        self.get_logger().info(f"Published '{command}' to /photo/command")

    # ==================== CALLBACKS ====================

    def clicked_point_callback(self, msg: PointStamped):
        """Add waypoint from RViz clicked point"""
        wp = Waypoint(
            x=msg.point.x,
            y=msg.point.y,
            theta=0.0,
            name=f"WP{len(self.waypoints) + 1}"
        )
        self.waypoints.append(wp)
        self.get_logger().info(
            f"Added waypoint from RViz click: ({wp.x:.2f}, {wp.y:.2f})")
        self.get_logger().info(f"Total waypoints: {len(self.waypoints)}")

    def waypoint_list_callback(self, msg: String):
        """
        Receive waypoint list as JSON.

        Supports:
        1. Pose-based: [{"x": 1.0, "y": 2.0, "theta": 0.0, "name": "WP1"}]
        2. Node ID-based: [{"node_id": 0, "name": "Start"}, {"node_id": 5}]
        3. Mixed: [{"node_id": 0}, {"x": 3.0, "y": 5.0}]
        """
        try:
            data = json.loads(msg.data)
            self.waypoints = []
            for wp_data in data:
                wp = Waypoint(
                    x=float(wp_data.get('x', 0.0)),
                    y=float(wp_data.get('y', 0.0)),
                    theta=float(wp_data.get('theta', 0.0)),
                    name=wp_data.get('name', f"WP{len(self.waypoints) + 1}"),
                    node_id=wp_data.get('node_id', None),
                )
                # Convert node_id to int if present
                if wp.node_id is not None:
                    wp.node_id = int(wp.node_id)

                    # Pre-resolve node_id to (x,y) from graph
                    pos = self.resolve_node_to_pose(wp.node_id)
                    if pos is not None:
                        wp.x, wp.y = pos
                    else:
                        self.get_logger().warn(
                            f"Cannot resolve node_id={wp.node_id}. "
                            f"Will retry at navigation time.")

                self.waypoints.append(wp)

            self.get_logger().info(f"Received {len(self.waypoints)} waypoints")
            for i, wp in enumerate(self.waypoints):
                if wp.node_id is not None:
                    self.get_logger().info(
                        f"  WP{i + 1}: node_id={wp.node_id} "
                        f"→ ({wp.x:.2f}, {wp.y:.2f}) ({wp.name})")
                else:
                    self.get_logger().info(
                        f"  WP{i + 1}: ({wp.x:.2f}, {wp.y:.2f}, "
                        f"θ={wp.theta:.2f}) ({wp.name})")

        except Exception as e:
            self.get_logger().error(f"Error parsing waypoint list: {e}")

    def config_callback(self, msg: String):
        """Receive configuration as JSON"""
        try:
            config = json.loads(msg.data)

            if 'wait_mode' in config:
                self.wait_mode = WaitMode(config['wait_mode'])
                self.get_logger().info(f"Wait mode set to: {self.wait_mode.value}")

            if 'wait_time' in config:
                self.wait_time = float(config['wait_time'])
                self.get_logger().info(f"Wait time set to: {self.wait_time}s")

            if 'loop_continuous' in config:
                self.loop_continuous = bool(config['loop_continuous'])
                self.get_logger().info(f"Loop continuous: {self.loop_continuous}")

            if 'on_graph_distance' in config:
                self.on_graph_distance = float(config['on_graph_distance'])
                self.get_logger().info(
                    f"On-graph distance: {self.on_graph_distance}m")

        except Exception as e:
            self.get_logger().error(f"Error parsing config: {e}")

    def control_callback(self, msg: String):
        """Handle control commands: start, stop, pause, resume, skip"""
        command = msg.data.lower().strip()
        self.get_logger().info(f"Received control command: {command}")

        if command == 'start':
            self.start_navigation()
        elif command == 'stop':
            self.stop_navigation()
        elif command == 'pause':
            self.pause_navigation()
        elif command == 'resume':
            self.resume_navigation()
        elif command == 'skip':
            self.skip_to_next()
        else:
            self.get_logger().warn(f"Unknown control command: {command}")

    def photo_command_callback(self, msg: String):
        """Listen for photo command responses"""
        command = msg.data.lower().strip()
        if command == 'stop' and self.wait_mode == WaitMode.PHOTO_COMMAND:
            self.get_logger().info("Received 'stop' from /photo/command")
            self.photo_stop_received = True

    # ==================== NAVIGATION CONTROL ====================

    def start_navigation(self):
        """Start waypoint navigation"""
        if not self.waypoints:
            self.get_logger().error("No waypoints to navigate!")
            return

        if self.state not in (WaypointState.IDLE, WaypointState.STOPPED):
            self.get_logger().warn(
                f"Cannot start — current state: {self.state.value}")
            return

        # Wait for NavigateToPose action server
        self.get_logger().info("Waiting for NavigateToPose action server...")
        if not self.nav_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("NavigateToPose action server not available!")
            return
        self.get_logger().info("NavigateToPose action server connected")

        # Re-resolve any unresolved node IDs
        for wp in self.waypoints:
            if wp.node_id is not None and wp.x == 0.0 and wp.y == 0.0:
                pos = self.resolve_node_to_pose(wp.node_id)
                if pos is not None:
                    wp.x, wp.y = pos
                    self.get_logger().info(
                        f"Resolved node_id={wp.node_id} → "
                        f"({wp.x:.2f}, {wp.y:.2f})")

        self.current_waypoint_index = 0
        self.loop_count = 0
        self.is_on_graph = False  # Robot starts off-graph
        self.route_failure_count = 0
        self.state = WaypointState.NAVIGATING

        self.get_logger().info(
            f"Starting waypoint navigation with {len(self.waypoints)} waypoints")
        self.get_logger().info(
            f"First waypoint uses FREESPACE (first-mile), "
            f"then ROUTE for subsequent waypoints")
        self.navigate_to_current_waypoint()

    def stop_navigation(self):
        """Stop waypoint navigation"""
        self.get_logger().warn("Stopping waypoint navigation")
        self.publish_photo_command('no')

        if self.navigation_goal_handle is not None:
            self.navigation_goal_handle.cancel_goal_async()
            self.navigation_goal_handle = None

        self.state = WaypointState.STOPPED

    def pause_navigation(self):
        """Pause waypoint navigation"""
        if self.state in (WaypointState.NAVIGATING, WaypointState.WAITING):
            self.get_logger().info("Pausing waypoint navigation")
            self.publish_photo_command('no')

            if self.navigation_goal_handle is not None:
                self.navigation_goal_handle.cancel_goal_async()

            self.state = WaypointState.PAUSED

    def resume_navigation(self):
        """Resume waypoint navigation"""
        if self.state == WaypointState.PAUSED:
            self.get_logger().info("Resuming waypoint navigation")
            self.state = WaypointState.NAVIGATING
            self.navigate_to_current_waypoint()

    def skip_to_next(self):
        """Skip to next waypoint"""
        if self.state in (WaypointState.NAVIGATING, WaypointState.WAITING,
                          WaypointState.PAUSED):
            self.get_logger().info("Skipping to next waypoint")
            self.publish_photo_command('no')

            if self.navigation_goal_handle is not None:
                self.navigation_goal_handle.cancel_goal_async()

            self.advance_to_next_waypoint()

    def retry_current_waypoint_once(self):
        """Retry current waypoint (called by timer, runs once)"""
        if self.state == WaypointState.STOPPED:
            return
        self.get_logger().info(
            f"Retrying waypoint {self.current_waypoint_index + 1}")
        self.state = WaypointState.NAVIGATING
        self.navigate_to_current_waypoint()

    # ==================== NAVIGATION DISPATCH ====================

    def _select_bt_for_waypoint(self) -> str:
        """Select the appropriate BT XML for the current waypoint.

        Strategy:
          - First waypoint (index 0) on first loop: FREESPACE (first-mile)
          - After consecutive route failures: FREESPACE (fallback)
          - Otherwise: ROUTE BT

        Returns:
            BT XML path string, or '' to use default BT.
        """
        wp_idx = self.current_waypoint_index
        wp = self.waypoints[wp_idx]

        # First waypoint on first loop — robot may be off the graph
        if wp_idx == 0 and self.loop_count == 0 and not self.is_on_graph:
            self.get_logger().info(
                f"[FIRST-MILE] WP{wp_idx+1}: Using FREESPACE BT "
                f"(robot not yet on graph)")
            return self.freespace_bt_xml

        # Too many route failures — fallback to freespace
        if self.route_failure_count >= self.max_route_failures:
            self.get_logger().info(
                f"[FALLBACK] WP{wp_idx+1}: Using FREESPACE BT "
                f"(route failed {self.route_failure_count} times)")
            return self.freespace_bt_xml

        # Normal operation — use route BT
        if self.route_bt_xml:
            self.get_logger().info(
                f"[ROUTE] WP{wp_idx+1}: Using ROUTE BT")
            return self.route_bt_xml

        # No route BT configured — use default (freespace)
        return self.freespace_bt_xml

    def navigate_to_current_waypoint(self):
        """Send NavigateToPose goal with the appropriate BT XML."""
        if self.current_waypoint_index >= len(self.waypoints):
            self.handle_loop_completion()
            return

        wp = self.waypoints[self.current_waypoint_index]
        wp_num = self.current_waypoint_index + 1
        total = len(self.waypoints)

        # Resolve node_id to (x,y) if needed
        if wp.node_id is not None and wp.x == 0.0 and wp.y == 0.0:
            pos = self.resolve_node_to_pose(wp.node_id)
            if pos is not None:
                wp.x, wp.y = pos
            else:
                self.get_logger().error(
                    f"Cannot resolve node_id={wp.node_id}! "
                    f"Skipping waypoint {wp_num}.")
                self.advance_to_next_waypoint()
                return

        # Select BT
        bt_xml = self._select_bt_for_waypoint()

        # Build NavigateToPose goal
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = wp.x
        goal_msg.pose.pose.position.y = wp.y
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation.z = math.sin(wp.theta / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(wp.theta / 2.0)

        # Set behavior tree (empty string = use default)
        goal_msg.behavior_tree = bt_xml

        if wp.node_id is not None:
            self.get_logger().info(
                f"Navigating to WP{wp_num}/{total}: {wp.name} "
                f"(node_id={wp.node_id} → {wp.x:.2f}, {wp.y:.2f})")
        else:
            self.get_logger().info(
                f"Navigating to WP{wp_num}/{total}: {wp.name} "
                f"({wp.x:.2f}, {wp.y:.2f})")

        send_goal_future = self.nav_client.send_goal_async(
            goal_msg,
            feedback_callback=self.nav_feedback_callback
        )
        send_goal_future.add_done_callback(self.nav_goal_response_callback)

    # ==================== NAVIGATION CALLBACKS ====================

    def nav_goal_response_callback(self, future):
        """Handle NavigateToPose goal response"""
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error("Navigation goal rejected!")
            self.get_logger().info("Retrying in 5 seconds...")
            self.create_timer(5.0, self.retry_current_waypoint_once)
            return

        self.navigation_goal_handle = goal_handle
        self.get_logger().debug("Navigation goal accepted")

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.nav_result_callback)

    def nav_feedback_callback(self, feedback_msg):
        """Handle NavigateToPose feedback"""
        pass

    def nav_result_callback(self, future):
        """Handle NavigateToPose result"""
        result = future.result()
        self.navigation_goal_handle = None

        if self.state in (WaypointState.PAUSED, WaypointState.STOPPED):
            return

        if result.status == 4:  # SUCCEEDED
            wp_num = self.current_waypoint_index + 1
            wp = self.waypoints[self.current_waypoint_index]
            self.get_logger().info(
                f"Arrived at waypoint {wp_num}: {wp.name}")

            # Robot is now on the graph (reached a graph node)
            if not self.is_on_graph:
                self.is_on_graph = True
                self.get_logger().info(
                    "Robot is now ON the route graph — "
                    "switching to ROUTE mode for subsequent waypoints")

            # Reset route failure counter on success
            self.route_failure_count = 0

            self.handle_waypoint_arrival()

        elif result.status == 5:  # CANCELED
            self.get_logger().info("Navigation canceled")

        else:
            wp_num = self.current_waypoint_index + 1
            self.get_logger().warn(
                f"Navigation failed (status={result.status}) "
                f"for waypoint {wp_num}")

            # Track route failures for fallback logic
            self.route_failure_count += 1
            if self.route_failure_count >= self.max_route_failures:
                self.get_logger().warn(
                    f"Route failed {self.route_failure_count} times — "
                    f"will use FREESPACE for next attempt")

            self.get_logger().info(
                f"Retrying waypoint {wp_num} in 5 seconds...")
            self.create_timer(5.0, self.retry_current_waypoint_once)

    # ==================== WAYPOINT ARRIVAL HANDLING ====================

    def handle_waypoint_arrival(self):
        """Handle arrival at a waypoint"""
        self.state = WaypointState.WAITING

        if self.wait_mode == WaitMode.PHOTO_COMMAND:
            self.handle_photo_command_wait()
        else:
            self.handle_time_based_wait()

    def handle_photo_command_wait(self):
        """Wait for photo command stop signal"""
        self.get_logger().info(
            "Photo command mode: Publishing 'yes' to /photo/command, "
            "waiting for 'stop'")

        self.photo_stop_received = False
        self.publish_photo_command('yes')

        def wait_for_stop():
            timeout = 300.0
            start_time = time.time()

            while (not self.photo_stop_received and
                   self.state == WaypointState.WAITING):
                if time.time() - start_time > timeout:
                    self.get_logger().warn(
                        "Photo command timeout — proceeding to next waypoint")
                    break
                time.sleep(0.1)

            if self.state == WaypointState.WAITING:
                self.publish_photo_command('no')
                self.advance_to_next_waypoint()

        thread = threading.Thread(target=wait_for_stop, daemon=True)
        thread.start()

    def handle_time_based_wait(self):
        """Wait for configured time with photo command signals"""
        wp_num = self.current_waypoint_index + 1
        total_wp = len(self.waypoints)

        self.get_logger().info(
            f"Time-based mode: Waypoint {wp_num}/{total_wp} reached. "
            f"Publishing 'yes', waiting {self.wait_time}s")

        self.publish_photo_command('yes')

        def wait_timer():
            remaining = self.wait_time

            while remaining > 0 and self.state == WaypointState.WAITING:
                sleep_time = min(1.0, remaining)
                time.sleep(sleep_time)
                remaining -= sleep_time

                if (remaining > 0 and int(remaining) % 5 == 0 and
                        int(remaining) != int(remaining + sleep_time)):
                    self.get_logger().info(
                        f"Wait time remaining: {int(remaining)}s")

            if self.state == WaypointState.WAITING:
                self.get_logger().info(
                    "Wait time completed. Publishing 'no' to /photo/command")
                self.publish_photo_command('no')
                time.sleep(0.5)
                self.advance_to_next_waypoint()

        thread = threading.Thread(target=wait_timer, daemon=True)
        thread.start()

    def advance_to_next_waypoint(self):
        """Move to next waypoint"""
        self.current_waypoint_index += 1

        if self.current_waypoint_index >= len(self.waypoints):
            self.handle_loop_completion()
        else:
            self.state = WaypointState.NAVIGATING
            self.navigate_to_current_waypoint()

    def handle_loop_completion(self):
        """Handle completion of all waypoints"""
        self.loop_count += 1
        self.get_logger().info(f"Completed loop {self.loop_count}")

        if self.loop_continuous:
            self.get_logger().info("Starting next loop...")
            self.current_waypoint_index = 0
            self.route_failure_count = 0
            # Robot is already on graph for subsequent loops
            self.state = WaypointState.NAVIGATING
            self.navigate_to_current_waypoint()
        else:
            self.get_logger().info("Waypoint navigation completed!")
            self.publish_photo_command('no')
            self.state = WaypointState.COMPLETED

    # ==================== STATUS PUBLISHING ====================

    def publish_status(self):
        """Publish current status"""
        bt_mode = "freespace"
        if self.is_on_graph and self.route_failure_count < self.max_route_failures:
            bt_mode = "route"

        status_data = {
            'state': self.state.value,
            'current_waypoint': self.current_waypoint_index + 1,
            'total_waypoints': len(self.waypoints),
            'loop_count': self.loop_count,
            'wait_mode': self.wait_mode.value,
            'bt_mode': bt_mode,
            'is_on_graph': self.is_on_graph,
            'route_failures': self.route_failure_count,
            'loop_continuous': self.loop_continuous,
            'graph_nodes_known': len(self.graph_node_positions),
        }

        status_msg = String()
        status_msg.data = json.dumps(status_data)
        self.status_pub.publish(status_msg)

        # Current waypoint index
        wp_msg = Int32()
        wp_msg.data = self.current_waypoint_index
        self.current_wp_pub.publish(wp_msg)

        # Progress (0.0 to 1.0)
        if self.waypoints:
            progress = (
                self.current_waypoint_index +
                (0.5 if self.state == WaypointState.WAITING else 0)
            ) / len(self.waypoints)
        else:
            progress = 0.0
        progress_msg = Float32()
        progress_msg.data = float(progress)
        self.progress_pub.publish(progress_msg)


def main(args=None):
    rclpy.init(args=args)
    node = WaypointNavigator()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()