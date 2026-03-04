#!/usr/bin/env python3
"""
POI (Point of Interest) Manager for Manriix Photography Robot - v2

Smooth Photographer Logic:
- Human stability tracking (wait before switching)
- POI queue system (don't forget opportunities)
- Score comparison with hysteresis
- Smooth transition support (no abrupt changes)

Key Principles:
1. Humans are transient - don't wait too long
2. Scenic is permanent - can return later
3. Complete nearby tasks first
4. Smooth movement always
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
import numpy as np
import time
from typing import List, Dict, Optional, Tuple
from dataclasses import dataclass, field
from collections import deque

from geometry_msgs.msg import PoseStamped, Point, Twist
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import Int32, String, Bool, Float32
from nav_msgs.msg import OccupancyGrid

import tf2_ros


@dataclass
class POI:
    """Represents a Point of Interest for photography"""
    position: np.ndarray
    poi_type: str = 'human'  # 'human' or 'scenic'
    score: float = 0.0
    human_count: int = 1
    activity_level: float = 0.0
    first_seen: float = field(default_factory=time.time)
    last_seen: float = field(default_factory=time.time)
    last_captured_time: float = 0.0
    capture_count: int = 0
    stable: bool = False  # True if seen consistently for stability_time
    
    def time_since_capture(self) -> float:
        if self.last_captured_time == 0:
            return float('inf')
        return time.time() - self.last_captured_time
    
    def time_visible(self) -> float:
        """How long has this POI been continuously visible"""
        return time.time() - self.first_seen
    
    def is_stale(self, timeout: float = 2.0) -> bool:
        return (time.time() - self.last_seen) > timeout


class POIQueue:
    """Queue for managing pending POIs"""
    def __init__(self, max_size: int = 10):
        self.queue: deque = deque(maxlen=max_size)
        
    def add(self, poi: POI):
        """Add POI to queue if not duplicate"""
        for existing in self.queue:
            if np.linalg.norm(existing.position - poi.position) < 2.0:
                # Update existing instead of adding duplicate
                existing.score = max(existing.score, poi.score)
                existing.last_seen = time.time()
                return
        self.queue.append(poi)
        
    def get_best(self) -> Optional[POI]:
        """Get highest scoring POI from queue"""
        if not self.queue:
            return None
        return max(self.queue, key=lambda p: p.score)
    
    def remove(self, position: np.ndarray, radius: float = 2.0):
        """Remove POI near position"""
        self.queue = deque(
            [p for p in self.queue if np.linalg.norm(p.position - position) > radius],
            maxlen=self.queue.maxlen
        )
        
    def cleanup_stale(self, timeout: float = 60.0):
        """Remove stale POIs from queue"""
        self.queue = deque(
            [p for p in self.queue if not p.is_stale(timeout)],
            maxlen=self.queue.maxlen
        )
        
    def __len__(self):
        return len(self.queue)


class POIManagerNode(Node):
    """
    Manages Points of Interest for photography with smooth transitions.
    
    NEW Features:
    - Human stability tracking (must be visible for X seconds)
    - POI queue (remember opportunities)
    - Hysteresis scoring (don't flip-flop)
    - Distance-to-target awareness
    
    Subscriptions:
        - /human_clusters/markers (MarkerArray)
        - /map (OccupancyGrid)
        - /mission/current_target (PoseStamped) - Current navigation target
        
    Publications:
        - /poi/target (PoseStamped) - Best photo spot
        - /poi/score (Float32) - Score of best POI
        - /poi/type (String) - 'human' or 'scenic'
        - /poi/should_capture (Bool) - True if score > threshold
        - /poi/interrupt_ok (Bool) - True if safe to switch targets
        - /poi/queue_size (Int32) - Number of queued POIs
        - /poi/markers (MarkerArray) - Visualization
    """
    
    def __init__(self):
        super().__init__('poi_manager_node')
        
        # ═══════════════════════════════════════════════════════════════════
        # PARAMETERS - Smooth Photographer Settings
        # ═══════════════════════════════════════════════════════════════════
        
        # Thresholds
        self.declare_parameter('human_score_threshold', 0.6)      # Min score to consider human POI
        self.declare_parameter('human_interrupt_threshold', 0.75) # Min score to interrupt current task
        self.declare_parameter('scenic_score_threshold', 0.4)
        
        # Timing
        self.declare_parameter('human_stability_time', 2.0)      # Seconds human must be visible
        self.declare_parameter('min_capture_interval', 60.0)     # Seconds between captures at same spot
        self.declare_parameter('scenic_check_interval', 120.0)   # Seconds without humans before scenic
        self.declare_parameter('target_lock_time', 5.0)          # Min time on one target (no flip-flop)
        
        # Distance
        self.declare_parameter('photo_distance', 2.5)            # Meters from target for photo
        self.declare_parameter('completion_threshold', 1.5)      # If closer than this, complete current
        
        # Hysteresis (prevents flip-flopping)
        self.declare_parameter('switch_score_margin', 0.15)      # New POI must beat current by this much
        
        # Load parameters
        self.human_threshold = self.get_parameter('human_score_threshold').value
        self.interrupt_threshold = self.get_parameter('human_interrupt_threshold').value
        self.scenic_threshold = self.get_parameter('scenic_score_threshold').value
        self.stability_time = self.get_parameter('human_stability_time').value
        self.min_capture_interval = self.get_parameter('min_capture_interval').value
        self.scenic_interval = self.get_parameter('scenic_check_interval').value
        self.target_lock_time = self.get_parameter('target_lock_time').value
        self.photo_distance = self.get_parameter('photo_distance').value
        self.completion_threshold = self.get_parameter('completion_threshold').value
        self.switch_margin = self.get_parameter('switch_score_margin').value
        
        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # ═══════════════════════════════════════════════════════════════════
        # STATE
        # ═══════════════════════════════════════════════════════════════════
        
        self.tracked_humans: Dict[int, POI] = {}  # id -> POI with stability tracking
        self.poi_queue = POIQueue(max_size=10)
        self.captured_locations: List[Tuple[np.ndarray, float]] = []
        self.last_human_seen = time.time()
        self.map_data: Optional[OccupancyGrid] = None
        self.robot_position = np.array([0.0, 0.0])
        
        # Current target tracking (for hysteresis)
        self.current_target_position: Optional[np.ndarray] = None
        self.current_target_score: float = 0.0
        self.current_target_type: str = ''
        self.target_set_time: float = 0.0
        
        # ═══════════════════════════════════════════════════════════════════
        # SUBSCRIBERS
        # ═══════════════════════════════════════════════════════════════════
        
        self.sub_humans = self.create_subscription(
            MarkerArray,
            '/human_clusters/markers',
            self.human_callback,
            10
        )
        self.sub_map = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            QoSProfile(depth=1, durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL)
        )
        self.sub_human_count = self.create_subscription(
            Int32,
            '/human_clusters/count',
            self.human_count_callback,
            10
        )
        # Subscribe to current mission target for distance awareness
        self.sub_current_target = self.create_subscription(
            PoseStamped,
            '/mission/current_target',
            self.current_target_callback,
            10
        )
        
        # ═══════════════════════════════════════════════════════════════════
        # PUBLISHERS
        # ═══════════════════════════════════════════════════════════════════
        
        self.pub_target = self.create_publisher(PoseStamped, '/poi/target', 10)
        self.pub_score = self.create_publisher(Float32, '/poi/score', 10)
        self.pub_type = self.create_publisher(String, '/poi/type', 10)
        self.pub_should_capture = self.create_publisher(Bool, '/poi/should_capture', 10)
        self.pub_interrupt_ok = self.create_publisher(Bool, '/poi/interrupt_ok', 10)
        self.pub_queue_size = self.create_publisher(Int32, '/poi/queue_size', 10)
        self.pub_markers = self.create_publisher(MarkerArray, '/poi/markers', 10)
        
        # Timer for periodic evaluation
        self.timer = self.create_timer(0.5, self.evaluate_and_publish)
        
        self.get_logger().info("="*60)
        self.get_logger().info("POI Manager v2 - Smooth Photographer Logic")
        self.get_logger().info("="*60)
        self.get_logger().info(f"  Human threshold: {self.human_threshold}")
        self.get_logger().info(f"  Interrupt threshold: {self.interrupt_threshold}")
        self.get_logger().info(f"  Stability time: {self.stability_time}s")
        self.get_logger().info(f"  Completion threshold: {self.completion_threshold}m")
        self.get_logger().info(f"  Target lock time: {self.target_lock_time}s")
        self.get_logger().info("="*60)
        
    # ═══════════════════════════════════════════════════════════════════════
    # CALLBACKS
    # ═══════════════════════════════════════════════════════════════════════
        
    def human_callback(self, msg: MarkerArray):
        """Process human cluster markers with stability tracking"""
        current_time = time.time()
        seen_ids = set()
        
        for marker in msg.markers:
            if marker.ns == 'human_clusters' and marker.action == Marker.ADD:
                marker_id = marker.id
                seen_ids.add(marker_id)
                
                position = np.array([
                    marker.pose.position.x,
                    marker.pose.position.y
                ])
                
                if marker_id in self.tracked_humans:
                    # Update existing tracked human
                    poi = self.tracked_humans[marker_id]
                    # Smooth position update
                    alpha = 0.3
                    poi.position = alpha * position + (1 - alpha) * poi.position
                    poi.last_seen = current_time
                    poi.activity_level = marker.color.r
                    
                    # Check stability (visible long enough)
                    if poi.time_visible() >= self.stability_time:
                        poi.stable = True
                else:
                    # New human - start tracking
                    poi = POI(
                        position=position,
                        poi_type='human',
                        activity_level=marker.color.r,
                        first_seen=current_time,
                        last_seen=current_time,
                        stable=False  # Not stable yet
                    )
                    self.tracked_humans[marker_id] = poi
                    self.get_logger().debug(f"New human detected, tracking for stability...")
                    
                self.last_human_seen = current_time
        
        # Remove stale tracked humans
        stale_ids = [
            hid for hid, poi in self.tracked_humans.items()
            if poi.is_stale(timeout=3.0)
        ]
        for hid in stale_ids:
            del self.tracked_humans[hid]
            
    def human_count_callback(self, msg: Int32):
        """Track human count"""
        if msg.data > 0:
            self.last_human_seen = time.time()
            
    def map_callback(self, msg: OccupancyGrid):
        """Store map for scene evaluation"""
        self.map_data = msg
        
    def current_target_callback(self, msg: PoseStamped):
        """Track current navigation target for distance awareness"""
        self.current_target_position = np.array([
            msg.pose.position.x,
            msg.pose.position.y
        ])
        
    # ═══════════════════════════════════════════════════════════════════════
    # HELPER METHODS
    # ═══════════════════════════════════════════════════════════════════════
        
    def get_robot_position(self) -> Optional[np.ndarray]:
        """Get current robot position in map frame"""
        try:
            transform = self.tf_buffer.lookup_transform(
                'map',
                'base_footprint',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
            return np.array([
                transform.transform.translation.x,
                transform.transform.translation.y
            ])
        except Exception:
            return None
            
    def get_distance_to_current_target(self) -> float:
        """Get distance to current navigation target"""
        if self.current_target_position is None:
            return float('inf')
            
        robot_pos = self.get_robot_position()
        if robot_pos is None:
            return float('inf')
            
        return float(np.linalg.norm(robot_pos - self.current_target_position))
            
    def calculate_photo_position(self, target: np.ndarray) -> np.ndarray:
        """Calculate optimal photo position (2.5m from target towards robot)"""
        robot_pos = self.get_robot_position()
        if robot_pos is None:
            robot_pos = self.robot_position
        else:
            self.robot_position = robot_pos
            
        direction = robot_pos - target
        dist = np.linalg.norm(direction)
        
        if dist < 0.1:
            direction = np.array([1.0, 0.0])
        else:
            direction = direction / dist
            
        photo_pos = target + direction * self.photo_distance
        return photo_pos
        
    def is_recently_captured(self, position: np.ndarray, threshold: float = 3.0) -> bool:
        """Check if this location was recently captured"""
        current_time = time.time()
        for cap_pos, cap_time in self.captured_locations:
            if current_time - cap_time < self.min_capture_interval:
                if np.linalg.norm(position - cap_pos) < threshold:
                    return True
        return False
        
    # ═══════════════════════════════════════════════════════════════════════
    # SCORING LOGIC
    # ═══════════════════════════════════════════════════════════════════════
        
    def score_human_poi(self, poi: POI) -> float:
        """
        Score a human-based POI.
        
        Factors:
        - Base score for having humans
        - Activity bonus (interesting poses)
        - Distance bonus (prefer closer)
        - Stability bonus (confirmed detection)
        - Recent capture penalty
        """
        score = 0.0
        
        # Base score
        score += 0.35
        
        # Stability bonus - IMPORTANT: unstable humans get reduced score
        if poi.stable:
            score += 0.25
        else:
            # Not stable yet - significantly reduce score
            remaining = max(0, self.stability_time - poi.time_visible())
            self.get_logger().debug(f"Human not stable yet, {remaining:.1f}s remaining")
            score -= 0.2
        
        # Activity bonus (moving/talking people are interesting, but not too much)
        # Sweet spot is moderate activity (0.2-0.5)
        if 0.1 < poi.activity_level < 0.6:
            score += 0.15  # Good activity level
        elif poi.activity_level >= 0.6:
            score += 0.05  # Too active - might be leaving
                
        # Distance bonus (prefer closer targets, but not too close)
        robot_pos = self.get_robot_position()
        if robot_pos is not None:
            dist = np.linalg.norm(poi.position - robot_pos)
            if 2.0 < dist < 8.0:
                score += 0.15  # Optimal distance range
            elif dist <= 2.0:
                score += 0.05  # Very close - already good
                
        # Recent capture penalty
        if self.is_recently_captured(poi.position):
            score -= 0.4
            
        return max(0.0, min(1.0, score))
        
    def generate_scenic_poi(self) -> Optional[POI]:
        """Generate a scenic POI when no humans around"""
        if self.map_data is None:
            return None
            
        robot_pos = self.get_robot_position()
        if robot_pos is None:
            return None
            
        poi = POI(
            position=robot_pos.copy(),
            poi_type='scenic',
            stable=True  # Scenic is always "stable"
        )
        poi.score = 0.3
        
        if not self.is_recently_captured(robot_pos):
            poi.score += 0.2
            
        return poi
    
    # ═══════════════════════════════════════════════════════════════════════
    # DECISION LOGIC - Smooth Photographer
    # ═══════════════════════════════════════════════════════════════════════
        
    def should_interrupt_current_target(self, new_poi: POI, new_score: float) -> bool:
        """
        Decide if we should interrupt current navigation for new POI.
        
        Rules:
        1. If close to current target (< completion_threshold), DON'T interrupt
        2. If target lock time hasn't passed, DON'T interrupt
        3. If new POI isn't significantly better (score margin), DON'T interrupt
        4. Only stable humans can interrupt
        """
        current_time = time.time()
        
        # Rule 1: Don't interrupt if almost at target
        dist_to_target = self.get_distance_to_current_target()
        if dist_to_target < self.completion_threshold:
            self.get_logger().debug(
                f"Won't interrupt: {dist_to_target:.1f}m from target (threshold: {self.completion_threshold}m)"
            )
            return False
            
        # Rule 2: Don't interrupt during target lock time
        time_on_target = current_time - self.target_set_time
        if time_on_target < self.target_lock_time:
            self.get_logger().debug(
                f"Won't interrupt: target lock ({time_on_target:.1f}s / {self.target_lock_time}s)"
            )
            return False
            
        # Rule 3: New POI must be significantly better
        if new_score < self.current_target_score + self.switch_margin:
            self.get_logger().debug(
                f"Won't interrupt: score {new_score:.2f} not better than "
                f"{self.current_target_score:.2f} + {self.switch_margin} margin"
            )
            return False
            
        # Rule 4: Only stable humans can interrupt (unless very high score)
        if new_poi.poi_type == 'human' and not new_poi.stable:
            if new_score < 0.85:  # Unless extremely high score
                self.get_logger().debug("Won't interrupt: human not stable yet")
                return False
                
        # Rule 5: Must meet interrupt threshold
        if new_score < self.interrupt_threshold:
            self.get_logger().debug(
                f"Won't interrupt: score {new_score:.2f} below interrupt threshold {self.interrupt_threshold}"
            )
            return False
            
        self.get_logger().info(
            f"✓ Interrupt OK: score {new_score:.2f} > current {self.current_target_score:.2f}, "
            f"dist to target: {dist_to_target:.1f}m"
        )
        return True
        
    def evaluate_and_publish(self):
        """Main evaluation loop - applies smooth photographer logic"""
        current_time = time.time()
        
        # Cleanup
        self.poi_queue.cleanup_stale(timeout=60.0)
        
        # Get all stable human POIs
        stable_pois: List[POI] = []
        unstable_pois: List[POI] = []
        
        for poi in self.tracked_humans.values():
            poi.score = self.score_human_poi(poi)
            if poi.stable:
                stable_pois.append(poi)
            else:
                unstable_pois.append(poi)
                
        # Find best POI among stable ones
        best_poi: Optional[POI] = None
        best_score = 0.0
        
        for poi in stable_pois:
            if poi.score > best_score:
                best_score = poi.score
                best_poi = poi
                
        # Queue unstable but promising POIs (for later)
        for poi in unstable_pois:
            if poi.score > 0.4:  # Potentially interesting
                self.poi_queue.add(poi)
                
        # Check for scenic opportunity
        time_since_human = current_time - self.last_human_seen
        
        if best_score < self.human_threshold and time_since_human > self.scenic_interval:
            scenic_poi = self.generate_scenic_poi()
            if scenic_poi and scenic_poi.score > self.scenic_threshold:
                if scenic_poi.score > best_score:
                    best_poi = scenic_poi
                    best_score = scenic_poi.score
                    
        # If no good POI, check queue
        if best_poi is None and len(self.poi_queue) > 0:
            queued = self.poi_queue.get_best()
            if queued and queued.score > self.human_threshold:
                best_poi = queued
                best_score = queued.score
                
        # Determine should_capture and interrupt_ok
        should_capture = False
        interrupt_ok = False
        
        if best_poi is not None:
            if best_poi.poi_type == 'human' and best_score >= self.human_threshold:
                should_capture = True
                interrupt_ok = self.should_interrupt_current_target(best_poi, best_score)
            elif best_poi.poi_type == 'scenic' and best_score >= self.scenic_threshold:
                should_capture = True
                interrupt_ok = self.should_interrupt_current_target(best_poi, best_score)
                
        # Publish results
        self.publish_results(best_poi, best_score, should_capture, interrupt_ok)
        self.publish_markers(stable_pois, unstable_pois)
        
    def publish_results(self, poi: Optional[POI], score: float, 
                       should_capture: bool, interrupt_ok: bool):
        """Publish POI information"""
        
        # Score
        score_msg = Float32()
        score_msg.data = score
        self.pub_score.publish(score_msg)
        
        # Should capture
        capture_msg = Bool()
        capture_msg.data = should_capture
        self.pub_should_capture.publish(capture_msg)
        
        # Interrupt OK
        interrupt_msg = Bool()
        interrupt_msg.data = interrupt_ok
        self.pub_interrupt_ok.publish(interrupt_msg)
        
        # Queue size
        queue_msg = Int32()
        queue_msg.data = len(self.poi_queue)
        self.pub_queue_size.publish(queue_msg)
        
        if poi is not None and (should_capture or interrupt_ok):
            # Only publish new target if it's actionable
            photo_pos = self.calculate_photo_position(poi.position)
            
            pose_msg = PoseStamped()
            pose_msg.header.frame_id = 'map'
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.pose.position.x = float(photo_pos[0])
            pose_msg.pose.position.y = float(photo_pos[1])
            pose_msg.pose.position.z = 0.0
            
            # Orientation facing the target
            direction = poi.position - photo_pos
            yaw = np.arctan2(direction[1], direction[0])
            pose_msg.pose.orientation.z = np.sin(yaw / 2)
            pose_msg.pose.orientation.w = np.cos(yaw / 2)
            
            self.pub_target.publish(pose_msg)
            
            # Update tracking
            self.current_target_position = photo_pos
            self.current_target_score = score
            self.current_target_type = poi.poi_type
            self.target_set_time = time.time()
            
            # Type
            type_msg = String()
            type_msg.data = poi.poi_type
            self.pub_type.publish(type_msg)
        else:
            # Still publish type for UI
            type_msg = String()
            type_msg.data = poi.poi_type if poi else 'none'
            self.pub_type.publish(type_msg)
            
    def publish_markers(self, stable_pois: List[POI], unstable_pois: List[POI]):
        """Publish visualization markers"""
        marker_array = MarkerArray()
        
        # Delete old
        delete_marker = Marker()
        delete_marker.header.frame_id = 'map'
        delete_marker.action = Marker.DELETEALL
        marker_array.markers.append(delete_marker)
        
        marker_id = 0
        
        # Stable POI markers (solid green)
        for poi in stable_pois:
            marker = self._create_poi_marker(
                poi, marker_id, 
                color=(0.0, 1.0, 0.0, 0.8),  # Green = stable
                label=f"STABLE\n{poi.score:.2f}"
            )
            marker_array.markers.append(marker)
            marker_id += 1
            
            # Photo position arrow
            photo_marker = self._create_photo_arrow(poi, marker_id)
            marker_array.markers.append(photo_marker)
            marker_id += 1
            
        # Unstable POI markers (yellow, dashed effect via transparency)
        for poi in unstable_pois:
            remaining = max(0, self.stability_time - poi.time_visible())
            marker = self._create_poi_marker(
                poi, marker_id,
                color=(1.0, 1.0, 0.0, 0.5),  # Yellow + transparent = unstable
                label=f"WAIT {remaining:.1f}s"
            )
            marker_array.markers.append(marker)
            marker_id += 1
            
        # Queue markers (blue)
        for i, poi in enumerate(self.poi_queue.queue):
            marker = self._create_poi_marker(
                poi, marker_id,
                color=(0.3, 0.3, 1.0, 0.5),  # Blue = queued
                label=f"Q{i+1}"
            )
            marker.scale.x = 0.3  # Smaller
            marker.scale.y = 0.3
            marker_array.markers.append(marker)
            marker_id += 1
            
        self.pub_markers.publish(marker_array)
        
    def _create_poi_marker(self, poi: POI, marker_id: int, 
                          color: Tuple[float, float, float, float],
                          label: str) -> Marker:
        """Create a POI visualization marker"""
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'poi_targets'
        marker.id = marker_id
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        marker.pose.position.x = float(poi.position[0])
        marker.pose.position.y = float(poi.position[1])
        marker.pose.position.z = 0.25
        marker.scale.x = 0.5
        marker.scale.y = 0.5
        marker.scale.z = 0.5
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = color[3]
        return marker
        
    def _create_photo_arrow(self, poi: POI, marker_id: int) -> Marker:
        """Create arrow showing photo position"""
        photo_pos = self.calculate_photo_position(poi.position)
        
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'photo_positions'
        marker.id = marker_id
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose.position.x = float(photo_pos[0])
        marker.pose.position.y = float(photo_pos[1])
        marker.pose.position.z = 0.1
        
        direction = poi.position - photo_pos
        yaw = np.arctan2(direction[1], direction[0])
        marker.pose.orientation.z = np.sin(yaw / 2)
        marker.pose.orientation.w = np.cos(yaw / 2)
        
        marker.scale.x = 0.8
        marker.scale.y = 0.15
        marker.scale.z = 0.15
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.5
        marker.color.a = 0.8
        return marker
        
    def mark_captured(self, position: np.ndarray):
        """Mark a position as captured"""
        self.captured_locations.append((position.copy(), time.time()))
        self.poi_queue.remove(position)
        if len(self.captured_locations) > 50:
            self.captured_locations.pop(0)


def main(args=None):
    rclpy.init(args=args)
    node = POIManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


# #!/usr/bin/env python3
# """
# POI (Point of Interest) Manager for Manriix Photography Robot

# Evaluates human clusters and scene quality to identify good photo spots.
# Publishes best POI for the mission controller.
# """

# import rclpy
# from rclpy.node import Node
# from rclpy.qos import QoSProfile, ReliabilityPolicy
# import numpy as np
# import time
# from typing import List, Dict, Optional, Tuple

# from geometry_msgs.msg import PoseStamped, Point
# from visualization_msgs.msg import MarkerArray, Marker
# from std_msgs.msg import Int32, String, Bool, Float32
# from nav_msgs.msg import OccupancyGrid

# import tf2_ros


# class POI:
#     """Represents a Point of Interest for photography"""
#     def __init__(self, position: np.ndarray, poi_type: str = 'human'):
#         self.position = position  # [x, y] in map frame
#         self.type = poi_type  # 'human' or 'scenic'
#         self.score = 0.0
#         self.human_count = 0
#         self.activity_level = 0.0
#         self.last_captured_time = 0.0
#         self.capture_count = 0
        
#     def time_since_capture(self) -> float:
#         if self.last_captured_time == 0:
#             return float('inf')
#         return time.time() - self.last_captured_time


# class POIManagerNode(Node):
#     """
#     Manages Points of Interest for photography.
    
#     Subscriptions:
#         - /human_clusters/markers (MarkerArray)
#         - /map (OccupancyGrid)
        
#     Publications:
#         - /poi/best (PoseStamped) - Best photo spot
#         - /poi/score (Float32) - Score of best POI
#         - /poi/type (String) - 'human' or 'scenic'
#         - /poi/should_capture (Bool) - True if score > threshold
#         - /poi/markers (MarkerArray) - Visualization
#     """
    
#     def __init__(self):
#         super().__init__('poi_manager_node')
        
#         # Parameters
#         self.declare_parameter('human_score_threshold', 0.5)
#         self.declare_parameter('scenic_score_threshold', 0.4)
#         self.declare_parameter('min_capture_interval', 60.0)  # seconds
#         self.declare_parameter('photo_distance', 2.5)  # meters from target
#         self.declare_parameter('scenic_check_interval', 120.0)  # seconds without humans
        
#         self.human_threshold = self.get_parameter('human_score_threshold').value
#         self.scenic_threshold = self.get_parameter('scenic_score_threshold').value
#         self.min_capture_interval = self.get_parameter('min_capture_interval').value
#         self.photo_distance = self.get_parameter('photo_distance').value
#         self.scenic_interval = self.get_parameter('scenic_check_interval').value
        
#         # TF
#         self.tf_buffer = tf2_ros.Buffer()
#         self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
#         # State
#         self.current_pois: List[POI] = []
#         self.captured_locations: List[Tuple[np.ndarray, float]] = []  # (position, time)
#         self.last_human_seen = time.time()
#         self.map_data: Optional[OccupancyGrid] = None
#         self.robot_position = np.array([0.0, 0.0])
        
#         # Subscribers
#         self.sub_humans = self.create_subscription(
#             MarkerArray,
#             '/human_clusters/markers',
#             self.human_callback,
#             10
#         )
#         self.sub_map = self.create_subscription(
#             OccupancyGrid,
#             '/map',
#             self.map_callback,
#             QoSProfile(depth=1, durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL)
#         )
#         self.sub_human_count = self.create_subscription(
#             Int32,
#             '/human_clusters/count',
#             self.human_count_callback,
#             10
#         )
        
#         # Publishers
#         self.pub_best_poi = self.create_publisher(PoseStamped, '/poi/best', 10)
#         self.pub_score = self.create_publisher(Float32, '/poi/score', 10)
#         self.pub_type = self.create_publisher(String, '/poi/type', 10)
#         self.pub_should_capture = self.create_publisher(Bool, '/poi/should_capture', 10)
#         self.pub_markers = self.create_publisher(MarkerArray, '/poi/markers', 10)
        
#         # Timer for periodic evaluation
#         self.timer = self.create_timer(0.5, self.evaluate_and_publish)
        
#         self.get_logger().info("POI Manager initialized")
#         self.get_logger().info(f"  Human threshold: {self.human_threshold}")
#         self.get_logger().info(f"  Photo distance: {self.photo_distance}m")
        
#     def human_callback(self, msg: MarkerArray):
#         """Process human cluster markers"""
#         self.current_pois = []
        
#         for marker in msg.markers:
#             if marker.ns == 'human_clusters' and marker.action == Marker.ADD:
#                 position = np.array([
#                     marker.pose.position.x,
#                     marker.pose.position.y
#                 ])
                
#                 poi = POI(position, 'human')
#                 poi.activity_level = marker.color.r  # Red component = activity
#                 self.current_pois.append(poi)
#                 self.last_human_seen = time.time()
                
#     def human_count_callback(self, msg: Int32):
#         """Track human count for logging"""
#         if msg.data > 0:
#             self.last_human_seen = time.time()
            
#     def map_callback(self, msg: OccupancyGrid):
#         """Store map for scene evaluation"""
#         self.map_data = msg
        
#     def get_robot_position(self) -> Optional[np.ndarray]:
#         """Get current robot position in map frame"""
#         try:
#             transform = self.tf_buffer.lookup_transform(
#                 'map',
#                 'base_footprint',
#                 rclpy.time.Time(),
#                 timeout=rclpy.duration.Duration(seconds=0.1)
#             )
#             return np.array([
#                 transform.transform.translation.x,
#                 transform.transform.translation.y
#             ])
#         except Exception:
#             return None
            
#     def calculate_photo_position(self, target: np.ndarray) -> np.ndarray:
#         """Calculate optimal photo position (2.5m from target towards robot)"""
#         robot_pos = self.get_robot_position()
#         if robot_pos is None:
#             robot_pos = self.robot_position
#         else:
#             self.robot_position = robot_pos
            
#         # Direction from target to robot
#         direction = robot_pos - target
#         dist = np.linalg.norm(direction)
        
#         if dist < 0.1:
#             # Robot is at target, use random direction
#             direction = np.array([1.0, 0.0])
#         else:
#             direction = direction / dist
            
#         # Photo position is photo_distance meters from target
#         photo_pos = target + direction * self.photo_distance
#         return photo_pos
        
#     def is_recently_captured(self, position: np.ndarray, threshold: float = 3.0) -> bool:
#         """Check if this location was recently captured"""
#         current_time = time.time()
#         for cap_pos, cap_time in self.captured_locations:
#             if current_time - cap_time < self.min_capture_interval:
#                 if np.linalg.norm(position - cap_pos) < threshold:
#                     return True
#         return False
        
#     def score_human_poi(self, poi: POI) -> float:
#         """Score a human-based POI"""
#         score = 0.0
        
#         # Base score for having humans
#         score += 0.4
        
#         # Activity bonus (moving/talking people are more interesting)
#         score += poi.activity_level * 0.2
        
#         # Distance penalty (prefer closer targets)
#         robot_pos = self.get_robot_position()
#         if robot_pos is not None:
#             dist = np.linalg.norm(poi.position - robot_pos)
#             if dist < 10.0:
#                 score += (1.0 - dist / 10.0) * 0.2
                
#         # Recent capture penalty
#         if self.is_recently_captured(poi.position):
#             score -= 0.3
            
#         return max(0.0, min(1.0, score))
        
#     def generate_scenic_poi(self) -> Optional[POI]:
#         """Generate a scenic POI when no humans around"""
#         if self.map_data is None:
#             return None
            
#         robot_pos = self.get_robot_position()
#         if robot_pos is None:
#             return None
            
#         # Find open areas in the map
#         # For now, use robot's current position as scenic spot
#         # TODO: Implement proper scene interest scoring
        
#         poi = POI(robot_pos.copy(), 'scenic')
#         poi.score = 0.3  # Base scenic score
        
#         # Bonus if we haven't captured here
#         if not self.is_recently_captured(robot_pos):
#             poi.score += 0.2
            
#         return poi
        
#     def evaluate_and_publish(self):
#         """Evaluate all POIs and publish best one"""
#         best_poi: Optional[POI] = None
#         best_score = 0.0
        
#         # Score human POIs
#         for poi in self.current_pois:
#             poi.score = self.score_human_poi(poi)
#             if poi.score > best_score:
#                 best_score = poi.score
#                 best_poi = poi
                
#         # Check for scenic opportunity if no good human POI
#         time_since_human = time.time() - self.last_human_seen
        
#         if best_score < self.human_threshold and time_since_human > self.scenic_interval:
#             scenic_poi = self.generate_scenic_poi()
#             if scenic_poi and scenic_poi.score > self.scenic_threshold:
#                 if scenic_poi.score > best_score:
#                     best_poi = scenic_poi
#                     best_score = scenic_poi.score
                    
#         # Determine if we should capture
#         should_capture = False
#         if best_poi is not None:
#             if best_poi.type == 'human' and best_score >= self.human_threshold:
#                 should_capture = True
#             elif best_poi.type == 'scenic' and best_score >= self.scenic_threshold:
#                 should_capture = True
                
#         # Publish results
#         self.publish_results(best_poi, best_score, should_capture)
#         self.publish_markers()
        
#     def publish_results(self, poi: Optional[POI], score: float, should_capture: bool):
#         """Publish POI information"""
#         # Score
#         score_msg = Float32()
#         score_msg.data = score
#         self.pub_score.publish(score_msg)
        
#         # Should capture
#         capture_msg = Bool()
#         capture_msg.data = should_capture
#         self.pub_should_capture.publish(capture_msg)
        
#         if poi is not None:
#             # Best POI position (photo position, not target position)
#             photo_pos = self.calculate_photo_position(poi.position)
            
#             pose_msg = PoseStamped()
#             pose_msg.header.frame_id = 'map'
#             pose_msg.header.stamp = self.get_clock().now().to_msg()
#             pose_msg.pose.position.x = photo_pos[0]
#             pose_msg.pose.position.y = photo_pos[1]
#             pose_msg.pose.position.z = 0.0
            
#             # Orientation facing the target
#             direction = poi.position - photo_pos
#             yaw = np.arctan2(direction[1], direction[0])
#             pose_msg.pose.orientation.z = np.sin(yaw / 2)
#             pose_msg.pose.orientation.w = np.cos(yaw / 2)
            
#             self.pub_best_poi.publish(pose_msg)
            
#             # Type
#             type_msg = String()
#             type_msg.data = poi.type
#             self.pub_type.publish(type_msg)
            
#     def publish_markers(self):
#         """Publish visualization markers"""
#         marker_array = MarkerArray()
        
#         # Delete old
#         delete_marker = Marker()
#         delete_marker.header.frame_id = 'map'
#         delete_marker.action = Marker.DELETEALL
#         marker_array.markers.append(delete_marker)
        
#         # POI markers
#         for i, poi in enumerate(self.current_pois):
#             # Target marker
#             marker = Marker()
#             marker.header.frame_id = 'map'
#             marker.header.stamp = self.get_clock().now().to_msg()
#             marker.ns = 'poi_targets'
#             marker.id = i
#             marker.type = Marker.SPHERE
#             marker.action = Marker.ADD
#             marker.pose.position.x = poi.position[0]
#             marker.pose.position.y = poi.position[1]
#             marker.pose.position.z = 0.2
#             marker.scale.x = 0.3
#             marker.scale.y = 0.3
#             marker.scale.z = 0.3
#             marker.color.r = 1.0
#             marker.color.g = 0.5
#             marker.color.b = 0.0
#             marker.color.a = 0.8
#             marker_array.markers.append(marker)
            
#             # Photo position marker
#             photo_pos = self.calculate_photo_position(poi.position)
#             photo_marker = Marker()
#             photo_marker.header.frame_id = 'map'
#             photo_marker.header.stamp = self.get_clock().now().to_msg()
#             photo_marker.ns = 'photo_positions'
#             photo_marker.id = i
#             photo_marker.type = Marker.ARROW
#             photo_marker.action = Marker.ADD
#             photo_marker.pose.position.x = photo_pos[0]
#             photo_marker.pose.position.y = photo_pos[1]
#             photo_marker.pose.position.z = 0.1
            
#             # Arrow pointing at target
#             direction = poi.position - photo_pos
#             yaw = np.arctan2(direction[1], direction[0])
#             photo_marker.pose.orientation.z = np.sin(yaw / 2)
#             photo_marker.pose.orientation.w = np.cos(yaw / 2)
            
#             photo_marker.scale.x = 0.8
#             photo_marker.scale.y = 0.15
#             photo_marker.scale.z = 0.15
#             photo_marker.color.r = 0.0
#             photo_marker.color.g = 1.0
#             photo_marker.color.b = 0.0
#             photo_marker.color.a = 0.8
#             marker_array.markers.append(photo_marker)
            
#         self.pub_markers.publish(marker_array)
        
#     def mark_captured(self, position: np.ndarray):
#         """Mark a position as captured"""
#         self.captured_locations.append((position.copy(), time.time()))
#         # Keep only last 50 captures
#         if len(self.captured_locations) > 50:
#             self.captured_locations.pop(0)


# def main(args=None):
#     rclpy.init(args=args)
#     node = POIManagerNode()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()


# if __name__ == '__main__':
#     main()
