#!/usr/bin/env python3
"""
Exploration Node for Manriix Photography Robot - System Integration

Enhanced frontier-based exploration adapted from friend's proven system.
Integrated with POI manager for intelligent navigation.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
import numpy as np
from typing import Optional, List, Tuple
import random

from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker, MarkerArray

# System messages
from manriix_perception.msg import POIArray, SystemStatus

import tf2_ros


class ExplorationNode(Node):
    """
    Enhanced exploration node integrated with photographer system.
    
    Subscriptions:
        - /map (OccupancyGrid)
        - /poi_manager/optimal_positions (POIArray) - For priority areas
        - /system/status (SystemStatus) - System state awareness
        
    Publications:
        - /exploration/goal (PoseStamped)
        - /exploration/markers (MarkerArray)
        - /navigation/exploration_status (Bool)
    """
    
    def __init__(self):
        super().__init__('exploration_node')
        
        # Enhanced parameters for photography robot
        self.declare_parameter('min_frontier_size', 5)
        self.declare_parameter('exploration_radius', 20.0)  # Larger for photography
        self.declare_parameter('min_goal_distance', 2.0)   # Wider spacing
        self.declare_parameter('goal_timeout', 45.0)       # More patient
        self.declare_parameter('poi_priority_weight', 0.7) # POI influence
        
        self.min_frontier_size = self.get_parameter('min_frontier_size').value
        self.exploration_radius = self.get_parameter('exploration_radius').value
        self.min_goal_distance = self.get_parameter('min_goal_distance').value
        self.goal_timeout = self.get_parameter('goal_timeout').value
        self.poi_weight = self.get_parameter('poi_priority_weight').value
        
        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # State
        self.map_data: Optional[OccupancyGrid] = None
        self.current_goal: Optional[np.ndarray] = None
        self.goal_time: float = 0.0
        self.visited_goals: List[np.ndarray] = []
        self.poi_positions: List[np.ndarray] = []
        self.system_status: Optional[str] = None
        
        # Subscribers
        self.sub_map = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        )
        
        self.sub_poi = self.create_subscription(
            POIArray,
            '/poi_manager/poi_array',  # Fixed: correct POIArray topic
            self.poi_callback,
            10
        )
        
        self.sub_system_status = self.create_subscription(
            SystemStatus,
            '/system/status',
            self.system_status_callback,
            10
        )
        
        # Publishers
        self.pub_goal = self.create_publisher(PoseStamped, '/exploration/goal', 10)
        self.pub_markers = self.create_publisher(MarkerArray, '/exploration/markers', 10)
        self.pub_status = self.create_publisher(Bool, '/navigation/exploration_status', 10)
        
        # Timer for goal generation
        self.timer = self.create_timer(3.0, self.generate_goal_if_needed)
        
        self.get_logger().info("Exploration Node initialized")
        self.get_logger().info(f"  Exploration radius: {self.exploration_radius}m")
        self.get_logger().info(f"  POI priority weight: {self.poi_weight}")
        
    def map_callback(self, msg: OccupancyGrid):
        """Store map data"""
        self.map_data = msg
        
    def poi_callback(self, msg: POIArray):
        """Update POI positions for priority navigation"""
        self.poi_positions = []
        for poi in msg.poi_list:
            self.poi_positions.append(np.array([poi.x, poi.y]))
            
    def system_status_callback(self, msg: SystemStatus):
        """Monitor system status to adjust exploration behavior"""
        self.system_status = msg.system_state
        
        # Adjust exploration behavior based on system state
        if msg.system_state in ['PHOTO_SEQUENCE', 'WAITING_FOR_STABILITY']:
            # System is working - reduce exploration aggressiveness
            self.exploration_radius = 15.0
        elif msg.system_state in ['IDLE', 'EVALUATING_TARGET']:
            # System needs targets - increase exploration
            self.exploration_radius = 25.0
        
    def get_robot_position(self) -> Optional[np.ndarray]:
        """Get current robot position"""
        try:
            transform = self.tf_buffer.lookup_transform(
                'map', 'base_footprint',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
            return np.array([
                transform.transform.translation.x,
                transform.transform.translation.y
            ])
        except Exception:
            return None
            
    def world_to_map(self, x: float, y: float) -> Tuple[int, int]:
        """Convert world coordinates to map indices"""
        if self.map_data is None:
            return (0, 0)
        mx = int((x - self.map_data.info.origin.position.x) / self.map_data.info.resolution)
        my = int((y - self.map_data.info.origin.position.y) / self.map_data.info.resolution)
        return (mx, my)
        
    def map_to_world(self, mx: int, my: int) -> Tuple[float, float]:
        """Convert map indices to world coordinates"""
        if self.map_data is None:
            return (0.0, 0.0)
        x = mx * self.map_data.info.resolution + self.map_data.info.origin.position.x
        y = my * self.map_data.info.resolution + self.map_data.info.origin.position.y
        return (x, y)
        
    def get_cell_value(self, mx: int, my: int) -> int:
        """Get occupancy value at map cell"""
        if self.map_data is None:
            return -1
        width = self.map_data.info.width
        height = self.map_data.info.height
        if 0 <= mx < width and 0 <= my < height:
            return self.map_data.data[my * width + mx]
        return -1
        
    def is_free(self, mx: int, my: int) -> bool:
        """Check if cell is free space"""
        val = self.get_cell_value(mx, my)
        return val == 0
        
    def is_unknown(self, mx: int, my: int) -> bool:
        """Check if cell is unknown"""
        val = self.get_cell_value(mx, my)
        return val == -1
        
    def is_frontier(self, mx: int, my: int) -> bool:
        """Check if cell is a frontier (free cell adjacent to unknown)"""
        if not self.is_free(mx, my):
            return False
        # Check neighbors
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                if dx == 0 and dy == 0:
                    continue
                if self.is_unknown(mx + dx, my + dy):
                    return True
        return False
        
    def find_frontiers(self) -> List[np.ndarray]:
        """Find all frontier points"""
        if self.map_data is None:
            return []
            
        frontiers = []
        width = self.map_data.info.width
        height = self.map_data.info.height
        
        robot_pos = self.get_robot_position()
        if robot_pos is None:
            return []
            
        # Search within exploration radius
        robot_mx, robot_my = self.world_to_map(robot_pos[0], robot_pos[1])
        radius_cells = int(self.exploration_radius / self.map_data.info.resolution)
        
        for mx in range(max(0, robot_mx - radius_cells), min(width, robot_mx + radius_cells)):
            for my in range(max(0, robot_my - radius_cells), min(height, robot_my + radius_cells)):
                if self.is_frontier(mx, my):
                    wx, wy = self.map_to_world(mx, my)
                    frontiers.append(np.array([wx, wy]))
                    
        return frontiers
        
    def score_frontier_with_poi(self, frontier: np.ndarray, robot_pos: np.ndarray) -> float:
        """Score frontier based on distance and POI proximity"""
        # Base score from distance (closer is better)
        dist_to_robot = np.linalg.norm(frontier - robot_pos)
        distance_score = max(0, 1.0 - dist_to_robot / self.exploration_radius)
        
        # POI proximity bonus
        poi_score = 0.0
        if self.poi_positions:
            # Find closest POI
            poi_distances = [np.linalg.norm(frontier - poi) for poi in self.poi_positions]
            closest_poi_dist = min(poi_distances)
            
            # Bonus for being near POIs (within 5m is good)
            if closest_poi_dist < 5.0:
                poi_score = max(0, 1.0 - closest_poi_dist / 5.0)
        
        # Combined score
        total_score = (1.0 - self.poi_weight) * distance_score + self.poi_weight * poi_score
        return total_score
        
    def cluster_frontiers(self, frontiers: List[np.ndarray]) -> List[np.ndarray]:
        """Cluster nearby frontier points and return scored centroids"""
        if not frontiers:
            return []
            
        robot_pos = self.get_robot_position()
        if robot_pos is None:
            return []
            
        clusters = []
        used = [False] * len(frontiers)
        
        for i, f in enumerate(frontiers):
            if used[i]:
                continue
            cluster = [f]
            used[i] = True
            
            for j, f2 in enumerate(frontiers):
                if not used[j] and np.linalg.norm(f - f2) < 1.5:  # Wider clustering for photography
                    cluster.append(f2)
                    used[j] = True
                    
            if len(cluster) >= self.min_frontier_size:
                centroid = np.mean(cluster, axis=0)
                clusters.append(centroid)
                
        return clusters
        
    def find_poi_guided_goal(self) -> Optional[np.ndarray]:
        """Find goal near unexplored POI areas"""
        if not self.poi_positions:
            return None
            
        robot_pos = self.get_robot_position()
        if robot_pos is None:
            return None
            
        # Look for areas near POIs that need exploration
        for poi in self.poi_positions:
            # Generate exploration points around POI
            for angle in np.linspace(0, 2 * np.pi, 8):
                distance = random.uniform(3.0, 8.0)  # Explore around POIs
                goal = poi + distance * np.array([np.cos(angle), np.sin(angle)])
                
                mx, my = self.world_to_map(goal[0], goal[1])
                if self.is_free(mx, my):
                    # Check if far enough from visited goals
                    too_close = False
                    for visited in self.visited_goals[-15:]:
                        if np.linalg.norm(goal - visited) < self.min_goal_distance:
                            too_close = True
                            break
                    
                    if not too_close:
                        return goal
                        
        return None
        
    def find_random_free_goal(self) -> Optional[np.ndarray]:
        """Find a random free space goal"""
        if self.map_data is None:
            return None
            
        robot_pos = self.get_robot_position()
        if robot_pos is None:
            return None
            
        # Try random points
        for _ in range(100):
            angle = random.uniform(0, 2 * np.pi)
            distance = random.uniform(self.min_goal_distance, self.exploration_radius)
            
            goal = robot_pos + distance * np.array([np.cos(angle), np.sin(angle)])
            mx, my = self.world_to_map(goal[0], goal[1])
            
            # Check if free and not too close to visited goals
            if self.is_free(mx, my):
                too_close = False
                for visited in self.visited_goals[-20:]:
                    if np.linalg.norm(goal - visited) < self.min_goal_distance:
                        too_close = True
                        break
                if not too_close:
                    return goal
                    
        return None
        
    def generate_goal_if_needed(self):
        """Generate new exploration goal if needed"""
        robot_pos = self.get_robot_position()
        if robot_pos is None:
            return
            
        # Check if current goal is reached or timed out
        need_new_goal = False
        
        if self.current_goal is None:
            need_new_goal = True
        elif np.linalg.norm(robot_pos - self.current_goal) < 0.8:  # Larger tolerance for photography
            self.get_logger().info("Exploration goal reached!")
            self.visited_goals.append(self.current_goal.copy())
            need_new_goal = True
        elif (rclpy.clock.Clock().now().nanoseconds / 1e9 - self.goal_time) > self.goal_timeout:
            self.get_logger().info("Exploration goal timed out")
            need_new_goal = True
            
        if need_new_goal:
            self.generate_new_goal()
            
        # Publish current state
        status_msg = Bool()
        status_msg.data = self.current_goal is not None
        self.pub_status.publish(status_msg)
        
        self.publish_markers()
        
    def generate_new_goal(self):
        """Generate a new exploration goal with POI awareness"""
        robot_pos = self.get_robot_position()
        if robot_pos is None:
            return
            
        # Priority order: 1) Frontiers near POIs, 2) Any frontiers, 3) POI-guided goals, 4) Random
        
        # Try frontiers first
        frontiers = self.find_frontiers()
        clusters = self.cluster_frontiers(frontiers)
        
        if clusters:
            # Score clusters by distance and POI proximity
            scores = [self.score_frontier_with_poi(c, robot_pos) for c in clusters]
            best_idx = np.argmax(scores)
            self.current_goal = clusters[best_idx]
            self.get_logger().info(f"New frontier goal (score: {scores[best_idx]:.2f}): ({self.current_goal[0]:.2f}, {self.current_goal[1]:.2f})")
        else:
            # Try POI-guided exploration
            poi_goal = self.find_poi_guided_goal()
            if poi_goal is not None:
                self.current_goal = poi_goal
                self.get_logger().info(f"New POI-guided goal: ({self.current_goal[0]:.2f}, {self.current_goal[1]:.2f})")
            else:
                # Fall back to random free space
                self.current_goal = self.find_random_free_goal()
                if self.current_goal is not None:
                    self.get_logger().info(f"New random goal: ({self.current_goal[0]:.2f}, {self.current_goal[1]:.2f})")
                else:
                    self.get_logger().warn("Could not find exploration goal")
                    return
                
        self.goal_time = rclpy.clock.Clock().now().nanoseconds / 1e9
        
        # Publish goal
        if self.current_goal is not None:
            goal_msg = PoseStamped()
            goal_msg.header.frame_id = 'map'
            goal_msg.header.stamp = self.get_clock().now().to_msg()
            goal_msg.pose.position.x = self.current_goal[0]
            goal_msg.pose.position.y = self.current_goal[1]
            goal_msg.pose.orientation.w = 1.0
            self.pub_goal.publish(goal_msg)
            
    def publish_markers(self):
        """Publish visualization markers"""
        marker_array = MarkerArray()
        
        # Delete old
        delete_marker = Marker()
        delete_marker.header.frame_id = 'map'
        delete_marker.action = Marker.DELETEALL
        marker_array.markers.append(delete_marker)
        
        # Current goal
        if self.current_goal is not None:
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'exploration_goal'
            marker.id = 0
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            marker.pose.position.x = self.current_goal[0]
            marker.pose.position.y = self.current_goal[1]
            marker.pose.position.z = 0.25
            marker.scale.x = 0.7  # Larger for photography robot
            marker.scale.y = 0.7
            marker.scale.z = 0.5
            marker.color.r = 0.0
            marker.color.g = 0.6
            marker.color.b = 1.0
            marker.color.a = 0.8
            marker_array.markers.append(marker)
            
        # POI influence zones
        for i, poi in enumerate(self.poi_positions):
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'poi_zones'
            marker.id = i + 100
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            marker.pose.position.x = poi[0]
            marker.pose.position.y = poi[1]
            marker.pose.position.z = 0.1
            marker.scale.x = 10.0  # 5m radius
            marker.scale.y = 10.0
            marker.scale.z = 0.2
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 0.1
            marker_array.markers.append(marker)
            
        self.pub_markers.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = ExplorationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()