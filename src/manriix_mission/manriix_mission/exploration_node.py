#!/usr/bin/env python3
"""
Exploration Node for Manriix Photography Robot

Generates exploration goals for unknown environments.
Uses frontier-based exploration when map is incomplete,
random valid goals otherwise.
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

import tf2_ros


class ExplorationNode(Node):
    """
    Generates exploration goals for autonomous navigation.
    
    Subscriptions:
        - /map (OccupancyGrid)
        
    Publications:
        - /exploration/goal (PoseStamped)
        - /exploration/markers (MarkerArray)
        
    Services:
        - /exploration/get_goal (trigger new goal generation)
    """
    
    def __init__(self):
        super().__init__('exploration_node')
        
        # Parameters
        self.declare_parameter('min_frontier_size', 5)
        self.declare_parameter('exploration_radius', 15.0)  # meters
        self.declare_parameter('min_goal_distance', 1.5)  # meters
        self.declare_parameter('goal_timeout', 30.0)  # seconds
        
        self.min_frontier_size = self.get_parameter('min_frontier_size').value
        self.exploration_radius = self.get_parameter('exploration_radius').value
        self.min_goal_distance = self.get_parameter('min_goal_distance').value
        self.goal_timeout = self.get_parameter('goal_timeout').value
        
        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # State
        self.map_data: Optional[OccupancyGrid] = None
        self.current_goal: Optional[np.ndarray] = None
        self.goal_time: float = 0.0
        self.visited_goals: List[np.ndarray] = []
        
        # Subscribers
        self.sub_map = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        )
        
        # Publishers
        self.pub_goal = self.create_publisher(PoseStamped, '/exploration/goal', 10)
        self.pub_markers = self.create_publisher(MarkerArray, '/exploration/markers', 10)
        self.pub_has_goal = self.create_publisher(Bool, '/exploration/has_goal', 10)
        
        # Timer for goal generation
        self.timer = self.create_timer(2.0, self.generate_goal_if_needed)
        
        self.get_logger().info("Exploration Node initialized")
        
    def map_callback(self, msg: OccupancyGrid):
        """Store map data"""
        self.map_data = msg
        
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
        
    def cluster_frontiers(self, frontiers: List[np.ndarray]) -> List[np.ndarray]:
        """Cluster nearby frontier points and return centroids"""
        if not frontiers:
            return []
            
        clusters = []
        used = [False] * len(frontiers)
        
        for i, f in enumerate(frontiers):
            if used[i]:
                continue
            cluster = [f]
            used[i] = True
            
            for j, f2 in enumerate(frontiers):
                if not used[j] and np.linalg.norm(f - f2) < 1.0:
                    cluster.append(f2)
                    used[j] = True
                    
            if len(cluster) >= self.min_frontier_size:
                centroid = np.mean(cluster, axis=0)
                clusters.append(centroid)
                
        return clusters
        
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
                for visited in self.visited_goals[-20:]:  # Check last 20 goals
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
        elif np.linalg.norm(robot_pos - self.current_goal) < 0.5:
            self.get_logger().info("Exploration goal reached!")
            self.visited_goals.append(self.current_goal.copy())
            need_new_goal = True
        elif (rclpy.clock.Clock().now().nanoseconds / 1e9 - self.goal_time) > self.goal_timeout:
            self.get_logger().info("Exploration goal timed out")
            need_new_goal = True
            
        if need_new_goal:
            self.generate_new_goal()
            
        # Publish current state
        has_goal_msg = Bool()
        has_goal_msg.data = self.current_goal is not None
        self.pub_has_goal.publish(has_goal_msg)
        
        self.publish_markers()
        
    def generate_new_goal(self):
        """Generate a new exploration goal"""
        # First try frontiers
        frontiers = self.find_frontiers()
        clusters = self.cluster_frontiers(frontiers)
        
        robot_pos = self.get_robot_position()
        
        if clusters and robot_pos is not None:
            # Pick closest frontier cluster
            distances = [np.linalg.norm(c - robot_pos) for c in clusters]
            closest_idx = np.argmin(distances)
            self.current_goal = clusters[closest_idx]
            self.get_logger().info(f"New frontier goal: ({self.current_goal[0]:.2f}, {self.current_goal[1]:.2f})")
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
            marker.scale.x = 0.5
            marker.scale.y = 0.5
            marker.scale.z = 0.5
            marker.color.r = 0.0
            marker.color.g = 0.5
            marker.color.b = 1.0
            marker.color.a = 0.8
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
