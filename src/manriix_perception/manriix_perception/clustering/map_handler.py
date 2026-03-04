#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
from nav_msgs.msg import OccupancyGrid
from typing import Dict, Tuple, List, Optional 
from manriix_perception.clustering.performance_utils import CircularBuffer, SpatialIndex

class MapHandler:
    """Initialize the Map Handler"""
    def __init__(self, config: Dict, node: Node = None):
        self.config = config 
        self.node = node

        # Map data storage
        self.map_data = None
        self.map_msg = None  # Store full OccupancyGrid message for visualization
        self.map_width = 0
        self.map_height = 0
        self.map_resolution = 0
        self.map_origin = None
        self.last_update_time = None

        # Performance optimizations
        self.obstacle_index = SpatialIndex()
        self.free_space_buffer = CircularBuffer(100)
        self.map_update_counter = 0

        # Map configs
        self.free_threshold = config["map"]["free_threshold"]
        self.occupied_threshold = config["map"]["occupied_threshold"]
        self.unknown_cell_strategy = config["map"]["unknown_cell_strategy"]

        # Subscriber --> map
        if self.node:
            self.map_sub = self.node.create_subscription(
                OccupancyGrid,
                '/map',
                self.map_callback,
                10
            )
            self.node.get_logger().info("MapHandler initialized.")

    def map_callback(self, msg):
        """Process map and update spatial indexing"""
        self.map_msg = msg  # Store full message for visualization
        self.map_data = msg.data
        self.map_width = msg.info.width
        self.map_height = msg.info.height
        self.map_resolution = msg.info.resolution
        self.map_origin = np.array([
            msg.info.origin.position.x,
            msg.info.origin.position.y,
            msg.info.origin.position.z
        ])
        
        if self.node:
            self.last_update_time = self.node.get_clock().now()
        self.map_update_counter += 1

        # Build obstacle spatial index
        self.update_spatial_index()
        
        if self.node:
            self.node.get_logger().info(f"Map updated: {self.map_width}x{self.map_height}, res: {self.map_resolution:.2f}m/cell")

    def update_spatial_index(self):
        """Build the spatial index of obstacles for fast queries"""
        if self.map_data is None:
            return 
        
        obstacle_points = []
        for y in range(self.map_height):
            for x in range(self.map_width):
                idx = y * self.map_width + x
                if idx < len(self.map_data):
                    cell_val = self.map_data[idx]
                    if cell_val >= self.occupied_threshold:
                        wx, wy = self.grid_to_world(x, y)
                        obstacle_points.append([wx, wy])
        
        if obstacle_points:
            self.obstacle_index.build_index(np.array(obstacle_points))

    def world_to_grid(self, world_x: float, world_y: float) -> Tuple[int, int]:
        """Convert world coords to grid coords"""
        if self.map_data is None:
            return (-1, -1)
        
        offset_x = world_x - self.map_origin[0]
        offset_y = world_y - self.map_origin[1]

        grid_x = int(offset_x / self.map_resolution)
        grid_y = int(offset_y / self.map_resolution)

        return (grid_x, grid_y)
    
    def grid_to_world(self, grid_x: float, grid_y: float) -> Tuple[float, float]:
        """Convert grid coords to world coords"""
        if self.map_data is None:
            return (0.0, 0.0)
        
        world_x = self.map_origin[0] + (grid_x + 0.5) * self.map_resolution
        world_y = self.map_origin[1] + (grid_y + 0.5) * self.map_resolution

        return (world_x, world_y)
    
    def is_in_grid_bounds(self, grid_x: int, grid_y: int) -> bool:
        """Checks grid coords are within map bounds"""
        return (
            self.map_data is not None and 
            0 <= grid_x < self.map_width and
            0 <= grid_y < self.map_height)
        
    def get_cell_value(self, grid_x: int, grid_y: int) -> int:
        """Get occupancy value of grid cell"""
        if not self.is_in_grid_bounds(grid_x, grid_y):
            return -1
        
        idx = grid_y * self.map_width + grid_x
        return self.map_data[idx] 
        
    def is_position_free(self, world_x: float, world_y: float, safety_margin: int = 0) -> bool:
        """Checks the world position is in free space"""
        try:
            if self.map_data is None:
                return True
            
            grid_x, grid_y = self.world_to_grid(world_x, world_y)

            if not self.is_in_grid_bounds(grid_x, grid_y):
                return False 
            
            center_value = self.get_cell_value(grid_x, grid_y)

            if center_value == -1:
                if self.unknown_cell_strategy == "cautious":
                    return False
                elif self.unknown_cell_strategy == "exploratory":
                    return True
                elif self.unknown_cell_strategy == "adaptive":
                    return True

            if center_value >= self.free_threshold:
                return False 
            
            if safety_margin > 0:
                for dx in range(-safety_margin, safety_margin + 1):
                    for dy in range(-safety_margin, safety_margin + 1):
                        if dx == 0 and dy == 0:
                            continue 
                        
                        if dx**2 + dy**2 <= safety_margin**2:
                            nx, ny = grid_x + dx, grid_y + dy
                            if self.is_in_grid_bounds(nx, ny):
                                value = self.get_cell_value(nx, ny)

                                if value == -1 and self.unknown_cell_strategy == "cautious":
                                    return False

                                if value >= self.free_threshold:
                                    return False    
                                
            return True 
        except Exception as e:
            if self.node:
                self.node.get_logger().error(f"Error checking if position is free: {e}")
            return False 
            
    def get_obstacle_distance(self, world_x: float, world_y: float, max_distance: float = 5.0) -> float:
        """Get distance to nearest obstacle using spatial index"""
        try:
            if self.obstacle_index.kdtree is None:
                return max_distance
        
            indices, distances = self.obstacle_index.query_neighbors(
                np.array([world_x, world_y]), k=1
            )
            
            if not distances:
                return max_distance 
            
            return min(distances[0], max_distance)
            
        except Exception as e:
            if self.node:
                self.node.get_logger().error(f"Error getting obstacle distance: {e}")
            return max_distance
