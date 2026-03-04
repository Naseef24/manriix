#!/usr/bin/env python3

import numpy as np
from typing import Dict, List, Tuple, Optional
from rclpy.node import Node
from manriix_perception.clustering.performance_utils import CircularBuffer, ParallelProcessor


class ObstacleAnalyzer:
    """Analyzes obstacle patterns and calculates env complexity"""

    def __init__(self, config: Dict, map_handler, node: Node = None):
        
        self.config = config
        self.map_handler = map_handler
        self.node = node

        # Configs
        obstacle_config = config.get('obstacle', {})
        self.min_safety_margin = obstacle_config.get('min_safety_margin', 1)
        self.max_safety_margin = obstacle_config.get('max_safety_margin', 3)
        self.density_radius = obstacle_config.get('density_radius', 2.0)
        self.complexity_method = obstacle_config.get('complexity_calculation', 'entropy')

        # Performance optimizations
        self.parallel_processor = ParallelProcessor()
        self.density_cache = CircularBuffer(100)  # Cache recent density calculations
        self.complexity_cache = CircularBuffer(100)  # Cache recent complexity calculations

        if self.node:
            self.node.get_logger().info("ObstacleAnalyzer initialized")

    def calculate_obstacle_density(self, world_x: float, world_y: float, radius: float) -> float:
        """Calculate obstacle density in an area with caching"""
        try:
            cache_key = f"density_{world_x:.1f}_{world_y:.1f}_{radius:.1f}"

            # Find key in cache
            for item in self.density_cache.get_data():
                if item[0] == cache_key:
                    return item[1]
                
            # Not in cache, calculate density
            if self.map_handler.map_data is None:
                return 0.0
        
            grid_x, grid_y = self.map_handler.world_to_grid(world_x, world_y)
            if not self.map_handler.is_in_grid_bounds(grid_x, grid_y):
                return 0.0
            
            # Convert the radius from meters to grid cells
            cell_radius = int(radius / self.map_handler.map_resolution)
            total_cells = 0
            obstacle_cells = 0 

            # Count occupied cells in radius
            for dx in range(-cell_radius, cell_radius + 1):
                for dy in range(-cell_radius, cell_radius + 1):
                    dist_sq = dx**2 + dy**2 
                    if dist_sq <= cell_radius**2:
                        nx, ny = grid_x + dx, grid_y + dy 
                        if self.map_handler.is_in_grid_bounds(nx, ny):
                            total_cells += 1
                            value = self.map_handler.get_cell_value(nx, ny)
                            if value >= self.map_handler.occupied_threshold:
                                obstacle_cells += 1

            # Calculate density
            density = obstacle_cells / total_cells if total_cells > 0 else 0.0

            # Store in cache
            self.density_cache.append((cache_key, density))

            return density
        except Exception as e:
            if self.node:
                self.node.get_logger().error(f"Error calculating obstacle density: {e}")
            return 0.0
        
    def analyze_env_complexity(self, world_x: float, world_y: float, radius: float) -> float:
        """Calculate env complexity based on entropy/variance/gradient"""

        try:
            cache_key = f"complexity_{world_x:.1f}_{world_y:.1f}_{radius:.1f}"

            # Find key in cache
            for item in self.complexity_cache.get_data():
                if item[0] == cache_key:
                    return item[1]
                
            # Not in cache, calculate complexity
            if self.complexity_method == "entropy":
                complexity = self.cal_entropy(world_x, world_y, radius)
            elif self.complexity_method == "variance":
                complexity = self.cal_variance(world_x, world_y, radius)
            elif self.complexity_method == "gradient":
                complexity = self.cal_gradient(world_x, world_y, radius)
            else:
                complexity = 0.0  # Default

                
            # Store in cache
            self.complexity_cache.append((cache_key, complexity))
            return complexity 
        
        except Exception as e:
            if self.node:
                self.node.get_logger().error(f"Error analyzing environment complexity: {e}")
            return 0.0
        
    def collect_occupancy_values(self, world_x: float, world_y: float, radius: float) -> List[int]:
        """Collect occupancy values in the area"""
        try:
            if self.map_handler.map_data is None:
                return [] 
            
            grid_x, grid_y = self.map_handler.world_to_grid(world_x, world_y)
            if not self.map_handler.is_in_grid_bounds(grid_x, grid_y):
                return []
            
            cell_radius = int(radius / self.map_handler.map_resolution) 

            if cell_radius > 10:  # Threshold for using parallel processing (for larger areas)
                chunks = []
                current_chunk = []
                chunk_size = 50  # IMPORTANT: change based on performance

                for dx in range(-cell_radius, cell_radius + 1):
                    for dy in range(-cell_radius, cell_radius + 1):
                        if dx**2 + dy**2 <= cell_radius**2:
                            current_chunk.append((dx, dy))
                            if len(current_chunk) >= chunk_size:
                                chunks.append(current_chunk)
                                current_chunk = []

                if current_chunk:
                    chunks.append(current_chunk)

                # Process chunks in parallel
                def process_chunk(offsets):
                    chunk_values = []
                    for dx, dy in offsets:
                        nx, ny = grid_x + dx, grid_y + dy
                        if self.map_handler.is_in_grid_bounds(nx, ny):
                            value = self.map_handler.get_cell_value(nx, ny)
                            if value >= 0:
                                chunk_values.append(value)
                    return chunk_values
                
                # Results from parallel processing
                results = self.parallel_processor.map_async(process_chunk, chunks)
                values = []
                for chunk_values in results:
                    values.extend(chunk_values)

                return values 
            else:
                # Smaller areas
                values = []
                for dx in range(-cell_radius, cell_radius + 1):
                    for dy in range(-cell_radius, cell_radius + 1):
                        dist_sq = dx * dx + dy * dy
                        if dist_sq <= cell_radius * cell_radius:
                            nx, ny = grid_x + dx, grid_y + dy
                            if self.map_handler.is_in_grid_bounds(nx, ny):
                                value = self.map_handler.get_cell_value(nx, ny)
                                if value >= 0:  # Exclude unknown values
                                    values.append(value)
                return values
        except Exception as e:
            if self.node:
                self.node.get_logger().error(f"Error collecting occupancy values: {e}")
            return []
        
    
    def cal_entropy(self, world_x: float, world_y: float, radius: float) -> float:
        """Calculate entropy of occupancy values"""
        try:
            # Get values 
            values = self.collect_occupancy_values(world_x, world_y, radius)
            if not values:
                return 0.0
            
            # Calculate entropy
            # Bin values to reduce noise
            bins = [0, 20, 40, 60, 80, 100]
            hist, _ = np.histogram(values, bins=bins)
            probs = hist / len(values)
            # Remove zero probabilities to avoid log(0)
            probs = probs[probs > 0]
            entropy = -np.sum(probs * np.log2(probs))
            
            # Normalize to 0-1 range
            max_entropy = np.log2(len(bins) - 1)
            normalized_entropy = entropy / max_entropy if max_entropy > 0 else 0
            
            return normalized_entropy
            
        except Exception as e:
            if self.node:
                self.node.get_logger().error(f"Error calculating entropy: {e}")
            return 0.0

    def cal_variance(self, world_x: float, world_y: float, radius: float) -> float:
        """Calculate variance of occupancy values"""
        try:
            # Get values 
            values = self.collect_occupancy_values(world_x, world_y, radius)
            if not values:
                return 0.0
            
            # Calculate variance and normalize to 0-1
            # Max possible variance would be with half values at 0 and half at 100
            variance = np.var(values)
            max_variance = 50 * 50  # Theoretical max variance
            normalized_variance = min(1.0, variance / max_variance)
            
            return normalized_variance
            
        except Exception as e:
            if self.node:
                self.node.get_logger().error(f"Error calculating variance: {e}")
            return 0.0
        
    def cal_gradient(self, world_x: float, world_y: float, radius: float) -> float:
        """Calculate average gradient magnitudes in the area"""
        try:
            if self.map_handler.map_data is None:
                return 0.0
                
            grid_x, grid_y = self.map_handler.world_to_grid(world_x, world_y)
            if not self.map_handler.is_in_grid_bounds(grid_x, grid_y):
                return 0.0
                
            # Convert radius from meters to grid cells
            cell_radius = int(radius / self.map_handler.map_resolution)
            
            # For larger areas, use parallel processing
            if cell_radius > 10:
                # Create chunks of work for parallel processing
                chunks = []
                current_chunk = []
                chunk_size = 50  # Adjust based on performance testing
                
                for dx in range(-cell_radius + 1, cell_radius):
                    for dy in range(-cell_radius + 1, cell_radius):
                        if dx * dx + dy * dy <= cell_radius * cell_radius:
                            current_chunk.append((dx, dy))
                            if len(current_chunk) >= chunk_size:
                                chunks.append(current_chunk)
                                current_chunk = []
                
                if current_chunk:  # Add the last partial chunk
                    chunks.append(current_chunk)
                
                # Process chunks in parallel
                def process_chunk(offsets):
                    chunk_gradient_sum = 0.0
                    chunk_count = 0
                    for dx, dy in offsets:
                        nx, ny = grid_x + dx, grid_y + dy
                        if (self.map_handler.is_in_grid_bounds(nx, ny) and 
                            self.map_handler.is_in_grid_bounds(nx + 1, ny) and 
                            self.map_handler.is_in_grid_bounds(nx, ny + 1)):
                            
                            # Get values
                            v_center = self.map_handler.get_cell_value(nx, ny)
                            v_right = self.map_handler.get_cell_value(nx + 1, ny)
                            v_down = self.map_handler.get_cell_value(nx, ny + 1)
                            
                            # Skip unknown cells
                            if v_center >= 0 and v_right >= 0 and v_down >= 0:
                                # Calculate gradient
                                gx = v_right - v_center
                                gy = v_down - v_center
                                gradient_mag = np.sqrt(gx * gx + gy * gy)
                                
                                chunk_gradient_sum += gradient_mag
                                chunk_count += 1
                    
                    return (chunk_gradient_sum, chunk_count)
                
                # Collect results
                results = self.parallel_processor.map_async(process_chunk, chunks)
                gradient_sum = 0.0
                count = 0
                for chunk_sum, chunk_count in results:
                    gradient_sum += chunk_sum
                    count += chunk_count
            else:
                # For smaller areas, use sequential processing
                gradient_sum = 0.0
                count = 0
                for dx in range(-cell_radius + 1, cell_radius):
                    for dy in range(-cell_radius + 1, cell_radius):
                        if dx * dx + dy * dy <= cell_radius * cell_radius:
                            nx, ny = grid_x + dx, grid_y + dy
                            if (self.map_handler.is_in_grid_bounds(nx, ny) and 
                                self.map_handler.is_in_grid_bounds(nx + 1, ny) and 
                                self.map_handler.is_in_grid_bounds(nx, ny + 1)):
                                
                                # Get values
                                v_center = self.map_handler.get_cell_value(nx, ny)
                                v_right = self.map_handler.get_cell_value(nx + 1, ny)
                                v_down = self.map_handler.get_cell_value(nx, ny + 1)
                                
                                # Skip unknown cells
                                if v_center >= 0 and v_right >= 0 and v_down >= 0:
                                    # Calculate gradient
                                    gx = v_right - v_center
                                    gy = v_down - v_center
                                    gradient_mag = np.sqrt(gx * gx + gy * gy)
                                    
                                    gradient_sum += gradient_mag
                                    count += 1
            
            if count > 0:
                avg_gradient = gradient_sum / count
                # Normalize by maximum possible gradient (100)
                return min(1.0, avg_gradient / 100.0)
            
            return 0.0
            
        except Exception as e:
            if self.node:
                self.node.get_logger().error(f"Error calculating gradient: {e}")
            return 0.0

    def calculate_safety_margin(self, world_x: float, world_y: float) -> int:
        """Calculates adaptive safety margin based on env complexity"""

        try:
            complexity = self.analyze_env_complexity(
                world_x,
                world_y,
                radius=self.density_radius * self.map_handler.map_resolution
            )
                    
            margin = self.min_safety_margin + complexity * (self.max_safety_margin - self.min_safety_margin)
            return int(round(margin))
        except Exception as e:
            if self.node:
                self.node.get_logger().error(f"Error calculating safety margin: {e}")
            return self.min_safety_margin
        

    def check_line_of_sight(self, start_x: float, start_y: float, end_x: float, end_y: float) -> bool:
        """Check clear there is a line between two points"""
        try:
            if self.map_handler.map_data is None:
                return True
             
            start_gx, start_gy = self.map_handler.world_to_grid(start_x, start_y)
            end_gx, end_gy = self.map_handler.world_to_grid(end_x, end_y)

            # Check points are valid
            if not (
                self.map_handler.is_in_grid_bounds(start_gx, start_gy) and
                self.map_handler.is_in_grid_bounds(end_gx, end_gy)
            ): 
                return False 

            # Bresenham's line algorithm --> check cells along the line
            dx = abs(end_gx - start_gx)
            dy = abs(end_gy - start_gy)
            sx = 1 if start_gx < end_gx else -1 
            sy = 1 if start_gy < end_gy else -1 
            err = dx - dy 

            x, y = start_gx, start_gy 

            while x != end_gx or y != end_gy:
                # Check current cell
                cell_value = self.map_handler.get_cell_value(x, y)
                if cell_value >= self.map_handler.occupied_threshold:  # if occupied
                    return False
            
                # Move to next cell
                e2 = 2 * err
                
                if e2 > -dy:
                    err -= dy
                    x += sx
                if e2 < dx:
                    err += dx
                    y += sy

            return True
        except Exception as e:
            if self.node:
                self.node.get_logger().error(f"Error checking line of sight: {e}")
            return False
