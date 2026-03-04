#!/usr/bin/env python3

import numpy as np
from typing import Dict, List, Tuple, Optional
from rclpy.node import Node
from manriix_perception.clustering.performance_utils import CircularBuffer, MemoryManager


class PositionEvaluator:
    """Position evaluation score calculation"""

    def __init__(self, config: Dict, map_handler, obstacle_analyzer, node: Node = None):
        self.config = config 
        self.map_handler = map_handler
        self.obstacle_analyzer = obstacle_analyzer
        self.node = node

        # Configs
        self.history_length = config.get('position', {}).get('history_length', 10)

        # Formation scores
        self.formation_scores = config.get('position', {}).get('formation_scores', {})

        # Weights
        self.weights = config.get('position', {}).get('weights', {})
        
        # Normalize weights to ensure the sum is 1.0 --> to ensure the normalize predictable results.
        weight_sum = sum(self.weights.values()) if self.weights else 1.0
        if abs(weight_sum - 1.0) > 0.001 and weight_sum > 0:
            if self.node:
                self.node.get_logger().warn(f"Position weights sum to {weight_sum}, normalizing to 1.0")
            for key in self.weights:
                self.weights[key] /= weight_sum

        # Position history tracking
        self.memory_manager = MemoryManager(max_history=self.history_length)
        self.memory_manager.register_buffer('position_history')

        self.cluster_history = {}  # track_id -> list of sizes
        self.max_history = config.get('position', {}).get('composition_history_length', 10)

        if self.node:
            self.node.get_logger().info("PositionEvaluator initialized")

    def calculate_priority_score(self, position: Dict, robot_position: np.ndarray) -> float:
        """Calculate priority score"""

        try:
            # Extract position coords
            pos_2d = position['position'][:2] if len(position['position']) > 2 else position['position']
            robot_2d = robot_position[:2] if len(robot_position) > 2 else robot_position

            ## Scoring
            # 1. Distance score
            distance = np.linalg.norm(pos_2d - robot_2d)
            distance_score = 1.0 / (1.0 + 0.1 * distance)

            # 2. Size score
            size_score = position['cluster_size'] / 10.0

            # 3. Density score
            density_score = position.get('density', 0.0)

            # 4. Formation score
            formation_score = self.formation_scores.get(position.get('formation', 'unknown'), 0.6)

            # 5. Stability score
            stability_score = position.get('stability', 0.5)

            # 6. Composition score
            composition_score = self.calculate_composition_rate({
                'track_id': position.get('track_id'),
                'size': position['cluster_size']
            })

            # 7. Obstacle proximity score
            obstacle_distance = self.map_handler.get_obstacle_distance(
                pos_2d[0], pos_2d[1], max_distance=5.0
            )

            # High score --> for positions farther from obstacles
            obstacle_proximity_score = min(1.0, obstacle_distance / 5.0)

            # 8. Environment complexity score
            complexity = self.obstacle_analyzer.analyze_env_complexity(
                pos_2d[0], pos_2d[1], radius=1.0
            )

            # Low score --> for complex env (Harder to navigate)
            environment_complexity_score = 1.0 - complexity

            # 9. Path feasibility score (IF ENABLED)
            path_feasibility_score = 1.0
            if self.config.get('position', {}).get('reachability_check', False):
                path_feasibility_score = self.evaluate_reachability(pos_2d, robot_2d)

            # 10. Temporal consistency score
            temporal_consistency_score = self.calculate_temporal_consistency(pos_2d)

            # 11. Backup penalty
            backup_penalty = 0.8 if position.get('is_backup', False) else 1.0

            ### FINAL SCORE
            final_score = (
                self.weights.get('distance', 0.1) * distance_score +
                self.weights.get('size', 0.1) * size_score +
                self.weights.get('density', 0.1) * density_score +
                self.weights.get('formation', 0.1) * formation_score +
                self.weights.get('stability', 0.1) * stability_score +
                self.weights.get('composition', 0.1) * composition_score +
                self.weights.get('obstacle_proximity', 0.1) * obstacle_proximity_score +
                self.weights.get('environment_complexity', 0.1) * environment_complexity_score +
                self.weights.get('path_feasibility', 0.1) * path_feasibility_score +
                self.weights.get('temporal_consistency', 0.1) * temporal_consistency_score
            ) * backup_penalty

            # Store position for history tracking
            self.track_position(pos_2d)

            return final_score
        
        except Exception as e:
            if self.node:
                self.node.get_logger().error(f"Error calculating priority score: {e}")
            return 0.0
        

    def calculate_composition_rate(self, cluster: Dict) -> float:
        """Calculate how frequently cluster composition changes"""
        try:
            track_id = cluster.get('track_id')
            if track_id is None:
                return 0.0
                
            # Initialize history if needed
            if track_id not in self.cluster_history:
                self.cluster_history[track_id] = []
            
            # Update history
            history = self.cluster_history[track_id]
            history.append(cluster['size'])
            if len(history) > self.max_history:
                history.pop(0)
                
            # Calculate change rate
            if len(history) < 2:
                return 0.0
                
            changes = sum(1 for i in range(len(history)-1) 
                         if history[i] != history[i+1])
            change_rate = changes / (len(history) - 1)
            
            return 1.0 - change_rate  # Higher score for stable composition
            
        except Exception as e:
            if self.node:
                self.node.get_logger().error(f"Error calculating composition rate: {e}")
            return 0.0
        
    def track_position(self, position: np.ndarray):
        """Track position in history for temporal consistency"""
        self.memory_manager.store_data('position_history', position)

    def calculate_temporal_consistency(self, position: np.ndarray) -> float:
        """Calculate temporal consistency score based on position history"""

        # Get recent positions
        position_history = self.memory_manager.get_data('position_history')

        if not position_history:
            return 1.0  # No history, assume consistent
        
        distances = []

        for pos in position_history:
            dist = np.linalg.norm(position - pos)
            distances.append(dist)

        if not distances:
            return 1.0 
        
        avg_distance = np.mean(distances)

        # High score --> for positions closer to recent history
        # Scale so that positions within 1m get high scores
        return 1.0 / (1.0 + avg_distance)
    
    def evaluate_reachability(self, position: np.ndarray, robot_position: np.ndarray) -> float:
        """Evaluate how reachable a position is (simplified path check)"""
        # First, check direct line of sight
        has_line_of_sight = self.obstacle_analyzer.check_line_of_sight(
            robot_position[0], robot_position[1],
            position[0], position[1]
        )
        
        if has_line_of_sight:
            return 1.0  # Perfectly reachable
            
        # If no direct path, do additional checks
        # For now, we'll use a simplified approximation
        # Distance between points
        distance = np.linalg.norm(position - robot_position)
        
        # Check several intermediate points
        num_checks = min(5, max(3, int(distance)))
        check_points = []
        
        for i in range(1, num_checks):
            t = i / num_checks
            check_point = robot_position * (1-t) + position * t
            check_points.append(check_point)
            
        # Count how many check points are in free space
        free_count = 0
        for point in check_points:
            if self.map_handler.is_position_free(point[0], point[1]):
                free_count += 1
                
        # Calculate reachability score
        return free_count / len(check_points) if check_points else 0.0
