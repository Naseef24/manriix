#!/usr/bin/env python3

import numpy as np
from typing import List, Dict, Optional, Tuple
import rospy

class PositionOptimizer:
    """Optimizes camera positions with multiple candidates and enhanced scoring"""
    
    def __init__(self, config: Dict):
        """Initialize position optimizer"""
        self.config = config
        
        # Camera parameters
        self.fov_horizontal = np.radians(config['camera']['fov']['horizontal'])
        self.min_distance = config['camera']['working_distance']['min']
        self.max_distance = config['camera']['working_distance']['max']
        
        # Position generation parameters
        self.angle_samples = 8  # Number of angles to sample for each cluster
        self.min_angle_sep = np.pi/6  # Minimum separation between angles
        
        # History for composition rate tracking
        self.cluster_history = {}  # track_id -> list of sizes
        self.max_history = 10      # Maximum history length
        
        rospy.loginfo("PositionOptimizer initialized")

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
            rospy.logerr(f"Error calculating composition rate: {e}")
            return 0.0

    def generate_candidate_positions(self, cluster: Dict) -> List[Dict]:
        """Generate multiple candidate positions around cluster"""
        try:
            candidates = []
            center = cluster['center']
            radius = cluster['radius']
            required_distance = cluster['required_distance']
            
            # Generate positions at different angles
            base_angle = np.arctan2(center[1], center[0])
            
            for i in range(self.angle_samples):
                # Calculate angle with even distribution
                angle = base_angle + (2 * np.pi * i / self.angle_samples)
                
                # Generate primary position
                direction = np.array([np.cos(angle), np.sin(angle)])
                position = center - direction * required_distance
                
                # Generate backup positions at different distances
                distances = [
                    required_distance,
                    required_distance * 1.2,  # Slightly further
                    required_distance * 0.8   # Slightly closer
                ]
                
                for dist in distances:
                    if self.min_distance <= dist <= self.max_distance:
                        backup_pos = center - direction * dist
                        
                        if self.is_valid_position(backup_pos):
                            candidates.append({
                                'position': backup_pos,
                                'orientation': angle,
                                'distance': dist,
                                'cluster_center': center,
                                'cluster_radius': radius,
                                'cluster_size': cluster['size'],
                                'formation': cluster.get('formation', 'unknown'),
                                'stability': cluster.get('stability', 0.0),
                                'confidence': cluster.get('confidence', 0.0),
                                'density': cluster.get('density', 0.0),
                                'is_backup': dist != required_distance
                            })
            
            return candidates
            
        except Exception as e:
            rospy.logerr(f"Error generating candidate positions: {e}")
            return []

    def is_valid_position(self, position: np.ndarray) -> bool:
        """Check if a position is valid"""
        try:
            # Check distance bounds
            if np.linalg.norm(position) > self.max_distance:
                return False
                
            # Add more checks here (obstacles, boundaries, etc.)
            
            return True
            
        except Exception as e:
            rospy.logerr(f"Error checking position validity: {e}")
            return False

    def calculate_priority_score(self, position: Dict, robot_position: np.ndarray) -> float:
        """Calculate comprehensive priority score"""
        try:
            # Ensure 2D comparison
            pos_2d = position['position'][:2] if len(position['position']) > 2 else position['position']
            robot_2d = robot_position[:2] if len(robot_position) > 2 else robot_position
            
            # Distance score
            distance = np.linalg.norm(pos_2d - robot_2d)
            distance_score = 1.0 / (1.0 + 0.1 * distance)
            
            # Size and density scores
            size_score = position['cluster_size'] / 10.0
            density_score = position.get('density', 0.0)
            
            # Formation score
            formation_scores = {
                'line': 1.0,
                'circle': 0.9,
                'small_group': 0.8,
                'scattered': 0.7,
                'unknown': 0.6
            }
            formation_score = formation_scores.get(position['formation'], 0.6)
            
            # Stability and composition scores
            stability_score = position.get('stability', 0.5)
            composition_score = self.calculate_composition_rate(position)
            
            # Backup position penalty
            backup_penalty = 0.8 if position.get('is_backup', False) else 1.0
            
            # Weighted combination
            weights = {
                'distance': 0.25,
                'size': 0.15,
                'density': 0.15,
                'formation': 0.15,
                'stability': 0.15,
                'composition': 0.15
            }
            
            final_score = (
                weights['distance'] * distance_score +
                weights['size'] * size_score +
                weights['density'] * density_score +
                weights['formation'] * formation_score +
                weights['stability'] * stability_score +
                weights['composition'] * composition_score
            ) * backup_penalty
            
            return final_score
            
        except Exception as e:
            rospy.logerr(f"Error calculating priority score: {e}")
            return 0.0

    def filter_similar_positions(self, positions: List[Dict],
                               min_angle_diff: float = np.pi/6,
                               min_dist_diff: float = 1.0) -> List[Dict]:
        """Filter out too similar positions"""
        try:
            if not positions:
                return []
                
            filtered = [positions[0]]
            
            for pos in positions[1:]:
                # Check distance and angle difference with existing positions
                is_different = all(
                    (np.linalg.norm(pos['position'] - kept['position']) >= min_dist_diff or
                     abs(pos['orientation'] - kept['orientation']) >= min_angle_diff)
                    for kept in filtered
                )
                
                if is_different:
                    filtered.append(pos)
            
            return filtered
            
        except Exception as e:
            rospy.logerr(f"Error filtering positions: {e}")
            return positions

    def prioritize_positions(self, clusters: List[Dict],
                           current_position: np.ndarray) -> List[Dict]:
        """Generate and prioritize multiple positions for all clusters"""
        try:
            all_candidates = []
            
            # Generate candidates for each cluster
            for cluster in clusters:
                candidates = self.generate_candidate_positions(cluster)
                for candidate in candidates:
                    # Calculate priority score
                    candidate['priority_score'] = self.calculate_priority_score(
                        candidate, 
                        current_position
                    )
                    # Calculate distance to robot
                    pos_2d = candidate['position'][:2] if len(candidate['position']) > 2 else candidate['position']
                    robot_2d = current_position[:2] if len(current_position) > 2 else current_position
                    candidate['distance_to_robot'] = np.linalg.norm(pos_2d - robot_2d)
                    
                all_candidates.extend(candidates)
            
            # Sort by priority score
            all_candidates.sort(key=lambda x: x['priority_score'], reverse=True)
            
            # Filter similar positions
            filtered_candidates = self.filter_similar_positions(all_candidates)
            
            # Log priorities
            # for i, pos in enumerate(filtered_candidates):
            #     rospy.loginfo(
            #         f"Priority {i+1}:"
            #         f"\n  Position (map frame): ({pos['position'][0]:.2f}, {pos['position'][1]:.2f})"
            #         f"\n  Orientation (global): {np.degrees(pos['orientation']):.1f}°"
            #         f"\n  Size: {pos['cluster_size']}"
            #         f"\n  Distance: {pos['distance_to_robot']:.2f}m"
            #         f"\n  Formation: {pos['formation']}"
            #         f"\n  Stability: {pos['stability']:.2f}"
            #         f"\n  Density: {pos.get('density', 0.0):.2f}"
            #         f"\n  Score: {pos['priority_score']:.2f}"
            #         f"\n  Backup: {pos.get('is_backup', False)}"
            #     )
            
            return filtered_candidates
            
        except Exception as e:
            rospy.logerr(f"Error prioritizing positions: {e}")
            return []