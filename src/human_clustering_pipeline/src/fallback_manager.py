#!/usr/bin/env python3

import numpy as np
from typing import List, Dict, Optional, Tuple
import rospy

class FallbackManager:
    """Manages fallback strategies when target cluster changes"""
    
    def __init__(self, config: Dict):
        """Initialize fallback manager"""
        self.config = config
        
        # Priority weights
        self.weights = {
            'size': 0.4,        # Prefer larger clusters
            'distance': 0.3,    # Prefer closer clusters
            'stability': 0.2,   # Prefer stable clusters
            'direction': 0.1    # Prefer clusters in current direction
        }
        
        # Thresholds
        self.min_cluster_size = 2
        self.min_stability = 0.6
        self.max_direction_change = np.pi/2  # 90 degrees
        self.distance_penalty = 0.2          # Penalty for distance
        
        rospy.loginfo("FallbackManager initialized")

    def find_best_alternative(self, 
                            current_clusters: List[Dict],
                            robot_position: np.ndarray,
                            current_direction: Optional[np.ndarray] = None,
                            excluded_id: Optional[int] = None) -> Optional[Dict]:
        """Find best alternative cluster when target is lost/split"""
        try:
            if not current_clusters:
                return None
                
            best_cluster = None
            best_score = -1
            
            for cluster in current_clusters:
                # Skip excluded cluster and small clusters
                if (cluster.get('track_id') == excluded_id or
                    cluster['size'] < self.min_cluster_size):
                    continue
                    
                # Calculate priority score
                score = self._calculate_priority_score(
                    cluster, 
                    robot_position,
                    current_direction
                )
                
                if score > best_score:
                    best_score = score
                    best_cluster = cluster
            
            return best_cluster if best_score > 0 else None
            
        except Exception as e:
            rospy.logerr(f"Error finding alternative cluster: {e}")
            return None

    def _calculate_priority_score(self,
                                cluster: Dict,
                                robot_position: np.ndarray,
                                current_direction: Optional[np.ndarray]) -> float:
        """Calculate priority score for cluster selection"""
        try:
            # Size score (normalized by expected max size)
            size_score = min(cluster['size'] / 5.0, 1.0)
            
            # Distance score (decreases with distance)
            distance = np.linalg.norm(cluster['center'] - robot_position)
            distance_score = 1.0 / (1.0 + self.distance_penalty * distance)
            
            # Stability score
            stability_score = cluster.get('stability', 1.0)
            
            # Direction change score
            direction_score = 1.0
            if current_direction is not None:
                target_direction = cluster['center'] - robot_position
                if np.linalg.norm(target_direction) > 0:
                    target_direction = target_direction / np.linalg.norm(target_direction)
                    angle_diff = np.arccos(
                        np.clip(
                            np.dot(current_direction, target_direction),
                            -1.0, 1.0
                        )
                    )
                    direction_score = 1.0 - (angle_diff / self.max_direction_change)
            
            # Weighted combination
            total_score = (
                self.weights['size'] * size_score +
                self.weights['distance'] * distance_score +
                self.weights['stability'] * stability_score +
                self.weights['direction'] * direction_score
            )
            
            return max(0.0, total_score)
            
        except Exception as e:
            rospy.logerr(f"Error calculating priority score: {e}")
            return 0.0

    def generate_backup_positions(self,
                                cluster: Dict,
                                robot_position: np.ndarray) -> List[Dict]:
        """Generate backup positions around target cluster"""
        try:
            backup_positions = []
            
            if not cluster:
                return backup_positions
                
            center = cluster['center']
            required_distance = cluster.get('required_distance', 3.0)
            
            # Generate positions at different angles
            angles = np.linspace(0, 2*np.pi, 8, endpoint=False)
            distances = [
                required_distance * 0.8,  # Closer
                required_distance,        # Optimal
                required_distance * 1.2   # Further
            ]
            
            for angle in angles:
                for dist in distances:
                    direction = np.array([np.cos(angle), np.sin(angle)])
                    position = center - direction * dist
                    
                    # Calculate position score
                    dist_to_robot = np.linalg.norm(position - robot_position)
                    score = 1.0 / (1.0 + 0.1 * dist_to_robot)
                    
                    backup_positions.append({
                        'position': position,
                        'orientation': angle,
                        'distance': dist,
                        'score': score
                    })
            
            # Sort by score
            backup_positions.sort(key=lambda x: x['score'], reverse=True)
            
            return backup_positions
            
        except Exception as e:
            rospy.logerr(f"Error generating backup positions: {e}")
            return []

    def should_switch_target(self,
                           current_target: Dict,
                           new_candidate: Dict,
                           robot_position: np.ndarray,
                           current_direction: Optional[np.ndarray] = None) -> bool:
        """Determine if should switch to new target cluster"""
        try:
            if not current_target or not new_candidate:
                return False
                
            # Calculate scores
            current_score = self._calculate_priority_score(
                current_target,
                robot_position,
                current_direction
            )
            
            new_score = self._calculate_priority_score(
                new_candidate,
                robot_position,
                current_direction
            )
            
            # Switch if new target is significantly better (>20% improvement)
            return new_score > current_score * 1.2
            
        except Exception as e:
            rospy.logerr(f"Error checking target switch: {e}")
            return False

    def get_fallback_strategy(self,
                            error_type: str,
                            context: Dict) -> Dict:
        """Get appropriate fallback strategy for error type"""
        try:
            strategy = {
                'action': 'none',
                'target': None,
                'positions': []
            }
            
            if error_type == 'cluster_split':
                # Find new target among sub-clusters
                new_target = self.find_best_alternative(
                    context.get('current_clusters', []),
                    context.get('robot_position', np.zeros(2)),
                    context.get('current_direction'),
                    context.get('split_cluster_id')
                )
                
                if new_target:
                    strategy['action'] = 'switch_target'
                    strategy['target'] = new_target
                    strategy['positions'] = self.generate_backup_positions(
                        new_target,
                        context.get('robot_position', np.zeros(2))
                    )
                    
            elif error_type == 'target_lost':
                # Find completely new target
                new_target = self.find_best_alternative(
                    context.get('current_clusters', []),
                    context.get('robot_position', np.zeros(2)),
                    context.get('current_direction')
                )
                
                if new_target:
                    strategy['action'] = 'new_target'
                    strategy['target'] = new_target
                    strategy['positions'] = self.generate_backup_positions(
                        new_target,
                        context.get('robot_position', np.zeros(2))
                    )
            
            return strategy
            
        except Exception as e:
            rospy.logerr(f"Error getting fallback strategy: {e}")
            return {'action': 'none'}