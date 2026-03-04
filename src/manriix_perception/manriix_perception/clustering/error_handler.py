#!/usr/bin/env python3

import numpy as np
from typing import List, Dict, Optional, Tuple
from rclpy.node import Node
from collections import deque


class ErrorHandler:
    """Handles cluster errors and recovery behaviors"""
    
    def __init__(self, config: Dict, node: Node = None):
        """Initialize error handler"""
        self.config = config
        self.node = node
        
        # Thresholds
        self.stability_threshold = 0.6    # Minimum stability for reliable cluster
        self.split_threshold = 0.5        # Size reduction to consider as split
        self.distance_threshold = 2.0     # Maximum expected movement between frames
        
        # Store last known states
        self.last_known_clusters = {}     # cluster_id -> cluster data
        self.last_target_cluster = None   # Currently targeted cluster
        self.backup_positions = []        # List of backup positions
        
        if self.node:
            self.node.get_logger().info("ErrorHandler initialized")

    def check_cluster_changes(self, 
                            current_clusters: List[Dict],
                            robot_position: np.ndarray) -> Dict:
        """Check for significant cluster changes and recommend actions"""
        try:
            changes = {
                'target_lost': False,
                'split_detected': False,
                'needs_replan': False,
                'recommended_action': None,
                'backup_target': None
            }
            
            if not current_clusters:
                return changes

            # Update cluster states
            current_cluster_dict = {
                c.get('track_id'): c for c in current_clusters 
                if 'track_id' in c
            }
            
            # Check for target cluster changes
            if self.last_target_cluster:
                target_id = self.last_target_cluster.get('track_id')
                
                if target_id in current_cluster_dict:
                    current_target = current_cluster_dict[target_id]
                    
                    # Check for significant changes
                    if self._check_cluster_split(
                        self.last_target_cluster, 
                        current_target
                    ):
                        changes['split_detected'] = True
                        changes['needs_replan'] = True
                        
                        # Find best backup target
                        backup = self._find_backup_target(
                            current_clusters,
                            robot_position,
                            excluded_id=target_id
                        )
                        if backup:
                            changes['backup_target'] = backup
                            changes['recommended_action'] = 'switch_target'
                else:
                    changes['target_lost'] = True
                    changes['needs_replan'] = True
                    
                    # Find closest stable cluster
                    backup = self._find_backup_target(
                        current_clusters,
                        robot_position
                    )
                    if backup:
                        changes['backup_target'] = backup
                        changes['recommended_action'] = 'switch_target'
            
            # Update last known states
            self.last_known_clusters = current_cluster_dict
            
            return changes
            
        except Exception as e:
            if self.node:
                self.node.get_logger().error(f"Error checking cluster changes: {e}")
            return {'needs_replan': False}

    def _check_cluster_split(self, 
                           previous: Dict, 
                           current: Dict) -> bool:
        """Check if a cluster has split"""
        try:
            if not previous or not current:
                return False
                
            # Check size reduction
            size_ratio = current['size'] / previous['size']
            if size_ratio < (1 - self.split_threshold):
                return True
                
            # Check stability
            stability = current.get('stability', 1.0)
            if stability < self.stability_threshold:
                return True
                
            return False
            
        except Exception as e:
            if self.node:
                self.node.get_logger().error(f"Error checking cluster split: {e}")
            return False

    def _find_backup_target(self, 
                          clusters: List[Dict],
                          robot_position: np.ndarray,
                          excluded_id: Optional[int] = None) -> Optional[Dict]:
        """Find best backup target cluster"""
        try:
            if not clusters:
                return None
                
            best_score = -1
            best_cluster = None
            
            for cluster in clusters:
                # Skip excluded cluster
                if excluded_id and cluster.get('track_id') == excluded_id:
                    continue
                    
                # Calculate priority score
                distance = np.linalg.norm(
                    cluster['center'] - robot_position[:2]
                )
                stability = cluster.get('stability', 1.0)
                size = cluster['size']
                
                # Priority to closer, larger, stable clusters
                score = (0.4 * size + 0.3 * stability) * (1 / (1 + 0.3 * distance))
                
                if score > best_score:
                    best_score = score
                    best_cluster = cluster
            
            return best_cluster
            
        except Exception as e:
            if self.node:
                self.node.get_logger().error(f"Error finding backup target: {e}")
            return None

    def generate_backup_positions(self, 
                                cluster: Dict,
                                current_position: np.ndarray) -> List[Dict]:
        """Generate backup positions for a given cluster"""
        try:
            backup_positions = []
            
            if not cluster:
                return backup_positions
                
            center = cluster['center']
            radius = cluster['radius']
            required_distance = cluster.get('required_distance', 3.0)
            
            # Generate positions at different angles
            angles = np.linspace(0, 2*np.pi, 8, endpoint=False)
            
            for angle in angles:
                # Calculate position
                direction = np.array([np.cos(angle), np.sin(angle)])
                position = center - direction * required_distance
                
                # Calculate score based on distance from current position
                distance = np.linalg.norm(position - current_position[:2])
                score = 1.0 / (1.0 + 0.2 * distance)
                
                backup_positions.append({
                    'position': position,
                    'orientation': angle,
                    'distance': required_distance,
                    'score': score
                })
            
            # Sort by score
            backup_positions.sort(key=lambda x: x['score'], reverse=True)
            
            return backup_positions
            
        except Exception as e:
            if self.node:
                self.node.get_logger().error(f"Error generating backup positions: {e}")
            return []

    def handle_tracking_loss(self, 
                           last_known_position: np.ndarray,
                           current_clusters: List[Dict]) -> Dict:
        """Handle temporary tracking loss"""
        try:
            action = {
                'action': 'none',
                'target': None
            }
            
            if not current_clusters:
                return action
            
            # Look for clusters near last known position
            nearest_cluster = None
            min_distance = float('inf')
            
            for cluster in current_clusters:
                distance = np.linalg.norm(
                    cluster['center'] - last_known_position
                )
                
                if distance < min_distance and distance < self.distance_threshold:
                    min_distance = distance
                    nearest_cluster = cluster
            
            if nearest_cluster:
                action['action'] = 'resume_tracking'
                action['target'] = nearest_cluster
            
            return action
            
        except Exception as e:
            if self.node:
                self.node.get_logger().error(f"Error handling tracking loss: {e}")
            return {'action': 'none'}

    def update_target_cluster(self, cluster: Dict):
        """Update the currently targeted cluster"""
        try:
            self.last_target_cluster = cluster.copy() if cluster else None
        except Exception as e:
            if self.node:
                self.node.get_logger().error(f"Error updating target cluster: {e}")

    def get_recovery_action(self, error_type: str, context: Dict) -> Dict:
        """Get appropriate recovery action for error type"""
        try:
            if error_type == 'split':
                return {
                    'action': 'find_new_target',
                    'current_position': context.get('robot_position'),
                    'available_clusters': context.get('current_clusters', [])
                }
            elif error_type == 'lost_tracking':
                return {
                    'action': 'wait_and_search',
                    'last_known': context.get('last_known_position'),
                    'search_radius': self.distance_threshold
                }
            else:
                return {'action': 'none'}
                
        except Exception as e:
            if self.node:
                self.node.get_logger().error(f"Error getting recovery action: {e}")
            return {'action': 'none'}
