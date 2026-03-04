#!/usr/bin/env python3

import numpy as np
from typing import List, Dict, Optional, Tuple
from rclpy.node import Node
from dataclasses import dataclass
import time


@dataclass
class FallbackStrategy:
    """Unified fallback strategy result"""
    action: str                    # 'switch_target', 'new_target', 'backup_position', 'escalate_recovery', 'none'
    target: Optional[Dict]         # Target cluster if applicable
    positions: List[Dict]          # Backup positions if applicable
    priority_score: float          # Strategy priority score
    reason: str                    # Fallback reason
    

class FallbackManager:
    """Centralized fallback management for all system components"""
    
    def __init__(self, config: Dict, node: Node = None):
        """Initialize unified fallback manager"""
        self.config = config
        self.node = node
        
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
        
        if self.node:
            self.node.get_logger().info("FallbackManager initialized")

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
            if self.node:
                self.node.get_logger().error(f"Error finding alternative cluster: {e}")
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
            distance = np.linalg.norm(cluster['center'] - robot_position[:2])
            distance_score = 1.0 / (1.0 + self.distance_penalty * distance)
            
            # Stability score
            stability_score = cluster.get('stability', 1.0)
            
            # Direction change score
            direction_score = 1.0
            if current_direction is not None:
                target_direction = cluster['center'] - robot_position[:2]
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
            if self.node:
                self.node.get_logger().error(f"Error calculating priority score: {e}")
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
                    dist_to_robot = np.linalg.norm(position - robot_position[:2])
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
            if self.node:
                self.node.get_logger().error(f"Error generating backup positions: {e}")
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
            if self.node:
                self.node.get_logger().error(f"Error checking target switch: {e}")
            return False

    def get_fallback_strategy(self,
                            error_type: str,
                            context: Dict) -> FallbackStrategy:
        """Unified fallback strategy combining all fallback methods"""
        try:
            if error_type == 'cluster_split':
                return self._handle_cluster_split(context)
            elif error_type == 'target_lost':
                return self._handle_target_lost(context)
            elif error_type == 'navigation_failed':
                return self._handle_navigation_failed(context)
            elif error_type == 'position_blocked':
                return self._handle_position_blocked(context)
            elif error_type == 'stability_timeout':
                return self._handle_stability_timeout(context)
            else:
                return self._handle_unknown_error(context)
                
        except Exception as e:
            if self.node:
                self.node.get_logger().error(f"Error getting fallback strategy: {e}")
            return FallbackStrategy(
                action='escalate_recovery',
                target=None,
                positions=[],
                priority_score=0.0,
                reason=f"Exception in fallback strategy: {str(e)}"
            )

    def _handle_cluster_split(self, context: Dict) -> FallbackStrategy:
        """Handle cluster split scenario"""
        new_target = self.find_best_alternative(
            context.get('current_clusters', []),
            context.get('robot_position', np.zeros(2)),
            context.get('current_direction'),
            context.get('split_cluster_id')
        )
        
        if new_target:
            positions = self.generate_backup_positions(
                new_target,
                context.get('robot_position', np.zeros(2))
            )
            return FallbackStrategy(
                action='switch_target',
                target=new_target,
                positions=positions,
                priority_score=self._calculate_priority_score(
                    new_target,
                    context.get('robot_position', np.zeros(2)),
                    context.get('current_direction')
                ),
                reason='Best sub-cluster selected after split'
            )
        else:
            return FallbackStrategy(
                action='escalate_recovery',
                target=None,
                positions=[],
                priority_score=0.0,
                reason='No suitable sub-cluster found after split'
            )

    def _handle_target_lost(self, context: Dict) -> FallbackStrategy:
        """Handle complete target loss"""
        new_target = self.find_best_alternative(
            context.get('current_clusters', []),
            context.get('robot_position', np.zeros(2)),
            context.get('current_direction')
        )
        
        if new_target:
            positions = self.generate_backup_positions(
                new_target,
                context.get('robot_position', np.zeros(2))
            )
            return FallbackStrategy(
                action='new_target',
                target=new_target,
                positions=positions,
                priority_score=self._calculate_priority_score(
                    new_target,
                    context.get('robot_position', np.zeros(2)),
                    context.get('current_direction')
                ),
                reason='New target selected after target loss'
            )
        else:
            # Try high-density position memory
            memory_positions = context.get('high_density_positions', [])
            if memory_positions:
                return FallbackStrategy(
                    action='backup_position',
                    target=None,
                    positions=memory_positions[:5],  # Top 5 positions
                    priority_score=0.6,
                    reason='Using high-density position memory'
                )
            else:
                return FallbackStrategy(
                    action='escalate_recovery',
                    target=None,
                    positions=[],
                    priority_score=0.0,
                    reason='No targets or fallback positions available'
                )

    def _handle_navigation_failed(self, context: Dict) -> FallbackStrategy:
        """Handle navigation failure"""
        if context.get('current_target'):
            # Generate alternative positions around same target
            positions = self.generate_backup_positions(
                context['current_target'],
                context.get('robot_position', np.zeros(2))
            )
            return FallbackStrategy(
                action='backup_position',
                target=context['current_target'],
                positions=positions,
                priority_score=0.7,
                reason='Alternative positions for navigation failure'
            )
        else:
            return self._handle_target_lost(context)

    def _handle_position_blocked(self, context: Dict) -> FallbackStrategy:
        """Handle blocked position scenario"""
        if context.get('current_target'):
            # Generate positions further away from obstacles
            positions = self.generate_backup_positions(
                context['current_target'],
                context.get('robot_position', np.zeros(2))
            )
            # Filter positions that are further from the blocking obstacle
            filtered_positions = [pos for pos in positions if pos['distance'] > 3.5]
            
            return FallbackStrategy(
                action='backup_position',
                target=context['current_target'],
                positions=filtered_positions if filtered_positions else positions,
                priority_score=0.5,
                reason='Alternative positions to avoid obstacles'
            )
        else:
            return self._handle_target_lost(context)

    def _handle_stability_timeout(self, context: Dict) -> FallbackStrategy:
        """Handle stability timeout during positioning"""
        return FallbackStrategy(
            action='switch_target',
            target=None,  # Will be filled by caller
            positions=[],
            priority_score=0.4,
            reason='Switching due to stability timeout'
        )

    def _handle_unknown_error(self, context: Dict) -> FallbackStrategy:
        """Handle unknown error types"""
        return FallbackStrategy(
            action='escalate_recovery',
            target=None,
            positions=[],
            priority_score=0.0,
            reason='Unknown error - escalating to recovery manager'
        )

    def get_high_priority_fallback(self,
                                 current_clusters: List[Dict],
                                 robot_position: np.ndarray,
                                 excluded_ids: List[int] = None) -> FallbackStrategy:
        """Get highest priority fallback option integrating all strategies"""
        try:
            excluded_ids = excluded_ids or []
            fallback_options = []
            
            # 1. Smart cluster switching
            for cluster in current_clusters:
                if cluster.get('track_id') not in excluded_ids:
                    score = self._calculate_priority_score(
                        cluster, robot_position, None
                    )
                    if score > 0.3:  # Minimum viability threshold
                        positions = self.generate_backup_positions(cluster, robot_position)
                        fallback_options.append(FallbackStrategy(
                            action='switch_target',
                            target=cluster,
                            positions=positions,
                            priority_score=score,
                            reason=f'Smart switch to cluster {cluster.get("track_id")}'
                        ))
            
            # 2. Backup position generation for highest scoring cluster
            if fallback_options:
                best_cluster_option = max(fallback_options, key=lambda x: x.priority_score)
                return best_cluster_option
            
            # 3. Recovery escalation
            return FallbackStrategy(
                action='escalate_recovery',
                target=None,
                positions=[],
                priority_score=0.0,
                reason='No viable cluster targets - escalating to recovery manager'
            )
            
        except Exception as e:
            if self.node:
                self.node.get_logger().error(f"Error in high priority fallback: {e}")
            return FallbackStrategy(
                action='escalate_recovery',
                target=None,
                positions=[],
                priority_score=0.0,
                reason=f'Exception in fallback calculation: {str(e)}'
            )
