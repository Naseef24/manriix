#!/usr/bin/env python3

import numpy as np
from typing import List, Dict, Optional, Tuple
from rclpy.node import Node
from collections import deque
from manriix_perception.clustering.performance_utils import MemoryManager


class ClusterMonitor:
    """Monitors cluster stability and changes over time"""
    
    def __init__(self, config: Dict, node: Node = None):
        """Initialize cluster monitor"""
        self.config = config
        self.node = node
        
        # Monitoring parameters
        self.stability_threshold = 0.6  # Minimum stability score
        self.history_length = 10        # Frames to track
        self.split_threshold = 0.5      # Distance threshold for split detection
        
        # Initialize memory manager
        self.memory_manager = MemoryManager(max_history=self.history_length)
        self.memory_manager.register_buffer('cluster_history')
        self.memory_manager.register_buffer('stability_scores')
        
        # Store latest states
        self.current_clusters = {}  # cluster_id -> cluster data
        self.cluster_velocities = {}  # cluster_id -> velocity vector
        self.cluster_stabilities = {}  # cluster_id -> stability score
        
        # Timestamp tracking
        self.last_update_time = None
        
        if self.node:
            self.node.get_logger().info("ClusterMonitor initialized")

    def update_cluster_history(self, clusters: List[Dict]):
        """Update cluster history and calculate changes"""
        try:
            current_time = None
            if self.node:
                current_time = self.node.get_clock().now()
            
            # Store new clusters
            cluster_data = {
                c['track_id']: {
                    'center': c['center'],
                    'size': c['size'],
                    'radius': c['radius'],
                    'timestamp': current_time
                } for c in clusters if 'track_id' in c
            }
            
            self.memory_manager.store_data('cluster_history', cluster_data)
            self.current_clusters = cluster_data
            self.last_update_time = current_time
            
            # Calculate velocities and stability
            self._update_cluster_metrics()
            
            return self._detect_significant_changes()
            
        except Exception as e:
            if self.node:
                self.node.get_logger().error(f"Error updating cluster history: {e}")
            return {}

    def _update_cluster_metrics(self):
        """Update cluster velocities and stability scores"""
        try:
            history = self.memory_manager.get_data('cluster_history')
            if len(history) < 2:
                return
            
            # Calculate time delta
            prev_time = None
            curr_time = None
            
            if history[-1] and history[-2]:
                # Get timestamps from first available cluster in each frame
                for cluster_id in history[-1].keys():
                    curr_time = history[-1][cluster_id].get('timestamp')
                    break
                for cluster_id in history[-2].keys():
                    prev_time = history[-2][cluster_id].get('timestamp')
                    break
            
            # Default dt if timestamps not available
            dt = 0.033  # ~30 FPS default
            if curr_time and prev_time:
                try:
                    dt = (curr_time.nanoseconds - prev_time.nanoseconds) / 1e9
                except:
                    dt = 0.033
            
            if dt <= 0:
                dt = 0.033
            
            for cluster_id in history[-1].keys():
                if cluster_id in history[-2]:
                    # Calculate velocity
                    curr_pos = history[-1][cluster_id]['center']
                    prev_pos = history[-2][cluster_id]['center']
                    velocity = (curr_pos - prev_pos) / dt
                    self.cluster_velocities[cluster_id] = velocity
                    
                    # Calculate stability score
                    stability = self._calculate_stability_score(cluster_id)
                    self.cluster_stabilities[cluster_id] = stability
                    
        except Exception as e:
            if self.node:
                self.node.get_logger().error(f"Error updating cluster metrics: {e}")

    def _calculate_stability_score(self, cluster_id: int) -> float:
        """Calculate cluster stability score"""
        try:
            history = self.memory_manager.get_data('cluster_history')
            if len(history) < 2:
                return 1.0
            
            # Get cluster history
            cluster_history = []
            for frame in history:
                if cluster_id in frame:
                    cluster_history.append(frame[cluster_id])
            
            if len(cluster_history) < 2:
                return 1.0
            
            # Calculate position stability
            positions = np.array([c['center'] for c in cluster_history])
            position_var = np.var(positions, axis=0).mean()
            
            # Calculate size stability
            sizes = [c['size'] for c in cluster_history]
            size_var = np.var(sizes) if len(sizes) > 1 else 0
            
            # Calculate radius stability
            radii = [c['radius'] for c in cluster_history]
            radius_var = np.var(radii) if len(radii) > 1 else 0
            
            # Combined stability score
            stability = 1.0 / (1.0 + position_var + size_var + radius_var)
            return np.clip(stability, 0.0, 1.0)
            
        except Exception as e:
            if self.node:
                self.node.get_logger().error(f"Error calculating stability score: {e}")
            return 0.0

    def _detect_significant_changes(self) -> Dict:
        """Detect significant changes in clusters"""
        try:
            history = self.memory_manager.get_data('cluster_history')
            if len(history) < 2:
                return {}
            
            changes = {}
            
            # Check for splits, merges, and dispersals
            for cluster_id in self.current_clusters:
                if cluster_id not in history[-2]:
                    continue
                    
                curr_state = self.current_clusters[cluster_id]
                stability = self.cluster_stabilities.get(cluster_id, 1.0)
                
                # Detect cluster splits
                if stability < self.stability_threshold:
                    if curr_state['size'] < history[-2][cluster_id]['size']:
                        changes[cluster_id] = {
                            'type': 'split',
                            'severity': 1.0 - stability,
                            'original_size': history[-2][cluster_id]['size'],
                            'current_size': curr_state['size']
                        }
                
                # Detect rapid movement
                if cluster_id in self.cluster_velocities:
                    velocity = np.linalg.norm(self.cluster_velocities[cluster_id])
                    if velocity > self.split_threshold:
                        changes[cluster_id] = {
                            'type': 'moving',
                            'velocity': velocity,
                            'direction': self.cluster_velocities[cluster_id]
                        }
            
            # Detect disappeared clusters
            for prev_id in history[-2]:
                if prev_id not in self.current_clusters:
                    changes[prev_id] = {
                        'type': 'disappeared',
                        'last_known': history[-2][prev_id]
                    }
            
            return changes
            
        except Exception as e:
            if self.node:
                self.node.get_logger().error(f"Error detecting changes: {e}")
            return {}

    def get_cluster_status(self, cluster_id: int) -> Dict:
        """Get comprehensive status for a cluster"""
        try:
            if cluster_id not in self.current_clusters:
                return {}
                
            return {
                'current_state': self.current_clusters[cluster_id],
                'stability': self.cluster_stabilities.get(cluster_id, 1.0),
                'velocity': self.cluster_velocities.get(cluster_id, np.zeros(2)),
                'is_stable': self.cluster_stabilities.get(cluster_id, 1.0) >= self.stability_threshold
            }
            
        except Exception as e:
            if self.node:
                self.node.get_logger().error(f"Error getting cluster status: {e}")
            return {}

    def predict_cluster_position(self, cluster_id: int, time_ahead: float) -> Optional[np.ndarray]:
        """Predict cluster position after given time"""
        try:
            if cluster_id not in self.current_clusters:
                return None
                
            if cluster_id not in self.cluster_velocities:
                return self.current_clusters[cluster_id]['center']
                
            current_pos = self.current_clusters[cluster_id]['center']
            velocity = self.cluster_velocities[cluster_id]
            
            # Simple linear prediction
            predicted_pos = current_pos + velocity * time_ahead
            return predicted_pos
            
        except Exception as e:
            if self.node:
                self.node.get_logger().error(f"Error predicting cluster position: {e}")
            return None
