#!/usr/bin/env python3
"""
Hungarian algorithm for optimal cluster-to-track assignment
Solves the assignment problem to maintain consistent IDs
"""

import numpy as np
from rclpy.node import Node
from typing import Dict, List, Tuple, Optional
from scipy.optimize import linear_sum_assignment
import time

try:
    import cupy as cp
    CUPY_AVAILABLE = True
except ImportError:
    CUPY_AVAILABLE = False


class HungarianMatcher:
    """
    Matches detected clusters to existing tracks using Hungarian algorithm
    Minimizes total assignment cost based on distance and appearance
    """
    
    def __init__(self, max_distance: float = 2.0, use_gpu: bool = False, node: Node = None):
        """
        Initialize Hungarian matcher
        
        Args:
            max_distance: Maximum distance for valid assignment
            use_gpu: Use GPU acceleration for cost matrix computation
            node: ROS2 node for logging
        """
        self.max_distance = max_distance
        self.use_gpu = use_gpu and CUPY_AVAILABLE
        self.node = node
        
        # Cost weights
        self.distance_weight = 0.7
        self.size_weight = 0.2
        self.velocity_weight = 0.1
        
        # Assignment thresholds
        self.new_track_threshold = 0.8  # Cost threshold for new track
        self.lost_track_threshold = 1.5  # Cost threshold for lost track
        
        # Statistics
        self.total_assignments = 0
        self.successful_matches = 0
        self.new_tracks_created = 0
        self.tracks_lost = 0
        
        if self.node:
            self.node.get_logger().info(f"Hungarian matcher initialized (GPU: {self.use_gpu})")
    
    def compute_cost_matrix(
        self,
        detected_clusters: List[Dict],
        track_predictions: Dict[int, np.ndarray],
        track_velocities: Optional[Dict[int, np.ndarray]] = None
    ) -> np.ndarray:
        """
        Compute cost matrix for assignment problem
        
        Args:
            detected_clusters: List of detected cluster dicts
            track_predictions: Dict of track_id -> predicted position
            track_velocities: Optional dict of track_id -> velocity
            
        Returns:
            Cost matrix (n_clusters x n_tracks)
        """
        n_clusters = len(detected_clusters)
        n_tracks = len(track_predictions)
        
        if n_clusters == 0 or n_tracks == 0:
            return np.array([[]])
        
        # Initialize cost matrix
        cost_matrix = np.full((n_clusters, n_tracks), self.max_distance)
        
        # Extract cluster positions and sizes
        cluster_positions = np.array([c['center'] for c in detected_clusters])
        cluster_sizes = np.array([c['size'] for c in detected_clusters])
        
        # Extract track predictions
        track_ids = list(track_predictions.keys())
        track_positions = np.array([track_predictions[tid] for tid in track_ids])
        
        if self.use_gpu:
            # GPU-accelerated cost computation
            cost_matrix = self._compute_cost_gpu(
                cluster_positions,
                cluster_sizes,
                track_positions,
                track_velocities,
                track_ids
            )
        else:
            # CPU cost computation
            for i, cluster in enumerate(detected_clusters):
                for j, track_id in enumerate(track_ids):
                    # Distance cost
                    distance = np.linalg.norm(
                        cluster_positions[i] - track_positions[j]
                    )
                    
                    if distance > self.max_distance:
                        continue
                    
                    # Base cost is distance
                    cost = distance * self.distance_weight
                    
                    # Add size difference cost if available
                    if 'prev_size' in track_predictions and track_id in track_predictions['prev_size']:
                        size_diff = abs(cluster_sizes[i] - track_predictions['prev_size'][track_id])
                        cost += size_diff * self.size_weight
                    
                    # Add velocity consistency cost if available
                    if track_velocities and track_id in track_velocities:
                        velocity = track_velocities[track_id]
                        velocity_magnitude = np.linalg.norm(velocity)
                        if velocity_magnitude > 0.1:  # Only if track is moving
                            # Check if motion is consistent
                            expected_direction = velocity / velocity_magnitude
                            actual_motion = cluster_positions[i] - track_positions[j]
                            actual_magnitude = np.linalg.norm(actual_motion)
                            if actual_magnitude > 0:
                                actual_direction = actual_motion / actual_magnitude
                                consistency = np.dot(expected_direction, actual_direction)
                                # Convert to cost (1 - consistency)
                                cost += (1 - max(0, consistency)) * self.velocity_weight
                    
                    cost_matrix[i, j] = cost
        
        return cost_matrix
    
    def _compute_cost_gpu(
        self,
        cluster_positions: np.ndarray,
        cluster_sizes: np.ndarray,
        track_positions: np.ndarray,
        track_velocities: Optional[Dict[int, np.ndarray]],
        track_ids: List[int]
    ) -> np.ndarray:
        """
        GPU-accelerated cost matrix computation
        
        Args:
            cluster_positions: Nx3 array of cluster centers
            cluster_sizes: N array of cluster sizes
            track_positions: Mx3 array of track predictions
            track_velocities: Optional velocity dict
            track_ids: Track IDs
            
        Returns:
            Cost matrix
        """
        try:
            # Transfer to GPU
            cluster_pos_gpu = cp.asarray(cluster_positions)
            track_pos_gpu = cp.asarray(track_positions)
            
            n_clusters = len(cluster_positions)
            n_tracks = len(track_positions)
            
            # Compute pairwise distances on GPU
            # Using broadcasting for efficiency
            diff = cluster_pos_gpu[:, np.newaxis, :] - track_pos_gpu[np.newaxis, :, :]
            distances = cp.sqrt(cp.sum(diff ** 2, axis=2))
            
            # Initialize cost matrix
            cost_matrix_gpu = distances * self.distance_weight
            
            # Apply max distance threshold
            cost_matrix_gpu = cp.where(
                distances <= self.max_distance,
                cost_matrix_gpu,
                self.max_distance
            )
            
            # Transfer back to CPU
            cost_matrix = cost_matrix_gpu.get()
            
            return cost_matrix
            
        except Exception as e:
            if self.node:
                self.node.get_logger().warn(f"GPU cost computation failed: {e}, using CPU")
            # Fallback to CPU
            cost_matrix = np.zeros((len(cluster_positions), len(track_positions)))
            for i in range(len(cluster_positions)):
                for j in range(len(track_positions)):
                    distance = np.linalg.norm(cluster_positions[i] - track_positions[j])
                    cost_matrix[i, j] = min(distance * self.distance_weight, self.max_distance)
            return cost_matrix
    
    def match_clusters_to_tracks(
        self,
        detected_clusters: List[Dict],
        track_predictions: Dict[int, np.ndarray],
        track_velocities: Optional[Dict[int, np.ndarray]] = None
    ) -> Tuple[Dict[int, int], List[int], List[int]]:
        """
        Match detected clusters to existing tracks
        
        Args:
            detected_clusters: List of detected clusters
            track_predictions: Predicted positions for tracks
            track_velocities: Optional track velocities
            
        Returns:
            matches: Dict of cluster_idx -> track_id
            unmatched_clusters: List of cluster indices
            unmatched_tracks: List of track IDs
        """
        start_time = time.time()
        
        n_clusters = len(detected_clusters)
        n_tracks = len(track_predictions)
        
        if n_clusters == 0 and n_tracks == 0:
            return {}, [], []
        
        if n_clusters == 0:
            # All tracks are unmatched
            return {}, [], list(track_predictions.keys())
        
        if n_tracks == 0:
            # All clusters are unmatched (new tracks)
            return {}, list(range(n_clusters)), []
        
        # Compute cost matrix
        cost_matrix = self.compute_cost_matrix(
            detected_clusters,
            track_predictions,
            track_velocities
        )
        
        # Solve assignment problem
        row_indices, col_indices = linear_sum_assignment(cost_matrix)
        
        # Process assignments
        matches = {}
        matched_clusters = set()
        matched_tracks = set()
        
        track_ids = list(track_predictions.keys())
        
        for row, col in zip(row_indices, col_indices):
            cost = cost_matrix[row, col]
            
            # Check if assignment is valid
            if cost < self.new_track_threshold:
                cluster_idx = row
                track_id = track_ids[col]
                
                matches[cluster_idx] = track_id
                matched_clusters.add(cluster_idx)
                matched_tracks.add(track_id)
                
                self.successful_matches += 1
        
        # Find unmatched clusters (potential new tracks)
        unmatched_clusters = [
            i for i in range(n_clusters)
            if i not in matched_clusters
        ]
        
        # Find unmatched tracks (potentially lost)
        unmatched_tracks = [
            tid for tid in track_ids
            if tid not in matched_tracks
        ]
        
        # Update statistics
        self.total_assignments += len(matches)
        self.new_tracks_created += len(unmatched_clusters)
        self.tracks_lost += len(unmatched_tracks)
        
        # Log timing
        match_time = time.time() - start_time
        if match_time > 0.01:  # Log if takes more than 10ms
            if self.node:
                self.node.get_logger().debug(f"Hungarian matching took {match_time*1000:.1f}ms")
        
        return matches, unmatched_clusters, unmatched_tracks
    
    def match_with_history(
        self,
        detected_clusters: List[Dict],
        track_history: Dict[int, List[np.ndarray]],
        max_history: int = 5
    ) -> Dict[int, int]:
        """
        Match using track history for improved robustness
        
        Args:
            detected_clusters: Detected clusters
            track_history: History of positions for each track
            max_history: Maximum history to consider
            
        Returns:
            Dict of cluster_idx -> track_id
        """
        if not track_history:
            return {}
        
        # Predict positions based on history
        track_predictions = {}
        track_velocities = {}
        
        for track_id, history in track_history.items():
            if len(history) == 0:
                continue
            
            # Use last position as prediction
            track_predictions[track_id] = history[-1]
            
            # Estimate velocity from history
            if len(history) >= 2:
                # Simple velocity estimate from last two positions
                velocity = history[-1] - history[-2]
                track_velocities[track_id] = velocity
            else:
                track_velocities[track_id] = np.zeros(3)
        
        # Perform matching
        matches, _, _ = self.match_clusters_to_tracks(
            detected_clusters,
            track_predictions,
            track_velocities
        )
        
        return matches
    
    def get_statistics(self) -> Dict:
        """Get matching statistics"""
        
        return {
            'total_assignments': self.total_assignments,
            'successful_matches': self.successful_matches,
            'match_rate': self.successful_matches / self.total_assignments if self.total_assignments > 0 else 0,
            'new_tracks_created': self.new_tracks_created,
            'tracks_lost': self.tracks_lost,
            'use_gpu': self.use_gpu
        }
    
    def reset_statistics(self):
        """Reset statistics counters"""
        
        self.total_assignments = 0
        self.successful_matches = 0
        self.new_tracks_created = 0
        self.tracks_lost = 0
