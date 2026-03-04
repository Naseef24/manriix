#!/usr/bin/env python3
"""
BIRCH Streaming Implementation for Real-time Human Clustering
Replaces DBSCAN with incremental clustering using partial_fit()
"""

import numpy as np
from sklearn.cluster import Birch
from typing import Dict, List, Tuple, Optional
from rclpy.node import Node
from collections import deque
import time


class BIRCHStreaming:
    """
    Streaming BIRCH clustering with incremental updates.
    Designed for real-time processing at 30+ FPS with 50+ people.
    """
    
    def __init__(self, config: Dict, node: Node = None):
        """Initialize BIRCH with streaming capabilities"""
        self.node = node
        
        # Extract BIRCH parameters from config (with defaults)
        birch_config = config.get('birch', {})
        self.threshold = birch_config.get('threshold', 1.5)
        self.branching_factor = birch_config.get('branching_factor', 50)
        self.n_clusters = birch_config.get('n_clusters', None)
        
        # Adaptive parameters
        self.adaptive_enabled = birch_config.get('adaptive_threshold', True)
        self.min_threshold = birch_config.get('min_threshold', 1.0)
        self.max_threshold = birch_config.get('max_threshold', 2.0)
        
        # Micro-batching for efficiency
        self.micro_batch_size = birch_config.get('micro_batch_size', 3)
        self.micro_batch = []
        
        # Sliding window for memory management
        self.window_size = birch_config.get('window_size', 150)  # 5 seconds at 30 FPS
        self.frame_buffer = deque(maxlen=self.window_size)
        
        # Initialize BIRCH
        self.birch = Birch(
            n_clusters=self.n_clusters,
            threshold=self.threshold,
            branching_factor=self.branching_factor,
            compute_labels=True,
            copy=False  # Don't copy data, use views for efficiency
        )
        
        # State tracking
        self.is_initialized = False
        self.frame_count = 0
        self.last_rebuild_frame = 0
        self.rebuild_interval = 300  # Rebuild every 10 seconds
        
        # Performance metrics
        self.processing_times = deque(maxlen=30)
        self.last_fps = 0
        
        # Cluster tracking
        self.prev_labels = None
        self.prev_centers = None
        
        if self.node:
            self.node.get_logger().info(f"BIRCH Streaming initialized with threshold={self.threshold}, "
                                        f"branching_factor={self.branching_factor}")
    
    def detect_clusters(self, positions: np.ndarray) -> Tuple[List[Dict], np.ndarray]:
        """
        Main clustering method - replaces DBSCAN detect_clusters
        
        Args:
            positions: Nx3 array of human positions in map frame
            
        Returns:
            clusters: List of cluster dictionaries with properties
            labels: Array of cluster labels for each point
        """
        start_time = time.time()
        
        if len(positions) == 0:
            return [], np.array([])
        
        # Add to micro-batch
        self.micro_batch.append(positions)
        
        # Process when batch is ready or forced
        if len(self.micro_batch) >= self.micro_batch_size or not self.is_initialized:
            labels = self._process_batch()
        else:
            # Return previous labels if available
            if self.prev_labels is not None and len(self.prev_labels) == len(positions):
                labels = self.prev_labels
            else:
                labels = self._process_batch()
        
        # Extract cluster information
        clusters = self._extract_clusters(positions, labels)
        
        # Update performance metrics
        processing_time = time.time() - start_time
        self.processing_times.append(processing_time)
        self._update_fps()
        
        # Store for next frame
        self.prev_labels = labels
        self.prev_centers = [c['center'] for c in clusters] if clusters else None
        
        return clusters, labels
    
    def _process_batch(self) -> np.ndarray:
        """Process accumulated micro-batch"""
        
        if not self.micro_batch:
            return np.array([])
        
        # Combine micro-batch data
        batch_data = np.vstack(self.micro_batch) if len(self.micro_batch) > 1 else self.micro_batch[0]
        
        # Adaptive threshold based on crowd density
        if self.adaptive_enabled:
            self._adapt_threshold(len(batch_data))
        
        # Incremental update or initial fit
        if not self.is_initialized:
            # First frame - full fit
            self.birch.fit(batch_data)
            self.is_initialized = True
            labels = self.birch.labels_
            if self.node:
                self.node.get_logger().info("BIRCH initialized with first frame")
        else:
            # Incremental update with partial_fit
            self.birch.partial_fit(batch_data)
            labels = self.birch.predict(batch_data)
        
        # Add to frame buffer
        self.frame_buffer.extend(self.micro_batch)
        
        # Periodic rebuild for long-term stability
        self.frame_count += len(self.micro_batch)
        if self.frame_count - self.last_rebuild_frame >= self.rebuild_interval:
            self._periodic_rebuild()
        
        # Clear micro-batch
        self.micro_batch = []
        
        return labels[-len(batch_data):]  # Return labels for current batch only
    
    def _adapt_threshold(self, n_points: int):
        """Dynamically adjust BIRCH threshold based on crowd density"""
        
        # Adaptive thresholding based on crowd size
        if n_points < 10:
            # Sparse crowd - looser clustering
            new_threshold = self.max_threshold
        elif n_points < 30:
            # Moderate crowd - standard clustering
            new_threshold = (self.min_threshold + self.max_threshold) / 2
        else:
            # Dense crowd - tighter clustering
            new_threshold = self.min_threshold
        
        # Only update if significant change
        if abs(new_threshold - self.threshold) > 0.1:
            self.threshold = new_threshold
            # Recreate BIRCH with new threshold
            self.birch = Birch(
                n_clusters=self.n_clusters,
                threshold=self.threshold,
                branching_factor=self.branching_factor,
                compute_labels=True,
                copy=False
            )
            # Refit with buffer data
            if self.frame_buffer:
                all_data = np.vstack(list(self.frame_buffer))
                self.birch.fit(all_data)
            
            if self.node:
                self.node.get_logger().info(f"Adapted BIRCH threshold to {self.threshold:.2f} for {n_points} points")
    
    def _periodic_rebuild(self):
        """Periodically rebuild CF-tree to maintain quality"""
        
        if not self.frame_buffer:
            return
        
        if self.node:
            self.node.get_logger().debug("Performing periodic BIRCH rebuild")
        
        # Combine recent frames
        all_data = np.vstack(list(self.frame_buffer))
        
        # Rebuild BIRCH
        self.birch = Birch(
            n_clusters=self.n_clusters,
            threshold=self.threshold,
            branching_factor=self.branching_factor,
            compute_labels=True,
            copy=False
        )
        self.birch.fit(all_data)
        
        self.last_rebuild_frame = self.frame_count
    
    def _extract_clusters(self, positions: np.ndarray, labels: np.ndarray) -> List[Dict]:
        """Extract cluster information from labels"""
        
        clusters = []
        unique_labels = np.unique(labels)
        
        for label in unique_labels:
            if label == -1:  # Noise points in BIRCH
                continue
                
            mask = labels == label
            cluster_points = positions[mask]
            
            if len(cluster_points) == 0:
                continue
            
            # Calculate cluster properties
            center = np.mean(cluster_points, axis=0)
            
            # Calculate radius (dynamic based on points)
            if len(cluster_points) == 1:
                radius = 0.5  # Personal space for single person
            else:
                distances = np.linalg.norm(cluster_points - center, axis=1)
                radius = np.percentile(distances, 90)  # 90th percentile radius
            
            # Determine formation
            formation = self._determine_formation(cluster_points)
            
            # Calculate density
            if radius > 0:
                density = len(cluster_points) / (np.pi * radius**2)
            else:
                density = 1.0
            
            # Create cluster dictionary
            cluster = {
                'id': int(label),
                'center': center,
                'radius': float(radius),
                'size': len(cluster_points),
                'points': cluster_points,
                'formation': formation,
                'density': float(density),
                'confidence': 0.8,  # Will be updated by tracker
                'stability': 0.7,    # Will be updated by tracker
                'required_distance': self._calculate_required_distance(
                    radius, len(cluster_points)
                )
            }
            
            clusters.append(cluster)
        
        return clusters
    
    def _determine_formation(self, points: np.ndarray) -> str:
        """Determine cluster formation type"""
        
        if len(points) <= 2:
            return 'small_group'
        
        # Calculate spread in x and y
        x_spread = np.ptp(points[:, 0])
        y_spread = np.ptp(points[:, 1])
        
        # Aspect ratio
        if x_spread > 0 and y_spread > 0:
            aspect_ratio = max(x_spread, y_spread) / min(x_spread, y_spread)
        else:
            aspect_ratio = 1.0
        
        # Formation detection
        if aspect_ratio > 3.0:
            return 'line'
        elif len(points) >= 5 and aspect_ratio < 1.5:
            # Check if circular
            center = np.mean(points, axis=0)
            distances = np.linalg.norm(points - center, axis=1)
            std_dist = np.std(distances)
            if std_dist < 0.3:
                return 'circle'
        
        if len(points) > 10:
            return 'scattered'
        
        return 'small_group'
    
    def _calculate_required_distance(self, radius: float, size: int) -> float:
        """Calculate optimal photography distance for cluster"""
        
        # Base distance on cluster radius and size
        base_distance = 2.0 + radius * 1.5
        
        # Adjust for group size
        if size > 10:
            base_distance *= 1.2
        elif size > 20:
            base_distance *= 1.4
        
        # Clamp to camera working distance
        return np.clip(base_distance, 2.0, 6.0)
    
    def _update_fps(self):
        """Update FPS calculation"""
        
        if len(self.processing_times) > 0:
            avg_time = np.mean(self.processing_times)
            self.last_fps = 1.0 / avg_time if avg_time > 0 else 0
    
    def get_performance_stats(self) -> Dict:
        """Get performance statistics"""
        
        stats = {
            'fps': self.last_fps,
            'avg_processing_time': np.mean(self.processing_times) if self.processing_times else 0,
            'frame_count': self.frame_count,
            'buffer_size': len(self.frame_buffer),
            'is_initialized': self.is_initialized,
            'threshold': self.threshold,
            'n_clusters': len(self.prev_centers) if self.prev_centers else 0
        }
        
        return stats
    
    def reset(self):
        """Reset clustering state"""
        
        self.birch = Birch(
            n_clusters=self.n_clusters,
            threshold=self.threshold,
            branching_factor=self.branching_factor,
            compute_labels=True,
            copy=False
        )
        
        self.is_initialized = False
        self.frame_count = 0
        self.last_rebuild_frame = 0
        self.micro_batch = []
        self.frame_buffer.clear()
        self.prev_labels = None
        self.prev_centers = None
        
        if self.node:
            self.node.get_logger().info("BIRCH Streaming reset")
