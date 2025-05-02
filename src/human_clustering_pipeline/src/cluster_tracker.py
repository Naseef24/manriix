#!/usr/bin/env python3

import numpy as np
from scipy.optimize import linear_sum_assignment
import rospy
from typing import List, Dict, Optional
from collections import deque

import sys
sys.path.append("/home/hype/Desktop/lasa/depth_cam/depth_ws/src/human_clustering_pipeline")

from src.performance_utils import CircularBuffer, SpatialIndex, ParallelProcessor, MemoryManager

class ClusterTracker:
    """Optimized temporal tracking and filtering of clusters"""
    
    def __init__(self, config: Dict):
        """Initialize cluster tracker with optimizations"""
        self.config = config
        
        # Tracking parameters
        self.max_tracking_history = 10
        self.max_cluster_age = 5
        self.min_confidence = 0.3
        self.formation_threshold = 0.7
        
        # Performance optimizations
        self.memory_manager = MemoryManager(max_history=self.max_tracking_history)
        self.spatial_index = SpatialIndex()
        self.parallel_processor = ParallelProcessor()
        
        # Initialize memory buffers for efficient storage
        self.memory_manager.register_buffer('cluster_positions')
        self.memory_manager.register_buffer('cluster_patterns')
        self.memory_manager.register_buffer('cluster_velocities')
        
        # Process control
        self.frame_count = 0
        self.process_every_n_frames = 2
        self.next_cluster_id = 0
        
        rospy.loginfo("ClusterTracker initialized with performance optimizations")

    def calculate_cluster_velocity(self, cluster_id: int) -> np.ndarray:
        """Calculate cluster velocity using optimized history access"""
        try:
            positions = self.memory_manager.get_data('cluster_positions')
            if not positions:
                return np.zeros(2)
            
            cluster_positions = [p[1] for p in positions if p[0] == cluster_id]
            if len(cluster_positions) < 2:
                return np.zeros(2)
            
            # Calculate velocity from last two positions
            dt = 0.1  # assuming 10Hz update rate
            velocity = (cluster_positions[-1] - cluster_positions[-2]) / dt
            return velocity
            
        except Exception as e:
            rospy.logerr(f"Error calculating cluster velocity: {e}")
            return np.zeros(2)

    def detect_formation_pattern(self, points: np.ndarray) -> str:
        """Detect formation pattern with parallel processing for large clusters"""
        try:
            if len(points) < 3:
                return "small_group"
            
            # Process large clusters in parallel
            if len(points) > 10:
                chunks = list(self.parallel_processor.process_chunks(points, 5))
                
                def process_chunk(chunk):
                    centered = chunk - np.mean(chunk, axis=0)
                    _, s, _ = np.linalg.svd(centered)
                    linearity = s[0] / (s[1] + 1e-10)
                    return linearity
                
                linearities = self.parallel_processor.map_async(process_chunk, chunks)
                avg_linearity = np.mean([l for l in linearities if l is not None])
                
                # Calculate circularity
                center = np.mean(points, axis=0)
                radii = np.linalg.norm(points - center, axis=1)
                circularity = 1 - np.std(radii) / np.mean(radii)
                
                if avg_linearity > self.formation_threshold:
                    return "line"
                elif circularity > self.formation_threshold:
                    return "circle"
                else:
                    return "scattered"
            else:
                # Process small clusters directly
                center = np.mean(points, axis=0)
                distances = np.linalg.norm(points - center, axis=1)
                
                # Check linearity using PCA
                centered = points - center
                _, s, _ = np.linalg.svd(centered)
                linearity = s[0] / (s[1] + 1e-10)
                
                # Check circularity
                circularity = 1 - np.std(distances) / np.mean(distances)
                
                if linearity > self.formation_threshold:
                    return "line"
                elif circularity > self.formation_threshold:
                    return "circle"
                else:
                    return "scattered"
                
        except Exception as e:
            rospy.logerr(f"Error detecting formation pattern: {e}")
            return "unknown"

    def calculate_cluster_stability(self, cluster_id: int) -> float:
        """Calculate stability score using efficient memory access"""
        try:
            positions = self.memory_manager.get_data('cluster_positions')
            if not positions:
                return 0.0
                
            cluster_data = [(p[1], p[2]) for p in positions if p[0] == cluster_id]
            if len(cluster_data) < 2:
                return 0.0
            
            # Calculate position variance
            positions_array = np.array([p[0] for p in cluster_data])
            position_var = np.var(positions_array, axis=0).mean()
            
            # Calculate size variance
            sizes = [p[1] for p in cluster_data]
            size_var = np.var(sizes) if len(sizes) > 1 else 0
            
            # Combined stability score
            stability = 1.0 / (1.0 + position_var + size_var)
            return np.clip(stability, 0.0, 1.0)
            
        except Exception as e:
            rospy.logerr(f"Error calculating cluster stability: {e}")
            return 0.0

    def match_clusters(self, current_clusters: List[Dict], 
                      max_distance: float = 2.0) -> List[Dict]:
        """Match clusters using spatial indexing and efficient storage"""
        try:
            if not current_clusters:
                return []
            
            # Get previous cluster positions from memory manager
            prev_positions = self.memory_manager.get_data('cluster_positions')
            
            if not prev_positions:
                # First frame, initialize tracking
                for cluster in current_clusters:
                    cluster_id = self.next_cluster_id
                    self.next_cluster_id += 1
                    cluster['track_id'] = cluster_id
                    
                    # Store in memory manager
                    self.memory_manager.store_data('cluster_positions', 
                        (cluster_id, cluster['center'], cluster['size']))
                    
                return current_clusters
            
            # Build spatial index for previous positions
            prev_centers = np.array([p[1] for p in prev_positions])
            self.spatial_index.build_index(prev_centers)
            
            # Process clusters in parallel for large datasets
            if len(current_clusters) > 10:
                def match_cluster(cluster):
                    # Find nearest previous clusters
                    nearest_indices = self.spatial_index.query_neighbors(
                        cluster['center'], 
                        k=3
                    )[0]
                    
                    if not nearest_indices:
                        return cluster, True  # New cluster
                    
                    # Find best match
                    distances = [
                        np.linalg.norm(cluster['center'] - prev_centers[i])
                        for i in nearest_indices
                    ]
                    best_idx = nearest_indices[np.argmin(distances)]
                    min_distance = min(distances)
                    
                    if min_distance <= max_distance:
                        cluster_id = prev_positions[best_idx][0]
                        cluster['track_id'] = cluster_id
                        return cluster, False  # Matched cluster
                    else:
                        return cluster, True  # New cluster
                
                # Process clusters in parallel
                results = self.parallel_processor.map_async(
                    match_cluster,
                    current_clusters
                )
                
                # Handle results
                for cluster, is_new in results:
                    if is_new:
                        cluster_id = self.next_cluster_id
                        self.next_cluster_id += 1
                        cluster['track_id'] = cluster_id
                    
                    # Store updated position
                    self.memory_manager.store_data('cluster_positions',
                        (cluster['track_id'], cluster['center'], cluster['size']))
                    
            else:
                # Process sequentially for small datasets
                for cluster in current_clusters:
                    # Find nearest previous clusters
                    nearest_indices = self.spatial_index.query_neighbors(
                        cluster['center'],
                        k=3
                    )[0]
                    
                    if nearest_indices:
                        # Find best match
                        distances = [
                            np.linalg.norm(cluster['center'] - prev_centers[i])
                            for i in nearest_indices
                        ]
                        best_idx = nearest_indices[np.argmin(distances)]
                        min_distance = min(distances)
                        
                        if min_distance <= max_distance:
                            cluster_id = prev_positions[best_idx][0]
                            cluster['track_id'] = cluster_id
                        else:
                            cluster_id = self.next_cluster_id
                            self.next_cluster_id += 1
                            cluster['track_id'] = cluster_id
                    else:
                        cluster_id = self.next_cluster_id
                        self.next_cluster_id += 1
                        cluster['track_id'] = cluster_id
                    
                    # Store updated position
                    self.memory_manager.store_data('cluster_positions',
                        (cluster['track_id'], cluster['center'], cluster['size']))
            
            # Clean up old tracks periodically
            if self.frame_count % 50 == 0:
                self.memory_manager.cleanup_old_data()
            
            return current_clusters
            
        except Exception as e:
            rospy.logerr(f"Error matching clusters: {e}")
            return current_clusters

    def update_clusters(self, clusters: List[Dict]) -> List[Dict]:
        """Update cluster tracking with performance optimizations"""
        try:
            # Skip frames if needed
            self.frame_count += 1
            if self.frame_count % self.process_every_n_frames != 0:
                return clusters

            # Match and update clusters
            tracked_clusters = self.match_clusters(clusters)
            
            # Process clusters in parallel for large datasets
            if len(tracked_clusters) > 10:
                def process_cluster(cluster):
                    track_id = cluster.get('track_id')
                    if track_id is not None:
                        cluster['velocity'] = self.calculate_cluster_velocity(track_id)
                        cluster['stability'] = self.calculate_cluster_stability(track_id)
                        cluster['pattern'] = self.detect_formation_pattern(cluster['points'])
                        
                        # Calculate confidence
                        positions = self.memory_manager.get_data('cluster_positions')
                        history_length = len([p for p in positions if p[0] == track_id])
                        cluster['confidence'] = min(
                            history_length / self.max_tracking_history,
                            1.0
                        )
                    return cluster
                
                # Process clusters in parallel
                processed_clusters = self.parallel_processor.map_async(
                    process_cluster,
                    tracked_clusters
                )
                
                return [c for c in processed_clusters if c is not None]
                
            else:
                # Process sequentially for small datasets
                for cluster in tracked_clusters:
                    track_id = cluster.get('track_id')
                    if track_id is not None:
                        cluster['velocity'] = self.calculate_cluster_velocity(track_id)
                        cluster['stability'] = self.calculate_cluster_stability(track_id)
                        cluster['pattern'] = self.detect_formation_pattern(cluster['points'])
                        
                        # Calculate confidence
                        positions = self.memory_manager.get_data('cluster_positions')
                        history_length = len([p for p in positions if p[0] == track_id])
                        cluster['confidence'] = min(
                            history_length / self.max_tracking_history,
                            1.0
                        )
                
                return tracked_clusters
            
        except Exception as e:
            rospy.logerr(f"Error updating clusters: {e}")
            return clusters
        