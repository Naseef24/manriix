#!/usr/bin/env python3

import numpy as np
from typing import Dict, List
from scipy.optimize import linear_sum_assignment
import rclpy
from rclpy.node import Node

class ClusterTracker:
    """Track clusters over time using Hungarian algorithm"""
    
    def __init__(self, config: Dict, node: Node = None):
        """Initialize cluster tracker"""
        self.config = config
        self.node = node
        
        # Tracking parameters
        tracking_config = config.get('tracking', {})
        self.max_tracks = tracking_config.get('max_tracks', 100)
        self.max_age = tracking_config.get('max_age', 10)
        self.min_hits = tracking_config.get('min_hits', 3)
        
        # Track storage
        self.tracks = {}
        self.next_id = 0
        self.frame_count = 0
        
        if self.node:
            self.node.get_logger().info("ClusterTracker initialized")
    
    def update_clusters(self, clusters: List[Dict]) -> List[Dict]:
        """
        Update cluster tracks
        
        Args:
            clusters: List of detected clusters
            
        Returns:
            List of tracked clusters with IDs
        """
        self.frame_count += 1
        
        if not clusters:
            self._age_tracks()
            return []
        
        # Match clusters to existing tracks
        matched_tracks, unmatched_clusters = self._match_clusters(clusters)
        
        # Update matched tracks
        for track_id, cluster_idx in matched_tracks:
            self.tracks[track_id]['cluster'] = clusters[cluster_idx]
            self.tracks[track_id]['age'] = 0
            self.tracks[track_id]['hits'] += 1
            self.tracks[track_id]['last_seen'] = self.frame_count
        
        # Create new tracks for unmatched clusters
        for cluster_idx in unmatched_clusters:
            self._create_track(clusters[cluster_idx])
        
        # Age and remove old tracks
        self._age_tracks()
        
        # Return confirmed tracks
        tracked_clusters = []
        for track_id, track in self.tracks.items():
            if track['hits'] >= self.min_hits:
                cluster = track['cluster'].copy()
                cluster['track_id'] = track_id
                tracked_clusters.append(cluster)
        
        return tracked_clusters
    
    def _match_clusters(self, clusters: List[Dict]):
        """Match clusters to existing tracks using Hungarian algorithm"""
        if not self.tracks or not clusters:
            return [], list(range(len(clusters)))
        
        # Build cost matrix
        track_ids = list(self.tracks.keys())
        cost_matrix = np.zeros((len(track_ids), len(clusters)))
        
        for i, track_id in enumerate(track_ids):
            track_center = self.tracks[track_id]['cluster']['center']
            
            for j, cluster in enumerate(clusters):
                cluster_center = cluster['center']
                distance = np.linalg.norm(track_center - cluster_center)
                cost_matrix[i, j] = distance
        
        # Solve assignment problem
        row_ind, col_ind = linear_sum_assignment(cost_matrix)
        
        # Filter matches by distance threshold
        max_distance = 2.0
        matched = []
        matched_cluster_indices = set()
        
        for i, j in zip(row_ind, col_ind):
            if cost_matrix[i, j] < max_distance:
                matched.append((track_ids[i], j))
                matched_cluster_indices.add(j)
        
        # Find unmatched clusters
        unmatched = [i for i in range(len(clusters)) if i not in matched_cluster_indices]
        
        return matched, unmatched
    
    def _create_track(self, cluster: Dict):
        """Create a new track"""
        track_id = self.next_id
        self.next_id += 1
        
        self.tracks[track_id] = {
            'cluster': cluster,
            'age': 0,
            'hits': 1,
            'last_seen': self.frame_count
        }
    
    def _age_tracks(self):
        """Age tracks and remove old ones"""
        to_remove = []
        
        for track_id, track in self.tracks.items():
            track['age'] += 1
            
            if track['age'] > self.max_age:
                to_remove.append(track_id)
        
        for track_id in to_remove:
            del self.tracks[track_id]
