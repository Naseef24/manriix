#!/usr/bin/env python3

import numpy as np
from sklearn.cluster import DBSCAN
from typing import List, Dict, Tuple
import rospy

import sys
sys.path.append("/home/hype/Desktop/lasa/depth_cam/depth_ws/src/human_clustering_pipeline")

from src.performance_utils import SpatialIndex, ParallelProcessor

class ClusterDetector:
    """Enhanced human clustering with dynamic margins and spatial analysis"""
    
    def __init__(self, config: Dict):
        """Initialize cluster detector with configuration"""
        self.config = config
        
        # Camera parameters
        self.fov_horizontal = np.radians(config['camera']['fov']['horizontal'])
        self.min_distance = config['camera']['working_distance']['min']
        self.max_distance = config['camera']['working_distance']['max']
        
        # DBSCAN parameters
        self.min_samples = 1  # Allow single-person clusters
        self.cluster_eps = 2 * self.min_distance * np.tan(self.fov_horizontal/2)
        
        # Spatial parameters
        self.min_separation = 1.0  # Minimum separation between clusters
        self.personal_space = 0.5  # Personal space radius per person
        
        # Performance optimizations
        self.spatial_index = SpatialIndex()
        self.parallel_processor = ParallelProcessor()
        self.frame_count = 0
        self.process_every_n_frames = 2  # Process every nth frame
        
        rospy.loginfo("Enhanced ClusterDetector initialized with optimizations")

    def calculate_dynamic_radius(self, points: np.ndarray) -> float:
        """Calculate dynamic radius with parallel processing"""
        try:
            if len(points) == 0:
                return 0.0
                
            if len(points) == 1:
                return self.personal_space
            
            # Calculate spatial distribution using parallel processing for large groups
            if len(points) > 10:
                chunks = list(self.parallel_processor.process_chunks(points, 5))
                
                # Process chunks in parallel
                def process_chunk(chunk):
                    center = np.mean(chunk, axis=0)
                    distances = np.linalg.norm(chunk - center, axis=1)
                    return np.mean(distances), len(chunk)
                
                results = self.parallel_processor.map_async(process_chunk, chunks)
                
                # Combine results
                total_distance = 0
                total_points = 0
                for mean_dist, n_points in results:
                    total_distance += mean_dist * n_points
                    total_points += n_points
                
                spatial_spread = total_distance / total_points
            else:
                # For small groups, process directly
                center = np.mean(points, axis=0)
                distances = np.linalg.norm(points - center, axis=1)
                spatial_spread = np.std(distances)
            
            # Base radius on number of people and their distribution
            n_people = len(points)
            base_radius = self.personal_space * np.sqrt(n_people)
            radius = base_radius + spatial_spread
            
            # Add margin based on group size
            margin = 0.2 * np.log2(n_people + 1)
            radius *= (1 + margin)
            
            return radius
            
        except Exception as e:
            rospy.logerr(f"Error calculating dynamic radius: {e}")
            return self.personal_space

    def find_minimum_circle(self, points: np.ndarray) -> Tuple[np.ndarray, float]:
        """Calculate minimum enclosing circle with spatial indexing"""
        try:
            if len(points) == 0:
                return np.array([0, 0]), 0
                
            if len(points) == 1:
                return points[0], self.personal_space
            
            # Use spatial index for efficient neighbor queries
            self.spatial_index.build_index(points)
            
            # Calculate center
            center = np.mean(points, axis=0)
            
            # Use spatial index to find points near the boundary
            radius = self.calculate_dynamic_radius(points)
            
            return center, radius
            
        except Exception as e:
            rospy.logerr(f"Error in find_minimum_circle: {e}")
            return np.array([0, 0]), 0

    def adjust_cluster_separation(self, clusters: List[Dict]) -> List[Dict]:
        """Adjust cluster separation with spatial indexing"""
        try:
            if len(clusters) <= 1:
                return clusters
            
            # Build spatial index for cluster centers
            centers = np.array([c['center'] for c in clusters])
            spatial_index = SpatialIndex()
            spatial_index.build_index(centers)
            
            # Process each cluster
            for i, cluster in enumerate(clusters):
                # Find nearby clusters efficiently using spatial index
                nearby_indices = spatial_index.query_radius(
                    cluster['center'], 
                    cluster['radius'] * 2 + self.min_separation
                )
                
                for j in nearby_indices:
                    if i != j:
                        center1 = cluster['center']
                        center2 = clusters[j]['center']
                        distance = np.linalg.norm(center1 - center2)
                        combined_radius = cluster['radius'] + clusters[j]['radius']
                        
                        if distance < combined_radius + self.min_separation:
                            overlap = combined_radius + self.min_separation - distance
                            total_size = cluster['size'] + clusters[j]['size']
                            
                            if total_size > 0:
                                ratio1 = cluster['size'] / total_size
                                ratio2 = clusters[j]['size'] / total_size
                                clusters[i]['radius'] -= overlap * ratio1
                                clusters[j]['radius'] -= overlap * ratio2
            
            return clusters
            
        except Exception as e:
            rospy.logerr(f"Error adjusting cluster separation: {e}")
            return clusters

    def calculate_required_distance(self, radius: float) -> float:
        """Calculate required camera distance"""
        try:
            distance = radius / np.tan(self.fov_horizontal/2)
            distance *= 1.2  # Add 20% margin
            return np.clip(distance, self.min_distance, self.max_distance)
            
        except Exception as e:
            rospy.logerr(f"Error in calculate_required_distance: {e}")
            return self.min_distance

    def analyze_cluster_formation(self, points: np.ndarray) -> str:
        """Analyze formation pattern using parallel processing for large clusters"""
        try:
            if len(points) < 3:
                return "small_group"
            
            # For large clusters, use parallel processing
            if len(points) > 10:
                chunks = list(self.parallel_processor.process_chunks(points, 5))
                
                def analyze_chunk(chunk):
                    centered = chunk - np.mean(chunk, axis=0)
                    _, s, _ = np.linalg.svd(centered)
                    linearity = s[0] / (s[1] + 1e-10)
                    return linearity
                
                linearities = self.parallel_processor.map_async(analyze_chunk, chunks)
                avg_linearity = np.mean([l for l in linearities if l is not None])
                
                # Check circularity
                center = np.mean(points, axis=0)
                radii = np.linalg.norm(points - center, axis=1)
                circularity = 1 - np.std(radii) / np.mean(radii)
                
                if avg_linearity > 2.0:
                    return "line"
                elif circularity > 0.8:
                    return "circle"
                else:
                    return "scattered"
            else:
                # For small clusters, use original method
                centered = points - np.mean(points, axis=0)
                _, s, _ = np.linalg.svd(centered)
                linearity = s[0] / (s[1] + 1e-10)
                
                center = np.mean(points, axis=0)
                radii = np.linalg.norm(points - center, axis=1)
                circularity = 1 - np.std(radii) / np.mean(radii)
                
                if linearity > 2.0:
                    return "line"
                elif circularity > 0.8:
                    return "circle"
                else:
                    return "scattered"
                
        except Exception as e:
            rospy.logerr(f"Error analyzing cluster formation: {e}")
            return "unknown"

    def detect_clusters(self, positions: Dict[int, Dict]) -> List[Dict]:
        """Detect and analyze clusters with performance optimizations"""
        try:
            if not positions:
                return []

            # Skip frames if needed
            self.frame_count += 1
            if self.frame_count % self.process_every_n_frames != 0:
                return []

            # Extract positions array
            points = np.array([[pos['map_position'][0], pos['map_position'][1]] 
                             for pos in positions.values()])
            ids = list(positions.keys())
            
            # Build spatial index for efficient neighbor queries
            self.spatial_index.build_index(points)
            
            # Run DBSCAN
            clustering = DBSCAN(
                eps=self.cluster_eps,
                min_samples=self.min_samples
            ).fit(points)
            
            clusters = []
            unique_labels = np.unique(clustering.labels_)
            
            # Process clusters in parallel for large datasets
            if len(points) > 20:
                def process_cluster(label):
                    if label == -1:  # Skip noise points
                        return None
                        
                    mask = clustering.labels_ == label
                    cluster_points = points[mask]
                    cluster_ids = [id for i, id in enumerate(ids) if mask[i]]
                    
                    center, radius = self.find_minimum_circle(cluster_points)
                    required_distance = self.calculate_required_distance(radius)
                    formation = self.analyze_cluster_formation(cluster_points)
                    
                    area = np.pi * radius**2
                    density = len(cluster_points) / area if area > 0 else 0
                    distances = np.linalg.norm(cluster_points - center, axis=1)
                    spread = np.mean(distances)
                    
                    return {
                        'center': center,
                        'radius': radius,
                        'points': cluster_points,
                        'ids': cluster_ids,
                        'size': len(cluster_points),
                        'required_distance': required_distance,
                        'formation': formation,
                        'density': density,
                        'spread': spread
                    }
                
                # Process clusters in parallel
                cluster_results = self.parallel_processor.map_async(
                    process_cluster,
                    [label for label in unique_labels if label != -1]
                )
                
                clusters = [c for c in cluster_results if c is not None]
                
            else:
                # Process sequentially for small datasets
                for label in unique_labels:
                    if label == -1:
                        continue
                        
                    mask = clustering.labels_ == label
                    cluster_points = points[mask]
                    cluster_ids = [id for i, id in enumerate(ids) if mask[i]]
                    
                    center, radius = self.find_minimum_circle(cluster_points)
                    required_distance = self.calculate_required_distance(radius)
                    formation = self.analyze_cluster_formation(cluster_points)
                    
                    area = np.pi * radius**2
                    density = len(cluster_points) / area if area > 0 else 0
                    distances = np.linalg.norm(cluster_points - center, axis=1)
                    spread = np.mean(distances)
                    
                    clusters.append({
                        'center': center,
                        'radius': radius,
                        'points': cluster_points,
                        'ids': cluster_ids,
                        'size': len(cluster_points),
                        'required_distance': required_distance,
                        'formation': formation,
                        'density': density,
                        'spread': spread
                    })
            
            # Adjust cluster separation
            clusters = self.adjust_cluster_separation(clusters)
            
            # if self.config['debug']['print_positions']:
            #     self._print_cluster_info(clusters)
            
            return clusters
            
        except Exception as e:
            rospy.logerr(f"Error in cluster detection: {e}")
            return []

    def _print_cluster_info(self, clusters: List[Dict]):
        """Print debug information about detected clusters"""
        try:
            rospy.loginfo("\n=== Cluster Detection Results ===")
            for i, cluster in enumerate(clusters):
                rospy.loginfo(f"\nCluster {i+1}:")
                rospy.loginfo(f"  Size: {cluster['size']} humans")
                rospy.loginfo(f"  Center: ({cluster['center'][0]:.2f}, {cluster['center'][1]:.2f})")
                rospy.loginfo(f"  Radius: {cluster['radius']:.2f}m")
                rospy.loginfo(f"  Formation: {cluster['formation']}")
                rospy.loginfo(f"  Density: {cluster['density']:.2f} humans/m²")
                rospy.loginfo(f"  Spread: {cluster['spread']:.2f}m")
                rospy.loginfo(f"  Required Distance: {cluster['required_distance']:.2f}m")
                rospy.loginfo(f"  Member IDs: {cluster['ids']}")
                
        except Exception as e:
            rospy.logerr(f"Error printing cluster info: {e}")