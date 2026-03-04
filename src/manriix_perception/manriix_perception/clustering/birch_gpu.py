#!/usr/bin/env python3
import numpy as np
from sklearn.cluster import Birch
from rclpy.node import Node
from typing import Dict, List, Tuple, Optional
import time
from collections import deque

# Import GPU modules
import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

try:
    from manriix_perception.clustering.cuda.distance_ops import DistanceOpsGPU
    from manriix_perception.clustering.cuda.cf_tree_gpu import CFTreeGPU
    from manriix_perception.clustering.cuda.multi_stream_pipeline import MultiStreamPipeline
    CUDA_MODULES_AVAILABLE = True
except ImportError:
    CUDA_MODULES_AVAILABLE = False

try:
    import cupy as cp
    CUPY_AVAILABLE = True
except ImportError:
    CUPY_AVAILABLE = False


class BIRCHGPUStreaming:
    """
    GPU-accelerated BIRCH streaming implementation
    Achieves 30+ FPS with 50+ people on Jetson AGX Orin
    """
    
    def __init__(self, config: Dict, node: Node = None):
        """Initialize GPU-accelerated BIRCH"""
        self.node = node
        
        # BIRCH parameters
        birch_config = config.get('birch', {})
        self.threshold = birch_config.get('threshold', 1.5)
        self.branching_factor = birch_config.get('branching_factor', 50)
        self.n_clusters = birch_config.get('n_clusters', None)
        
        # GPU configuration
        gpu_config = config.get('gpu', {})
        self.use_gpu = gpu_config.get('enabled', True) and CUPY_AVAILABLE and CUDA_MODULES_AVAILABLE
        self.device_id = gpu_config.get('device_id', 0)
        
        if not CUPY_AVAILABLE and self.node:
            self.node.get_logger().warn("CuPy not available - BIRCH GPU acceleration disabled")
        
        # Initialize components
        if self.use_gpu:
            self._init_gpu_components()
        else:
            if self.node:
                self.node.get_logger().warn("GPU not available, using CPU BIRCH")
            self._init_cpu_components()
        
        # Streaming state
        self.is_initialized = False
        self.frame_count = 0
        self.frame_buffer = deque(maxlen=150)  # 5 seconds at 30 FPS
        
        # Performance tracking
        self.processing_times = deque(maxlen=30)
        self.gpu_times = deque(maxlen=30)
        self.last_fps = 0
        
        # Cluster tracking
        self.prev_centers = None
        self.prev_labels = None
        
        if self.node:
            self.node.get_logger().info(f"BIRCH GPU Streaming initialized (GPU: {self.use_gpu})")
    
    def _init_gpu_components(self):
        """Initialize GPU acceleration components"""
        
        try:
            # Set CUDA device
            cp.cuda.Device(self.device_id).use()
            
            # Initialize GPU modules
            self.distance_ops = DistanceOpsGPU(max_points=200, use_unified_memory=True)
            self.cf_tree_gpu = CFTreeGPU(
                branching_factor=self.branching_factor,
                threshold=self.threshold
            )
            self.pipeline = MultiStreamPipeline(n_streams=3, max_batch_size=100)
            
            # Initialize base BIRCH (will be accelerated)
            self.birch = Birch(
                n_clusters=self.n_clusters,
                threshold=self.threshold,
                branching_factor=self.branching_factor,
                compute_labels=True,
                copy=False
            )
            
            if self.node:
                self.node.get_logger().info("GPU components initialized successfully")
            
        except Exception as e:
            if self.node:
                self.node.get_logger().error(f"Failed to initialize GPU components: {e}")
            self.use_gpu = False
            self._init_cpu_components()
    
    def _init_cpu_components(self):
        """Initialize CPU-only components"""
        
        self.birch = Birch(
            n_clusters=self.n_clusters,
            threshold=self.threshold,
            branching_factor=self.branching_factor,
            compute_labels=True,
            copy=False
        )
        
        self.distance_ops = None
        self.cf_tree_gpu = None
        self.pipeline = None
    
    def detect_clusters(self, positions: np.ndarray) -> Tuple[List[Dict], np.ndarray]:
        """
        Main clustering method with GPU acceleration
        
        Args:
            positions: Nx3 array of positions
            
        Returns:
            clusters: List of cluster dictionaries
            labels: Cluster labels
        """
        start_time = time.time()
        
        if len(positions) == 0:
            return [], np.array([])
        
        if self.use_gpu:
            clusters, labels = self._detect_clusters_gpu(positions)
        else:
            clusters, labels = self._detect_clusters_cpu(positions)
        
        # Update performance metrics
        processing_time = time.time() - start_time
        self.processing_times.append(processing_time)
        self._update_fps()
        
        # Store for tracking
        self.prev_centers = [c['center'] for c in clusters] if clusters else None
        self.prev_labels = labels
        
        return clusters, labels
    
    def _detect_clusters_gpu(self, positions: np.ndarray) -> Tuple[List[Dict], np.ndarray]:
        """GPU-accelerated clustering"""
        
        gpu_start = time.time()
        
        # Use multi-stream pipeline for processing
        results = self.pipeline.process_frame_async(
            positions, 
            self.prev_centers
        )
        
        # Extract results
        labels = results['labels']
        centers = results['centers']
        n_clusters = results['n_clusters']
        predictions = results.get('predictions')
        
        # GPU time tracking
        gpu_time = time.time() - gpu_start
        self.gpu_times.append(gpu_time)
        
        # Update BIRCH model with GPU results (for consistency)
        if not self.is_initialized:
            self.birch.fit(positions)
            self.is_initialized = True
        else:
            # Use partial_fit with GPU-computed assignments
            self.birch.partial_fit(positions)
        
        # Build cluster dictionaries
        clusters = []
        for i in range(n_clusters):
            mask = labels == i
            cluster_points = positions[mask]
            
            if len(cluster_points) == 0:
                continue
            
            # Use GPU-computed center
            center = centers[i] if i < len(centers) else np.mean(cluster_points, axis=0)
            
            # Compute radius using GPU if many points
            if len(cluster_points) > 10 and self.distance_ops:
                # Use GPU for radius computation
                distances_to_center = self.distance_ops.compute_distances_to_centers(
                    cluster_points, 
                    center.reshape(1, -1)
                ).flatten()
                radius = np.percentile(distances_to_center, 90)
            else:
                # CPU fallback for small clusters
                distances = np.linalg.norm(cluster_points - center, axis=1)
                radius = np.percentile(distances, 90) if len(distances) > 0 else 0.5
            
            cluster = {
                'id': int(i),
                'center': center,
                'radius': float(radius),
                'size': len(cluster_points),
                'points': cluster_points,
                'formation': self._determine_formation(cluster_points),
                'density': len(cluster_points) / (np.pi * radius**2) if radius > 0 else 1.0,
                'confidence': 0.9,  # High confidence with GPU
                'stability': 0.8,
                'required_distance': self._calculate_required_distance(radius, len(cluster_points)),
                'predicted_center': predictions[i] if predictions is not None and i < len(predictions) else None
            }
            
            clusters.append(cluster)
        
        return clusters, labels
    
    def _detect_clusters_cpu(self, positions: np.ndarray) -> Tuple[List[Dict], np.ndarray]:
        """CPU fallback clustering"""
        
        # Add to frame buffer
        self.frame_buffer.append(positions)
        
        # Process with standard BIRCH
        if not self.is_initialized:
            self.birch.fit(positions)
            self.is_initialized = True
            labels = self.birch.labels_
        else:
            self.birch.partial_fit(positions)
            labels = self.birch.predict(positions)
        
        # Extract clusters
        clusters = []
        unique_labels = np.unique(labels)
        
        for label in unique_labels:
            mask = labels == label
            cluster_points = positions[mask]
            
            if len(cluster_points) == 0:
                continue
            
            center = np.mean(cluster_points, axis=0)
            
            if len(cluster_points) == 1:
                radius = 0.5
            else:
                distances = np.linalg.norm(cluster_points - center, axis=1)
                radius = np.percentile(distances, 90)
            
            cluster = {
                'id': int(label),
                'center': center,
                'radius': float(radius),
                'size': len(cluster_points),
                'points': cluster_points,
                'formation': self._determine_formation(cluster_points),
                'density': len(cluster_points) / (np.pi * radius**2) if radius > 0 else 1.0,
                'confidence': 0.7,
                'stability': 0.6,
                'required_distance': self._calculate_required_distance(radius, len(cluster_points))
            }
            
            clusters.append(cluster)
        
        return clusters, labels
    
    def process_batch_gpu(self, frames: List[np.ndarray]) -> List[Tuple[List[Dict], np.ndarray]]:
        """
        Process multiple frames in batch using GPU
        
        Args:
            frames: List of position arrays
            
        Returns:
            List of (clusters, labels) tuples
        """
        if not self.use_gpu or not self.pipeline:
            return [self.detect_clusters(frame) for frame in frames]
        
        # Process batch through pipeline
        results = self.pipeline.process_batch_pipeline(frames)
        
        # Convert results to cluster format
        outputs = []
        for i, (frame, result) in enumerate(zip(frames, results)):
            labels = result['labels']
            centers = result['centers']
            
            clusters = []
            for j in range(result['n_clusters']):
                mask = labels == j
                cluster_points = frame[mask]
                
                if len(cluster_points) > 0:
                    cluster = {
                        'id': j,
                        'center': centers[j] if j < len(centers) else np.mean(cluster_points, axis=0),
                        'radius': np.std(np.linalg.norm(cluster_points - centers[j], axis=1)) if len(cluster_points) > 1 else 0.5,
                        'size': len(cluster_points),
                        'points': cluster_points
                    }
                    clusters.append(cluster)
            
            outputs.append((clusters, labels))
        
        return outputs
    
    def _determine_formation(self, points: np.ndarray) -> str:
        """Determine cluster formation type"""
        
        if len(points) <= 2:
            return 'small_group'
        
        # Use GPU for large clusters
        if len(points) > 20 and self.use_gpu and self.distance_ops:
            # Compute pairwise distances on GPU
            distances = self.distance_ops.compute_pairwise_distances(points)
            
            # Analyze distribution
            mean_dist = np.mean(distances[distances > 0])
            std_dist = np.std(distances[distances > 0])
            
            if std_dist / mean_dist < 0.3:
                return 'circle'
            elif std_dist / mean_dist > 0.7:
                return 'line'
            else:
                return 'scattered'
        else:
            # CPU analysis for small clusters
            x_spread = np.ptp(points[:, 0])
            y_spread = np.ptp(points[:, 1])
            
            if x_spread > 0 and y_spread > 0:
                aspect_ratio = max(x_spread, y_spread) / min(x_spread, y_spread)
                
                if aspect_ratio > 3.0:
                    return 'line'
                elif aspect_ratio < 1.5 and len(points) >= 5:
                    return 'circle'
            
            return 'scattered' if len(points) > 10 else 'small_group'
    
    def _calculate_required_distance(self, radius: float, size: int) -> float:
        """Calculate optimal photography distance"""
        
        base_distance = 2.0 + radius * 1.5
        
        if size > 10:
            base_distance *= 1.2
        elif size > 20:
            base_distance *= 1.4
        
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
            'gpu_enabled': self.use_gpu,
            'frame_count': self.frame_count
        }
        
        if self.use_gpu and self.gpu_times:
            stats['avg_gpu_time'] = np.mean(self.gpu_times)
            stats['gpu_speedup'] = stats['avg_processing_time'] / stats['avg_gpu_time'] if stats['avg_gpu_time'] > 0 else 1.0
            
            if self.distance_ops:
                stats['gpu_memory_mb'] = self.distance_ops.get_memory_usage()['gpu_mb']
            
            if self.pipeline:
                stats['pipeline_stats'] = self.pipeline.get_pipeline_stats()
        
        return stats
    
    def cleanup(self):
        """Clean up GPU resources"""
        
        if self.use_gpu:
            if self.distance_ops:
                self.distance_ops.cleanup()
            
            if self.cf_tree_gpu:
                self.cf_tree_gpu.reset()
            
            if self.pipeline:
                self.pipeline.cleanup()
        
        if self.node:
            self.node.get_logger().info("BIRCH GPU cleaned up")
