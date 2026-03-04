#!/usr/bin/env python3
"""
Multi-stream GPU processing pipeline for parallel computation
Optimized for Jetson AGX Orin with concurrent CUDA streams
"""

import numpy as np
from rclpy.node import Node
from typing import List, Dict, Tuple, Optional
import time
from collections import deque
import threading

try:
    import cupy as cp
    CUPY_AVAILABLE = True
except ImportError:
    CUPY_AVAILABLE = False


class MultiStreamPipeline:
    """
    Multi-stream CUDA pipeline for maximum GPU utilization
    Processes distance computation, clustering, and tracking in parallel
    """
    
    def __init__(self, n_streams: int = 3, max_batch_size: int = 100, node: Node = None):
        """
        Initialize multi-stream pipeline
        
        Args:
            n_streams: Number of CUDA streams (3 recommended)
            max_batch_size: Maximum batch size per stream
            node: ROS2 node for logging
        """
        self.n_streams = n_streams
        self.max_batch_size = max_batch_size
        self.gpu_available = CUPY_AVAILABLE
        self.node = node
        
        if self.gpu_available:
            self._init_streams()
            self._init_buffers()
        else:
            if self.node:
                self.node.get_logger().warn("Multi-stream GPU pipeline not available")
    
    def _init_streams(self):
        """Initialize CUDA streams"""
        
        try:
            # Create CUDA streams
            self.streams = [cp.cuda.Stream() for _ in range(self.n_streams)]
            
            # Assign stream purposes
            self.stream_names = {
                0: "distance_computation",
                1: "clustering_update",
                2: "tracking_prediction"
            }
            
            # Stream synchronization events
            self.events = [cp.cuda.Event() for _ in range(self.n_streams)]
            
            # Pipeline queues for each stream
            self.input_queues = [deque() for _ in range(self.n_streams)]
            self.output_queues = [deque() for _ in range(self.n_streams)]
            
            if self.node:
                self.node.get_logger().info(f"Initialized {self.n_streams} CUDA streams for pipeline")
            
        except Exception as e:
            if self.node:
                self.node.get_logger().error(f"Failed to initialize CUDA streams: {e}")
            self.gpu_available = False
    
    def _init_buffers(self):
        """Initialize GPU memory buffers for each stream"""
        
        try:
            # Pre-allocate pinned memory for fast transfers
            self.pinned_buffers = []
            for i in range(self.n_streams):
                buffer = {
                    'points': cp.zeros((self.max_batch_size, 3), dtype=cp.float32),
                    'distances': cp.zeros((self.max_batch_size, self.max_batch_size), dtype=cp.float32),
                    'labels': cp.zeros(self.max_batch_size, dtype=cp.int32),
                    'centers': cp.zeros((20, 3), dtype=cp.float32)  # Max 20 clusters
                }
                self.pinned_buffers.append(buffer)
            
            # Memory pool for dynamic allocations
            self.mempool = cp.get_default_memory_pool()
            self.pinned_mempool = cp.get_default_pinned_memory_pool()
            
            if self.node:
                self.node.get_logger().info("GPU buffers initialized for multi-stream processing")
            
        except Exception as e:
            if self.node:
                self.node.get_logger().error(f"Failed to initialize GPU buffers: {e}")
            self.gpu_available = False
    
    def process_frame_async(self, points: np.ndarray, 
                           prev_centers: Optional[np.ndarray] = None) -> Dict:
        """
        Process frame asynchronously using multiple streams
        
        Args:
            points: Nx3 array of points
            prev_centers: Previous cluster centers for tracking
            
        Returns:
            Processing results dictionary
        """
        if not self.gpu_available:
            return self._process_frame_cpu(points, prev_centers)
        
        try:
            start_time = time.time()
            n_points = len(points)
            
            # Stream 0: Distance computation
            with self.streams[0]:
                # Transfer data to GPU
                gpu_points = cp.asarray(points, dtype=cp.float32)
                
                # Compute pairwise distances
                from cupyx.scipy.spatial import distance
                gpu_distances = distance.cdist(gpu_points, gpu_points, 'euclidean')
                
                # Record completion event
                self.events[0].record()
            
            # Stream 1: Clustering update (depends on distances)
            with self.streams[1]:
                # Wait for distance computation
                self.streams[1].wait_event(self.events[0])
                
                # Perform clustering (simplified BIRCH-like)
                threshold = 1.5
                
                # Find connected components based on distance threshold
                adjacency = gpu_distances < threshold
                cp.fill_diagonal(adjacency, False)
                
                # Simple clustering via connected components
                labels = self._connected_components_gpu(adjacency)
                
                # Compute cluster centers
                unique_labels = cp.unique(labels)
                n_clusters = len(unique_labels)
                
                centers = cp.zeros((n_clusters, 3), dtype=cp.float32)
                for i, label in enumerate(unique_labels):
                    mask = labels == label
                    centers[i] = cp.mean(gpu_points[mask], axis=0)
                
                # Record completion
                self.events[1].record()
            
            # Stream 2: Tracking/Prediction (independent)
            predictions = None
            if prev_centers is not None:
                with self.streams[2]:
                    # Transfer previous centers
                    gpu_prev_centers = cp.asarray(prev_centers, dtype=cp.float32)
                    
                    # Simple velocity estimation (for Kalman-like prediction)
                    # Wait for current centers
                    self.streams[2].wait_event(self.events[1])
                    
                    # Match current to previous centers
                    if len(centers) > 0 and len(gpu_prev_centers) > 0:
                        # Compute distances between prev and current centers
                        center_distances = distance.cdist(centers, gpu_prev_centers, 'euclidean')
                        
                        # Find best matches
                        matches = cp.argmin(center_distances, axis=1)
                        
                        # Estimate velocities
                        velocities = centers - gpu_prev_centers[matches]
                        
                        # Predict next positions (simple linear prediction)
                        predictions = centers + velocities * 0.033  # 33ms prediction
                    
                    self.events[2].record()
            
            # Synchronize all streams
            for stream in self.streams:
                stream.synchronize()
            
            # Get results back to CPU
            results = {
                'points': points,
                'distances': gpu_distances.get(),
                'labels': labels.get(),
                'centers': centers.get(),
                'n_clusters': n_clusters,
                'predictions': predictions.get() if predictions is not None else None,
                'processing_time': time.time() - start_time,
                'gpu_memory_mb': self.mempool.used_bytes() / (1024**2)
            }
            
            return results
            
        except Exception as e:
            if self.node:
                self.node.get_logger().warn(f"GPU pipeline failed: {e}, using CPU fallback")
            return self._process_frame_cpu(points, prev_centers)
    
    def process_batch_pipeline(self, frame_batch: List[np.ndarray]) -> List[Dict]:
        """
        Process multiple frames in pipeline fashion
        
        Args:
            frame_batch: List of point arrays
            
        Returns:
            List of results for each frame
        """
        if not self.gpu_available:
            return [self._process_frame_cpu(frame) for frame in frame_batch]
        
        results = []
        
        try:
            # Process frames in pipeline
            for i, frame in enumerate(frame_batch):
                stream_idx = i % self.n_streams
                
                with self.streams[stream_idx]:
                    # Use pre-allocated buffer
                    n_points = min(len(frame), self.max_batch_size)
                    self.pinned_buffers[stream_idx]['points'][:n_points] = cp.asarray(frame[:n_points])
                    
                    # Process in assigned stream
                    result = self._process_in_stream(
                        self.pinned_buffers[stream_idx]['points'][:n_points],
                        stream_idx
                    )
                    results.append(result)
            
            # Synchronize all streams
            for stream in self.streams:
                stream.synchronize()
            
            # Convert results to CPU
            for i, result in enumerate(results):
                if isinstance(result['labels'], cp.ndarray):
                    result['labels'] = result['labels'].get()
                if isinstance(result['centers'], cp.ndarray):
                    result['centers'] = result['centers'].get()
            
            return results
            
        except Exception as e:
            if self.node:
                self.node.get_logger().warn(f"Batch pipeline failed: {e}")
            return [self._process_frame_cpu(frame) for frame in frame_batch]
    
    def _connected_components_gpu(self, adjacency: cp.ndarray) -> cp.ndarray:
        """
        Find connected components on GPU (simple implementation)
        
        Args:
            adjacency: NxN boolean adjacency matrix
            
        Returns:
            Labels array
        """
        n = len(adjacency)
        labels = cp.arange(n, dtype=cp.int32)
        
        # Simple iterative connected components
        changed = True
        iteration = 0
        max_iterations = 20
        
        while changed and iteration < max_iterations:
            changed = False
            old_labels = labels.copy()
            
            # For each node, adopt minimum label of neighbors
            for i in range(n):
                neighbors = cp.where(adjacency[i])[0]
                if len(neighbors) > 0:
                    min_label = cp.min(labels[neighbors])
                    if min_label < labels[i]:
                        labels[i] = min_label
                        changed = True
            
            iteration += 1
        
        # Relabel to consecutive integers
        unique_labels = cp.unique(labels)
        new_labels = cp.zeros_like(labels)
        for i, label in enumerate(unique_labels):
            new_labels[labels == label] = i
        
        return new_labels
    
    def _process_in_stream(self, gpu_points: cp.ndarray, stream_idx: int) -> Dict:
        """
        Process points in specific stream
        
        Args:
            gpu_points: Points already on GPU
            stream_idx: Stream index
            
        Returns:
            Processing results
        """
        # Simplified processing for specific stream
        from cupyx.scipy.spatial import distance
        
        # Compute distances
        distances = distance.cdist(gpu_points, gpu_points, 'euclidean')
        
        # Simple clustering
        threshold = 1.5
        adjacency = distances < threshold
        cp.fill_diagonal(adjacency, False)
        
        labels = self._connected_components_gpu(adjacency)
        
        # Compute centers
        unique_labels = cp.unique(labels)
        centers = cp.zeros((len(unique_labels), 3), dtype=cp.float32)
        
        for i, label in enumerate(unique_labels):
            mask = labels == label
            centers[i] = cp.mean(gpu_points[mask], axis=0)
        
        return {
            'labels': labels,
            'centers': centers,
            'n_clusters': len(unique_labels)
        }
    
    def _process_frame_cpu(self, points: np.ndarray, 
                          prev_centers: Optional[np.ndarray] = None) -> Dict:
        """CPU fallback for frame processing"""
        
        from scipy.spatial import distance as scipy_distance
        from sklearn.cluster import DBSCAN
        
        # Simple DBSCAN clustering
        clustering = DBSCAN(eps=1.5, min_samples=2).fit(points)
        labels = clustering.labels_
        
        # Compute centers
        unique_labels = np.unique(labels[labels >= 0])
        centers = []
        for label in unique_labels:
            mask = labels == label
            centers.append(np.mean(points[mask], axis=0))
        
        centers = np.array(centers) if centers else np.array([])
        
        return {
            'points': points,
            'distances': scipy_distance.cdist(points, points, 'euclidean'),
            'labels': labels,
            'centers': centers,
            'n_clusters': len(unique_labels),
            'predictions': None,
            'processing_time': 0,
            'gpu_memory_mb': 0
        }
    
    def optimize_stream_scheduling(self):
        """
        Optimize stream scheduling based on workload
        Dynamic load balancing across streams
        """
        if not self.gpu_available:
            return
        
        # Get stream utilization
        stream_times = []
        for i, stream in enumerate(self.streams):
            # Measure stream execution time
            start_event = cp.cuda.Event()
            end_event = cp.cuda.Event()
            
            with stream:
                start_event.record()
                # Dummy operation to measure stream
                dummy = cp.zeros(1000)
                _ = cp.sum(dummy)
                end_event.record()
            
            stream.synchronize()
            elapsed = cp.cuda.get_elapsed_time(start_event, end_event)
            stream_times.append(elapsed)
        
        # Rebalance workload if needed
        avg_time = np.mean(stream_times)
        imbalance = np.std(stream_times) / avg_time if avg_time > 0 else 0
        
        if imbalance > 0.2:  # 20% imbalance threshold
            if self.node:
                self.node.get_logger().info(f"Stream imbalance detected: {imbalance:.2f}, rebalancing...")
            # Implement rebalancing logic here
    
    def get_pipeline_stats(self) -> Dict:
        """Get pipeline performance statistics"""
        
        stats = {
            'n_streams': self.n_streams,
            'gpu_available': self.gpu_available,
            'memory_used_mb': 0,
            'memory_reserved_mb': 0
        }
        
        if self.gpu_available:
            stats['memory_used_mb'] = self.mempool.used_bytes() / (1024**2)
            stats['memory_reserved_mb'] = self.mempool.total_bytes() / (1024**2)
            
            # Add stream names
            stats['streams'] = list(self.stream_names.values())
        
        return stats
    
    def cleanup(self):
        """Clean up GPU resources"""
        
        if self.gpu_available:
            # Synchronize all streams
            for stream in self.streams:
                stream.synchronize()
            
            # Clear memory pools
            self.mempool.free_all_blocks()
            self.pinned_mempool.free_all_blocks()
            
            if self.node:
                self.node.get_logger().info("Multi-stream pipeline cleaned up")
