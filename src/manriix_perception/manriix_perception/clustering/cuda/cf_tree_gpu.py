#!/usr/bin/env python3
"""
GPU-accelerated CF-Tree operations for BIRCH clustering
Optimized for streaming data on Jetson AGX Orin
"""

import numpy as np
from rclpy.node import Node
from typing import Dict, List, Tuple, Optional
import time

try:
    import cupy as cp
    from numba import cuda
    import numba
    CUDA_AVAILABLE = True
except ImportError:
    CUDA_AVAILABLE = False


class CFTreeGPU:
    """
    GPU-accelerated Clustering Feature Tree operations
    Handles CF statistics updates and node operations on GPU
    """
    
    def __init__(self, branching_factor: int = 50, threshold: float = 1.5, node: Node = None):
        """
        Initialize GPU CF-Tree
        
        Args:
            branching_factor: Maximum entries per node
            threshold: Radius threshold for subclusters
            node: ROS2 node for logging
        """
        self.branching_factor = branching_factor
        self.threshold = threshold
        self.gpu_available = CUDA_AVAILABLE
        self.node = node
        
        if not CUDA_AVAILABLE and self.node:
            self.node.get_logger().warn("CUDA acceleration not available for CF-Tree")
        
        if self.gpu_available:
            self._init_gpu_structures()
            self._compile_kernels()
        else:
            if self.node:
                self.node.get_logger().warn("CF-Tree GPU acceleration not available")
    
    def _init_gpu_structures(self):
        """Initialize GPU data structures for CF-Tree"""
        
        try:
            # Pre-allocate GPU arrays for CF statistics
            max_nodes = 1000  # Maximum CF nodes
            
            # CF statistics: N (count), LS (linear sum), SS (squared sum)
            self.cf_n = cp.zeros(max_nodes, dtype=cp.int32)
            self.cf_ls = cp.zeros((max_nodes, 3), dtype=cp.float32)
            self.cf_ss = cp.zeros(max_nodes, dtype=cp.float32)
            
            # Node structure
            self.node_centers = cp.zeros((max_nodes, 3), dtype=cp.float32)
            self.node_radii = cp.zeros(max_nodes, dtype=cp.float32)
            self.node_parent = cp.full(max_nodes, -1, dtype=cp.int32)
            self.node_children = cp.full((max_nodes, self.branching_factor), -1, dtype=cp.int32)
            
            self.n_nodes = 0
            self.max_nodes = max_nodes
            
            if self.node:
                self.node.get_logger().info(f"CF-Tree GPU structures initialized (max_nodes={max_nodes})")
            
        except Exception as e:
            if self.node:
                self.node.get_logger().error(f"Failed to initialize CF-Tree GPU: {e}")
            self.gpu_available = False
    
    def _compile_kernels(self):
        """Compile CUDA kernels for CF operations"""
        
        if not CUDA_AVAILABLE:
            return
        
        # Kernel for updating CF statistics
        @cuda.jit
        def update_cf_stats_kernel(points, assignments, cf_n, cf_ls, cf_ss):
            """Update CF statistics in parallel"""
            tid = cuda.grid(1)
            
            if tid < points.shape[0]:
                cluster_id = assignments[tid]
                if cluster_id >= 0:
                    # Atomic add for count
                    cuda.atomic.add(cf_n, cluster_id, 1)
                    
                    # Atomic add for linear sum
                    for dim in range(3):
                        cuda.atomic.add(cf_ls[cluster_id], dim, points[tid, dim])
                    
                    # Atomic add for squared sum
                    sq_sum = 0.0
                    for dim in range(3):
                        sq_sum += points[tid, dim] * points[tid, dim]
                    cuda.atomic.add(cf_ss, cluster_id, sq_sum)
        
        # Kernel for computing centroids
        @cuda.jit
        def compute_centroids_kernel(cf_n, cf_ls, centroids):
            """Compute centroids from CF statistics"""
            tid = cuda.grid(1)
            
            if tid < centroids.shape[0]:
                n = cf_n[tid]
                if n > 0:
                    for dim in range(3):
                        centroids[tid, dim] = cf_ls[tid, dim] / n
                else:
                    for dim in range(3):
                        centroids[tid, dim] = 0.0
        
        # Kernel for radius computation
        @cuda.jit
        def compute_radius_kernel(cf_n, cf_ls, cf_ss, radii):
            """Compute radius from CF statistics"""
            tid = cuda.grid(1)
            
            if tid < radii.shape[0]:
                n = cf_n[tid]
                if n > 0:
                    # Radius = sqrt((SS/N) - (LS/N)^2)
                    ss_n = cf_ss[tid] / n
                    ls_n_sq = 0.0
                    for dim in range(3):
                        ls_n = cf_ls[tid, dim] / n
                        ls_n_sq += ls_n * ls_n
                    
                    variance = ss_n - ls_n_sq
                    if variance > 0:
                        radii[tid] = numba.cuda.libdevice.sqrt(variance)
                    else:
                        radii[tid] = 0.0
                else:
                    radii[tid] = 0.0
        
        # Store compiled kernels
        self.update_cf_stats_kernel = update_cf_stats_kernel
        self.compute_centroids_kernel = compute_centroids_kernel
        self.compute_radius_kernel = compute_radius_kernel
        
        if self.node:
            self.node.get_logger().info("CF-Tree CUDA kernels compiled")
    
    def update_cf_statistics(self, points: np.ndarray, assignments: np.ndarray):
        """
        Update CF statistics on GPU
        
        Args:
            points: Nx3 array of points
            assignments: N array of cluster assignments
        """
        if not self.gpu_available:
            return self._update_cf_statistics_cpu(points, assignments)
        
        try:
            # Transfer to GPU
            gpu_points = cp.asarray(points, dtype=cp.float32)
            gpu_assignments = cp.asarray(assignments, dtype=cp.int32)
            
            # Reset statistics
            n_clusters = int(cp.max(gpu_assignments).get()) + 1
            self.cf_n[:n_clusters] = 0
            self.cf_ls[:n_clusters] = 0
            self.cf_ss[:n_clusters] = 0
            
            # Launch kernel
            threads_per_block = 256
            blocks_per_grid = (len(points) + threads_per_block - 1) // threads_per_block
            
            self.update_cf_stats_kernel[blocks_per_grid, threads_per_block](
                gpu_points, gpu_assignments,
                self.cf_n, self.cf_ls, self.cf_ss
            )
            
            # Compute centroids and radii
            self.compute_centroids_kernel[blocks_per_grid, threads_per_block](
                self.cf_n, self.cf_ls, self.node_centers
            )
            
            self.compute_radius_kernel[blocks_per_grid, threads_per_block](
                self.cf_n, self.cf_ls, self.cf_ss, self.node_radii
            )
            
            cuda.synchronize()
            
            return {
                'n': self.cf_n[:n_clusters].get(),
                'centers': self.node_centers[:n_clusters].get(),
                'radii': self.node_radii[:n_clusters].get()
            }
            
        except Exception as e:
            if self.node:
                self.node.get_logger().warn(f"GPU CF update failed: {e}")
            return self._update_cf_statistics_cpu(points, assignments)
    
    def merge_subclusters_gpu(self, threshold: float) -> np.ndarray:
        """
        Merge nearby subclusters on GPU
        
        Args:
            threshold: Distance threshold for merging
            
        Returns:
            New cluster assignments
        """
        if not self.gpu_available:
            return None
        
        try:
            # Compute pairwise distances between centers
            centers = self.node_centers[:self.n_nodes]
            from cupyx.scipy.spatial import distance
            distances = distance.cdist(centers, centers, 'euclidean')
            
            # Find pairs to merge
            cp.fill_diagonal(distances, cp.inf)
            merge_pairs = cp.where(distances < threshold)
            
            # Merge CF statistics
            merged = cp.zeros(self.n_nodes, dtype=cp.bool_)
            new_assignments = cp.arange(self.n_nodes, dtype=cp.int32)
            
            for i, j in zip(merge_pairs[0].get(), merge_pairs[1].get()):
                if i < j and not merged[i] and not merged[j]:
                    # Merge j into i
                    self.cf_n[i] += self.cf_n[j]
                    self.cf_ls[i] += self.cf_ls[j]
                    self.cf_ss[i] += self.cf_ss[j]
                    
                    # Recompute centroid and radius
                    if self.cf_n[i] > 0:
                        self.node_centers[i] = self.cf_ls[i] / self.cf_n[i]
                        
                        ss_n = self.cf_ss[i] / self.cf_n[i]
                        ls_n_sq = cp.sum((self.cf_ls[i] / self.cf_n[i])**2)
                        variance = ss_n - ls_n_sq
                        self.node_radii[i] = cp.sqrt(cp.maximum(variance, 0))
                    
                    merged[j] = True
                    new_assignments[j] = i
            
            return new_assignments.get()
            
        except Exception as e:
            if self.node:
                self.node.get_logger().warn(f"GPU merge failed: {e}")
            return None
    
    def insert_batch_gpu(self, points: np.ndarray) -> np.ndarray:
        """
        Insert batch of points into CF-Tree on GPU
        
        Args:
            points: Nx3 array of new points
            
        Returns:
            Cluster assignments for points
        """
        if not self.gpu_available:
            return np.zeros(len(points), dtype=np.int32)
        
        try:
            gpu_points = cp.asarray(points, dtype=cp.float32)
            
            # Find nearest CF node for each point
            from cupyx.scipy.spatial import distance
            
            if self.n_nodes > 0:
                centers = self.node_centers[:self.n_nodes]
                distances = distance.cdist(gpu_points, centers, 'euclidean')
                assignments = cp.argmin(distances, axis=1)
                
                # Check threshold constraint
                min_distances = cp.min(distances, axis=1)
                
                # Create new nodes for points exceeding threshold
                new_node_mask = min_distances > self.threshold
                n_new = int(cp.sum(new_node_mask).get())
                
                if n_new > 0 and self.n_nodes + n_new < self.max_nodes:
                    # Add new CF nodes
                    new_points = gpu_points[new_node_mask]
                    new_indices = cp.arange(self.n_nodes, self.n_nodes + n_new)
                    
                    self.node_centers[self.n_nodes:self.n_nodes + n_new] = new_points
                    self.cf_n[self.n_nodes:self.n_nodes + n_new] = 1
                    self.cf_ls[self.n_nodes:self.n_nodes + n_new] = new_points
                    
                    # Update assignments
                    assignments[new_node_mask] = new_indices[:n_new]
                    self.n_nodes += n_new
            else:
                # First point creates first node
                self.node_centers[0] = gpu_points[0]
                self.cf_n[0] = len(points)
                self.cf_ls[0] = cp.sum(gpu_points, axis=0)
                self.cf_ss[0] = cp.sum(gpu_points**2)
                self.n_nodes = 1
                assignments = cp.zeros(len(points), dtype=cp.int32)
            
            return assignments.get()
            
        except Exception as e:
            if self.node:
                self.node.get_logger().warn(f"GPU batch insert failed: {e}")
            return np.zeros(len(points), dtype=np.int32)
    
    def _update_cf_statistics_cpu(self, points: np.ndarray, 
                                 assignments: np.ndarray) -> Dict:
        """CPU fallback for CF statistics update"""
        
        n_clusters = np.max(assignments) + 1 if len(assignments) > 0 else 0
        
        cf_stats = {
            'n': np.zeros(n_clusters, dtype=np.int32),
            'centers': np.zeros((n_clusters, 3), dtype=np.float32),
            'radii': np.zeros(n_clusters, dtype=np.float32)
        }
        
        for cluster_id in range(n_clusters):
            mask = assignments == cluster_id
            cluster_points = points[mask]
            
            if len(cluster_points) > 0:
                cf_stats['n'][cluster_id] = len(cluster_points)
                cf_stats['centers'][cluster_id] = np.mean(cluster_points, axis=0)
                
                if len(cluster_points) > 1:
                    distances = np.linalg.norm(
                        cluster_points - cf_stats['centers'][cluster_id], 
                        axis=1
                    )
                    cf_stats['radii'][cluster_id] = np.std(distances)
        
        return cf_stats
    
    def get_cf_tree_stats(self) -> Dict:
        """Get current CF-Tree statistics"""
        
        stats = {
            'n_nodes': self.n_nodes,
            'branching_factor': self.branching_factor,
            'threshold': self.threshold,
            'gpu_available': self.gpu_available
        }
        
        if self.gpu_available and self.n_nodes > 0:
            stats['avg_cluster_size'] = float(cp.mean(self.cf_n[:self.n_nodes]).get())
            stats['avg_radius'] = float(cp.mean(self.node_radii[:self.n_nodes]).get())
        
        return stats
    
    def reset(self):
        """Reset CF-Tree"""
        
        if self.gpu_available:
            self.cf_n.fill(0)
            self.cf_ls.fill(0)
            self.cf_ss.fill(0)
            self.node_centers.fill(0)
            self.node_radii.fill(0)
            self.n_nodes = 0
        
        if self.node:
            self.node.get_logger().info("CF-Tree GPU reset")
