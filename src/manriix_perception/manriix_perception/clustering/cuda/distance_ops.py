#!/usr/bin/env python3
"""
GPU-accelerated distance computations using CuPy
Optimized for Jetson AGX Orin's compute capability 8.7
"""

import numpy as np
from rclpy.node import Node
from typing import Optional, Tuple, List, Dict
import time

try:
    import cupy as cp
    from cupyx.scipy.spatial import distance as cp_distance
    CUPY_AVAILABLE = True
except ImportError:
    CUPY_AVAILABLE = False


class DistanceOpsGPU:
    """
    GPU-accelerated distance operations for clustering
    Uses CuPy for maximum performance on Jetson
    """
    
    def __init__(self, max_points: int = 200, use_unified_memory: bool = True, node: Node = None):
        """
        Initialize GPU distance operations
        
        Args:
            max_points: Maximum number of points to process
            use_unified_memory: Use unified memory for Jetson (zero-copy)
            node: ROS2 node for logging
        """
        self.max_points = max_points
        self.use_unified_memory = use_unified_memory
        self.gpu_available = CUPY_AVAILABLE
        self.node = node
        
        if self.gpu_available:
            self._init_gpu()
        else:
            if self.node:
                self.node.get_logger().warn("GPU distance operations not available - using CPU fallback")
    
    def _init_gpu(self):
        """Initialize GPU resources"""
        try:
            # Set memory pool
            if self.use_unified_memory:
                # Use managed memory for Jetson (unified memory)
                cp.cuda.set_allocator(cp.cuda.MemoryPool(cp.cuda.malloc_managed).malloc)
                if self.node:
                    self.node.get_logger().info("Using unified memory for GPU operations")
            else:
                # Use standard GPU memory pool
                mempool = cp.get_default_memory_pool()
                mempool.set_limit(size=4 * 1024**3)  # 4GB limit
            
            # Pre-allocate arrays for common operations
            self.gpu_points_buffer = cp.zeros((self.max_points, 3), dtype=cp.float32)
            self.gpu_distances_buffer = cp.zeros((self.max_points, self.max_points), dtype=cp.float32)
            
            # Create CUDA kernels for custom operations
            self._create_kernels()
            
            if self.node:
                self.node.get_logger().info(f"GPU distance operations initialized (max_points={self.max_points})")
            
        except Exception as e:
            if self.node:
                self.node.get_logger().error(f"Failed to initialize GPU: {e}")
            self.gpu_available = False
    
    def _create_kernels(self):
        """Create custom CUDA kernels using CuPy"""
        
        # Euclidean distance kernel
        self.euclidean_kernel = cp.RawKernel(r'''
        extern "C" __global__
        void euclidean_distance(const float* points1, const float* points2,
                                float* distances, int n1, int n2, int dim) {
            int i = blockIdx.x * blockDim.x + threadIdx.x;
            int j = blockIdx.y * blockDim.y + threadIdx.y;
            
            if (i < n1 && j < n2) {
                float sum = 0.0f;
                for (int d = 0; d < dim; d++) {
                    float diff = points1[i * dim + d] - points2[j * dim + d];
                    sum += diff * diff;
                }
                distances[i * n2 + j] = sqrtf(sum);
            }
        }
        ''', 'euclidean_distance')
        
        # Batch norm kernel for fast distance computation
        self.batch_norm_kernel = cp.RawKernel(r'''
        extern "C" __global__
        void batch_norm_distance(const float* points, const float* centers,
                                 float* distances, int n_points, int n_centers) {
            int tid = blockIdx.x * blockDim.x + threadIdx.x;
            
            if (tid < n_points * n_centers) {
                int point_idx = tid / n_centers;
                int center_idx = tid % n_centers;
                
                float dx = points[point_idx * 3] - centers[center_idx * 3];
                float dy = points[point_idx * 3 + 1] - centers[center_idx * 3 + 1];
                float dz = points[point_idx * 3 + 2] - centers[center_idx * 3 + 2];
                
                distances[tid] = sqrtf(dx*dx + dy*dy + dz*dz);
            }
        }
        ''', 'batch_norm_distance')
    
    def compute_pairwise_distances(self, points: np.ndarray) -> np.ndarray:
        """
        Compute pairwise distances between all points on GPU
        
        Args:
            points: Nx3 array of points (CPU)
            
        Returns:
            NxN distance matrix (CPU)
        """
        if not self.gpu_available:
            return self._compute_distances_cpu(points)
        
        try:
            n_points = len(points)
            
            # Transfer to GPU
            if n_points <= self.max_points:
                # Use pre-allocated buffer
                self.gpu_points_buffer[:n_points] = cp.asarray(points)
                gpu_points = self.gpu_points_buffer[:n_points]
            else:
                # Allocate new array
                gpu_points = cp.asarray(points, dtype=cp.float32)
            
            # Compute distances using CuPy's optimized function
            # This uses CUBLAS internally for best performance
            gpu_distances = cp_distance.cdist(gpu_points, gpu_points, 'euclidean')
            
            # Transfer back to CPU
            return gpu_distances.get()
            
        except Exception as e:
            if self.node:
                self.node.get_logger().warn(f"GPU distance computation failed: {e}, falling back to CPU")
            return self._compute_distances_cpu(points)
    
    def compute_distances_to_centers(self, points: np.ndarray, 
                                   centers: np.ndarray) -> np.ndarray:
        """
        Compute distances from points to cluster centers on GPU
        
        Args:
            points: Nx3 array of points
            centers: Mx3 array of centers
            
        Returns:
            NxM distance matrix
        """
        if not self.gpu_available:
            return self._compute_distances_to_centers_cpu(points, centers)
        
        try:
            # Transfer to GPU
            gpu_points = cp.asarray(points, dtype=cp.float32)
            gpu_centers = cp.asarray(centers, dtype=cp.float32)
            
            # Use custom kernel for better performance
            n_points = len(points)
            n_centers = len(centers)
            
            # Allocate output
            gpu_distances = cp.zeros((n_points, n_centers), dtype=cp.float32)
            
            # Launch kernel
            threads_per_block = 256
            blocks = (n_points * n_centers + threads_per_block - 1) // threads_per_block
            
            self.batch_norm_kernel(
                (blocks,), (threads_per_block,),
                (gpu_points, gpu_centers, gpu_distances, n_points, n_centers)
            )
            
            return gpu_distances.get()
            
        except Exception as e:
            if self.node:
                self.node.get_logger().warn(f"GPU center distance failed: {e}")
            return self._compute_distances_to_centers_cpu(points, centers)
    
    def find_nearest_neighbors(self, points: np.ndarray, k: int = 5) -> Tuple[np.ndarray, np.ndarray]:
        """
        Find k-nearest neighbors for each point using GPU
        
        Args:
            points: Nx3 array of points
            k: Number of neighbors
            
        Returns:
            distances: NxK array of distances
            indices: NxK array of neighbor indices
        """
        if not self.gpu_available:
            return self._find_nearest_neighbors_cpu(points, k)
        
        try:
            # Compute pairwise distances
            gpu_points = cp.asarray(points, dtype=cp.float32)
            distances = cp_distance.cdist(gpu_points, gpu_points, 'euclidean')
            
            # Set diagonal to infinity (exclude self)
            cp.fill_diagonal(distances, cp.inf)
            
            # Find k nearest for each point
            k = min(k, len(points) - 1)
            
            # Use argpartition for efficiency
            indices = cp.argpartition(distances, k, axis=1)[:, :k]
            
            # Get corresponding distances
            row_indices = cp.arange(len(points))[:, cp.newaxis]
            nearest_distances = distances[row_indices, indices]
            
            # Sort by distance
            sort_indices = cp.argsort(nearest_distances, axis=1)
            indices = indices[row_indices, sort_indices]
            nearest_distances = nearest_distances[row_indices, sort_indices]
            
            return nearest_distances.get(), indices.get()
            
        except Exception as e:
            if self.node:
                self.node.get_logger().warn(f"GPU KNN failed: {e}")
            return self._find_nearest_neighbors_cpu(points, k)
    
    def compute_radius_neighbors(self, points: np.ndarray, radius: float) -> List[List[int]]:
        """
        Find all neighbors within radius for each point
        
        Args:
            points: Nx3 array of points
            radius: Search radius
            
        Returns:
            List of neighbor indices for each point
        """
        if not self.gpu_available:
            return self._compute_radius_neighbors_cpu(points, radius)
        
        try:
            # Compute pairwise distances
            gpu_points = cp.asarray(points, dtype=cp.float32)
            distances = cp_distance.cdist(gpu_points, gpu_points, 'euclidean')
            
            # Find neighbors within radius
            neighbors = []
            for i in range(len(points)):
                mask = distances[i] < radius
                mask[i] = False  # Exclude self
                neighbor_indices = cp.where(mask)[0].get()
                neighbors.append(neighbor_indices.tolist())
            
            return neighbors
            
        except Exception as e:
            if self.node:
                self.node.get_logger().warn(f"GPU radius neighbors failed: {e}")
            return self._compute_radius_neighbors_cpu(points, radius)
    
    def batch_distance_computation(self, point_batches: List[np.ndarray]) -> List[np.ndarray]:
        """
        Compute distances for multiple batches in parallel using streams
        
        Args:
            point_batches: List of point arrays
            
        Returns:
            List of distance matrices
        """
        if not self.gpu_available:
            return [self._compute_distances_cpu(batch) for batch in point_batches]
        
        try:
            # Create streams for parallel processing
            streams = [cp.cuda.Stream() for _ in range(min(len(point_batches), 3))]
            results = []
            
            for i, batch in enumerate(point_batches):
                stream = streams[i % len(streams)]
                
                with stream:
                    gpu_batch = cp.asarray(batch, dtype=cp.float32)
                    gpu_distances = cp_distance.cdist(gpu_batch, gpu_batch, 'euclidean')
                    results.append(gpu_distances)
            
            # Synchronize streams and get results
            for stream in streams:
                stream.synchronize()
            
            return [r.get() for r in results]
            
        except Exception as e:
            if self.node:
                self.node.get_logger().warn(f"Batch GPU computation failed: {e}")
            return [self._compute_distances_cpu(batch) for batch in point_batches]
    
    # CPU fallback methods
    def _compute_distances_cpu(self, points: np.ndarray) -> np.ndarray:
        """CPU fallback for pairwise distances"""
        from scipy.spatial import distance
        return distance.cdist(points, points, 'euclidean')
    
    def _compute_distances_to_centers_cpu(self, points: np.ndarray, 
                                         centers: np.ndarray) -> np.ndarray:
        """CPU fallback for point-to-center distances"""
        from scipy.spatial import distance
        return distance.cdist(points, centers, 'euclidean')
    
    def _find_nearest_neighbors_cpu(self, points: np.ndarray, 
                                   k: int) -> Tuple[np.ndarray, np.ndarray]:
        """CPU fallback for KNN"""
        from scipy.spatial import distance
        distances = distance.cdist(points, points, 'euclidean')
        np.fill_diagonal(distances, np.inf)
        
        k = min(k, len(points) - 1)
        indices = np.argpartition(distances, k, axis=1)[:, :k]
        
        nearest_distances = distances[np.arange(len(points))[:, np.newaxis], indices]
        sort_indices = np.argsort(nearest_distances, axis=1)
        
        return nearest_distances[:, sort_indices], indices[:, sort_indices]
    
    def _compute_radius_neighbors_cpu(self, points: np.ndarray, 
                                     radius: float) -> List[List[int]]:
        """CPU fallback for radius neighbors"""
        from scipy.spatial import distance
        distances = distance.cdist(points, points, 'euclidean')
        
        neighbors = []
        for i in range(len(points)):
            mask = distances[i] < radius
            mask[i] = False
            neighbors.append(np.where(mask)[0].tolist())
        
        return neighbors
    
    def get_memory_usage(self) -> Dict:
        """Get GPU memory usage"""
        if not self.gpu_available:
            return {'gpu_mb': 0, 'gpu_percent': 0}
        
        try:
            mempool = cp.get_default_memory_pool()
            used = mempool.used_bytes() / (1024**2)
            total = mempool.total_bytes() / (1024**2)
            
            return {
                'gpu_mb': used,
                'gpu_percent': (used / total * 100) if total > 0 else 0
            }
        except:
            return {'gpu_mb': 0, 'gpu_percent': 0}
    
    def cleanup(self):
        """Clean up GPU resources"""
        if self.gpu_available:
            try:
                mempool = cp.get_default_memory_pool()
                mempool.free_all_blocks()
                if self.node:
                    self.node.get_logger().info("GPU memory freed")
            except:
                pass
