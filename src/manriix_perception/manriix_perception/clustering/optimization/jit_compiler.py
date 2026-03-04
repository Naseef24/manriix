#!/usr/bin/env python3
"""
JIT compilation for performance-critical functions
Uses Numba for CPU JIT and CuPy for GPU JIT
"""

import numpy as np
from rclpy.node import Node
from typing import Tuple, Optional, Dict
import time

try:
    from numba import jit, njit, prange, cuda
    from numba.typed import List
    NUMBA_AVAILABLE = True
except ImportError:
    NUMBA_AVAILABLE = False
    # Create dummy decorators when Numba not available
    def njit(*args, **kwargs):
        def decorator(func):
            return func
        if len(args) == 1 and callable(args[0]):
            return args[0]
        return decorator
    
    def jit(*args, **kwargs):
        def decorator(func):
            return func
        if len(args) == 1 and callable(args[0]):
            return args[0]
        return decorator
    
    def prange(*args):
        return range(*args)
    
    # Create dummy cuda module
    class cuda:
        @staticmethod
        def jit(*args, **kwargs):
            def decorator(func):
                return func
            if len(args) == 1 and callable(args[0]):
                return args[0]
            return decorator
        
        @staticmethod
        def is_available():
            return False
        
        @staticmethod
        def grid(n):
            return 0

try:
    import cupy as cp
    CUPY_AVAILABLE = True
except ImportError:
    CUPY_AVAILABLE = False


class JITCompiler:
    """
    JIT compilation manager for critical functions
    Provides optimized versions of compute-intensive operations
    """
    
    def __init__(self, use_gpu: bool = True, node: Node = None):
        """Initialize JIT compiler"""
        self.node = node
        
        # Check cuda availability safely
        cuda_available = False
        if NUMBA_AVAILABLE:
            try:
                from numba import cuda
                cuda_available = cuda.is_available()
            except Exception:
                cuda_available = False
        
        self.use_gpu = use_gpu and CUPY_AVAILABLE and NUMBA_AVAILABLE and cuda_available
        self.use_numba = NUMBA_AVAILABLE
        
        if not NUMBA_AVAILABLE and self.node:
            self.node.get_logger().warn("Numba not available - JIT compilation disabled")
        
        if self.use_numba:
            # Compile critical functions on initialization
            self._compile_functions()
            if self.node:
                self.node.get_logger().info(f"JIT Compiler initialized (GPU: {self.use_gpu}, Numba: {self.use_numba})")
        else:
            if self.node:
                self.node.get_logger().warn("JIT compilation not available - using pure Python")
    
    def _compile_functions(self):
        """Pre-compile all JIT functions"""
        # Trigger compilation with dummy data
        dummy_points = np.random.randn(10, 3).astype(np.float32)
        dummy_center = np.array([0, 0, 0], dtype=np.float32)
        
        # Compile distance functions
        _ = self.compute_distances_jit(dummy_points, dummy_center)
        _ = self.pairwise_distances_jit(dummy_points, dummy_points)
        
        # Compile clustering functions
        _ = self.find_nearest_cluster_jit(dummy_points[0], dummy_points)
        
        if self.use_gpu:
            # Compile GPU kernels
            self._compile_gpu_kernels()
    
    def _compile_gpu_kernels(self):
        """Compile CUDA kernels"""
        if not cuda.is_available():
            return
        
        # Test data for compilation
        n = 100
        test_data = np.random.randn(n, 3).astype(np.float32)
        
        try:
            # Transfer to GPU and trigger compilation
            d_data = cuda.to_device(test_data)
            d_out = cuda.device_array((n, n), dtype=np.float32)
            
            # Compile kernel by running once
            threadsperblock = 32
            blockspergrid = (n + threadsperblock - 1) // threadsperblock
            self.distance_kernel_cuda[blockspergrid, threadsperblock](d_data, d_out, n)
            cuda.synchronize()
            
        except Exception as e:
            if self.node:
                self.node.get_logger().warn(f"GPU kernel compilation failed: {e}")
            self.use_gpu = False
    
    @staticmethod
    @njit(fastmath=True, parallel=True, cache=True)
    def compute_distances_jit(points: np.ndarray, center: np.ndarray) -> np.ndarray:
        """
        JIT-compiled distance computation
        
        Args:
            points: Nx3 array of points
            center: 3D center point
            
        Returns:
            N array of distances
        """
        n = len(points)
        distances = np.zeros(n, dtype=np.float32)
        
        for i in prange(n):
            dx = points[i, 0] - center[0]
            dy = points[i, 1] - center[1]
            dz = points[i, 2] - center[2]
            distances[i] = np.sqrt(dx*dx + dy*dy + dz*dz)
        
        return distances
    
    @staticmethod
    @njit(fastmath=True, parallel=True, cache=True)
    def pairwise_distances_jit(points1: np.ndarray, points2: np.ndarray) -> np.ndarray:
        """
        JIT-compiled pairwise distance matrix
        
        Args:
            points1: Nx3 array
            points2: Mx3 array
            
        Returns:
            NxM distance matrix
        """
        n = len(points1)
        m = len(points2)
        distances = np.zeros((n, m), dtype=np.float32)
        
        for i in prange(n):
            for j in range(m):
                dx = points1[i, 0] - points2[j, 0]
                dy = points1[i, 1] - points2[j, 1]
                dz = points1[i, 2] - points2[j, 2]
                distances[i, j] = np.sqrt(dx*dx + dy*dy + dz*dz)
        
        return distances
    
    @staticmethod
    @njit(fastmath=True, cache=True)
    def find_nearest_cluster_jit(point: np.ndarray, centers: np.ndarray) -> Tuple[int, float]:
        """
        Find nearest cluster center (JIT-compiled)
        
        Args:
            point: 3D point
            centers: Kx3 array of centers
            
        Returns:
            (nearest_idx, min_distance)
        """
        min_dist = np.inf
        min_idx = -1
        
        for i in range(len(centers)):
            dx = point[0] - centers[i, 0]
            dy = point[1] - centers[i, 1]
            dz = point[2] - centers[i, 2]
            dist = np.sqrt(dx*dx + dy*dy + dz*dz)
            
            if dist < min_dist:
                min_dist = dist
                min_idx = i
        
        return min_idx, min_dist
    
    @staticmethod
    @njit(fastmath=True, parallel=True, cache=True)
    def update_cluster_stats_jit(points: np.ndarray, labels: np.ndarray, 
                                 n_clusters: int) -> Tuple[np.ndarray, np.ndarray]:
        """
        Update cluster statistics (JIT-compiled)
        
        Args:
            points: Nx3 array of points
            labels: N array of cluster labels
            n_clusters: Number of clusters
            
        Returns:
            (centers, sizes) for each cluster
        """
        centers = np.zeros((n_clusters, 3), dtype=np.float32)
        sizes = np.zeros(n_clusters, dtype=np.int32)
        
        # Accumulate sums
        for i in range(len(points)):
            label = labels[i]
            if 0 <= label < n_clusters:
                centers[label] += points[i]
                sizes[label] += 1
        
        # Compute means
        for k in range(n_clusters):
            if sizes[k] > 0:
                centers[k] /= sizes[k]
        
        return centers, sizes
    
    @staticmethod
    @cuda.jit
    def distance_kernel_cuda(points, distances, n):
        """
        CUDA kernel for distance computation
        
        Args:
            points: Device array of points
            distances: Output distance matrix
            n: Number of points
        """
        i = cuda.grid(1)
        
        if i < n:
            for j in range(n):
                dx = points[i, 0] - points[j, 0]
                dy = points[i, 1] - points[j, 1]
                dz = points[i, 2] - points[j, 2]
                distances[i, j] = cuda.libdevice.sqrtf(dx*dx + dy*dy + dz*dz)
    
    @staticmethod
    @njit(fastmath=True, parallel=True, cache=True)
    def birch_threshold_check_jit(points: np.ndarray, center: np.ndarray, 
                                  threshold: float) -> np.ndarray:
        """
        Check which points are within BIRCH threshold (JIT-compiled)
        
        Args:
            points: Nx3 array of points
            center: Cluster center
            threshold: Distance threshold
            
        Returns:
            Boolean mask of points within threshold
        """
        n = len(points)
        mask = np.zeros(n, dtype=np.bool_)
        
        for i in prange(n):
            dx = points[i, 0] - center[0]
            dy = points[i, 1] - center[1]
            dz = points[i, 2] - center[2]
            dist = np.sqrt(dx*dx + dy*dy + dz*dz)
            mask[i] = dist <= threshold
        
        return mask
    
    @staticmethod
    @njit(fastmath=True, cache=True)
    def compute_cf_stats_jit(points: np.ndarray) -> Tuple[int, np.ndarray, np.ndarray]:
        """
        Compute Clustering Feature statistics (JIT-compiled)
        
        Args:
            points: Nx3 array of points
            
        Returns:
            (n, linear_sum, squared_sum)
        """
        n = len(points)
        linear_sum = np.zeros(3, dtype=np.float64)
        squared_sum = np.zeros(3, dtype=np.float64)
        
        for i in range(n):
            linear_sum[0] += points[i, 0]
            linear_sum[1] += points[i, 1]
            linear_sum[2] += points[i, 2]
            
            squared_sum[0] += points[i, 0] * points[i, 0]
            squared_sum[1] += points[i, 1] * points[i, 1]
            squared_sum[2] += points[i, 2] * points[i, 2]
        
        return n, linear_sum, squared_sum
    
    @staticmethod
    @njit(fastmath=True, parallel=True, cache=True)
    def kalman_predict_batch_jit(states: np.ndarray, F: np.ndarray, 
                                 dt: float) -> np.ndarray:
        """
        Batch Kalman prediction (JIT-compiled)
        
        Args:
            states: Nx6 array of states [x, y, z, vx, vy, vz]
            F: State transition matrix
            dt: Time step
            
        Returns:
            Predicted states
        """
        n = len(states)
        predicted = np.zeros_like(states)
        
        # Update F matrix for dt
        F_dt = F.copy()
        F_dt[0, 3] = dt
        F_dt[1, 4] = dt
        F_dt[2, 5] = dt
        
        for i in prange(n):
            # Matrix-vector multiplication
            for j in range(6):
                sum_val = 0.0
                for k in range(6):
                    sum_val += F_dt[j, k] * states[i, k]
                predicted[i, j] = sum_val
        
        return predicted
    
    @staticmethod
    @njit(fastmath=True, cache=True)
    def hungarian_cost_matrix_jit(cluster_centers: np.ndarray, 
                                  track_centers: np.ndarray,
                                  max_distance: float) -> np.ndarray:
        """
        Compute cost matrix for Hungarian algorithm (JIT-compiled)
        
        Args:
            cluster_centers: Nx3 array of cluster centers
            track_centers: Mx3 array of track predictions
            max_distance: Maximum assignment distance
            
        Returns:
            NxM cost matrix
        """
        n = len(cluster_centers)
        m = len(track_centers)
        cost_matrix = np.full((n, m), max_distance, dtype=np.float32)
        
        for i in range(n):
            for j in range(m):
                dx = cluster_centers[i, 0] - track_centers[j, 0]
                dy = cluster_centers[i, 1] - track_centers[j, 1]
                dz = cluster_centers[i, 2] - track_centers[j, 2]
                dist = np.sqrt(dx*dx + dy*dy + dz*dz)
                
                if dist < max_distance:
                    cost_matrix[i, j] = dist
        
        return cost_matrix


class OptimizedOperations:
    """
    Wrapper for optimized operations with automatic fallback
    """
    
    def __init__(self, use_jit: bool = True, use_gpu: bool = False, node: Node = None):
        """Initialize optimized operations"""
        self.use_jit = use_jit and NUMBA_AVAILABLE
        self.use_gpu = use_gpu and CUPY_AVAILABLE
        self.node = node
        
        if self.use_jit:
            self.jit_compiler = JITCompiler(use_gpu=use_gpu, node=node)
        else:
            self.jit_compiler = None
        
        # Performance tracking
        self.operation_times = {}
        self.operation_counts = {}
    
    def compute_distances(self, points: np.ndarray, center: np.ndarray) -> np.ndarray:
        """
        Compute distances with best available method
        
        Args:
            points: Nx3 array of points
            center: 3D center point
            
        Returns:
            N array of distances
        """
        start_time = time.time()
        
        if self.use_jit and self.jit_compiler:
            result = self.jit_compiler.compute_distances_jit(
                points.astype(np.float32), 
                center.astype(np.float32)
            )
        else:
            # Fallback to NumPy
            result = np.linalg.norm(points - center, axis=1)
        
        self._track_performance('compute_distances', time.time() - start_time)
        return result
    
    def pairwise_distances(self, points1: np.ndarray, points2: np.ndarray) -> np.ndarray:
        """
        Compute pairwise distances with best available method
        
        Args:
            points1: Nx3 array
            points2: Mx3 array
            
        Returns:
            NxM distance matrix
        """
        start_time = time.time()
        
        if self.use_gpu and CUPY_AVAILABLE:
            # GPU implementation
            points1_gpu = cp.asarray(points1)
            points2_gpu = cp.asarray(points2)
            
            # Broadcasting on GPU
            diff = points1_gpu[:, np.newaxis, :] - points2_gpu[np.newaxis, :, :]
            result = cp.sqrt(cp.sum(diff ** 2, axis=2)).get()
            
        elif self.use_jit and self.jit_compiler:
            # JIT-compiled CPU
            result = self.jit_compiler.pairwise_distances_jit(
                points1.astype(np.float32),
                points2.astype(np.float32)
            )
        else:
            # Fallback to SciPy
            from scipy.spatial.distance import cdist
            result = cdist(points1, points2)
        
        self._track_performance('pairwise_distances', time.time() - start_time)
        return result
    
    def find_nearest_cluster(self, point: np.ndarray, centers: np.ndarray) -> Tuple[int, float]:
        """
        Find nearest cluster with best available method
        
        Args:
            point: 3D point
            centers: Kx3 array of centers
            
        Returns:
            (nearest_idx, min_distance)
        """
        start_time = time.time()
        
        if self.use_jit and self.jit_compiler:
            result = self.jit_compiler.find_nearest_cluster_jit(
                point.astype(np.float32),
                centers.astype(np.float32)
            )
        else:
            # Fallback to NumPy
            distances = np.linalg.norm(centers - point, axis=1)
            min_idx = np.argmin(distances)
            min_dist = distances[min_idx]
            result = (min_idx, min_dist)
        
        self._track_performance('find_nearest_cluster', time.time() - start_time)
        return result
    
    def update_cluster_stats(self, points: np.ndarray, labels: np.ndarray, 
                           n_clusters: int) -> Tuple[np.ndarray, np.ndarray]:
        """
        Update cluster statistics with best available method
        
        Args:
            points: Nx3 array of points
            labels: N array of cluster labels
            n_clusters: Number of clusters
            
        Returns:
            (centers, sizes) for each cluster
        """
        start_time = time.time()
        
        if self.use_jit and self.jit_compiler:
            result = self.jit_compiler.update_cluster_stats_jit(
                points.astype(np.float32),
                labels.astype(np.int32),
                n_clusters
            )
        else:
            # Fallback to NumPy
            centers = np.zeros((n_clusters, 3), dtype=np.float32)
            sizes = np.zeros(n_clusters, dtype=np.int32)
            
            for k in range(n_clusters):
                mask = labels == k
                if np.any(mask):
                    centers[k] = np.mean(points[mask], axis=0)
                    sizes[k] = np.sum(mask)
            
            result = (centers, sizes)
        
        self._track_performance('update_cluster_stats', time.time() - start_time)
        return result
    
    def _track_performance(self, operation: str, elapsed_time: float):
        """Track operation performance"""
        if operation not in self.operation_times:
            self.operation_times[operation] = []
            self.operation_counts[operation] = 0
        
        self.operation_times[operation].append(elapsed_time)
        self.operation_counts[operation] += 1
        
        # Keep only last 100 measurements
        if len(self.operation_times[operation]) > 100:
            self.operation_times[operation] = self.operation_times[operation][-100:]
    
    def get_performance_stats(self) -> dict:
        """Get performance statistics"""
        stats = {}
        
        for op in self.operation_times:
            times = self.operation_times[op]
            if times:
                stats[op] = {
                    'mean_ms': np.mean(times) * 1000,
                    'std_ms': np.std(times) * 1000,
                    'min_ms': np.min(times) * 1000,
                    'max_ms': np.max(times) * 1000,
                    'count': self.operation_counts[op]
                }
        
        return stats