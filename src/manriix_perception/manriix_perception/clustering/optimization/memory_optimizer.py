#!/usr/bin/env python3
"""
Memory access optimization for Jetson AGX Orin
Optimizes cache usage and memory bandwidth
"""

import numpy as np
from rclpy.node import Node
from typing import Dict, List, Optional, Tuple
import psutil
import gc
from collections import deque

try:
    import cupy as cp
    CUPY_AVAILABLE = True
except ImportError:
    CUPY_AVAILABLE = False


class MemoryOptimizer:
    """
    Memory optimization for real-time processing
    Focuses on cache efficiency and memory bandwidth
    """
    
    def __init__(self, max_memory_mb: int = 4096, node: Node = None):
        """
        Initialize memory optimizer
        
        Args:
            max_memory_mb: Maximum memory usage in MB
            node: ROS2 node for logging
        """
        self.max_memory_mb = max_memory_mb
        self.max_memory_bytes = max_memory_mb * 1024 * 1024
        self.node = node
        
        # Memory pools
        self.array_pools = {}
        self.buffer_pools = {}
        
        # Cache optimization
        self.cache_line_size = 64  # bytes
        self.l2_cache_size = 512 * 1024  # 512KB on Jetson
        
        # Memory tracking
        self.memory_usage = deque(maxlen=100)
        self.allocation_counts = {}
        
        # GPU memory pool
        if CUPY_AVAILABLE:
            self._init_gpu_memory_pool()
        
        if self.node:
            self.node.get_logger().info(f"Memory Optimizer initialized (max: {max_memory_mb} MB)")
    
    def _init_gpu_memory_pool(self):
        """Initialize GPU memory pool for efficient allocation"""
        try:
            # Set memory pool
            mempool = cp.get_default_memory_pool()
            pinned_mempool = cp.get_default_pinned_memory_pool()
            
            # Limit memory usage
            mempool.set_limit(size=self.max_memory_bytes)
            
            # Pre-allocate common sizes
            common_sizes = [
                (100, 3),    # 100 3D points
                (200, 3),    # 200 3D points
                (50, 50),    # Distance matrix
                (100, 100),  # Larger distance matrix
                (6, 6),      # Kalman state covariance
            ]
            
            for shape in common_sizes:
                # Pre-allocate and free to warm up pool
                arr = cp.zeros(shape, dtype=cp.float32)
                del arr
            
            # Force collection
            mempool.free_all_blocks()
            
            if self.node:
                self.node.get_logger().info("GPU memory pool initialized")
            
        except Exception as e:
            if self.node:
                self.node.get_logger().warn(f"Failed to initialize GPU memory pool: {e}")
    
    def get_optimized_array(self, shape: Tuple, dtype: type = np.float32, 
                           use_gpu: bool = False) -> np.ndarray:
        """
        Get memory-optimized array from pool
        
        Args:
            shape: Array shape
            dtype: Data type
            use_gpu: Use GPU memory
            
        Returns:
            Optimized array
        """
        key = (shape, dtype, use_gpu)
        
        # Check if we have a pooled array
        if key in self.array_pools and self.array_pools[key]:
            array = self.array_pools[key].pop()
            array.fill(0)  # Clear contents
            return array
        
        # Allocate new array with optimal alignment
        if use_gpu and CUPY_AVAILABLE:
            array = cp.zeros(shape, dtype=dtype)
        else:
            # Align to cache line for CPU
            array = np.zeros(shape, dtype=dtype)
            
            # Ensure alignment for SIMD operations
            if array.nbytes >= self.cache_line_size:
                aligned_array = self._align_array(array)
                if aligned_array is not None:
                    array = aligned_array
        
        # Track allocation
        self._track_allocation(key, array.nbytes)
        
        return array
    
    def return_array(self, array: np.ndarray, shape: Tuple = None, 
                     dtype: type = None, use_gpu: bool = False):
        """
        Return array to pool for reuse
        
        Args:
            array: Array to return
            shape: Original shape (if None, uses array.shape)
            dtype: Original dtype (if None, uses array.dtype)
            use_gpu: Whether this is a GPU array
        """
        if shape is None:
            shape = array.shape
        if dtype is None:
            dtype = array.dtype
        
        key = (shape, dtype, use_gpu)
        
        if key not in self.array_pools:
            self.array_pools[key] = []
        
        # Keep pool size reasonable
        if len(self.array_pools[key]) < 5:
            self.array_pools[key].append(array)
    
    def _align_array(self, array: np.ndarray) -> Optional[np.ndarray]:
        """
        Align array to cache line boundary
        
        Args:
            array: Input array
            
        Returns:
            Aligned array or None if alignment not needed
        """
        try:
            # Check current alignment
            addr = array.__array_interface__['data'][0]
            
            if addr % self.cache_line_size == 0:
                return None  # Already aligned
            
            # Create aligned array
            dtype = array.dtype
            shape = array.shape
            itemsize = dtype.itemsize
            nbytes = array.nbytes
            
            # Allocate extra space for alignment
            buf = np.empty(nbytes + self.cache_line_size, dtype=np.uint8)
            addr = buf.__array_interface__['data'][0]
            
            # Calculate offset for alignment
            offset = (self.cache_line_size - addr % self.cache_line_size) % self.cache_line_size
            
            # Create view with correct alignment
            aligned = np.frombuffer(buf[offset:offset + nbytes], dtype=dtype).reshape(shape)
            np.copyto(aligned, array)
            
            return aligned
            
        except Exception:
            return None
    
    def optimize_memory_layout(self, points: np.ndarray) -> np.ndarray:
        """
        Optimize memory layout for better cache performance
        
        Args:
            points: Nx3 array of points
            
        Returns:
            Optimized array
        """
        # Ensure C-contiguous for better cache locality
        if not points.flags['C_CONTIGUOUS']:
            points = np.ascontiguousarray(points)
        
        # Use float32 instead of float64 to reduce memory bandwidth
        if points.dtype != np.float32:
            points = points.astype(np.float32)
        
        # Pad to multiple of cache line if beneficial
        n_points = len(points)
        if n_points > 100:  # Only for larger arrays
            # Calculate optimal padding
            bytes_per_point = points.itemsize * 3
            points_per_cacheline = self.cache_line_size // bytes_per_point
            
            if points_per_cacheline > 0:
                padded_size = ((n_points + points_per_cacheline - 1) // 
                             points_per_cacheline) * points_per_cacheline
                
                if padded_size > n_points:
                    # Create padded array
                    padded = self.get_optimized_array((padded_size, 3), np.float32)
                    padded[:n_points] = points
                    return padded[:n_points]  # Return view of actual data
        
        return points
    
    def optimize_batch_processing(self, data: List[np.ndarray], 
                                batch_size: int = 32) -> List[np.ndarray]:
        """
        Optimize batch processing for memory efficiency
        
        Args:
            data: List of arrays to process
            batch_size: Optimal batch size
            
        Returns:
            Reorganized data for efficient processing
        """
        if not data:
            return data
        
        # Calculate optimal batch size based on L2 cache
        element_size = data[0].nbytes if data else 0
        if element_size > 0:
            optimal_batch = min(
                batch_size,
                self.l2_cache_size // (element_size * 2)  # Leave room for output
            )
        else:
            optimal_batch = batch_size
        
        # Reorganize into cache-friendly batches
        batches = []
        for i in range(0, len(data), optimal_batch):
            batch = data[i:i + optimal_batch]
            
            # Stack for better memory locality
            if len(batch) > 1:
                stacked = np.stack(batch)
                batches.append(stacked)
            else:
                batches.append(batch[0])
        
        return batches
    
    def prefetch_data(self, arrays: List[np.ndarray]):
        """
        Prefetch data into cache
        
        Args:
            arrays: Arrays to prefetch
        """
        for array in arrays:
            if isinstance(array, np.ndarray):
                # Touch data to bring into cache
                _ = array.sum()
            elif CUPY_AVAILABLE and isinstance(array, cp.ndarray):
                # GPU prefetch
                cp.cuda.Stream.null.synchronize()
    
    def _track_allocation(self, key: Tuple, nbytes: int):
        """Track memory allocation"""
        if key not in self.allocation_counts:
            self.allocation_counts[key] = {'count': 0, 'total_bytes': 0}
        
        self.allocation_counts[key]['count'] += 1
        self.allocation_counts[key]['total_bytes'] += nbytes
    
    def optimize_gpu_transfer(self, data: np.ndarray, 
                            use_pinned: bool = True):
        """
        Optimize CPU to GPU transfer
        
        Args:
            data: CPU array
            use_pinned: Use pinned memory for faster transfer
            
        Returns:
            GPU array
        """
        if not CUPY_AVAILABLE:
            return data
        
        try:
            if use_pinned:
                # Use pinned memory for faster transfer
                pinned_mem = cp.cuda.alloc_pinned_memory(data.nbytes)
                pinned_array = np.frombuffer(pinned_mem, data.dtype, data.size).reshape(data.shape)
                pinned_array[...] = data
                gpu_array = cp.asarray(pinned_array)
            else:
                gpu_array = cp.asarray(data)
            
            return gpu_array
            
        except Exception as e:
            if self.node:
                self.node.get_logger().debug(f"GPU transfer optimization failed: {e}")
            return cp.asarray(data)
    
    def clear_pools(self):
        """Clear all memory pools"""
        self.array_pools.clear()
        self.buffer_pools.clear()
        
        if CUPY_AVAILABLE:
            mempool = cp.get_default_memory_pool()
            pinned_mempool = cp.get_default_pinned_memory_pool()
            mempool.free_all_blocks()
            pinned_mempool.free_all_blocks()
        
        # Force garbage collection
        gc.collect()
    
    def get_memory_stats(self) -> Dict:
        """Get memory usage statistics"""
        process = psutil.Process()
        memory_info = process.memory_info()
        
        stats = {
            'rss_mb': memory_info.rss / (1024 * 1024),
            'vms_mb': memory_info.vms / (1024 * 1024),
            'percent': process.memory_percent(),
            'available_mb': psutil.virtual_memory().available / (1024 * 1024),
            'pool_sizes': {str(k): len(v) for k, v in self.array_pools.items()},
            'allocation_counts': self.allocation_counts
        }
        
        if CUPY_AVAILABLE:
            mempool = cp.get_default_memory_pool()
            stats['gpu_used_mb'] = mempool.used_bytes() / (1024 * 1024)
            stats['gpu_total_mb'] = mempool.total_bytes() / (1024 * 1024)
        
        return stats


class DataLocalityOptimizer:
    """
    Optimize data locality for better cache usage
    """
    
    def __init__(self, node: Node = None):
        """Initialize data locality optimizer"""
        self.node = node
        self.access_patterns = {}
        self.reorder_indices = {}
    
    def optimize_point_order(self, points: np.ndarray, 
                           method: str = 'morton') -> Tuple[np.ndarray, np.ndarray]:
        """
        Reorder points for better spatial locality
        
        Args:
            points: Nx3 array of points
            method: Ordering method ('morton', 'hilbert', 'kdtree')
            
        Returns:
            (reordered_points, indices)
        """
        n = len(points)
        
        if method == 'morton':
            # Morton order (Z-order) for spatial locality
            indices = self._morton_order(points)
        elif method == 'kdtree':
            # KD-tree based ordering
            indices = self._kdtree_order(points)
        else:
            # Default to original order
            indices = np.arange(n)
        
        reordered = points[indices]
        return reordered, indices
    
    def _morton_order(self, points: np.ndarray) -> np.ndarray:
        """
        Compute Morton order (Z-order) for points
        
        Args:
            points: Nx3 array of points
            
        Returns:
            Ordering indices
        """
        # Normalize points to [0, 1]
        min_vals = points.min(axis=0)
        max_vals = points.max(axis=0)
        range_vals = max_vals - min_vals
        range_vals[range_vals == 0] = 1.0
        
        normalized = (points - min_vals) / range_vals
        
        # Convert to integer coordinates (10-bit precision)
        int_coords = (normalized * 1023).astype(np.uint32)
        
        # Compute Morton codes
        morton_codes = np.zeros(len(points), dtype=np.uint64)
        
        for i in range(10):  # 10 bits per dimension
            for dim in range(3):
                bit = (int_coords[:, dim] >> i) & 1
                morton_codes |= bit << (3 * i + dim)
        
        # Sort by Morton code
        indices = np.argsort(morton_codes)
        return indices
    
    def _kdtree_order(self, points: np.ndarray) -> np.ndarray:
        """
        KD-tree based ordering for spatial locality
        
        Args:
            points: Nx3 array of points
            
        Returns:
            Ordering indices
        """
        from scipy.spatial import cKDTree
        
        # Build KD-tree
        tree = cKDTree(points)
        
        # Depth-first traversal for locality
        indices = []
        
        def traverse(node):
            if hasattr(node, 'split_dim'):
                # Internal node
                traverse(node.lesser)
                traverse(node.greater)
            else:
                # Leaf node
                indices.extend(node.indices)
        
        # Start traversal
        if hasattr(tree, 'tree'):
            traverse(tree.tree)
        else:
            indices = list(range(len(points)))
        
        return np.array(indices)
    
    def optimize_cluster_access(self, clusters: List[Dict]) -> List[Dict]:
        """
        Optimize cluster data structure for cache-friendly access
        
        Args:
            clusters: List of cluster dictionaries
            
        Returns:
            Optimized clusters
        """
        if not clusters:
            return clusters
        
        # Sort clusters by size for better branch prediction
        clusters = sorted(clusters, key=lambda c: c.get('size', 0), reverse=True)
        
        # Pack cluster data for better locality
        for cluster in clusters:
            if 'points' in cluster and isinstance(cluster['points'], np.ndarray):
                # Ensure contiguous memory
                cluster['points'] = np.ascontiguousarray(cluster['points'], dtype=np.float32)
                
                # Reorder points within cluster for locality
                if len(cluster['points']) > 10:
                    reordered, _ = self.optimize_point_order(cluster['points'], 'morton')
                    cluster['points'] = reordered
        
        return clusters
