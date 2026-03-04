#!/usr/bin/env python3
"""
Memory pool management for efficient GPU memory usage
Implements circular buffers and unified memory for Jetson
"""

import numpy as np
from collections import deque
from typing import Optional, Tuple, Dict
from rclpy.node import Node

try:
    import cupy as cp
    CUPY_AVAILABLE = True
except ImportError:
    CUPY_AVAILABLE = False


class MemoryPool:
    """
    Manages memory allocation for streaming data processing.
    Uses circular buffers and memory pooling to minimize allocations.
    """
    
    def __init__(self, max_people: int = 100, history_size: int = 150, node: Node = None):
        """
        Initialize memory pool
        
        Args:
            max_people: Maximum number of people to track
            history_size: Size of history buffer (frames)
            node: ROS2 node for logging
        """
        self.max_people = max_people
        self.history_size = history_size
        self.node = node
        
        if not CUPY_AVAILABLE and self.node:
            self.node.get_logger().warn("CuPy not available - GPU memory pooling disabled")
        
        # Pre-allocate buffers
        self.position_buffer = CircularBuffer(
            shape=(max_people, 3),
            maxlen=history_size,
            dtype=np.float32
        )
        
        self.label_buffer = CircularBuffer(
            shape=(max_people,),
            maxlen=history_size,
            dtype=np.int32
        )
        
        # GPU memory pool if available
        if CUPY_AVAILABLE:
            self._init_gpu_memory()
        else:
            self.gpu_available = False
        
        if self.node:
            self.node.get_logger().info(f"MemoryPool initialized: max_people={max_people}, "
                                        f"history={history_size}, GPU={CUPY_AVAILABLE}")
    
    def _init_gpu_memory(self):
        """Initialize GPU memory pool"""
        
        try:
            # Use unified memory for zero-copy on Jetson
            self.mempool = cp.get_default_memory_pool()
            self.pinned_mempool = cp.get_default_pinned_memory_pool()
            
            # Pre-allocate GPU arrays
            self.gpu_position_buffer = cp.zeros(
                (self.history_size, self.max_people, 3),
                dtype=cp.float32
            )
            
            self.gpu_label_buffer = cp.zeros(
                (self.history_size, self.max_people),
                dtype=cp.int32
            )
            
            # Current frame pointers
            self.gpu_current_frame = 0
            
            self.gpu_available = True
            
            if self.node:
                self.node.get_logger().info(f"GPU memory pool initialized: "
                                            f"{self.mempool.total_bytes() / 1024**2:.1f} MB")
            
        except Exception as e:
            if self.node:
                self.node.get_logger().warn(f"Failed to initialize GPU memory: {e}")
            self.gpu_available = False
    
    def add_frame(self, positions: np.ndarray, labels: Optional[np.ndarray] = None):
        """
        Add a frame to the buffer
        
        Args:
            positions: Nx3 array of positions
            labels: Optional array of cluster labels
        """
        
        # Pad or truncate to max_people
        n_people = min(len(positions), self.max_people)
        
        padded_positions = np.zeros((self.max_people, 3), dtype=np.float32)
        padded_positions[:n_people] = positions[:n_people]
        
        self.position_buffer.append(padded_positions)
        
        if labels is not None:
            padded_labels = np.full(self.max_people, -1, dtype=np.int32)
            padded_labels[:n_people] = labels[:n_people]
            self.label_buffer.append(padded_labels)
        
        # Update GPU buffers if available
        if self.gpu_available:
            self._update_gpu_buffers(padded_positions, labels)
    
    def _update_gpu_buffers(self, positions: np.ndarray, labels: Optional[np.ndarray]):
        """Update GPU buffers with new frame"""
        
        try:
            # Copy to GPU using current frame index
            idx = self.gpu_current_frame % self.history_size
            
            self.gpu_position_buffer[idx] = cp.asarray(positions)
            
            if labels is not None:
                self.gpu_label_buffer[idx] = cp.asarray(labels)
            
            self.gpu_current_frame += 1
            
        except Exception as e:
            if self.node:
                self.node.get_logger().debug(f"GPU buffer update error: {e}")
    
    def get_recent_frames(self, n_frames: int = 10) -> np.ndarray:
        """
        Get recent frames from buffer
        
        Args:
            n_frames: Number of recent frames to retrieve
            
        Returns:
            Array of recent positions
        """
        
        return self.position_buffer.get_recent(n_frames)
    
    def get_gpu_data(self) -> Optional[Tuple]:
        """
        Get GPU data if available
        
        Returns:
            Tuple of (positions, labels) on GPU or None
        """
        
        if not self.gpu_available:
            return None
        
        try:
            # Get current frame data
            idx = (self.gpu_current_frame - 1) % self.history_size
            return (
                self.gpu_position_buffer[idx],
                self.gpu_label_buffer[idx]
            )
        except:
            return None
    
    def clear(self):
        """Clear all buffers"""
        
        self.position_buffer.clear()
        self.label_buffer.clear()
        
        if self.gpu_available:
            self.gpu_position_buffer.fill(0)
            self.gpu_label_buffer.fill(-1)
            self.gpu_current_frame = 0
    
    def get_memory_usage(self) -> Dict:
        """Get memory usage statistics"""
        
        cpu_bytes = (
            self.position_buffer.memory_usage() +
            self.label_buffer.memory_usage()
        )
        
        gpu_bytes = 0
        if self.gpu_available and CUPY_AVAILABLE:
            gpu_bytes = self.mempool.used_bytes()
        
        return {
            'cpu_mb': cpu_bytes / (1024**2),
            'gpu_mb': gpu_bytes / (1024**2),
            'total_mb': (cpu_bytes + gpu_bytes) / (1024**2)
        }


class CircularBuffer:
    """
    Efficient circular buffer implementation for streaming data
    """
    
    def __init__(self, shape: Tuple, maxlen: int, dtype=np.float32):
        """
        Initialize circular buffer
        
        Args:
            shape: Shape of each element
            maxlen: Maximum number of elements
            dtype: Data type
        """
        self.shape = shape
        self.maxlen = maxlen
        self.dtype = dtype
        
        # Pre-allocate array
        self.buffer = np.zeros((maxlen,) + shape, dtype=dtype)
        self.index = 0
        self.size = 0
    
    def append(self, data: np.ndarray):
        """Add data to buffer"""
        
        self.buffer[self.index % self.maxlen] = data
        self.index += 1
        self.size = min(self.size + 1, self.maxlen)
    
    def get_recent(self, n: int) -> np.ndarray:
        """Get n most recent entries"""
        
        n = min(n, self.size)
        if n == 0:
            return np.array([])
        
        if self.size < self.maxlen:
            # Buffer not full yet
            return self.buffer[max(0, self.size - n):self.size]
        else:
            # Circular buffer is full
            start_idx = (self.index - n) % self.maxlen
            if start_idx < self.index % self.maxlen:
                return self.buffer[start_idx:self.index % self.maxlen]
            else:
                return np.vstack([
                    self.buffer[start_idx:],
                    self.buffer[:self.index % self.maxlen]
                ])
    
    def clear(self):
        """Clear buffer"""
        
        self.buffer.fill(0)
        self.index = 0
        self.size = 0
    
    def memory_usage(self) -> int:
        """Get memory usage in bytes"""
        
        return self.buffer.nbytes
    
    def __len__(self):
        return self.size
