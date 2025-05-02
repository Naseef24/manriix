#!/usr/bin/env python3

import numpy as np
from typing import List, Dict, Tuple 
from collections import deque 
import threading 
from sklearn.neighbors import KDTree 
import rospy 
import multiprocessing 
from concurrent.futures import ThreadPoolExecutor 

class CircularBuffer: 
    """Thread safe circular buffer for efficient data storage"""

    def __init__(self, max_size:int):
        self.buffer = deque(maxlen=max_size)
        self.lock = threading.Lock()

    def append(self, item):
        with self.lock:
            self.buffer.append(item)
    
    def get_data(self):
        with self.lock:
            return list(self.buffer) 
        
    def clear(self):
        with self.lock:
            self.buffer.clear()

class SpatialIndex:
    """KD-tree based spatial indexing for efficient neighbor searches"""

    def __init__(self):
        self.kdtree = None 
        self.points = None 
        self.lock = threading.Lock()

    def build_index(self, points: np.ndarray):
        """Build KD-tree index from points"""
        with self.lock:
            self.points = points 
            self.kdtree = KDTree(points)    
    
    def query_radius(self, point: np.ndarray, radius:float) -> List[int]:
        """find all points within radius of query point"""
        with self.lock:
            if self.kdtree is None:
                return []
            indices = self.kdtree.query_radius([point], radius)[0]
            return indices.tolist()
        
    def query_neighbors(self, point: np.ndarray, k: int) -> Tuple[List[int], List[float]]:
        """Find k nearest neighbors of query point"""
        with self.lock:
            if self.kdtree is None:
                return [], []
            distances, indices = self.kdtree.query([point], k=k)
            return indices[0].tolist(), distances[0].tolist()
        
class ParallelProcessor:
    """Handles parallel processing for computationally intensive tasks"""
    
    def __init__(self, max_workers: int = None):
        if max_workers is None:
            max_workers = multiprocessing.cpu_count()
        self.executor = ThreadPoolExecutor(max_workers=max_workers)
        self.lock = threading.Lock()
        
    def process_chunks(self, data: List, chunk_size: int):
        """Split data into chunks for parallel processing"""
        for i in range(0, len(data), chunk_size):
            yield data[i:i + chunk_size]
            
    def map_async(self, func, data_chunks):
        """Process data chunks in parallel"""
        try:
            futures = []
            for chunk in data_chunks:
                future = self.executor.submit(func, chunk)
                futures.append(future)
            
            results = []
            for future in futures:
                try:
                    result = future.result()
                    results.append(result)
                except Exception as e:
                    rospy.logerr(f"Error in parallel processing: {e}")
            
            return results
        except Exception as e:
            rospy.logerr(f"Error in parallel mapping: {e}")
            return []
        
class MemoryManager:
    """Manages memory usage and cleanup"""
    
    def __init__(self, max_history: int = 100):
        self.data_buffers = {}
        self.max_history = max_history
        self.lock = threading.Lock()
        
    def register_buffer(self, name: str):
        """Register a new circular buffer"""
        with self.lock:
            if name not in self.data_buffers:
                self.data_buffers[name] = CircularBuffer(self.max_history)
                
    def store_data(self, buffer_name: str, data):
        """Store data in specified buffer"""
        with self.lock:
            if buffer_name in self.data_buffers:
                self.data_buffers[buffer_name].append(data)
                
    def get_data(self, buffer_name: str) -> List:
        """Retrieve data from specified buffer"""
        with self.lock:
            if buffer_name in self.data_buffers:
                return self.data_buffers[buffer_name].get_data()
            return []
            
    def cleanup_old_data(self):
        """Clean up old data from all buffers"""
        with self.lock:
            for buffer in self.data_buffers.values():
                while len(buffer.buffer) > self.max_history:
                    buffer.buffer.popleft()        