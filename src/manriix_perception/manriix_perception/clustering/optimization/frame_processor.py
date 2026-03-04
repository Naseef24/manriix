#!/usr/bin/env python3
"""
Frame processing optimization with skipping and interpolation
Maintains real-time performance under heavy load
"""

import numpy as np
from rclpy.node import Node
from typing import Dict, List, Tuple, Optional
from collections import deque
import time
from scipy.interpolate import interp1d


class FrameProcessor:
    """
    Optimized frame processing with intelligent skipping and interpolation
    Ensures consistent 30+ FPS even with complex scenes
    """
    
    def __init__(self, target_fps: float = 30.0, min_fps: float = 15.0, node: Node = None):
        """
        Initialize frame processor
        
        Args:
            target_fps: Target frame rate
            min_fps: Minimum acceptable frame rate
            node: ROS2 node for logging
        """
        self.target_fps = target_fps
        self.min_fps = min_fps
        self.target_frame_time = 1.0 / target_fps
        self.max_frame_time = 1.0 / min_fps
        self.node = node
        
        # Frame skipping parameters
        self.skip_counter = 0
        self.max_skip = 2  # Maximum consecutive frames to skip
        self.current_skip_rate = 0
        
        # Interpolation buffers
        self.position_buffer = deque(maxlen=10)
        self.timestamp_buffer = deque(maxlen=10)
        self.interpolators = {}
        
        # Performance tracking
        self.frame_times = deque(maxlen=30)
        self.processing_times = deque(maxlen=30)
        self.skip_history = deque(maxlen=100)
        
        # Adaptive parameters
        self.load_factor = 0.0
        self.quality_level = 1.0  # 1.0 = full quality, 0.0 = minimum
        
        if self.node:
            self.node.get_logger().info(f"Frame Processor initialized (target: {target_fps} FPS)")
    
    def should_process_frame(self) -> bool:
        """
        Determine if current frame should be processed
        
        Returns:
            True if frame should be processed
        """
        # Always process if we haven't skipped recently
        if self.skip_counter >= self.max_skip:
            self.skip_counter = 0
            return True
        
        # Check current load
        if self.load_factor > 0.8:  # High load
            self.skip_counter += 1
            self.skip_history.append(1)
            return False
        elif self.load_factor > 0.5:  # Medium load
            # Skip every other frame
            should_skip = self.skip_counter % 2 == 1
            self.skip_counter += 1
            self.skip_history.append(1 if should_skip else 0)
            return not should_skip
        else:  # Low load
            self.skip_counter = 0
            self.skip_history.append(0)
            return True
    
    def interpolate_positions(self, timestamp: float) -> Optional[Dict[int, np.ndarray]]:
        """
        Interpolate positions for skipped frame
        
        Args:
            timestamp: Current timestamp
            
        Returns:
            Interpolated positions or None if insufficient data
        """
        if len(self.position_buffer) < 2:
            return None
        
        try:
            # Get timestamps and positions from buffer
            timestamps = np.array(self.timestamp_buffer)
            
            # Check if we have enough history
            if timestamp < timestamps[0] or timestamp > timestamps[-1] + 0.1:
                return None
            
            interpolated = {}
            
            for track_id in self.interpolators:
                if track_id in self.position_buffer[-1]:
                    # Get historical positions for this track
                    positions = []
                    valid_times = []
                    
                    for i, pos_dict in enumerate(self.position_buffer):
                        if track_id in pos_dict:
                            positions.append(pos_dict[track_id])
                            valid_times.append(timestamps[i])
                    
                    if len(positions) >= 2:
                        # Create interpolator
                        positions = np.array(positions)
                        valid_times = np.array(valid_times)
                        
                        # Linear interpolation for each dimension
                        interp_x = interp1d(valid_times, positions[:, 0], 
                                          kind='linear', fill_value='extrapolate')
                        interp_y = interp1d(valid_times, positions[:, 1], 
                                          kind='linear', fill_value='extrapolate')
                        interp_z = interp1d(valid_times, positions[:, 2], 
                                          kind='linear', fill_value='extrapolate')
                        
                        # Interpolate position
                        interpolated[track_id] = np.array([
                            interp_x(timestamp),
                            interp_y(timestamp),
                            interp_z(timestamp)
                        ])
            
            return interpolated
            
        except Exception as e:
            if self.node:
                self.node.get_logger().debug(f"Interpolation failed: {e}")
            return None
    
    def update_position_buffer(self, positions: Dict[int, np.ndarray], timestamp: float):
        """
        Update position buffer for interpolation
        
        Args:
            positions: Dictionary of track_id -> position
            timestamp: Frame timestamp
        """
        self.position_buffer.append(positions.copy())
        self.timestamp_buffer.append(timestamp)
        
        # Update interpolator list
        for track_id in positions:
            if track_id not in self.interpolators:
                self.interpolators[track_id] = True
        
        # Clean up old tracks
        if len(self.position_buffer) == self.position_buffer.maxlen:
            old_tracks = set(self.interpolators.keys())
            current_tracks = set()
            for pos_dict in self.position_buffer:
                current_tracks.update(pos_dict.keys())
            
            for track_id in old_tracks - current_tracks:
                del self.interpolators[track_id]
    
    def update_performance_metrics(self, frame_time: float, processing_time: float):
        """
        Update performance metrics and adjust parameters
        
        Args:
            frame_time: Total frame time
            processing_time: Processing time for frame
        """
        self.frame_times.append(frame_time)
        self.processing_times.append(processing_time)
        
        # Calculate load factor
        if self.frame_times:
            avg_frame_time = np.mean(self.frame_times)
            self.load_factor = min(1.0, avg_frame_time / self.target_frame_time)
        
        # Adjust quality based on load
        if self.load_factor > 0.9:
            self.quality_level = max(0.5, self.quality_level - 0.05)
        elif self.load_factor < 0.7:
            self.quality_level = min(1.0, self.quality_level + 0.02)
        
        # Adjust max skip rate
        if self.load_factor > 0.95:
            self.max_skip = min(3, self.max_skip + 1)
        elif self.load_factor < 0.6:
            self.max_skip = max(1, self.max_skip - 1)
    
    def get_quality_parameters(self) -> Dict:
        """
        Get current quality parameters for adaptive processing
        
        Returns:
            Dictionary of quality parameters
        """
        return {
            'quality_level': self.quality_level,
            'skip_rate': self.current_skip_rate,
            'max_skip': self.max_skip,
            'downsampling_factor': self._get_downsampling_factor(),
            'clustering_threshold': self._get_clustering_threshold(),
            'tracking_max_age': self._get_tracking_max_age()
        }
    
    def _get_downsampling_factor(self) -> int:
        """Get point cloud downsampling factor based on quality"""
        if self.quality_level > 0.8:
            return 1  # No downsampling
        elif self.quality_level > 0.6:
            return 2  # Skip every other point
        else:
            return 3  # Skip 2 out of 3 points
    
    def _get_clustering_threshold(self) -> float:
        """Get adaptive clustering threshold"""
        base_threshold = 1.5
        if self.quality_level < 0.7:
            # Increase threshold for faster clustering
            return base_threshold * 1.3
        return base_threshold
    
    def _get_tracking_max_age(self) -> int:
        """Get adaptive tracking max age"""
        base_age = 10
        if self.quality_level < 0.6:
            # Reduce max age to prune tracks faster
            return max(5, int(base_age * self.quality_level))
        return base_age
    
    def get_statistics(self) -> Dict:
        """Get frame processing statistics"""
        skip_rate = np.mean(self.skip_history) if self.skip_history else 0
        self.current_skip_rate = skip_rate
        
        stats = {
            'current_fps': 1.0 / np.mean(self.frame_times) if self.frame_times else 0,
            'target_fps': self.target_fps,
            'load_factor': self.load_factor,
            'quality_level': self.quality_level,
            'skip_rate': skip_rate,
            'avg_processing_time': np.mean(self.processing_times) * 1000 if self.processing_times else 0,
            'max_skip': self.max_skip,
            'interpolation_available': len(self.position_buffer) >= 2
        }
        
        return stats


class AdaptiveProcessor:
    """
    Adaptive processing based on scene complexity
    """
    
    def __init__(self, node: Node = None):
        """Initialize adaptive processor"""
        self.node = node
        self.complexity_history = deque(maxlen=30)
        self.current_complexity = 0.0
        
        # Adaptive thresholds
        self.complexity_thresholds = {
            'simple': 0.3,
            'moderate': 0.6,
            'complex': 0.85,
            'very_complex': 1.0
        }
        
        # Processing strategies for different complexity levels
        self.strategies = {
            'simple': {
                'downsample': 1,
                'birch_threshold': 1.5,
                'min_cluster_size': 1,
                'use_gpu': False,
                'use_kalman': True
            },
            'moderate': {
                'downsample': 1,
                'birch_threshold': 1.5,
                'min_cluster_size': 1,
                'use_gpu': True,
                'use_kalman': True
            },
            'complex': {
                'downsample': 2,
                'birch_threshold': 1.7,
                'min_cluster_size': 2,
                'use_gpu': True,
                'use_kalman': True
            },
            'very_complex': {
                'downsample': 3,
                'birch_threshold': 2.0,
                'min_cluster_size': 3,
                'use_gpu': True,
                'use_kalman': False  # Disable Kalman for speed
            }
        }
    
    def calculate_complexity(self, n_points: int, n_clusters: int, 
                           spatial_spread: float) -> float:
        """
        Calculate scene complexity
        
        Args:
            n_points: Number of detected points
            n_clusters: Number of clusters
            spatial_spread: Spatial distribution measure
            
        Returns:
            Complexity score [0, 1]
        """
        # Normalize factors
        point_factor = min(1.0, n_points / 100.0)  # Max 100 points
        cluster_factor = min(1.0, n_clusters / 20.0)  # Max 20 clusters
        spread_factor = min(1.0, spatial_spread / 10.0)  # Max 10m spread
        
        # Weighted combination
        complexity = (
            0.5 * point_factor +
            0.3 * cluster_factor +
            0.2 * spread_factor
        )
        
        self.complexity_history.append(complexity)
        self.current_complexity = np.mean(self.complexity_history)
        
        return self.current_complexity
    
    def get_processing_strategy(self) -> Dict:
        """
        Get processing strategy based on current complexity
        
        Returns:
            Strategy dictionary
        """
        if self.current_complexity < self.complexity_thresholds['simple']:
            strategy_name = 'simple'
        elif self.current_complexity < self.complexity_thresholds['moderate']:
            strategy_name = 'moderate'
        elif self.current_complexity < self.complexity_thresholds['complex']:
            strategy_name = 'complex'
        else:
            strategy_name = 'very_complex'
        
        strategy = self.strategies[strategy_name].copy()
        strategy['complexity_level'] = strategy_name
        strategy['complexity_score'] = self.current_complexity
        
        return strategy
    
    def should_use_optimization(self, optimization: str) -> bool:
        """
        Check if specific optimization should be used
        
        Args:
            optimization: Name of optimization
            
        Returns:
            True if optimization should be used
        """
        strategy = self.get_processing_strategy()
        
        optimization_map = {
            'gpu': strategy.get('use_gpu', False),
            'kalman': strategy.get('use_kalman', True),
            'downsampling': strategy.get('downsample', 1) > 1,
            'threshold_relaxation': strategy.get('birch_threshold', 1.5) > 1.5
        }
        
        return optimization_map.get(optimization, False)
