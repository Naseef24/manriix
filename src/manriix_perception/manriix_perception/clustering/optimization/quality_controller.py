#!/usr/bin/env python3
"""
Dynamic quality controller for maintaining target FPS
Automatically adjusts processing quality based on system load
"""

import numpy as np
from rclpy.node import Node
from typing import Dict, List, Optional
from collections import deque
import time
from enum import Enum


class QualityLevel(Enum):
    """Quality level enumeration"""
    ULTRA = 5
    HIGH = 4
    MEDIUM = 3
    LOW = 2
    MINIMUM = 1


class QualityController:
    """
    Dynamic quality adjustment for consistent real-time performance
    """
    
    def __init__(self, target_fps: float = 30.0, node: Node = None):
        """
        Initialize quality controller
        
        Args:
            target_fps: Target frame rate to maintain
            node: ROS2 node for logging
        """
        self.target_fps = target_fps
        self.target_frame_time = 1.0 / target_fps
        self.node = node
        
        # Current settings
        self.current_level = QualityLevel.HIGH
        self.current_settings = {}
        
        # Performance tracking
        self.fps_history = deque(maxlen=30)
        self.frame_time_history = deque(maxlen=30)
        self.quality_changes = deque(maxlen=10)
        
        # Control parameters
        self.min_stable_frames = 10  # Frames before quality change
        self.frames_since_change = 0
        self.hysteresis = 0.1  # 10% hysteresis to prevent oscillation
        
        # Quality presets
        self.quality_presets = self._init_quality_presets()
        
        # Apply initial settings
        self._apply_quality_level(self.current_level)
        
        if self.node:
            self.node.get_logger().info(f"Quality Controller initialized (target: {target_fps} FPS)")
    
    def _init_quality_presets(self) -> Dict:
        """Initialize quality level presets"""
        return {
            QualityLevel.ULTRA: {
                'point_downsample': 1,
                'birch_threshold': 1.5,
                'birch_branching': 50,
                'kalman_enabled': True,
                'gpu_enabled': True,
                'jit_enabled': True,
                'max_tracks': 100,
                'frame_skip': 0,
                'interpolation': False,
                'cf_tree_depth': 5,
                'min_cluster_size': 1,
                'adaptive_threshold': True,
                'multi_stream': True,
                'memory_optimization': True
            },
            QualityLevel.HIGH: {
                'point_downsample': 1,
                'birch_threshold': 1.5,
                'birch_branching': 50,
                'kalman_enabled': True,
                'gpu_enabled': True,
                'jit_enabled': True,
                'max_tracks': 80,
                'frame_skip': 0,
                'interpolation': False,
                'cf_tree_depth': 4,
                'min_cluster_size': 1,
                'adaptive_threshold': True,
                'multi_stream': True,
                'memory_optimization': True
            },
            QualityLevel.MEDIUM: {
                'point_downsample': 2,
                'birch_threshold': 1.7,
                'birch_branching': 40,
                'kalman_enabled': True,
                'gpu_enabled': True,
                'jit_enabled': True,
                'max_tracks': 60,
                'frame_skip': 1,
                'interpolation': True,
                'cf_tree_depth': 3,
                'min_cluster_size': 2,
                'adaptive_threshold': True,
                'multi_stream': False,
                'memory_optimization': True
            },
            QualityLevel.LOW: {
                'point_downsample': 3,
                'birch_threshold': 2.0,
                'birch_branching': 30,
                'kalman_enabled': False,
                'gpu_enabled': True,
                'jit_enabled': False,
                'max_tracks': 40,
                'frame_skip': 2,
                'interpolation': True,
                'cf_tree_depth': 2,
                'min_cluster_size': 3,
                'adaptive_threshold': False,
                'multi_stream': False,
                'memory_optimization': False
            },
            QualityLevel.MINIMUM: {
                'point_downsample': 4,
                'birch_threshold': 2.5,
                'birch_branching': 20,
                'kalman_enabled': False,
                'gpu_enabled': False,
                'jit_enabled': False,
                'max_tracks': 20,
                'frame_skip': 3,
                'interpolation': True,
                'cf_tree_depth': 2,
                'min_cluster_size': 4,
                'adaptive_threshold': False,
                'multi_stream': False,
                'memory_optimization': False
            }
        }
    
    def update_performance(self, frame_time: float):
        """
        Update performance metrics and adjust quality if needed
        
        Args:
            frame_time: Time taken for last frame
        """
        self.frame_time_history.append(frame_time)
        current_fps = 1.0 / frame_time if frame_time > 0 else 0
        self.fps_history.append(current_fps)
        
        self.frames_since_change += 1
        
        # Check if we should adjust quality
        if self.frames_since_change >= self.min_stable_frames:
            self._evaluate_quality_adjustment()
    
    def _evaluate_quality_adjustment(self):
        """Evaluate if quality adjustment is needed"""
        if not self.fps_history:
            return
        
        avg_fps = np.mean(self.fps_history)
        std_fps = np.std(self.fps_history)
        
        # Calculate performance ratio
        perf_ratio = avg_fps / self.target_fps
        
        # Determine if adjustment needed
        should_increase = perf_ratio > (1.0 + self.hysteresis) and std_fps < 5.0
        should_decrease = perf_ratio < (1.0 - self.hysteresis)
        
        if should_increase:
            self._increase_quality()
        elif should_decrease:
            self._decrease_quality()
    
    def _increase_quality(self):
        """Increase quality level if possible"""
        next_levels = {
            QualityLevel.MINIMUM: QualityLevel.LOW,
            QualityLevel.LOW: QualityLevel.MEDIUM,
            QualityLevel.MEDIUM: QualityLevel.HIGH,
            QualityLevel.HIGH: QualityLevel.ULTRA,
            QualityLevel.ULTRA: QualityLevel.ULTRA
        }
        
        new_level = next_levels[self.current_level]
        
        if new_level != self.current_level:
            if self.node:
                self.node.get_logger().info(f"Increasing quality: {self.current_level.name} -> {new_level.name}")
            self._apply_quality_level(new_level)
            self.quality_changes.append((time.time(), 'increase', new_level))
    
    def _decrease_quality(self):
        """Decrease quality level if possible"""
        prev_levels = {
            QualityLevel.ULTRA: QualityLevel.HIGH,
            QualityLevel.HIGH: QualityLevel.MEDIUM,
            QualityLevel.MEDIUM: QualityLevel.LOW,
            QualityLevel.LOW: QualityLevel.MINIMUM,
            QualityLevel.MINIMUM: QualityLevel.MINIMUM
        }
        
        new_level = prev_levels[self.current_level]
        
        if new_level != self.current_level:
            if self.node:
                self.node.get_logger().info(f"Decreasing quality: {self.current_level.name} -> {new_level.name}")
            self._apply_quality_level(new_level)
            self.quality_changes.append((time.time(), 'decrease', new_level))
    
    def _apply_quality_level(self, level: QualityLevel):
        """
        Apply quality level settings
        
        Args:
            level: New quality level
        """
        self.current_level = level
        self.current_settings = self.quality_presets[level].copy()
        self.frames_since_change = 0
    
    def get_settings(self) -> Dict:
        """
        Get current quality settings
        
        Returns:
            Dictionary of current settings
        """
        return self.current_settings.copy()
    
    def get_setting(self, key: str, default=None):
        """
        Get specific quality setting
        
        Args:
            key: Setting key
            default: Default value if key not found
            
        Returns:
            Setting value
        """
        return self.current_settings.get(key, default)
    
    def should_process_frame(self) -> bool:
        """
        Determine if current frame should be processed
        
        Returns:
            True if frame should be processed
        """
        skip_rate = self.current_settings.get('frame_skip', 0)
        
        if skip_rate == 0:
            return True
        
        # Simple frame skipping pattern
        frame_count = len(self.frame_time_history)
        return frame_count % (skip_rate + 1) == 0
    
    def get_adaptive_parameters(self, n_points: int) -> Dict:
        """
        Get adaptive parameters based on current load
        
        Args:
            n_points: Number of points to process
            
        Returns:
            Adaptive parameters
        """
        base_settings = self.current_settings.copy()
        
        # Further adjust based on point count
        if n_points > 100:
            # Heavy load - apply additional restrictions
            load_factor = min(2.0, n_points / 100.0)
            
            base_settings['birch_threshold'] *= (1.0 + 0.1 * (load_factor - 1))
            base_settings['min_cluster_size'] = max(
                base_settings['min_cluster_size'],
                int(load_factor)
            )
            
            if n_points > 150:
                # Very heavy load
                base_settings['kalman_enabled'] = False
                base_settings['adaptive_threshold'] = False
        
        return base_settings
    
    def get_statistics(self) -> Dict:
        """Get quality control statistics"""
        stats = {
            'current_level': self.current_level.name,
            'current_fps': np.mean(self.fps_history) if self.fps_history else 0,
            'target_fps': self.target_fps,
            'fps_stability': 1.0 / (1.0 + np.std(self.fps_history)) if self.fps_history else 0,
            'frames_since_change': self.frames_since_change,
            'quality_changes': len(self.quality_changes),
            'settings': self.current_settings
        }
        
        # Add performance metrics
        if self.frame_time_history:
            stats['avg_frame_time_ms'] = np.mean(self.frame_time_history) * 1000
            stats['max_frame_time_ms'] = np.max(self.frame_time_history) * 1000
            stats['min_frame_time_ms'] = np.min(self.frame_time_history) * 1000
        
        return stats
    
    def force_quality_level(self, level: QualityLevel):
        """
        Force specific quality level (for testing)
        
        Args:
            level: Quality level to force
        """
        if self.node:
            self.node.get_logger().info(f"Forcing quality level: {level.name}")
        self._apply_quality_level(level)


class PerformanceBalancer:
    """
    Balance performance across different components
    """
    
    def __init__(self, node: Node = None):
        """Initialize performance balancer"""
        self.node = node
        self.component_times = {
            'detection': deque(maxlen=30),
            'clustering': deque(maxlen=30),
            'tracking': deque(maxlen=30),
            'optimization': deque(maxlen=30)
        }
        
        self.component_weights = {
            'detection': 0.3,
            'clustering': 0.4,
            'tracking': 0.2,
            'optimization': 0.1
        }
    
    def update_component_time(self, component: str, time_ms: float):
        """
        Update component processing time
        
        Args:
            component: Component name
            time_ms: Processing time in milliseconds
        """
        if component in self.component_times:
            self.component_times[component].append(time_ms)
    
    def get_bottleneck(self) -> Optional[str]:
        """
        Identify performance bottleneck
        
        Returns:
            Name of bottleneck component or None
        """
        avg_times = {}
        
        for component, times in self.component_times.items():
            if times:
                avg_times[component] = np.mean(times)
        
        if not avg_times:
            return None
        
        # Find component with highest time
        bottleneck = max(avg_times, key=avg_times.get)
        
        # Check if it's significantly higher than others
        bottleneck_time = avg_times[bottleneck]
        other_times = [t for c, t in avg_times.items() if c != bottleneck]
        
        if other_times:
            avg_other = np.mean(other_times)
            if bottleneck_time > avg_other * 1.5:  # 50% higher than average
                return bottleneck
        
        return None
    
    def get_optimization_suggestions(self) -> List[str]:
        """
        Get optimization suggestions based on performance data
        
        Returns:
            List of optimization suggestions
        """
        suggestions = []
        bottleneck = self.get_bottleneck()
        
        if bottleneck:
            if bottleneck == 'detection':
                suggestions.append("Consider increasing point downsampling")
                suggestions.append("Enable GPU acceleration for detection")
            elif bottleneck == 'clustering':
                suggestions.append("Increase BIRCH threshold for faster clustering")
                suggestions.append("Reduce branching factor")
                suggestions.append("Enable adaptive thresholds")
            elif bottleneck == 'tracking':
                suggestions.append("Reduce maximum number of tracks")
                suggestions.append("Disable Kalman filtering temporarily")
                suggestions.append("Increase track pruning frequency")
            elif bottleneck == 'optimization':
                suggestions.append("Disable some optimization features")
                suggestions.append("Reduce quality level")
        
        return suggestions
    
    def get_component_distribution(self) -> Dict:
        """
        Get time distribution across components
        
        Returns:
            Component time distribution
        """
        distribution = {}
        total_time = 0
        
        for component, times in self.component_times.items():
            if times:
                avg_time = np.mean(times)
                distribution[component] = avg_time
                total_time += avg_time
        
        # Convert to percentages
        if total_time > 0:
            for component in distribution:
                distribution[component] = (distribution[component] / total_time) * 100
        
        return distribution
