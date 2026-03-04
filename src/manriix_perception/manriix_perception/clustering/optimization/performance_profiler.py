#!/usr/bin/env python3
"""
Performance profiler and bottleneck detection
Provides detailed timing analysis and optimization recommendations
"""

import numpy as np
from rclpy.node import Node
from typing import Dict, List, Optional, Callable, Any
from collections import deque, defaultdict
import time
import cProfile
import pstats
import io
from contextlib import contextmanager
import threading
import traceback


class PerformanceProfiler:
    """
    Comprehensive performance profiling system
    """
    
    def __init__(self, enable_detailed: bool = False, node: Node = None):
        """
        Initialize performance profiler
        
        Args:
            enable_detailed: Enable detailed profiling (higher overhead)
            node: ROS2 node for logging
        """
        self.enable_detailed = enable_detailed
        self.node = node
        
        # Timing storage
        self.timings = defaultdict(lambda: deque(maxlen=100))
        self.call_counts = defaultdict(int)
        self.total_times = defaultdict(float)
        
        # Hierarchical timing
        self.timing_stack = []
        self.current_section = None
        
        # Bottleneck detection
        self.bottleneck_threshold = 0.3  # 30% of total time
        self.slow_operations = deque(maxlen=50)
        
        # Thread safety
        self.lock = threading.Lock()
        
        # Python profiler for detailed analysis
        if enable_detailed:
            self.profiler = cProfile.Profile()
        else:
            self.profiler = None
        
        if self.node:
            self.node.get_logger().info(f"Performance Profiler initialized (detailed: {enable_detailed})")
    
    @contextmanager
    def profile_section(self, name: str):
        """
        Context manager for profiling code sections
        
        Args:
            name: Section name
            
        Usage:
            with profiler.profile_section('clustering'):
                # Code to profile
        """
        start_time = time.perf_counter()
        
        # Push to timing stack
        with self.lock:
            parent = self.current_section
            self.current_section = name
            self.timing_stack.append((name, start_time, parent))
        
        try:
            yield
        finally:
            end_time = time.perf_counter()
            elapsed = (end_time - start_time) * 1000  # Convert to ms
            
            with self.lock:
                # Record timing
                self.timings[name].append(elapsed)
                self.call_counts[name] += 1
                self.total_times[name] += elapsed
                
                # Check for slow operations
                if elapsed > 10.0:  # > 10ms is slow
                    self.slow_operations.append({
                        'name': name,
                        'time_ms': elapsed,
                        'timestamp': time.time(),
                        'parent': parent
                    })
                
                # Pop from stack
                if self.timing_stack:
                    self.timing_stack.pop()
                
                # Restore parent section
                self.current_section = parent
    
    def profile_function(self, func: Callable) -> Callable:
        """
        Decorator for profiling functions
        
        Args:
            func: Function to profile
            
        Returns:
            Wrapped function
        """
        def wrapper(*args, **kwargs):
            with self.profile_section(func.__name__):
                return func(*args, **kwargs)
        
        wrapper.__name__ = func.__name__
        return wrapper
    
    def start_detailed_profiling(self):
        """Start detailed CPU profiling"""
        if self.profiler:
            self.profiler.enable()
    
    def stop_detailed_profiling(self) -> str:
        """
        Stop detailed profiling and return report
        
        Returns:
            Profiling report string
        """
        if not self.profiler:
            return "Detailed profiling not enabled"
        
        self.profiler.disable()
        
        # Generate report
        s = io.StringIO()
        ps = pstats.Stats(self.profiler, stream=s).sort_stats('cumulative')
        ps.print_stats(20)  # Top 20 functions
        
        return s.getvalue()
    
    def detect_bottlenecks(self) -> List[Dict]:
        """
        Detect performance bottlenecks
        
        Returns:
            List of bottleneck information
        """
        bottlenecks = []
        
        with self.lock:
            if not self.total_times:
                return bottlenecks
            
            # Calculate total time
            total_time = sum(self.total_times.values())
            
            if total_time == 0:
                return bottlenecks
            
            # Find operations taking significant time
            for name, op_time in self.total_times.items():
                percentage = (op_time / total_time) * 100
                
                if percentage > self.bottleneck_threshold * 100:
                    avg_time = np.mean(self.timings[name]) if self.timings[name] else 0
                    
                    bottlenecks.append({
                        'operation': name,
                        'percentage': percentage,
                        'avg_time_ms': avg_time,
                        'total_time_ms': op_time,
                        'call_count': self.call_counts[name],
                        'severity': self._classify_severity(percentage)
                    })
        
        # Sort by percentage
        bottlenecks.sort(key=lambda x: x['percentage'], reverse=True)
        
        return bottlenecks
    
    def _classify_severity(self, percentage: float) -> str:
        """Classify bottleneck severity"""
        if percentage > 50:
            return 'critical'
        elif percentage > 30:
            return 'high'
        elif percentage > 20:
            return 'medium'
        else:
            return 'low'
    
    def get_optimization_recommendations(self) -> List[str]:
        """
        Get optimization recommendations based on profiling data
        
        Returns:
            List of recommendations
        """
        recommendations = []
        bottlenecks = self.detect_bottlenecks()
        
        for bottleneck in bottlenecks:
            name = bottleneck['operation']
            severity = bottleneck['severity']
            avg_time = bottleneck['avg_time_ms']
            
            if 'clustering' in name.lower():
                if severity == 'critical':
                    recommendations.append(f"CRITICAL: {name} taking {avg_time:.1f}ms - Enable GPU acceleration")
                    recommendations.append(f"Consider increasing BIRCH threshold")
                    recommendations.append(f"Enable frame skipping for clustering")
                elif severity == 'high':
                    recommendations.append(f"HIGH: {name} - Use adaptive thresholds")
                    recommendations.append(f"Reduce clustering frequency")
            
            elif 'tracking' in name.lower():
                if avg_time > 5:
                    recommendations.append(f"Tracking overhead high ({avg_time:.1f}ms) - Reduce max tracks")
                    recommendations.append(f"Consider disabling Kalman filtering")
            
            elif 'distance' in name.lower():
                if avg_time > 3:
                    recommendations.append(f"Distance computation slow - Enable JIT compilation")
                    recommendations.append(f"Use GPU for distance calculations")
            
            elif 'memory' in name.lower():
                recommendations.append(f"Memory operations slow - Enable memory pooling")
                recommendations.append(f"Optimize data layout for cache")
        
        # Check for slow operations
        if self.slow_operations:
            recent_slow = list(self.slow_operations)[-5:]  # Last 5 slow ops
            for op in recent_slow:
                recommendations.append(f"Slow operation: {op['name']} took {op['time_ms']:.1f}ms")
        
        return recommendations
    
    def get_timing_statistics(self, operation: str = None) -> Dict:
        """
        Get timing statistics for operations
        
        Args:
            operation: Specific operation or None for all
            
        Returns:
            Statistics dictionary
        """
        with self.lock:
            if operation:
                if operation not in self.timings:
                    return {}
                
                times = list(self.timings[operation])
                if not times:
                    return {}
                
                return {
                    'operation': operation,
                    'count': self.call_counts[operation],
                    'total_ms': self.total_times[operation],
                    'mean_ms': np.mean(times),
                    'std_ms': np.std(times),
                    'min_ms': np.min(times),
                    'max_ms': np.max(times),
                    'percentile_50': np.percentile(times, 50),
                    'percentile_90': np.percentile(times, 90),
                    'percentile_99': np.percentile(times, 99)
                }
            else:
                # All operations
                stats = {}
                for op in self.timings:
                    stats[op] = self.get_timing_statistics(op)
                return stats
    
    def get_call_graph(self) -> Dict:
        """
        Get simplified call graph
        
        Returns:
            Call graph structure
        """
        graph = {
            'nodes': [],
            'edges': []
        }
        
        with self.lock:
            # Add nodes
            for op in self.timings:
                avg_time = np.mean(self.timings[op]) if self.timings[op] else 0
                graph['nodes'].append({
                    'id': op,
                    'label': op,
                    'time_ms': avg_time,
                    'calls': self.call_counts[op]
                })
            
            # Add edges from slow operations (simplified)
            for slow_op in self.slow_operations:
                if slow_op['parent']:
                    graph['edges'].append({
                        'from': slow_op['parent'],
                        'to': slow_op['name'],
                        'weight': slow_op['time_ms']
                    })
        
        return graph
    
    def reset(self):
        """Reset all profiling data"""
        with self.lock:
            self.timings.clear()
            self.call_counts.clear()
            self.total_times.clear()
            self.slow_operations.clear()
            self.timing_stack.clear()
            self.current_section = None
            
            if self.profiler:
                self.profiler.clear()
    
    def generate_report(self) -> str:
        """
        Generate comprehensive performance report
        
        Returns:
            Report string
        """
        report = []
        report.append("="*60)
        report.append("PERFORMANCE PROFILING REPORT")
        report.append("="*60)
        
        # Overall statistics
        total_time = sum(self.total_times.values())
        total_calls = sum(self.call_counts.values())
        
        report.append(f"\nTotal time: {total_time:.1f}ms")
        report.append(f"Total calls: {total_calls}")
        
        # Bottlenecks
        bottlenecks = self.detect_bottlenecks()
        if bottlenecks:
            report.append("\n### BOTTLENECKS ###")
            for b in bottlenecks[:5]:  # Top 5
                report.append(f"  {b['operation']}: {b['percentage']:.1f}% "
                            f"({b['avg_time_ms']:.1f}ms avg, {b['call_count']} calls)")
        
        # Slow operations
        if self.slow_operations:
            report.append("\n### SLOW OPERATIONS ###")
            for op in list(self.slow_operations)[-5:]:  # Last 5
                report.append(f"  {op['name']}: {op['time_ms']:.1f}ms")
        
        # Recommendations
        recommendations = self.get_optimization_recommendations()
        if recommendations:
            report.append("\n### RECOMMENDATIONS ###")
            for rec in recommendations[:10]:  # Top 10
                report.append(f"  - {rec}")
        
        # Detailed statistics
        report.append("\n### OPERATION STATISTICS ###")
        stats = self.get_timing_statistics()
        sorted_ops = sorted(stats.items(), 
                          key=lambda x: x[1].get('total_ms', 0) if x[1] else 0,
                          reverse=True)
        
        for op, stat in sorted_ops[:10]:  # Top 10
            if stat:
                report.append(f"  {op}:")
                report.append(f"    Calls: {stat['count']}")
                report.append(f"    Mean: {stat['mean_ms']:.2f}ms")
                report.append(f"    P90: {stat['percentile_90']:.2f}ms")
                report.append(f"    Max: {stat['max_ms']:.2f}ms")
        
        report.append("="*60)
        
        return "\n".join(report)


class FPSMonitor:
    """
    Dedicated FPS monitoring and analysis
    """
    
    def __init__(self, target_fps: float = 30.0, node: Node = None):
        """
        Initialize FPS monitor
        
        Args:
            target_fps: Target frame rate
            node: ROS2 node for logging
        """
        self.target_fps = target_fps
        self.target_frame_time = 1.0 / target_fps
        self.node = node
        
        # FPS tracking
        self.frame_times = deque(maxlen=300)  # 10 seconds at 30 FPS
        self.fps_history = deque(maxlen=300)
        self.last_frame_time = None
        
        # Statistics
        self.frames_processed = 0
        self.frames_skipped = 0
        self.target_achieved_count = 0
        
        # Performance zones
        self.perf_zones = {
            'excellent': (target_fps * 1.1, float('inf')),
            'good': (target_fps * 0.9, target_fps * 1.1),
            'acceptable': (target_fps * 0.7, target_fps * 0.9),
            'poor': (target_fps * 0.5, target_fps * 0.7),
            'critical': (0, target_fps * 0.5)
        }
    
    def update(self, processed: bool = True):
        """
        Update FPS monitoring
        
        Args:
            processed: Whether frame was processed or skipped
        """
        current_time = time.time()
        
        if self.last_frame_time is not None:
            frame_time = current_time - self.last_frame_time
            self.frame_times.append(frame_time)
            
            fps = 1.0 / frame_time if frame_time > 0 else 0
            self.fps_history.append(fps)
            
            if fps >= self.target_fps:
                self.target_achieved_count += 1
        
        self.last_frame_time = current_time
        
        if processed:
            self.frames_processed += 1
        else:
            self.frames_skipped += 1
    
    def get_current_fps(self) -> float:
        """Get current FPS"""
        if not self.frame_times:
            return 0.0
        
        # Use recent frames for current FPS
        recent_times = list(self.frame_times)[-30:]  # Last second
        if recent_times:
            avg_time = np.mean(recent_times)
            return 1.0 / avg_time if avg_time > 0 else 0
        return 0.0
    
    def get_performance_zone(self) -> str:
        """Get current performance zone"""
        fps = self.get_current_fps()

        for zone, (min_fps, max_fps) in self.perf_zones.items():
            if min_fps <= fps < max_fps:
                return zone

        return 'unknown'

    def get_fps(self) -> float:
        """Get current FPS (alias for get_current_fps for compatibility)"""
        return self.get_current_fps()
    
    def get_statistics(self) -> Dict:
        """Get comprehensive FPS statistics"""
        if not self.fps_history:
            return {
                'current_fps': 0,
                'target_fps': self.target_fps,
                'performance_zone': 'unknown'
            }
        
        fps_array = np.array(self.fps_history)
        
        return {
            'current_fps': self.get_current_fps(),
            'target_fps': self.target_fps,
            'mean_fps': np.mean(fps_array),
            'std_fps': np.std(fps_array),
            'min_fps': np.min(fps_array),
            'max_fps': np.max(fps_array),
            'percentile_10': np.percentile(fps_array, 10),
            'percentile_50': np.percentile(fps_array, 50),
            'percentile_90': np.percentile(fps_array, 90),
            'frames_processed': self.frames_processed,
            'frames_skipped': self.frames_skipped,
            'skip_rate': self.frames_skipped / max(1, self.frames_processed + self.frames_skipped),
            'target_achievement_rate': self.target_achieved_count / max(1, len(self.fps_history)),
            'performance_zone': self.get_performance_zone(),
            'stability': 1.0 / (1.0 + np.std(fps_array) / max(1, np.mean(fps_array)))
        }
