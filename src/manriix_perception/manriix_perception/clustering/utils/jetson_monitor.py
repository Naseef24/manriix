#!/usr/bin/env python3
"""
Real-time performance monitoring for Jetson AGX Orin
Tracks GPU, CPU, memory, thermal, and FPS metrics
"""

import time
import threading
import psutil
import numpy as np
from collections import deque
from datetime import datetime
import subprocess
import os
from typing import Dict
from rclpy.node import Node

try:
    import cupy as cp
    CUPY_AVAILABLE = True
except ImportError:
    CUPY_AVAILABLE = False


class JetsonMonitor:
    """
    Real-time performance monitoring for Jetson AGX Orin
    Publishes metrics for debugging without affecting main functionality
    """
    
    def __init__(self, history_size=100, publish_rate=1.0, node: Node = None):
        """
        Initialize Jetson performance monitor
        
        Args:
            history_size: Number of samples to keep in history
            publish_rate: Rate to publish metrics (Hz)
            node: ROS2 node for logging
        """
        self.history_size = history_size
        self.publish_rate = publish_rate
        self.running = False
        self.monitor_thread = None
        self.node = node
        
        if not CUPY_AVAILABLE and self.node:
            self.node.get_logger().warn("CuPy not available - GPU memory monitoring disabled")
        
        # Metrics storage
        self.gpu_usage = deque(maxlen=history_size)
        self.gpu_memory = deque(maxlen=history_size)
        self.cpu_usage = deque(maxlen=history_size)
        self.cpu_memory = deque(maxlen=history_size)
        self.temperature = deque(maxlen=history_size)
        self.power = deque(maxlen=history_size)
        self.fps_history = deque(maxlen=history_size)
        self.timestamps = deque(maxlen=history_size)
        
        # Current metrics
        self.current_fps = 0
        self.current_processing_time = 0
        
        # Tegrastats parser
        self.tegrastats_process = None
        self.tegrastats_data = {}
        
        if self.node:
            self.node.get_logger().info("JetsonMonitor initialized")
    
    def parse_tegrastats_line(self, line: str) -> Dict:
        """Parse a line from tegrastats output"""
        
        data = {}
        
        try:
            # Parse RAM usage
            if 'RAM' in line:
                ram_part = line.split('RAM')[1].split('/')[0].strip()
                ram_used = int(ram_part.replace('MB', ''))
                data['ram_mb'] = ram_used
            
            # Parse CPU usage
            if 'CPU [' in line:
                cpu_part = line.split('CPU [')[1].split(']')[0]
                cpu_values = [int(x.replace('%', '')) for x in cpu_part.split(',') if '%' in x]
                if cpu_values:
                    data['cpu_percent'] = np.mean(cpu_values)
            
            # Parse GPU usage
            if 'GR3D_FREQ' in line:
                gpu_part = line.split('GR3D_FREQ')[1].split('%')[0].strip()
                data['gpu_percent'] = float(gpu_part)
            
            # Parse temperature
            if 'temp' in line.lower():
                # Extract all temperatures
                import re
                temps = re.findall(r'(\d+\.?\d*)C', line)
                if temps:
                    data['temp_c'] = float(temps[0])
            
            # Parse power
            if 'VDD_IN' in line:
                power_part = line.split('VDD_IN')[1].split('/')[0].strip()
                power_mw = int(power_part.replace('mW', ''))
                data['power_w'] = power_mw / 1000.0
                
        except Exception as e:
            if self.node:
                self.node.get_logger().debug(f"Error parsing tegrastats: {e}")
        
        return data
    
    def start_tegrastats(self):
        """Start tegrastats subprocess"""
        
        try:
            self.tegrastats_process = subprocess.Popen(
                ['tegrastats', '--interval', '1000'],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                bufsize=1
            )
            
            # Start reader thread
            def read_tegrastats():
                while self.running and self.tegrastats_process:
                    try:
                        line = self.tegrastats_process.stdout.readline()
                        if line:
                            self.tegrastats_data = self.parse_tegrastats_line(line)
                    except:
                        break
            
            reader_thread = threading.Thread(target=read_tegrastats)
            reader_thread.daemon = True
            reader_thread.start()
            
        except Exception as e:
            if self.node:
                self.node.get_logger().warn(f"Could not start tegrastats: {e}")
    
    def stop_tegrastats(self):
        """Stop tegrastats subprocess"""
        
        if self.tegrastats_process:
            self.tegrastats_process.terminate()
            self.tegrastats_process = None
    
    def get_cuda_memory(self) -> Dict:
        """Get CUDA memory usage using CuPy"""
        
        if not CUPY_AVAILABLE:
            return {'used_gb': 0, 'total_gb': 0, 'percent': 0}
        
        try:
            mempool = cp.get_default_memory_pool()
            used = mempool.used_bytes() / (1024**3)  # GB
            total = mempool.total_bytes() / (1024**3)  # GB
            
            # Try to get total GPU memory
            if total == 0:
                # Approximate for Jetson AGX Orin 64GB
                total = 62.0  # Shared memory
            
            percent = (used / total * 100) if total > 0 else 0
            
            return {
                'used_gb': used,
                'total_gb': total,
                'percent': percent
            }
        except Exception as e:
            if self.node:
                self.node.get_logger().debug(f"Error getting CUDA memory: {e}")
            return {'used_gb': 0, 'total_gb': 0, 'percent': 0}
    
    def get_system_memory(self) -> Dict:
        """Get system memory usage"""
        
        try:
            mem = psutil.virtual_memory()
            return {
                'used_gb': mem.used / (1024**3),
                'total_gb': mem.total / (1024**3),
                'percent': mem.percent
            }
        except:
            return {'used_gb': 0, 'total_gb': 0, 'percent': 0}
    
    def monitor_loop(self):
        """Continuous monitoring loop"""
        
        while self.running:
            try:
                timestamp = datetime.now()
                
                # Get system metrics
                cpu_percent = psutil.cpu_percent(interval=0.1)
                sys_mem = self.get_system_memory()
                cuda_mem = self.get_cuda_memory()
                
                # Get tegrastats metrics
                gpu_percent = self.tegrastats_data.get('gpu_percent', 0)
                temp_c = self.tegrastats_data.get('temp_c', 0)
                power_w = self.tegrastats_data.get('power_w', 0)
                
                # Store metrics
                self.timestamps.append(timestamp)
                self.cpu_usage.append(cpu_percent)
                self.cpu_memory.append(sys_mem['used_gb'])
                self.gpu_usage.append(gpu_percent)
                self.gpu_memory.append(cuda_mem['used_gb'])
                self.temperature.append(temp_c)
                self.power.append(power_w)
                self.fps_history.append(self.current_fps)
                
                # Sleep for next sample
                time.sleep(1.0 / self.publish_rate)
                
            except Exception as e:
                if self.node:
                    self.node.get_logger().debug(f"Monitor loop error: {e}")
                time.sleep(1.0)
    
    def start(self):
        """Start performance monitoring"""
        
        if not self.running:
            self.running = True
            
            # Start tegrastats
            self.start_tegrastats()
            
            # Start monitor thread
            self.monitor_thread = threading.Thread(target=self.monitor_loop)
            self.monitor_thread.daemon = True
            self.monitor_thread.start()
            
            if self.node:
                self.node.get_logger().info("Jetson performance monitoring started")
    
    def stop(self):
        """Stop performance monitoring"""
        
        self.running = False
        
        # Stop tegrastats
        self.stop_tegrastats()
        
        # Wait for thread to finish
        if self.monitor_thread:
            self.monitor_thread.join(timeout=2.0)
        
        if self.node:
            self.node.get_logger().info("Jetson performance monitoring stopped")
    
    def update_fps(self, fps: float, processing_time: float):
        """Update FPS metrics from main processing loop"""
        
        self.current_fps = fps
        self.current_processing_time = processing_time
    
    def get_current_stats(self) -> Dict:
        """Get current performance statistics"""
        
        stats = {
            'cpu_percent': self.cpu_usage[-1] if self.cpu_usage else 0,
            'gpu_percent': self.gpu_usage[-1] if self.gpu_usage else 0,
            'cpu_memory_gb': self.cpu_memory[-1] if self.cpu_memory else 0,
            'gpu_memory_gb': self.gpu_memory[-1] if self.gpu_memory else 0,
            'temperature_c': self.temperature[-1] if self.temperature else 0,
            'power_w': self.power[-1] if self.power else 0,
            'fps': self.current_fps,
            'processing_time_ms': self.current_processing_time * 1000
        }
        
        return stats
    
    def get_summary_stats(self) -> Dict:
        """Get summary statistics over history"""
        
        if not self.cpu_usage:
            return {}
        
        stats = {
            'cpu_avg': np.mean(self.cpu_usage),
            'cpu_max': np.max(self.cpu_usage),
            'gpu_avg': np.mean(self.gpu_usage) if self.gpu_usage else 0,
            'gpu_max': np.max(self.gpu_usage) if self.gpu_usage else 0,
            'memory_avg_gb': np.mean(self.cpu_memory),
            'memory_max_gb': np.max(self.cpu_memory),
            'temp_avg_c': np.mean(self.temperature) if self.temperature else 0,
            'temp_max_c': np.max(self.temperature) if self.temperature else 0,
            'power_avg_w': np.mean(self.power) if self.power else 0,
            'fps_avg': np.mean(self.fps_history) if self.fps_history else 0,
            'fps_min': np.min(self.fps_history) if self.fps_history else 0
        }
        
        return stats
    
    def print_stats(self):
        """Print current stats to console"""
        
        stats = self.get_current_stats()
        
        print(f"CPU: {stats['cpu_percent']:.1f}% | "
              f"GPU: {stats['gpu_percent']:.1f}% | "
              f"Mem: {stats['cpu_memory_gb']:.1f}GB | "
              f"Temp: {stats['temperature_c']:.1f}°C | "
              f"FPS: {stats['fps']:.1f}")
    
    def check_thermal_throttling(self) -> bool:
        """Check if system is thermally throttling"""
        
        if self.temperature and self.temperature[-1] > 85:
            if self.node:
                self.node.get_logger().warn(f"High temperature detected: {self.temperature[-1]}°C")
            return True
        return False
    
    def check_memory_pressure(self) -> bool:
        """Check if system is under memory pressure"""

        sys_mem = self.get_system_memory()
        if sys_mem['percent'] > 90:
            if self.node:
                self.node.get_logger().warn(f"High memory usage: {sys_mem['percent']:.1f}%")
            return True
        return False

    def get_metrics(self) -> Dict:
        """Get current metrics (alias for compatibility)"""
        metrics = self.get_current_stats()
        # Add gpu_usage field for compatibility
        metrics['gpu_usage'] = metrics.get('gpu_percent', 0)
        return metrics