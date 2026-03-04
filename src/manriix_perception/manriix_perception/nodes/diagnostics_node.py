#!/usr/bin/env python3
"""
System Diagnostics Node

Monitors and reports health of the ZED multi-camera detection system.
Publishes system status and performance metrics.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import Header
import time
import psutil
from typing import Dict, Any

# Message imports will be added when messages are built


class DiagnosticsNode(Node):
    """
    System diagnostics and monitoring node
    
    Monitors:
    - Camera status and detection rates
    - Fusion performance and quality
    - System resource usage (CPU, memory)
    - Error rates and system health
    """
    
    def __init__(self):
        super().__init__('zed_system_diagnostics')
        
        # Configuration parameters
        self.declare_parameter('system_status_topic', '/zed_system/status')
        self.declare_parameter('fusion_diagnostics_topic', '/zed_system/fusion_diagnostics')
        self.declare_parameter('publish_rate', 2.0)
        self.declare_parameter('enable_performance_logging', True)
        self.declare_parameter('log_performance_every', 100)
        
        # Get parameters
        self.system_status_topic = self.get_parameter('system_status_topic').value
        self.fusion_diagnostics_topic = self.get_parameter('fusion_diagnostics_topic').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.enable_logging = self.get_parameter('enable_performance_logging').value
        self.log_interval = self.get_parameter('log_performance_every').value
        
        # System monitoring state
        self.diagnostic_data = {
            'system_start_time': time.time(),
            'total_uptime': 0.0,
            'cpu_usage': 0.0,
            'memory_usage_mb': 0.0,
            'camera_status': {},
            'fusion_metrics': {},
            'error_counts': {},
            'last_log_time': 0.0,
            'log_counter': 0
        }
        
        # QoS profile
        self.qos = QoSProfile(depth=5)
        
        # Publishers (will be uncommented when messages are available)
        # self.status_publisher = self.create_publisher(
        #     MultiCameraStatus,
        #     self.system_status_topic,
        #     self.qos
        # )
        # 
        # self.diagnostics_publisher = self.create_publisher(
        #     FusionDiagnostics,
        #     self.fusion_diagnostics_topic,
        #     self.qos
        # )
        
        # Monitoring timer
        timer_period = 1.0 / self.publish_rate
        self.create_timer(timer_period, self.monitor_system)
        
        # Performance logging timer
        if self.enable_logging:
            self.create_timer(5.0, self.log_performance)
        
        self.get_logger().info(f"System diagnostics node started")
        self.get_logger().info(f"  Status topic: {self.system_status_topic}")
        self.get_logger().info(f"  Diagnostics topic: {self.fusion_diagnostics_topic}")
        self.get_logger().info(f"  Publish rate: {self.publish_rate} Hz")
    
    def monitor_system(self):
        """Monitor system performance and health"""
        try:
            # Update system metrics
            self._update_system_metrics()
            
            # Monitor camera status (placeholder)
            self._monitor_camera_status()
            
            # Monitor fusion performance (placeholder)
            self._monitor_fusion_performance()
            
            # Publish diagnostics (when messages are available)
            # self._publish_system_status()
            # self._publish_fusion_diagnostics()
            
        except Exception as e:
            self.get_logger().error(f"Error in system monitoring: {e}")
    
    def _update_system_metrics(self):
        """Update system resource usage metrics"""
        try:
            # System uptime
            current_time = time.time()
            self.diagnostic_data['total_uptime'] = current_time - self.diagnostic_data['system_start_time']
            
            # CPU usage
            self.diagnostic_data['cpu_usage'] = psutil.cpu_percent(interval=None)
            
            # Memory usage
            memory = psutil.virtual_memory()
            self.diagnostic_data['memory_usage_mb'] = memory.used / (1024 * 1024)
            
        except Exception as e:
            self.get_logger().warning(f"Error updating system metrics: {e}")
    
    def _monitor_camera_status(self):
        """Monitor camera detection status (placeholder)"""
        # This would monitor topics like /zedx_front/zed_node/obj_det/objects
        # For now, just placeholder data
        
        camera_names = ['zedx_front', 'zedx_left', 'zedx_right']
        
        for camera_name in camera_names:
            if camera_name not in self.diagnostic_data['camera_status']:
                self.diagnostic_data['camera_status'][camera_name] = {
                    'active': False,
                    'detection_rate': 0.0,
                    'last_detection_time': 0.0,
                    'total_detections': 0,
                    'error_count': 0
                }
        
        # TODO: Implement actual camera monitoring by subscribing to camera topics
    
    def _monitor_fusion_performance(self):
        """Monitor fusion algorithm performance (placeholder)"""
        # This would monitor fusion node performance
        # For now, just placeholder data
        
        self.diagnostic_data['fusion_metrics'] = {
            'input_detections_per_sec': 0.0,
            'output_detections_per_sec': 0.0,
            'fusion_latency_ms': 0.0,
            'sync_success_rate': 0.0,
            'spatial_fusion_efficiency': 0.0
        }
        
        # TODO: Implement actual fusion monitoring by subscribing to fusion diagnostics
    
    def log_performance(self):
        """Log performance metrics periodically"""
        if not self.enable_logging:
            return
        
        current_time = time.time()
        self.diagnostic_data['log_counter'] += 1
        
        # Log every N intervals or every 30 seconds
        should_log = (
            self.diagnostic_data['log_counter'] >= self.log_interval or
            current_time - self.diagnostic_data['last_log_time'] >= 30.0
        )
        
        if should_log:
            self._log_system_performance()
            self.diagnostic_data['last_log_time'] = current_time
            self.diagnostic_data['log_counter'] = 0
    
    def _log_system_performance(self):
        """Log current system performance"""
        data = self.diagnostic_data
        
        self.get_logger().info("=== ZED SYSTEM DIAGNOSTICS ===")
        
        # System metrics
        uptime_hours = data['total_uptime'] / 3600
        self.get_logger().info(
            f"System: Uptime {uptime_hours:.1f}h, "
            f"CPU {data['cpu_usage']:.1f}%, "
            f"Memory {data['memory_usage_mb']:.1f}MB"
        )
        
        # Camera status
        camera_count = len(data['camera_status'])
        active_cameras = sum(1 for status in data['camera_status'].values() if status['active'])
        self.get_logger().info(f"Cameras: {active_cameras}/{camera_count} active")
        
        for camera_name, status in data['camera_status'].items():
            status_str = "ACTIVE" if status['active'] else "INACTIVE"
            self.get_logger().info(
                f"  {camera_name}: {status_str} "
                f"(rate: {status['detection_rate']:.1f}Hz, "
                f"detections: {status['total_detections']}, "
                f"errors: {status['error_count']})"
            )
        
        # Fusion metrics
        fusion = data['fusion_metrics']
        self.get_logger().info(
            f"Fusion: Input {fusion['input_detections_per_sec']:.1f}/s, "
            f"Output {fusion['output_detections_per_sec']:.1f}/s, "
            f"Latency {fusion['fusion_latency_ms']:.1f}ms, "
            f"Sync {fusion['sync_success_rate']:.1%}, "
            f"Efficiency {fusion['spatial_fusion_efficiency']:.1%}"
        )
        
        self.get_logger().info("==============================")
    
    def get_system_health_summary(self) -> Dict[str, Any]:
        """Get overall system health summary"""
        data = self.diagnostic_data
        
        # Determine overall health status
        health_status = "good"
        issues = []
        
        # Check CPU usage
        if data['cpu_usage'] > 80.0:
            health_status = "degraded"
            issues.append(f"High CPU usage: {data['cpu_usage']:.1f}%")
        
        # Check memory usage
        if data['memory_usage_mb'] > 4000:  # 4GB threshold
            health_status = "degraded"
            issues.append(f"High memory usage: {data['memory_usage_mb']:.1f}MB")
        
        # Check camera status
        total_cameras = len(data['camera_status'])
        active_cameras = sum(1 for status in data['camera_status'].values() if status['active'])
        
        if active_cameras < total_cameras:
            if active_cameras == 0:
                health_status = "critical"
                issues.append("No cameras active")
            else:
                health_status = "degraded"
                issues.append(f"Only {active_cameras}/{total_cameras} cameras active")
        
        return {
            'status': health_status,
            'issues': issues,
            'uptime_hours': data['total_uptime'] / 3600,
            'cpu_usage': data['cpu_usage'],
            'memory_usage_mb': data['memory_usage_mb'],
            'active_cameras': active_cameras,
            'total_cameras': total_cameras
        }


def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)
    
    try:
        node = DiagnosticsNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error in diagnostics node: {e}")
    finally:
        if 'node' in locals():
            node.destroy_node()
        try:
            rclpy.shutdown()
        except:
            pass


if __name__ == '__main__':
    main()