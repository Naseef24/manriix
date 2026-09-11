#!/usr/bin/env python3
"""
Human Clustering Node - ROS2 Version
Direct conversion from ROS1 human_clustering_node.py
"""

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import yaml
import os
import numpy as np
from typing import Dict, List, Optional, Tuple
import threading
from collections import deque
from datetime import datetime
import sys
import gc
import psutil
import time

# Add the package lib directory to path for local imports

from manriix_perception.msg import HumansList
from manriix_perception.msg import OptimalPosition
from std_msgs.msg import String

# Import local modules from src subdirectory
from manriix_perception.clustering.world_transform import WorldTransform
from manriix_perception.clustering.cluster_detector import ClusterDetector
from manriix_perception.clustering.cluster_tracker import ClusterTracker
# Position optimization now handled by POI Manager
from manriix_perception.clustering.viz_2d import TopView2DVisualizer
from manriix_perception.clustering.performance_utils import MemoryManager, ParallelProcessor
from manriix_perception.clustering.map_handler import MapHandler

# Import optimization components
from manriix_perception.clustering.optimization import (
    JITCompiler, OptimizedOperations,
    FrameProcessor, AdaptiveProcessor,
    MemoryOptimizer, DataLocalityOptimizer,
    QualityController, QualityLevel, PerformanceBalancer,
    PerformanceProfiler, FPSMonitor
)
from manriix_perception.clustering.utils.jetson_monitor import JetsonMonitor

from manriix_perception.msg import ClusterArray, Cluster
from geometry_msgs.msg import Point, PoseStamped, Vector3
from manriix_perception.msg import TransformedHuman, TransformedHumansArray
from manriix_perception.msg import RobotStatus
from manriix_perception.msg import PerformanceMetrics, ClusterTracking
from std_msgs.msg import Float32, Bool, ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray


class HumanClusteringNode(Node):
    """Main node"""
    
    def __init__(self):
        super().__init__('human_clustering_node')
        self.get_logger().info("Starting Human Clustering Node...")
        self.declare_parameter('config_path', '')
        
        config_path = self.get_parameter('config_path').get_parameter_value().string_value
        if not config_path:
            from ament_index_python.packages import get_package_share_directory
            try:
                package_share = get_package_share_directory('manriix_perception')
                config_path = os.path.join(package_share, 'config', 'clustering_params.yaml')
            except Exception:
                config_path = 'config/clustering_params.yaml'

        self.get_logger().info(f"Loading config from: {config_path}")
        self.config = self.load_config(config_path)
        if not self.config:
            self.get_logger().error("Failed to load config file")
            return
        if not self.validate_config():
            self.get_logger().error("Configuration validation failed")
            return

        self.init_optimization_layer()
        self.memory_manager = MemoryManager(max_history=50)
        self.parallel_processor = ParallelProcessor(node=self)
        self.memory_manager.register_buffer('humans')
        self.memory_manager.register_buffer('clusters')
        self.memory_manager.register_buffer('optimal_positions')
        self.initialize_components()
        self.initialize_publishers()

        self.state_lock = threading.RLock()
        self.current_humans = {}
        self.current_clusters = []
        self.optimal_positions = []
        self.robot_position = None
        self.robot_orientation = None
        self.is_moving = False
        self.current_optimal_position = None
        self.position_locked = False
        self.publish_allowed = True
        self.is_scanning = False
        self.scan_start_time = None
        self.robot_stopped = False
        self.last_position = None
        self.last_position_time = None

        target_config = self.config.get('target_detection', {})
        self.scan_region_radius = target_config.get('scan_region_radius', 1.0)
        self.position_change_threshold = target_config.get('position_change_threshold', 0.05)
        self.position_stable_duration = Duration(seconds=target_config.get('position_stable_duration', 0.5))
        self.scan_duration = Duration(seconds=target_config.get('scan_duration', 120.0))
        self.enable_position_stability_check = target_config.get('enable_position_stability_check', True)
        self.check_orientation = target_config.get('check_orientation', False)
        if self.check_orientation:
            self.orientation_tolerance_degrees = target_config.get('orientation_tolerance_degrees', 15.0)
            self.orientation_tolerance_rad = np.radians(self.orientation_tolerance_degrees)

        recovery_config = self.config.get('recovery', {})
        self.recovery_state = "NORMAL"
        self.current_scan_angle_index = 0
        self.last_human_data_time = self.get_clock().now()
        self.recovery_attempts = 0
        self.orientation_angles = []
        self.no_data_timeout = Duration(seconds=recovery_config.get('no_data_timeout', 30.0))
        self.orientation_wait_time = Duration(seconds=recovery_config.get('orientation_wait_time', 3.0))
        self.max_recovery_attempts = recovery_config.get('max_recovery_attempts', 3)
        num_angles = recovery_config.get('orientation_scan_angles', 6)
        for i in range(num_angles):
            self.orientation_angles.append((2 * np.pi * i) / num_angles)

        self.start_scan_subscriber_available = False
        self.last_subscriber_check = self.get_clock().now()
        self.subscriber_check_interval = Duration(seconds=2.0)
        self.no_subscriber_move_timer = None
        self.no_subscriber_wait_duration = Duration(seconds=5.0)
        self.current_position_index = 0
        self.frame_count = 0
        self.process_every_n_frames = self.calculate_frame_skip_rate()
        self.processing_times = deque(maxlen=50)
        self.last_cleanup_time = self.get_clock().now()
        self.performance_monitor = PerformanceMonitor(node=self)
        self.last_valid_message_time = self.get_clock().now()
        self.message_timeout = Duration(seconds=5.0)

        self.initialize_subscribers()
        self.status_timer = self.create_timer(0.2, self.publish_status)
        self.get_logger().info("Human clustering node initialized successfully") 
               
    def init_optimization_layer(self):
        """Initialize all optimization components"""
        self.get_logger().info("Initializing optimization layer...")

        # Read config settings
        gpu_config = self.config.get('gpu', {})
        jit_config = self.config.get('jit', {})
        quality_config = self.config.get('quality_control', {})

        use_gpu = gpu_config.get('enabled', False)
        use_jit = jit_config.get('enabled', False)
        target_fps = quality_config.get('target_fps', 30.0)
        min_fps = quality_config.get('min_fps', 15.0)

        self.get_logger().info(f"Optimization config - GPU: {use_gpu}, JIT: {use_jit}, Target FPS: {target_fps}")

        # Initialize baseline CPU percentage (non-blocking after first call)
        try:
            _ = psutil.cpu_percent(interval=0.1)  # Quick initial measurement
        except Exception:
            pass

        # JIT Compilation
        self.jit_ops = OptimizedOperations(
            use_jit=use_jit,
            use_gpu=use_gpu,
            node=self
        )

        # Frame Processing
        self.frame_processor = FrameProcessor(
            target_fps=target_fps,
            min_fps=min_fps,
            node=self
        )
        self.adaptive_processor = AdaptiveProcessor(node=self)

        # Memory Optimization
        self.memory_optimizer = MemoryOptimizer(
            max_memory_mb=gpu_config.get('memory_limit', 4096),
            node=self
        )
        self.locality_optimizer = DataLocalityOptimizer(node=self)

        # Quality Control
        self.quality_controller = QualityController(target_fps=target_fps, node=self)
        self.performance_balancer = PerformanceBalancer(node=self)

        # Performance Profiling
        self.profiler = PerformanceProfiler(enable_detailed=False, node=self)
        self.fps_monitor = FPSMonitor(target_fps=target_fps, node=self)

        # Jetson Monitoring
        self.jetson_monitor = JetsonMonitor(node=self)

        # Start Jetson monitoring in background
        try:
            self.jetson_monitor.start()
        except Exception as e:
            self.get_logger().warn(f"Could not start Jetson monitoring: {e}")

        self.get_logger().info("Optimization layer initialized successfully")
    
    def setup_jetson_optimizations(self):
        """Setup Jetson-specific optimizations"""
        try:
            # Set CPU governor to performance mode if possible
            os.system("echo performance | sudo tee /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor > /dev/null 2>&1")
            
            # Configure garbage collection for real-time performance
            gc.set_threshold(700, 10, 10)  # More aggressive GC for memory-constrained system
            
            # Set process priority
            os.nice(-10)  # Higher priority (requires appropriate permissions)
            
        except Exception as e:
            self.get_logger().warn(f"Could not apply all Jetson optimizations: {e}")
    
    def calculate_frame_skip_rate(self) -> int:
        """Calculate optimal frame skip rate based on system performance"""
        try:
            # Get CPU usage - NON-BLOCKING (interval=None uses cached value)
            cpu_usage = psutil.cpu_percent(interval=None)

            # Adaptive frame skipping based on load
            if cpu_usage > 80:
                return 4  # Skip 3 out of 4 frames
            elif cpu_usage > 60:
                return 3  # Skip 2 out of 3 frames
            elif cpu_usage > 40:
                return 2  # Skip every other frame
            else:
                return 1  # Process every frame when load is low

        except Exception:
            return 2  # Default fallback
    
    def validate_config(self) -> bool:
        """Validate configuration parameters"""
        try:
            required_sections = ['transform', 'target_detection', 'recovery', 'movement']
            
            for section in required_sections:
                if section not in self.config:
                    self.get_logger().error(f"Missing required config section: {section}")
                    return False
            
            # Validate numeric parameters
            target_config = self.config.get('target_detection', {})
            if target_config.get('scan_duration', 0) <= 0:
                self.get_logger().error("scan_duration must be positive")
                return False
                
            if target_config.get('scan_region_radius', 0) <= 0:
                self.get_logger().error("scan_region_radius must be positive")
                return False
            
            return True
            
        except Exception as e:
            self.get_logger().error(f"Config validation error: {e}")
            return False

    def initialize_components(self):
        """Initialize all components with error handling"""
        try:
            self.get_logger().info("Initializing World Transform...")
            self.world_transform = WorldTransform(self.config, node=self)

            self.get_logger().info("Initializing Cluster Detector...")
            self.cluster_detector = ClusterDetector(self.config, node=self)

            self.get_logger().info("Initializing Cluster Tracker...")
            self.cluster_tracker = ClusterTracker(self.config, node=self)

            # Position optimization now handled by POI Manager

            self.get_logger().info("Initializing Map Handler...")
            self.map_handler = MapHandler(self.config, node=self)

            # Initialize matplotlib visualization (controlled by config)
            self.visualizer = None

            if self.config.get('visualization', {}).get('enabled', False):
                self.get_logger().info("Initializing Matplotlib 2D Visualizer...")
                try:
                    self.visualizer = TopView2DVisualizer(
                        max_range=self.config['visualization'].get('matplotlib', {}).get('max_range', 8.0),
                        node=self
                    )
                except Exception as e:
                    self.get_logger().warn(f"Failed to initialize matplotlib visualizer: {e}")
            else:
                self.get_logger().info("Local visualization disabled via config")
        except Exception as e:
            self.get_logger().error(f"Failed to initialize components: {e}")
            raise

    def initialize_publishers(self):
        """Initialize all publishers"""
        try:
            qos = QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                history=HistoryPolicy.KEEP_LAST,
                depth=5
            )
            qos_best_effort = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST,
                depth=3
            )
            
            # Keep optimal position for internal use
            self.optimal_pos_pub = self.create_publisher(
                OptimalPosition,
                '/human_clustering/optimal_position',
                qos
            )
            
            self.start_scan_pub = self.create_publisher(
                String,
                '/human_clustering/start_scan',
                qos
            )

            self.clusters_pub = self.create_publisher(
                ClusterArray,
                '/human_clustering/clusters',
                qos
            )

            self.transformed_humans_pub = self.create_publisher(
                TransformedHumansArray,
                '/human_clustering/transformed_positions',
                qos
            )

            self.robot_status_pub = self.create_publisher(
                RobotStatus,
                '/human_clustering/robot_status',
                qos
            )
            
            # Performance monitoring publishers
            # Performance monitoring publishers — BEST_EFFORT: high rate, loss acceptable
            self.performance_pub = self.create_publisher(
                PerformanceMetrics,
                '/human_clustering/performance',
                qos_best_effort
            )
            self.fps_pub = self.create_publisher(
                Float32,
                '/human_clustering/fps',
                qos_best_effort
            )
            self.tracking_pub = self.create_publisher(
                ClusterTracking,
                '/human_clustering/tracking',
                qos
            )

            # Web interface publishers (standard message types for ROSBridge)
            self.web_clusters_pub = self.create_publisher(
                MarkerArray,
                '/web/clusters',
                qos_best_effort
            )
            self.web_humans_pub = self.create_publisher(
                MarkerArray,
                '/web/humans',
                qos_best_effort
            )
            self.web_optimal_pub = self.create_publisher(
                Marker,
                '/web/optimal_position',
                qos_best_effort
            )
            self.get_logger().info("Web interface publishers initialized (/web/*)")
        except Exception as e:
            self.get_logger().error(f"Failed to initialize publishers: {e}")
            raise

    def initialize_subscribers(self):
        """Initialize subscribers"""
        try:
            qos = QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                history=HistoryPolicy.KEEP_LAST,
                depth=5
            )
            
            self.human_sub = self.create_subscription(
                HumansList,
                '/human_clustering/humans',
                self.human_callback,
                qos
            )
        except Exception as e:
            self.get_logger().error(f"Failed to initialize subscribers: {e}")
            raise
    
    def publish_start_scan(self, message: str):
        """Publish start scan message with validation"""
        try:
            if message not in ["yes", "no"]:
                self.get_logger().warn(f"Invalid scan message: {message}")
                return
                
            msg = String()
            msg.data = message
            self.start_scan_pub.publish(msg)
            self.get_logger().info(f"Published scan command: {message}")
        except Exception as e:
            self.get_logger().error(f"Error publishing start scan: {e}")

    def publish_cluster_data(self, clusters: List[Dict]):
        """Publish cluster data with validation"""
        if not clusters:
            return
            
        try:
            msg = ClusterArray()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "map"
            
            for cluster in clusters:
                if not self.validate_cluster(cluster):
                    continue
                    
                cluster_msg = Cluster()
                cluster_msg.id = cluster.get('track_id', 0)
                
                cluster_msg.center = Point()
                cluster_msg.center.x = float(cluster['center'][0])
                cluster_msg.center.y = float(cluster['center'][1])
                cluster_msg.center.z = 0.0
                
                cluster_msg.radius = float(cluster['radius'])
                cluster_msg.size = int(cluster['size'])
                cluster_msg.formation = cluster.get('formation', "unknown")
                cluster_msg.stability = float(cluster.get('stability', 0.5))
                cluster_msg.density = float(cluster.get('density', 0.0))
                    
                msg.clusters.append(cluster_msg)
                
            self.clusters_pub.publish(msg)
            
        except Exception as e:
            self.get_logger().error(f"Error publishing cluster data: {e}")
    
    def validate_cluster(self, cluster: Dict) -> bool:
        """Validate cluster data including NaN checks"""
        try:
            required_fields = ['center', 'radius', 'size']
            for field in required_fields:
                if field not in cluster:
                    return False

            # Check for valid center array
            if not isinstance(cluster['center'], (list, tuple, np.ndarray)) or len(cluster['center']) < 2:
                return False

            # Check for NaN/Inf in center coordinates
            center_x = float(cluster['center'][0])
            center_y = float(cluster['center'][1])

            if np.isnan(center_x) or np.isnan(center_y):
                self.get_logger().warn("Cluster rejected: NaN in center coordinates")
                return False

            if np.isinf(center_x) or np.isinf(center_y):
                self.get_logger().warn("Cluster rejected: Inf in center coordinates")
                return False

            # Check for unreasonable coordinates (>1km from origin)
            if abs(center_x) > 1000 or abs(center_y) > 1000:
                self.get_logger().warn(f"Cluster rejected: unreasonable center ({center_x}, {center_y})")
                return False

            # Check radius and size
            if cluster['radius'] <= 0 or cluster['size'] <= 0:
                return False

            # Check radius for NaN
            if np.isnan(cluster['radius']) or np.isinf(cluster['radius']):
                return False

            return True
        except Exception as e:
            self.get_logger().debug(f"Cluster validation error: {e}")
            return False

    def _publish_web_data(self, vis_positions: Dict, vis_optimal: List):
        """Publish data to web interface using standard message types for ROSBridge"""
        try:
            now = self.get_clock().now().to_msg()
            frame_id = "map"

            # Publish humans as MarkerArray
            humans_msg = MarkerArray()
            delete_marker = Marker()
            delete_marker.header.frame_id = frame_id
            delete_marker.action = Marker.DELETEALL
            humans_msg.markers.append(delete_marker)

            for i, (human_id, pos) in enumerate(vis_positions.items()):
                marker = Marker()
                marker.header.frame_id = frame_id
                marker.header.stamp = now
                marker.ns = "humans"
                marker.id = i
                marker.type = Marker.CYLINDER
                marker.action = Marker.ADD
                marker.pose.position.x = float(pos[0])
                marker.pose.position.y = float(pos[1])
                marker.pose.position.z = 0.9
                marker.pose.orientation.w = 1.0
                marker.scale = Vector3(x=0.4, y=0.4, z=1.8)
                marker.color = ColorRGBA(r=0.34, g=0.65, b=1.0, a=0.9)  # Blue color for humans
                humans_msg.markers.append(marker)

            self.web_humans_pub.publish(humans_msg)

            # Publish clusters as MarkerArray
            clusters_msg = MarkerArray()
            delete_marker2 = Marker()
            delete_marker2.header.frame_id = frame_id
            delete_marker2.action = Marker.DELETEALL
            clusters_msg.markers.append(delete_marker2)

            if self.current_clusters:
                for i, cluster in enumerate(self.current_clusters):
                    marker = Marker()
                    marker.header.frame_id = frame_id
                    marker.header.stamp = now
                    marker.ns = "clusters"
                    marker.id = i
                    marker.type = Marker.CYLINDER
                    marker.action = Marker.ADD

                    # Extract cluster center - handle numpy arrays, lists, tuples
                    center = cluster.get('center', None)
                    if center is not None and hasattr(center, '__len__') and len(center) >= 2:
                        cx = float(center[0])
                        cy = float(center[1])
                    else:
                        cx = float(cluster.get('x', 0))
                        cy = float(cluster.get('y', 0))

                    radius = cluster.get('radius', 1.0)
                    size = cluster.get('size', 1)

                    marker.pose.position.x = float(cx)
                    marker.pose.position.y = float(cy)
                    marker.pose.position.z = 0.1
                    marker.pose.orientation.w = 1.0
                    marker.scale = Vector3(x=float(radius * 2), y=float(radius * 2), z=0.1)
                    marker.color = ColorRGBA(r=0.2, g=0.7, b=0.9, a=0.4)
                    marker.text = str(size)
                    clusters_msg.markers.append(marker)

            self.web_clusters_pub.publish(clusters_msg)

            # Publish optimal position as Marker
            if vis_optimal and len(vis_optimal) > 0:
                opt = vis_optimal[0]

                # Extract position - handle numpy arrays, lists, tuples, dicts
                position = opt.get('position', None)
                if position is not None and hasattr(position, '__len__') and len(position) >= 2:
                    opt_x = float(position[0])
                    opt_y = float(position[1])
                else:
                    opt_x = float(opt.get('x', 0))
                    opt_y = float(opt.get('y', 0))

                score = opt.get('score', opt.get('priority_score', 1.0))

                opt_marker = Marker()
                opt_marker.header.frame_id = frame_id
                opt_marker.header.stamp = now
                opt_marker.ns = "optimal"
                opt_marker.id = 0
                opt_marker.type = Marker.SPHERE
                opt_marker.action = Marker.ADD
                opt_marker.pose.position.x = float(opt_x)
                opt_marker.pose.position.y = float(opt_y)
                opt_marker.pose.position.z = 0.3
                opt_marker.pose.orientation.w = 1.0
                size = 0.5 + float(score) * 0.3
                opt_marker.scale = Vector3(x=size, y=size, z=size)
                opt_marker.color = ColorRGBA(r=1.0, g=0.8, b=0.2, a=0.9)

                self.web_optimal_pub.publish(opt_marker)

        except Exception as e:
            self.get_logger().debug(f"Error publishing web data: {e}")

    def publish_transformed_positions(self, human_positions: Dict):
        """Publish human positions with validation"""
        if not human_positions:
            return
            
        try:
            msg = TransformedHumansArray()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "map"
            
            for human_id, data in human_positions.items():
                if not self.validate_human_position(data):
                    continue
                    
                human_msg = TransformedHuman()
                human_msg.tracking_id = human_id
                
                map_pos = data['map_position']
                human_msg.position = Point(
                    x=float(map_pos[0]),
                    y=float(map_pos[1]),
                    z=0.0
                )
                
                msg.humans.append(human_msg)
                
            self.transformed_humans_pub.publish(msg)
            
        except Exception as e:
            self.get_logger().error(f"Error publishing transformed positions: {e}")
    
    def validate_human_position(self, data: Dict) -> bool:
        """Validate human position data"""
        try:
            if 'map_position' not in data:
                return False
                
            pos = data['map_position']
            if not isinstance(pos, (list, tuple, np.ndarray)) or len(pos) < 2:
                return False
                
            # Check for reasonable position values
            if abs(pos[0]) > 1000 or abs(pos[1]) > 1000:  # 1km limit
                return False
                
            return True
        except Exception:
            return False

    def publish_status(self):
        """Publish robot status"""
        try:
            status_msg = RobotStatus()
            status_msg.is_moving = self.is_moving
            status_msg.target_reached = self.robot_stopped
            status_msg.is_scanning = self.is_scanning
            status_msg.robot_stopped = self.robot_stopped
            status_msg.scan_duration = self.scan_duration.nanoseconds / 1e9

            self.robot_status_pub.publish(status_msg)

        except Exception as e:
            self.get_logger().error(f"Error publishing status: {e}")

    def publish_optimal_position(self):
        """Publish optimal position with validation"""
        try:
            if self.current_optimal_position is None:
                self.get_logger().warn("Cannot publish: current_optimal_position is None")
                return

            if not self.validate_optimal_position(self.current_optimal_position):
                self.get_logger().error("Invalid optimal position data")
                return

            optimal_pos_msg = OptimalPosition()
            optimal_pos_msg.x = float(self.current_optimal_position['position'][0])
            optimal_pos_msg.y = float(self.current_optimal_position['position'][1])
            optimal_pos_msg.orientation = float(self.current_optimal_position['orientation'])
            optimal_pos_msg.priority_score = float(self.current_optimal_position['priority_score'])

            self.optimal_pos_pub.publish(optimal_pos_msg)
            self.get_logger().info(f"Published optimal position: ({optimal_pos_msg.x:.2f}, {optimal_pos_msg.y:.2f})")
            
        except Exception as e:
            self.get_logger().error(f"Error publishing optimal position: {e}")
    
    def validate_optimal_position(self, position: Dict) -> bool:
        """Validate optimal position data"""
        try:
            required_fields = ['position', 'orientation', 'priority_score']
            for field in required_fields:
                if field not in position:
                    return False
            
            pos = position['position']
            if not isinstance(pos, (list, tuple, np.ndarray)) or len(pos) < 2:
                return False
                
            # Check for reasonable values
            if abs(pos[0]) > 1000 or abs(pos[1]) > 1000:
                return False
                
            return True
        except Exception:
            return False

    def check_start_scan_subscriber_availability(self):
        """Check if start_scan topic has subscribers"""
        try:
            num_subscribers = self.start_scan_pub.get_subscription_count()
            previous_status = self.start_scan_subscriber_available
            self.start_scan_subscriber_available = num_subscribers > 0

            if previous_status != self.start_scan_subscriber_available:
                if self.start_scan_subscriber_available:
                    self.get_logger().info("start_scan subscriber detected - resuming normal operation")
                    if self.recovery_state == "NO_SUBSCRIBER":
                        self.recovery_state = "NORMAL"
                        self.cancel_no_subscriber_timer()
                else:
                    self.get_logger().warn("start_scan subscriber lost - entering no-subscriber mode")
                    self.recovery_state = "NO_SUBSCRIBER"
                    self.start_no_subscriber_behavior()

        except Exception as e:
            self.get_logger().error(f"Error checking start_scan subscriber availability: {e}")

    def start_no_subscriber_behavior(self):
        """Start behavior when no start_scan subscriber available"""
        try:
            self.get_logger().info("Starting no-subscriber behavior - continuous movement mode")
            self.current_position_index = 0

            if not self.current_optimal_position and self.optimal_positions is not None and len(self.optimal_positions) > 0:
                self.current_optimal_position = self.optimal_positions[0]

            if self.current_optimal_position:
                self.is_moving = True
                self.is_scanning = False
                self.publish_optimal_position()
                self.schedule_no_subscriber_move()

        except Exception as e:
            self.get_logger().error(f"Error starting no-subscriber behavior: {e}")

    def schedule_no_subscriber_move(self):
        """Schedule next movement in no-subscriber mode"""
        try:
            if self.no_subscriber_move_timer:
                self.no_subscriber_move_timer.cancel()

            self.no_subscriber_move_timer = self.create_timer(
                self.no_subscriber_wait_duration.nanoseconds / 1e9,
                self.no_subscriber_move_callback
            )
            self.get_logger().info(f"Scheduled next move in {self.no_subscriber_wait_duration.nanoseconds / 1e9} seconds")

        except Exception as e:
            self.get_logger().error(f"Error scheduling no-subscriber move: {e}")

    def no_subscriber_move_callback(self):
        """Callback for no-subscriber movement timer"""
        try:
            # Cancel timer after execution (one-shot)
            if self.no_subscriber_move_timer:
                self.no_subscriber_move_timer.cancel()
                self.no_subscriber_move_timer = None
            
            if self.recovery_state != "NO_SUBSCRIBER":
                return

            self.current_position_index += 1

            if self.optimal_positions is not None and len(self.optimal_positions) > 1:
                if self.current_position_index >= len(self.optimal_positions):
                    self.current_position_index = 0

                next_position = self.optimal_positions[self.current_position_index]

                if self.robot_position is not None:
                    distance = np.linalg.norm(
                        next_position['position'][:2] - self.robot_position[:2]
                    )

                    if distance < self.config['recovery']['fallback_distance_threshold']:
                        attempts = 0
                        while (distance < self.config['recovery']['fallback_distance_threshold'] and 
                               attempts < len(self.optimal_positions)):
                            self.current_position_index = (self.current_position_index + 1) % len(self.optimal_positions)
                            next_position = self.optimal_positions[self.current_position_index]
                            distance = np.linalg.norm(next_position['position'][:2] - self.robot_position[:2])
                            attempts += 1

                self.current_optimal_position = next_position
            else:
                # Fallback positions are managed by POI Manager.
                # In no-subscriber mode with no clusters, nothing to do here.
                self.get_logger().warn(
                    "No valid clusters in no-subscriber mode — waiting for POI Manager"
                )
                return

            if self.current_optimal_position:
                self.get_logger().info(f"Moving to next position in no-subscriber mode: "
                            f"({self.current_optimal_position['position'][0]:.2f}, "
                            f"{self.current_optimal_position['position'][1]:.2f})")
                self.is_moving = True
                self.publish_optimal_position()
                self.schedule_no_subscriber_move()

        except Exception as e:
            self.get_logger().error(f"Error in no-subscriber move callback: {e}")

    def cancel_no_subscriber_timer(self):
        """Cancel no-subscriber movement timer"""
        try:
            if self.no_subscriber_move_timer:
                self.no_subscriber_move_timer.cancel()
                self.no_subscriber_move_timer = None
                self.get_logger().info("Cancelled no-subscriber movement timer")
        except Exception as e:
            self.get_logger().error(f"Error cancelling no-subscriber timer: {e}")

    def check_target_reached(self) -> bool:
        """Check if robot has reached target position and is fully stopped"""
        try:
            if self.current_optimal_position is None or self.robot_position is None:
                return False

            if self.recovery_state == "NO_SUBSCRIBER":
                return False

            if self.is_scanning:
                time_elapsed = self.get_clock().now() - self.scan_start_time
                if time_elapsed > self.scan_duration:
                    self.get_logger().info(f"Scan time complete ({self.scan_duration.nanoseconds / 1e9:.1f} seconds). Changing to 'no'")
                    self.is_scanning = False
                    self.publish_start_scan("no")
                    return False
                else:
                    remaining = self.scan_duration - time_elapsed
                    # Throttled logging - only log occasionally
                    if self.frame_count % 300 == 0:
                        self.get_logger().info(f"Scanning in progress. {remaining.nanoseconds / 1e9:.1f} seconds remaining.")
                    return True

            # Calculate distance to target
            target_pos = self.current_optimal_position['position'][:2]
            current_pos = self.robot_position[:2]
            distance = np.linalg.norm(target_pos - current_pos)

            # Check if within region radius
            in_region = distance < self.scan_region_radius

            # Check orientation only if enabled in config
            orientation_aligned = True
            if self.check_orientation:
                target_orientation = self.current_optimal_position['orientation']
                current_orientation = self.robot_orientation
                angle_diff = abs(target_orientation - current_orientation)
                angle_diff = min(angle_diff, 2*np.pi - angle_diff)
                orientation_aligned = angle_diff < self.orientation_tolerance_rad

            if not in_region or not orientation_aligned:
                self.robot_stopped = False
                self.last_position = None
                self.last_position_time = None
                
                if not in_region and self.frame_count % 60 == 0:
                    self.get_logger().info(f"Not in target region. Distance: {distance:.2f}m "
                                        f"(tolerance: {self.scan_region_radius:.2f}m)")
                if self.check_orientation and not orientation_aligned and self.frame_count % 60 == 0:
                    self.get_logger().info(f"Orientation not aligned. "
                                        f"Diff: {np.degrees(angle_diff):.1f}° "
                                        f"(tolerance: {self.orientation_tolerance_degrees:.1f}°)")
                
                return False

            # Handle position stability checking based on configuration
            if self.enable_position_stability_check:
                # Original stability checking logic
                current_time = self.get_clock().now()

                if self.last_position is None:
                    self.last_position = current_pos.copy()
                    self.last_position_time = current_time
                    self.get_logger().info("Entered target region, monitoring stability...")
                    return False

                position_change = np.linalg.norm(current_pos - self.last_position)
                time_stationary = current_time - self.last_position_time

                if position_change < self.position_change_threshold:
                    if time_stationary > self.position_stable_duration and not self.robot_stopped:
                        self.robot_stopped = True
                        self.get_logger().info(f"Robot has fully stopped (stable for {time_stationary.nanoseconds / 1e9:.1f}s). "
                                    f"Ready for scanning.")

                        self.get_logger().info(f"SCAN STARTED: Duration set to {self.scan_duration.nanoseconds / 1e9} seconds")
                        self.publish_start_scan("yes")
                else:
                    self.last_position = current_pos.copy()
                    self.last_position_time = current_time
                    self.robot_stopped = False
                    if self.frame_count % 60 == 0:
                        self.get_logger().info(f"Robot still moving. Position change: {position_change:.3f}m "
                                            f"(threshold: {self.position_change_threshold:.3f}m)")

                return self.robot_stopped
            else:
                # Skip stability check - immediate scanning when in region
                if not self.robot_stopped:
                    self.robot_stopped = True
                    self.get_logger().info("Target reached, starting scan immediately (stability check disabled)")
                    self.publish_start_scan("yes")
                
                return True

        except Exception as e:
            self.get_logger().error(f"Error checking target reached: {e}")
            return False

    def handle_no_data_recovery(self):
        """Handle recovery when no human data received for extended period"""
        try:
            if self.recovery_state == "NORMAL":
                if self.config['recovery']['enable_orientation_recovery']:
                    self.get_logger().info("No human data timeout - starting orientation scanning recovery")
                    self.start_orientation_scanning()
                else:
                    self.try_fallback_positioning()
            elif self.recovery_state == "ORIENTATION_SCANNING":
                self.continue_orientation_scanning()
            elif self.recovery_state == "FALLBACK":
                self.continue_fallback_positioning()

        except Exception as e:
            self.get_logger().error(f"Error in no data recovery: {e}")

    def start_orientation_scanning(self):
        """Start orientation scanning at current position"""
        try:
            self.recovery_state = "ORIENTATION_SCANNING"
            self.current_scan_angle_index = 0
            self.recovery_attempts += 1

            self.get_logger().info(f"Starting orientation scanning (attempt {self.recovery_attempts}/"
                         f"{self.max_recovery_attempts})")

            if self.current_optimal_position:
                new_orientation = self.orientation_angles[self.current_scan_angle_index]
                # Create orientation position manually since POI Manager handles this
                orientation_position = self.current_optimal_position.copy()
                orientation_position['orientation'] = new_orientation

                if orientation_position:
                    self.current_optimal_position = orientation_position
                    self.publish_optimal_position()
                    self.get_logger().info(f"Scanning orientation {self.current_scan_angle_index + 1}/"
                                f"{len(self.orientation_angles)} - {np.degrees(new_orientation):.1f}°")

        except Exception as e:
            self.get_logger().error(f"Error starting orientation scanning: {e}")

    def continue_orientation_scanning(self):
        """Continue orientation scanning sequence"""
        try:
            self.current_scan_angle_index += 1

            if self.current_scan_angle_index < len(self.orientation_angles):
                if self.current_optimal_position:
                    new_orientation = self.orientation_angles[self.current_scan_angle_index]
                    # Create orientation position manually since POI Manager handles this
                    orientation_position = self.current_optimal_position.copy()
                    orientation_position['orientation'] = new_orientation

                    if orientation_position:
                        self.current_optimal_position = orientation_position
                        self.publish_optimal_position()
                        self.get_logger().info(f"Scanning orientation {self.current_scan_angle_index + 1}/"
                                    f"{len(self.orientation_angles)} - {np.degrees(new_orientation):.1f}°")
            else:
                self.get_logger().info("Completed orientation scanning, trying fallback positioning")
                self.try_fallback_positioning()

        except Exception as e:
            self.get_logger().error(f"Error continuing orientation scanning: {e}")

    def try_fallback_positioning(self):
        """Try moving to fallback positions"""
        try:
            if not self.config['recovery']['enable_fallback_positioning']:
                self.get_logger().warn("Fallback positioning disabled - no more recovery options")
                return

            self.recovery_state = "FALLBACK"
            self.get_logger().warn(
                "Fallback recovery triggered — POI Manager will provide next positions. "
                "Resetting recovery state."
            )
            self.recovery_state = "NORMAL"
            self.recovery_attempts = 0

        except Exception as e:
            self.get_logger().error(f"Error in fallback positioning: {e}")

    def continue_fallback_positioning(self):
        """Continue fallback positioning sequence"""
        try:
            if self.recovery_attempts >= self.max_recovery_attempts:
                self.get_logger().warn("Maximum recovery attempts reached - resetting to normal operation")
                self.recovery_state = "NORMAL"
                self.recovery_attempts = 0
                return

            self.get_logger().info("Starting orientation scanning at fallback position")
            self.start_orientation_scanning()

        except Exception as e:
            self.get_logger().error(f"Error continuing fallback positioning: {e}")

    def load_config(self, config_path: str) -> Dict:
        """Load configuration from yaml file, handling both flat and ROS2-wrapped formats."""
        try:
            if not os.path.exists(config_path):
                self.get_logger().error(f"Config file not found: {config_path}")
                return {}

            with open(config_path, 'r') as f:
                raw = yaml.safe_load(f)

            if not isinstance(raw, dict):
                self.get_logger().error(f"Config file is not a YAML dict: {config_path}")
                return {}

            # Unwrap ROS2 parameter file format if present:
            #   {node_name: {ros__parameters: {actual_config}}}
            # This file is loaded programmatically so the wrapper must be stripped.
            if len(raw) == 1:
                top_key = next(iter(raw))
                inner = raw[top_key]
                if isinstance(inner, dict) and 'ros__parameters' in inner:
                    self.get_logger().info(
                        f"Detected ROS2 parameter wrapper — unwrapping '{top_key}/ros__parameters'"
                    )
                    raw = inner['ros__parameters']

            self.get_logger().info(f"Loaded configuration from {config_path}")
            return raw

        except Exception as e:
            self.get_logger().error(f"Error loading config: {e}")
            return {}
    
    def validate_message_timestamp(self, msg) -> bool:
        """Validate message timestamp to detect stale data"""
        try:
            if not hasattr(msg, 'header') or not hasattr(msg.header, 'stamp'):
                return True  # No timestamp to check
                
            msg_time = Time.from_msg(msg.header.stamp)
            current_time = self.get_clock().now()
            
            # Check if message is too old
            if (current_time - msg_time) > self.message_timeout:
                # Throttled warning
                if self.frame_count % 150 == 0:
                    self.get_logger().warn("Received stale human detection message")
                return False
                
            return True
        except Exception as e:
            self.get_logger().warn(f"Error validating message timestamp: {e}")
            return True  # Default to accepting message

    def process_human_detections(self, humans_msg: List) -> Dict:
        """Process human detections with optimizations"""
        try:
            if not humans_msg or not humans_msg.humans:
                return {}

            human_positions = {}
            
            # Optimize for Jetson - avoid parallel processing for small datasets
            if len(humans_msg.humans) > 20:  # Increased threshold
                chunks = list(self.parallel_processor.process_chunks(
                    humans_msg.humans,
                    chunk_size=8  # Optimized for Jetson cores
                ))

                def process_chunk(chunk):
                    results = {}
                    for human in chunk:
                        try:
                            map_pos = self.world_transform.transform_to_map(
                                [(human.position.x, human.position.y, human.position.z)],
                                # self.config['transform']['camera_frame']
                                self.config['transform']['base_frame']
                            )

                            if map_pos and len(map_pos) > 0:
                                # Validate map position for NaN/Inf before storing
                                mx, my = map_pos[0]
                                if np.isnan(mx) or np.isnan(my) or np.isinf(mx) or np.isinf(my):
                                    continue
                                if abs(mx) > 1000 or abs(my) > 1000:
                                    continue
                                
                                # Distance filter — same as sequential path
                                if self.robot_position is not None:
                                    max_dist = self.config.get('camera_specs', {}).get(
                                        'effective_range', {}).get('max', 8.0)
                                    human_dist = np.sqrt(
                                        (mx - self.robot_position[0])**2 +
                                        (my - self.robot_position[1])**2)
                                    if human_dist > max_dist:
                                        continue

                                results[human.tracking_id] = {
                                    'map_position': (mx, my),
                                    'robot_relative': (
                                        human.position.x,
                                        human.position.y,
                                        human.position.z
                                    )
                                }                                
                        except Exception as e:
                            self.get_logger().warn(f"Error processing human {human.tracking_id}: {e}")
                            continue
                    return results

                chunk_results = self.parallel_processor.map_async(process_chunk, chunks)
                
                for result in chunk_results:
                    human_positions.update(result)
            else:
                # Sequential processing for smaller datasets
                for human in humans_msg.humans:
                    try:
                        map_pos = self.world_transform.transform_to_map(
                            [(human.position.x, human.position.y, human.position.z)],
                            # self.config['transform']['camera_frame']
                            self.config['transform']['base_frame']
                        )

                        if map_pos and len(map_pos) > 0:
                            # Validate map position for NaN/Inf before storing
                            mx, my = map_pos[0]
                            if np.isnan(mx) or np.isnan(my) or np.isinf(mx) or np.isinf(my):
                                self.get_logger().debug(f"Filtered NaN position for human {human.tracking_id}")
                                continue
                            if abs(mx) > 1000 or abs(my) > 1000:
                                self.get_logger().debug(f"Filtered unreasonable position for human {human.tracking_id}")
                                continue
                            
                            # Distance filter — reject humans beyond effective range
                            if self.robot_position is not None:
                                max_dist = self.config.get('camera_specs', {}).get(
                                    'effective_range', {}).get('max', 8.0)
                                human_dist = np.sqrt((mx - self.robot_position[0])**2 +
                                                    (my - self.robot_position[1])**2)
                                if human_dist > max_dist:
                                    self.get_logger().debug(
                                        f"Filtered far human {human.tracking_id} at {human_dist:.1f}m "
                                        f"(max {max_dist:.1f}m)")
                                    continue

                            human_positions[human.tracking_id] = {
                                'map_position': (mx, my),
                                'robot_relative': (
                                    human.position.x,
                                    human.position.y,
                                    human.position.z
                                )
                            }
                    except Exception as e:
                        self.get_logger().warn(f"Error processing human {human.tracking_id}: {e}")
                        continue

            return human_positions

        except Exception as e:
            self.get_logger().error(f"Error processing human detections: {e}")
            return {}

    def human_callback(self, msg: HumansList):
        """Handle incoming human detection messages with edge case handling"""
        try:
            callback_start = time.perf_counter()

            # Validate message timestamp
            if not self.validate_message_timestamp(msg):
                return

            # Performance monitoring - use frame processor for intelligent skipping
            self.frame_count += 1

            # Use optimization layer for frame skipping decision
            if hasattr(self, 'frame_processor') and self.frame_processor:
                if not self.frame_processor.should_process_frame():
                    # Update FPS monitor even for skipped frames
                    if hasattr(self, 'fps_monitor'):
                        self.fps_monitor.update(processed=False)
                    return
            else:
                # Fallback to simple frame skipping
                current_frame_skip = self.calculate_frame_skip_rate()
                if self.frame_count % current_frame_skip != 0:
                    return

            # Update FPS monitor for processed frame
            if hasattr(self, 'fps_monitor'):
                self.fps_monitor.update(processed=True)

            # Check subscriber availability periodically
            if (self.get_clock().now() - self.last_subscriber_check) > self.subscriber_check_interval:
                self.check_start_scan_subscriber_availability()
                self.last_subscriber_check = self.get_clock().now()

            # Update last human data time and reset recovery if humans detected
            if msg.humans:
                self.last_human_data_time = self.get_clock().now()
                self.last_valid_message_time = self.get_clock().now()
                
                if self.recovery_state not in ["NORMAL", "NO_SUBSCRIBER"]:
                    self.get_logger().info("Humans detected - resetting recovery state to normal")
                    self.recovery_state = "NORMAL" if self.start_scan_subscriber_available else "NO_SUBSCRIBER"
                    self.recovery_attempts = 0

            # Check for no-data timeout (but not in no-subscriber mode)
            if self.recovery_state != "NO_SUBSCRIBER":
                time_since_humans = (self.get_clock().now() - self.last_human_data_time).nanoseconds / 1e9
                if time_since_humans > self.no_data_timeout.nanoseconds / 1e9:
                    self.get_logger().warn(f"No human data for {time_since_humans:.1f} seconds - initiating recovery")
                    self.handle_no_data_recovery()
                    return

            # Get robot's current pose
            robot_pose = self.world_transform.get_robot_pose()
            if robot_pose is None:
                if self.frame_count % 150 == 0:
                    self.get_logger().warn("No robot pose available")
                return

            with self.state_lock:
                self.robot_position, self.robot_orientation = robot_pose
                self.robot_position = self.robot_position[:2]

            # Process human detections
            human_positions = self.process_human_detections(msg)
            
            if len(human_positions) > 0:
                self.get_logger().info(f"Processing {len(human_positions)} humans")

            with self.state_lock:
                self.current_humans = human_positions

            # Publish transformed positions (for visualization/debugging)
            self.publish_transformed_positions(human_positions)
            
            # Store in memory manager
            self.memory_manager.store_data('humans', human_positions)

            if human_positions:
                try:
                    # Handle clustering
                    clusters = None
                    if len(human_positions) == 1:
                        # Single person case - validate coordinates first
                        person_id, person_data = next(iter(human_positions.items()))
                        person_position = person_data['map_position']

                        # Validate position for NaN/Inf
                        try:
                            px = float(person_position[0])
                            py = float(person_position[1])

                            if np.isnan(px) or np.isnan(py) or np.isinf(px) or np.isinf(py):
                                self.get_logger().warn(f"Single person position has NaN/Inf: ({px}, {py})")
                                clusters = []
                            elif abs(px) > 1000 or abs(py) > 1000:
                                self.get_logger().warn(f"Single person position unreasonable: ({px}, {py})")
                                clusters = []
                            else:
                                clusters = [{
                                    'center': np.array([px, py]),
                                    'radius': 0.5,
                                    'points': np.array([[px, py]]),
                                    'ids': [person_id],
                                    'size': 1,
                                    'required_distance': 2.0,
                                    'formation': 'single_person',
                                    'density': 1.0,
                                    'spread': 0.0
                                }]
                        except (TypeError, ValueError, IndexError) as e:
                            self.get_logger().warn(f"Invalid single person position: {e}")
                            clusters = []
                    else:
                        # Multiple people - use clustering
                        clusters = self.cluster_detector.detect_clusters(human_positions)
                        if clusters is not None and len(clusters) > 0:
                            clusters = self.cluster_tracker.update_clusters(clusters)

                    if clusters is not None and len(clusters) > 0:
                        with self.state_lock:
                            self.current_clusters = clusters

                        self.publish_cluster_data(clusters)

                        # Calculate optimal positions based on recovery state
                        if self.recovery_state != "NO_SUBSCRIBER":
                            # Normal operation with subscriber - POI Manager handles position optimization
                            if not self.is_scanning:
                                # POI Manager will calculate optimal positions, just publish clusters
                                self.get_logger().debug("Publishing clusters to POI Manager for position optimization")
                                optimal_positions = []  # POI Manager handles this now

                                if len(clusters) > 0:
                                    self.get_logger().debug(f"Published {len(clusters)} clusters to POI Manager")
                            else:
                                optimal_positions = self.optimal_positions

                            # STATE MACHINE - Simple two-state system
                            if self.is_moving:
                                # Robot is moving to target
                                self.publish_start_scan("no")

                                if self.check_target_reached():
                                    self.get_logger().info("Target reached and robot stopped, starting scan")
                                    self.get_logger().info(f"STATE CHANGE: Moving -> Scanning, Duration: {self.scan_duration.nanoseconds / 1e9}s")
                                    self.is_moving = False
                                    self.is_scanning = True  # Directly transition to scanning
                                    self.scan_start_time = self.get_clock().now()
                                    self.publish_status()

                                # Continue publishing current position while moving
                                if self.current_optimal_position:
                                    self.publish_optimal_position()

                            elif self.is_scanning:
                                # Robot is scanning at current position
                                still_scanning = self.check_target_reached()
                                if not still_scanning:
                                    # Scan completed, move to next position immediately
                                    if optimal_positions is not None and len(optimal_positions) > 0:
                                        self.get_logger().info("Scan complete, moving to next position")
                                        self.current_optimal_position = optimal_positions[0]
                                        self.is_scanning = False
                                        self.is_moving = True
                                        self.publish_status()
                                        self.publish_optimal_position()
                                    else:
                                        self.get_logger().warn("No optimal positions available after scan")
                                        self.is_scanning = False

                            else:
                                # Initial state or reset - get first position
                                if optimal_positions is not None and len(optimal_positions) > 0 and not self.current_optimal_position:
                                    self.get_logger().info("Setting initial position")
                                    self.current_optimal_position = optimal_positions[0]
                                    self.is_moving = True
                                    self.publish_optimal_position()
                                elif optimal_positions is None or len(optimal_positions) == 0:
                                    if self.frame_count % 150 == 0:
                                        self.get_logger().info("No optimal positions calculated, waiting...")

                        else:
                            # No subscriber mode - POI Manager handles position optimization
                            self.get_logger().debug("No subscriber mode - POI Manager will handle continuous movement")
                            optimal_positions = []  # POI Manager handles this now

                        # Prepare visualization data (for web interface and optional local display)
                        vis_positions = {
                            track_id: (data['map_position'][0], data['map_position'][1])
                            for track_id, data in human_positions.items()
                        }
                        vis_optimal = [self.current_optimal_position] if self.current_optimal_position else []

                        # ALWAYS publish to web interface (independent of local visualization)
                        self._publish_web_data(vis_positions, vis_optimal)

                        # Update local matplotlib visualizer if enabled
                        if self.config.get('visualization', {}).get('enabled', False):
                            if hasattr(self, 'visualizer') and self.visualizer:
                                # Update positions (humans_dict, robot_pos, robot_orientation)
                                self.visualizer.update_positions(
                                    vis_positions,
                                    robot_pos=self.robot_position,
                                    robot_orientation=self.robot_orientation
                                )
                                # Update clusters separately
                                if self.current_clusters:
                                    cluster_list = [
                                        {'x': c.get('center', [0, 0])[0] if isinstance(c.get('center'), (list, tuple)) else c.get('x', 0),
                                         'y': c.get('center', [0, 0])[1] if isinstance(c.get('center'), (list, tuple)) else c.get('y', 0),
                                         'size': c.get('size', 1),
                                         'confidence': c.get('confidence', 0.5)}
                                        for c in self.current_clusters
                                    ]
                                    self.visualizer.update_clusters(cluster_list)
                                # Update optimal positions separately
                                if vis_optimal:
                                    opt_list = [
                                        {'x': opt.get('x', opt.get('position', [0, 0])[0] if isinstance(opt.get('position'), (list, tuple)) else 0),
                                         'y': opt.get('y', opt.get('position', [0, 0])[1] if isinstance(opt.get('position'), (list, tuple)) else 0),
                                         'score': opt.get('score', 1.0)}
                                        for opt in vis_optimal
                                    ]
                                    self.visualizer.update_optimal_positions(opt_list)

                except Exception as e:
                    self.get_logger().error(f"Error in cluster processing: {e}")
                    import traceback
                    self.get_logger().error(f"Traceback:\n{traceback.format_exc()}")
            
            else:
                # No humans detected - handle edge case
                if self.frame_count % 300 == 0:
                    self.get_logger().info("No humans detected in current frame")

            # Performance monitoring - calculate processing time
            process_time = time.perf_counter() - callback_start
            self.processing_times.append(process_time)

            # Update optimization components with performance data
            if hasattr(self, 'frame_processor') and self.frame_processor:
                frame_time = process_time  # For this frame
                self.frame_processor.update_performance_metrics(frame_time, process_time)

            if hasattr(self, 'quality_controller') and self.quality_controller:
                self.quality_controller.update_performance(process_time)

            # Periodic cleanup and monitoring
            if (self.get_clock().now() - self.last_cleanup_time).nanoseconds / 1e9 > 10.0:
                self.memory_manager.cleanup_old_data()

                if len(self.processing_times) > 0:
                    avg_process_time = np.mean(self.processing_times)
                    max_process_time = np.max(self.processing_times)

                    # Get FPS from monitor if available
                    current_fps = 0
                    if hasattr(self, 'fps_monitor') and self.fps_monitor:
                        current_fps = self.fps_monitor.get_current_fps()

                    # Get quality level if available
                    quality_level = "N/A"
                    if hasattr(self, 'quality_controller') and self.quality_controller:
                        quality_level = self.quality_controller.current_level.name

                    # Get cluster detector stats
                    cluster_stats = ""
                    if hasattr(self, 'cluster_detector') and self.cluster_detector:
                        stats = self.cluster_detector.get_performance_stats()
                        if stats.get('optimization_enabled'):
                            cluster_stats = f", Clustering: {stats.get('avg_ms', 0):.1f}ms (JIT:{stats.get('jit_enabled')}, GPU:{stats.get('gpu_enabled')})"

                    self.get_logger().info(
                        f"Performance - Avg: {avg_process_time*1000:.1f}ms, "
                        f"Max: {max_process_time*1000:.1f}ms, FPS: {current_fps:.1f}, "
                        f"Quality: {quality_level}{cluster_stats}"
                    )

                    # Adjust frame skipping if performance is poor
                    if max_process_time > 0.1:  # 100ms threshold
                        self.process_every_n_frames = min(self.process_every_n_frames + 1, 5)
                        self.get_logger().warn(f"Performance degraded, increasing frame skip to {self.process_every_n_frames}")

                self.last_cleanup_time = self.get_clock().now()
                # Garbage collection only when memory usage is high (avoid latency spikes)
                if psutil.virtual_memory().percent > 80:
                    gc.collect()

        except Exception as e:
            self.get_logger().error(f"Error in human callback: {e}")
            import traceback
            self.get_logger().error(traceback.format_exc())

    def print_debug_info(self):
        """Print debug information about current state"""
        try:
            self.get_logger().info("\n========== Debug Information ==========")

            if self.robot_position is not None:
                self.get_logger().info(
                    f"\nRobot Pose (map frame):"
                    f"\n  Position: ({self.robot_position[0]:.2f}, {self.robot_position[1]:.2f})"
                    f"\n  Orientation: {np.degrees(self.robot_orientation):.1f}°"
                )

            self.get_logger().info(f"\nDetected Humans ({len(self.current_humans)}):")
            for track_id, data in self.current_humans.items():
                self.get_logger().info(
                    f"ID: {track_id}"
                    f"\n  Map position: ({data['map_position'][0]:.2f}, {data['map_position'][1]:.2f})"
                    f"\n  Robot relative: ({data['robot_relative'][0]:.2f}, "
                    f"{data['robot_relative'][1]:.2f}, {data['robot_relative'][2]:.2f})"
                )

            self.get_logger().info(f"\nDetected Clusters ({len(self.current_clusters)}):")
            for i, cluster in enumerate(self.current_clusters):
                self.get_logger().info(
                    f"Cluster {i + 1}:"
                    f"\n  Size: {cluster['size']} humans"
                    f"\n  Center: ({cluster['center'][0]:.2f}, {cluster['center'][1]:.2f})"
                    f"\n  Radius: {cluster['radius']:.2f}m"
                    f"\n  Formation: {cluster.get('pattern', 'unknown')}"
                    f"\n  Required Distance: {cluster['required_distance']:.2f}m"
                    f"\n  Member IDs: {cluster['ids']}"
                )

            self.get_logger().info(f"\nOptimal Positions ({len(self.optimal_positions)}):")
            for i, pos in enumerate(self.optimal_positions):
                self.get_logger().info(
                    f"Position {i + 1}:"
                    f"\n  Location: ({pos['position'][0]:.2f}, {pos['position'][1]:.2f})"
                    f"\n  Orientation: {np.degrees(pos['orientation']):.1f}°"
                    f"\n  Distance: {pos['distance']:.2f}m"
                    f"\n  Priority Score: {pos['priority_score']:.2f}"
                )

            self.get_logger().info("\n=====================================")

        except Exception as e:
            self.get_logger().error(f"Error in debug printing: {e}")

    def update_map_visualization(self):
        """Periodically update map visualization data"""
        try:
            # Only update if visualization is enabled
            if not self.config.get('visualization', {}).get('enabled', False):
                return
                
            # Update matplotlib visualizer if available
            if hasattr(self, 'visualizer') and self.visualizer and hasattr(self, 'map_handler'):
                if self.map_handler.map_msg is not None:
                    self.visualizer.update_map_data(self.map_handler.map_msg)
        except Exception as e:
            self.get_logger().error(f"Error updating map visualization: {e}")

# run() was there before adding the lifecycle nodes

    def cleanup(self):
        """Clean up resources"""
        try:
            if hasattr(self, 'status_timer') and self.status_timer:
                self.status_timer.cancel()
            if hasattr(self, 'map_update_timer') and self.map_update_timer:
                self.map_update_timer.cancel()
            if hasattr(self, 'no_subscriber_move_timer') and self.no_subscriber_move_timer:
                self.no_subscriber_move_timer.cancel()

            # Clean up parallel processor
            if hasattr(self, 'parallel_processor'):
                self.parallel_processor.shutdown()

            # Stop Jetson monitoring
            if hasattr(self, 'jetson_monitor') and self.jetson_monitor:
                self.jetson_monitor.stop()

            # Clear memory pools
            if hasattr(self, 'memory_optimizer') and self.memory_optimizer:
                self.memory_optimizer.clear_pools()

            self.get_logger().info("Cleanup completed")
        except Exception as e:
            self.get_logger().error(f"Error during cleanup: {e}")


class PerformanceMonitor:
    """Performance monitoring for Jetson optimization"""
    
    def __init__(self, node: Node = None):
        self.cpu_usage = deque(maxlen=10)
        self.memory_usage = deque(maxlen=10)
        self.last_check = None
        self.node = node
    
    def update(self, current_time):
        """Update performance metrics"""
        try:
            if self.last_check is None:
                self.last_check = current_time
                return
            
            # Check every 5 seconds
            elapsed = (current_time - self.last_check).nanoseconds / 1e9
            if elapsed > 5.0:
                cpu = psutil.cpu_percent()
                memory = psutil.virtual_memory().percent
                
                self.cpu_usage.append(cpu)
                self.memory_usage.append(memory)
                
                if cpu > 85 or memory > 85:
                    if self.node:
                        self.node.get_logger().warn(f"High resource usage - CPU: {cpu:.1f}%, Memory: {memory:.1f}%")
                
                self.last_check = current_time
        except Exception as e:
            if self.node:
                self.node.get_logger().warn(f"Error monitoring performance: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = HumanClusteringNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()