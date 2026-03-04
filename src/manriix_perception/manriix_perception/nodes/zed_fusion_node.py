#!/usr/bin/env python3
"""
ZED Fusion Node - Unified Detection System

Main node that coordinates:
1. ZED X multi-camera detection using built-in AI models
2. Multi-camera spatial fusion and synchronization
3. Output to both clustering pipeline and Nav2 costmap

This replaces the separate detection systems with a unified ZED-based approach.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg import Header, String
from geometry_msgs.msg import Point
import time
import threading
import json
from typing import Dict, List, Any, Optional
import os
from pathlib import Path

# Local imports - updated for unified package
from manriix_perception.detection.zed_subscriber import ZedSubscriber, ZedDetection
from manriix_perception.detection.coordinate_transformer import CoordinateTransformer
from manriix_perception.fusion.synchronizer import FrameSynchronizer, SynchronizedFrameSet
from manriix_perception.fusion.spatial_fusion import SpatialFusion, FusedDetection
from manriix_perception.utils.config_loader import ConfigLoader, ConfigurationError
from manriix_perception.utils.logger_setup import LoggerSetup


class ZedFusionNode(Node):
    """
    Main ZED fusion node for unified detection system
    
    Coordinates the complete pipeline:
    1. Subscribe to multiple ZED cameras (built-in AI detection)
    2. Transform detections to common coordinate frame
    3. Synchronize frames across cameras
    4. Perform spatial fusion
    5. Output dual streams: HumansList + fused detections for costmap
    """
    
    def __init__(self):
        super().__init__('zed_fusion_node')
        
        # Declare and load ROS2 parameters
        self._declare_parameters()
        self.config = self._load_ros_parameters()
        
        # Setup logging
        self.logger = LoggerSetup.get_node_logger(self, self.config.get('logging', {}))
        LoggerSetup.log_system_info(self.logger, self.config)

        # Configuration sections
        self.camera_config = self.config.get('cameras', {})
        self.detection_config = self._load_detection_config()  # Load from detection_config.yaml
        self.fusion_config = self.config.get('fusion', {})
        self.transform_config = self.config.get('transform', {})
        self.output_config = self.config.get('output', {})
        
        # System components
        self.zed_subscriber = None
        self.coordinate_transformer = None
        self.frame_synchronizer = None
        self.spatial_fusion = None
        
        # QoS profiles
        self.sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        
        self.reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        
        # Output publishers
        self._setup_publishers()
        
        # System state
        self.system_active = False
        self.last_output_time = 0.0
        self.processing_lock = threading.Lock()
        
        # Performance monitoring
        self.performance_stats = {
            'total_fusions_processed': 0,
            'average_processing_time': 0.0,
            'output_rate_hz': 0.0,
            'last_performance_log': 0.0
        }
        
        # Initialize system components
        self._initialize_system()

        # Start performance monitoring timer
        self.create_timer(1.0, self._monitor_performance)

        # Start web interface publishing timer (10 Hz for responsive UI)
        self.create_timer(0.1, self._publish_web_camera_detections)

        self.logger.info("ZED Fusion Node initialized successfully")
    
    def _declare_parameters(self):
        """Declare all ROS2 parameters with default values"""
        
        # Logging parameters
        self.declare_parameter('logging.level', 'INFO')
        self.declare_parameter('logging.enable_performance_logs', True)
        self.declare_parameter('logging.log_detection_stats', True)
        
        # Camera parameters - ZED X Front (values match URDF: xyz="0.145 0.0 0.815" rpy="0 0 0")
        self.declare_parameter('cameras.zedx_front.enabled', True)
        self.declare_parameter('cameras.zedx_front.topic', '/zedx_front/zed_node/obj_det/objects')
        self.declare_parameter('cameras.zedx_front.frame_id', 'zedx_front_left_camera_frame')
        self.declare_parameter('cameras.zedx_front.position', [0.145, 0.0, 0.815])
        self.declare_parameter('cameras.zedx_front.orientation', [0.0, 0.0, 0.0, 1.0])  # 0° yaw
        self.declare_parameter('cameras.zedx_front.confidence_threshold', 60.0)
        self.declare_parameter('cameras.zedx_front.detection_classes', ['PERSON', 'VEHICLE', 'ANIMAL'])

        # Camera parameters - ZED X Left (values match URDF: xyz="0.0 0.175 0.815" rpy="0 0 2.094")
        self.declare_parameter('cameras.zedx_left.enabled', True)
        self.declare_parameter('cameras.zedx_left.topic', '/zedx_left/zed_node/obj_det/objects')
        self.declare_parameter('cameras.zedx_left.frame_id', 'zedx_left_left_camera_frame')
        self.declare_parameter('cameras.zedx_left.position', [0.0, 0.175, 0.815])
        self.declare_parameter('cameras.zedx_left.orientation', [0.0, 0.0, 0.8660254, 0.5])  # 120° yaw quaternion
        self.declare_parameter('cameras.zedx_left.confidence_threshold', 60.0)
        self.declare_parameter('cameras.zedx_left.detection_classes', ['PERSON', 'VEHICLE', 'ANIMAL'])

        # Camera parameters - ZED X Right (values match URDF: xyz="0.0 -0.175 0.815" rpy="0 0 -2.094")
        self.declare_parameter('cameras.zedx_right.enabled', True)
        self.declare_parameter('cameras.zedx_right.topic', '/zedx_right/zed_node/obj_det/objects')
        self.declare_parameter('cameras.zedx_right.frame_id', 'zedx_right_left_camera_frame')
        self.declare_parameter('cameras.zedx_right.position', [0.0, -0.175, 0.815])
        self.declare_parameter('cameras.zedx_right.orientation', [0.0, 0.0, -0.8660254, 0.5])  # -120° yaw quaternion
        self.declare_parameter('cameras.zedx_right.confidence_threshold', 60.0)
        self.declare_parameter('cameras.zedx_right.detection_classes', ['PERSON', 'VEHICLE', 'ANIMAL'])
        
        # Camera parameters - ZED X Rear (disabled by default)
        self.declare_parameter('cameras.zed_x_rear.enabled', False)
        self.declare_parameter('cameras.zed_x_rear.topic', '/zed_x_rear/detections')
        self.declare_parameter('cameras.zed_x_rear.frame_id', 'zed_x_rear_camera_frame')
        self.declare_parameter('cameras.zed_x_rear.position', [-0.3, 0.0, 0.8])
        self.declare_parameter('cameras.zed_x_rear.orientation', [0.0, 0.0, 3.14159, 1.0])
        self.declare_parameter('cameras.zed_x_rear.confidence_threshold', 60.0)
        self.declare_parameter('cameras.zed_x_rear.detection_classes', ['PERSON', 'VEHICLE', 'ANIMAL'])
        
        # Detection parameters
        self.declare_parameter('detection.enable_tracking', True)
        self.declare_parameter('detection.tracking_timeout', 2.0)
        self.declare_parameter('detection.min_detection_confidence', 35.0)
        self.declare_parameter('detection.max_detection_range', 10.0)
        self.declare_parameter('detection.enable_velocity_estimation', True)
        self.declare_parameter('detection.velocity_smoothing_window', 5)
        
        # Class-specific detection parameters
        self.declare_parameter('detection.person_detection.min_confidence', 40.0)
        self.declare_parameter('detection.person_detection.max_range', 8.0)
        self.declare_parameter('detection.person_detection.enable_pose_estimation', False)
        
        self.declare_parameter('detection.vehicle_detection.min_confidence', 70.0)
        self.declare_parameter('detection.vehicle_detection.max_range', 15.0)
        
        self.declare_parameter('detection.animal_detection.min_confidence', 65.0)
        self.declare_parameter('detection.animal_detection.max_range', 6.0)
        
        # Fusion parameters
        self.declare_parameter('fusion.synchronization.max_time_diff', 0.1)
        self.declare_parameter('fusion.synchronization.min_cameras_required', 1)
        self.declare_parameter('fusion.synchronization.sync_timeout', 0.5)
        
        self.declare_parameter('fusion.spatial_fusion.duplicate_threshold', 0.8)
        self.declare_parameter('fusion.spatial_fusion.confidence_fusion_method', 'weighted_average')
        self.declare_parameter('fusion.spatial_fusion.position_fusion_method', 'weighted_centroid')
        self.declare_parameter('fusion.spatial_fusion.enable_cross_camera_tracking', True)
        self.declare_parameter('fusion.spatial_fusion.tracking_association_threshold', 1.0)
        
        self.declare_parameter('fusion.temporal_smoothing.enable_smoothing', True)
        self.declare_parameter('fusion.temporal_smoothing.position_smoothing_factor', 0.3)
        self.declare_parameter('fusion.temporal_smoothing.velocity_smoothing_factor', 0.5)
        self.declare_parameter('fusion.temporal_smoothing.confidence_smoothing_factor', 0.2)
        
        # Transform parameters
        # Use base_footprint for Nav2 compatibility (at ground level z=0)
        self.declare_parameter('transform.target_frame', 'base_footprint')
        self.declare_parameter('transform.enable_tf_lookup', True)
        self.declare_parameter('transform.tf_timeout', 1.0)
        self.declare_parameter('transform.use_static_transforms', False)

        # Output parameters
        self.declare_parameter('output.humans_list.topic', '/human_clustering/humans')
        self.declare_parameter('output.humans_list.frame_id', 'base_footprint')
        self.declare_parameter('output.humans_list.publish_rate', 10.0)
        
        self.declare_parameter('output.debug.enabled', False)
        self.declare_parameter('output.debug.fused_detections_topic', '/zed_system/fused_detections')
        self.declare_parameter('output.debug.raw_detections_topic', '/zed_system/raw_detections')
        
        # Diagnostics parameters
        self.declare_parameter('diagnostics.enable_performance_metrics', True)
        self.declare_parameter('diagnostics.system_status_topic', '/zed_system/status')
        self.declare_parameter('diagnostics.fusion_diagnostics_topic', '/zed_system/fusion_diagnostics')
        self.declare_parameter('diagnostics.publish_rate', 1.0)
        self.declare_parameter('diagnostics.max_processing_time_ms', 50.0)
        self.declare_parameter('diagnostics.min_output_rate_hz', 5.0)
        self.declare_parameter('diagnostics.max_memory_usage_mb', 500.0)
    
    def _load_ros_parameters(self) -> Dict[str, Any]:
        """Load configuration from ROS2 parameters"""
        try:
            config = {
                'logging': {
                    'level': self.get_parameter('logging.level').value,
                    'enable_performance_logs': self.get_parameter('logging.enable_performance_logs').value,
                    'log_detection_stats': self.get_parameter('logging.log_detection_stats').value
                },
                'cameras': {
                    'zedx_front': {
                        'enabled': self.get_parameter('cameras.zedx_front.enabled').value,
                        'topic': self.get_parameter('cameras.zedx_front.topic').value,
                        'frame_id': self.get_parameter('cameras.zedx_front.frame_id').value,
                        'position': self.get_parameter('cameras.zedx_front.position').value,
                        'orientation': self.get_parameter('cameras.zedx_front.orientation').value,
                        'confidence_threshold': self.get_parameter('cameras.zedx_front.confidence_threshold').value,
                        'detection_classes': self.get_parameter('cameras.zedx_front.detection_classes').value
                    },
                    'zedx_left': {
                        'enabled': self.get_parameter('cameras.zedx_left.enabled').value,
                        'topic': self.get_parameter('cameras.zedx_left.topic').value,
                        'frame_id': self.get_parameter('cameras.zedx_left.frame_id').value,
                        'position': self.get_parameter('cameras.zedx_left.position').value,
                        'orientation': self.get_parameter('cameras.zedx_left.orientation').value,
                        'confidence_threshold': self.get_parameter('cameras.zedx_left.confidence_threshold').value,
                        'detection_classes': self.get_parameter('cameras.zedx_left.detection_classes').value
                    },
                    'zedx_right': {
                        'enabled': self.get_parameter('cameras.zedx_right.enabled').value,
                        'topic': self.get_parameter('cameras.zedx_right.topic').value,
                        'frame_id': self.get_parameter('cameras.zedx_right.frame_id').value,
                        'position': self.get_parameter('cameras.zedx_right.position').value,
                        'orientation': self.get_parameter('cameras.zedx_right.orientation').value,
                        'confidence_threshold': self.get_parameter('cameras.zedx_right.confidence_threshold').value,
                        'detection_classes': self.get_parameter('cameras.zedx_right.detection_classes').value
                    },
                    'zed_x_rear': {
                        'enabled': self.get_parameter('cameras.zed_x_rear.enabled').value,
                        'topic': self.get_parameter('cameras.zed_x_rear.topic').value,
                        'frame_id': self.get_parameter('cameras.zed_x_rear.frame_id').value,
                        'position': self.get_parameter('cameras.zed_x_rear.position').value,
                        'orientation': self.get_parameter('cameras.zed_x_rear.orientation').value,
                        'confidence_threshold': self.get_parameter('cameras.zed_x_rear.confidence_threshold').value,
                        'detection_classes': self.get_parameter('cameras.zed_x_rear.detection_classes').value
                    }
                },
                'detection': {
                    'enable_tracking': self.get_parameter('detection.enable_tracking').value,
                    'tracking_timeout': self.get_parameter('detection.tracking_timeout').value,
                    'min_detection_confidence': self.get_parameter('detection.min_detection_confidence').value,
                    'max_detection_range': self.get_parameter('detection.max_detection_range').value,
                    'enable_velocity_estimation': self.get_parameter('detection.enable_velocity_estimation').value,
                    'velocity_smoothing_window': self.get_parameter('detection.velocity_smoothing_window').value,
                    'person_detection': {
                        'min_confidence': self.get_parameter('detection.person_detection.min_confidence').value,
                        'max_range': self.get_parameter('detection.person_detection.max_range').value,
                        'enable_pose_estimation': self.get_parameter('detection.person_detection.enable_pose_estimation').value
                    },
                    'vehicle_detection': {
                        'min_confidence': self.get_parameter('detection.vehicle_detection.min_confidence').value,
                        'max_range': self.get_parameter('detection.vehicle_detection.max_range').value
                    },
                    'animal_detection': {
                        'min_confidence': self.get_parameter('detection.animal_detection.min_confidence').value,
                        'max_range': self.get_parameter('detection.animal_detection.max_range').value
                    }
                },
                'fusion': {
                    'synchronization': {
                        'max_time_diff': self.get_parameter('fusion.synchronization.max_time_diff').value,
                        'min_cameras_required': self.get_parameter('fusion.synchronization.min_cameras_required').value,
                        'sync_timeout': self.get_parameter('fusion.synchronization.sync_timeout').value
                    },
                    'spatial_fusion': {
                        'duplicate_threshold': self.get_parameter('fusion.spatial_fusion.duplicate_threshold').value,
                        'confidence_fusion_method': self.get_parameter('fusion.spatial_fusion.confidence_fusion_method').value,
                        'position_fusion_method': self.get_parameter('fusion.spatial_fusion.position_fusion_method').value,
                        'enable_cross_camera_tracking': self.get_parameter('fusion.spatial_fusion.enable_cross_camera_tracking').value,
                        'tracking_association_threshold': self.get_parameter('fusion.spatial_fusion.tracking_association_threshold').value
                    },
                    'temporal_smoothing': {
                        'enable_smoothing': self.get_parameter('fusion.temporal_smoothing.enable_smoothing').value,
                        'position_smoothing_factor': self.get_parameter('fusion.temporal_smoothing.position_smoothing_factor').value,
                        'velocity_smoothing_factor': self.get_parameter('fusion.temporal_smoothing.velocity_smoothing_factor').value,
                        'confidence_smoothing_factor': self.get_parameter('fusion.temporal_smoothing.confidence_smoothing_factor').value
                    }
                },
                'transform': {
                    'target_frame': self.get_parameter('transform.target_frame').value,
                    'enable_tf_lookup': self.get_parameter('transform.enable_tf_lookup').value,
                    'tf_timeout': self.get_parameter('transform.tf_timeout').value,
                    'use_static_transforms': self.get_parameter('transform.use_static_transforms').value
                },
                'output': {
                    'humans_list': {
                        'topic': self.get_parameter('output.humans_list.topic').value,
                        'frame_id': self.get_parameter('output.humans_list.frame_id').value,
                        'publish_rate': self.get_parameter('output.humans_list.publish_rate').value
                    },
                    'debug': {
                        'enabled': self.get_parameter('output.debug.enabled').value,
                        'fused_detections_topic': self.get_parameter('output.debug.fused_detections_topic').value,
                        'raw_detections_topic': self.get_parameter('output.debug.raw_detections_topic').value
                    }
                },
                'diagnostics': {
                    'enable_performance_metrics': self.get_parameter('diagnostics.enable_performance_metrics').value,
                    'system_status_topic': self.get_parameter('diagnostics.system_status_topic').value,
                    'fusion_diagnostics_topic': self.get_parameter('diagnostics.fusion_diagnostics_topic').value,
                    'publish_rate': self.get_parameter('diagnostics.publish_rate').value,
                    'max_processing_time_ms': self.get_parameter('diagnostics.max_processing_time_ms').value,
                    'min_output_rate_hz': self.get_parameter('diagnostics.min_output_rate_hz').value,
                    'max_memory_usage_mb': self.get_parameter('diagnostics.max_memory_usage_mb').value
                }
            }
            
            self.get_logger().info("Successfully loaded configuration from ROS2 parameters")
            return config
            
        except Exception as e:
            self.get_logger().error(f"Failed to load ROS2 parameters: {e}")
            raise

    def _load_detection_config(self) -> Dict[str, Any]:
        """
        Load detection pipeline configuration from detection_config.yaml.

        This config determines whether to use ZED built-in detection or YOLO custom detection,
        and contains pipeline-specific label mappings and thresholds.

        Returns:
            Detection configuration dictionary
        """
        import yaml

        # Try to find detection_config.yaml in package config directory
        config_paths = [
            # Installed package location
            Path(os.path.dirname(__file__)).parent.parent / 'share' / 'manriix_navigation' / 'config' / 'detection_config.yaml',
            # Development location
            Path(os.path.dirname(__file__)).parent.parent / 'config' / 'detection_config.yaml',
            # ROS2 workspace install location
            Path('/home/hype/Desktop/lasa/ros2_ws/install/manriix_navigation/share/manriix_navigation/config/detection_config.yaml'),
        ]

        detection_config = {}
        config_loaded = False

        for config_path in config_paths:
            if config_path.exists():
                try:
                    with open(config_path, 'r') as f:
                        detection_config = yaml.safe_load(f)
                    self.get_logger().info(f"Loaded detection config from: {config_path}")
                    config_loaded = True
                    break
                except Exception as e:
                    self.get_logger().warning(f"Failed to load {config_path}: {e}")

        if not config_loaded:
            self.get_logger().warning("detection_config.yaml not found, using defaults (ZED_INBUILT pipeline)")
            detection_config = {
                'detection_pipeline': 'ZED_INBUILT',
                'zed_inbuilt': {
                    'enabled_labels': ['PERSON'],
                    'human_label': 'PERSON',
                    'default_confidence': 50.0
                },
                'common': {
                    'min_detection_range': 0.3,
                    'max_detection_range': 10.0
                }
            }

        # Log the pipeline being used
        pipeline = detection_config.get('detection_pipeline', 'ZED_INBUILT')
        self.get_logger().info(f"Detection pipeline selected: {pipeline}")

        return detection_config

    def _setup_publishers(self):
        """Setup output publishers"""
        # Import messages here to avoid circular imports
        from manriix_perception.msg import HumansList, HumanTrackingData
        from manriix_perception.msg import ZedDetectionArray, ZedDetection as ZedDetectionMsg
        from manriix_perception.msg import MultiCameraStatus, FusionDiagnostics
        
        # Main output for human clustering pipeline (SAME TOPIC AS BEFORE)
        humans_config = self.output_config['humans_list']
        self.humans_publisher = self.create_publisher(
            HumansList,
            humans_config['topic'],
            self.reliable_qos
        )
        
        # Fused detections for costmap bridge - ALWAYS publish (required for obstacle avoidance)
        # Topic from debug config or default to '/zed_system/fused_detections'
        fused_topic = self.output_config.get('debug', {}).get(
            'fused_detections_topic', '/zed_system/fused_detections'
        )
        self.fused_detections_publisher = self.create_publisher(
            ZedDetectionArray,
            fused_topic,
            self.sensor_qos
        )
        self.logger.info(f"Fused detections publisher created on: {fused_topic}")
        
        # System status publishers
        diagnostics_config = self.config.get('diagnostics', {})
        if diagnostics_config.get('enable_performance_metrics', True):
            self.status_publisher = self.create_publisher(
                MultiCameraStatus,
                diagnostics_config.get('system_status_topic', '/zed_system/status'),
                QoSProfile(depth=5)
            )
            
            self.fusion_diagnostics_publisher = self.create_publisher(
                FusionDiagnostics,
                diagnostics_config.get('fusion_diagnostics_topic', '/zed_system/fusion_diagnostics'),
                QoSProfile(depth=5)
            )
        else:
            self.status_publisher = None
            self.fusion_diagnostics_publisher = None

        # Web interface publisher for camera detection counts (JSON format for ROSBridge)
        self.web_detections_pub = self.create_publisher(
            String,
            '/web/camera_detections',
            QoSProfile(depth=5)
        )

        # Web interface publisher for system health (JSON format for ROSBridge)
        # This is what the web UI subscribes to for component status
        self.web_system_health_pub = self.create_publisher(
            String,
            '/recovery/system_health',
            QoSProfile(depth=5)
        )

        self.logger.info("Publishers setup completed")
    
    def _initialize_system(self):
        """Initialize all system components"""
        try:
            # Get active camera names
            active_cameras = [
                name for name, config in self.camera_config.items()
                if config.get('enabled', True)
            ]
            
            if not active_cameras:
                raise ConfigurationError("No active cameras configured")
            
            self.logger.info(f"Initializing system for cameras: {active_cameras}")
            
            # Initialize coordinate transformer
            self.coordinate_transformer = CoordinateTransformer(
                self, self.transform_config, self.camera_config
            )
            
            # Initialize frame synchronizer
            self.frame_synchronizer = FrameSynchronizer(
                self.fusion_config['synchronization'], active_cameras
            )
            self.frame_synchronizer.register_sync_callback(self._synchronized_frames_callback)
            
            # Initialize spatial fusion
            self.spatial_fusion = SpatialFusion(self.fusion_config)
            
            # Initialize ZED subscriber (must be last)
            self.zed_subscriber = ZedSubscriber(
                self, self.camera_config, self.detection_config
            )
            self.zed_subscriber.register_detection_callback(self._detections_callback)
            
            self.system_active = True
            self.logger.info("System initialization completed successfully")
            
        except Exception as e:
            self.logger.error(f"System initialization failed: {e}")
            self.system_active = False
            raise
    
    def _detections_callback(self, detections: List[ZedDetection], camera_name: str):
        """
        Callback for new detections from ZED subscriber
        
        Args:
            detections: List of detections from a camera
            camera_name: Name of the camera
        """
        if not self.system_active or not detections:
            return
        
        try:
            # Transform detections to common coordinate frame
            transformed_detections = self.coordinate_transformer.transform_detections(detections)
            
            # Add to synchronizer
            current_time = time.time()
            synchronized_set = self.frame_synchronizer.add_camera_frame(
                camera_name, transformed_detections, current_time
            )
            
            # Synchronized set processing is handled by _synchronized_frames_callback
            
        except Exception as e:
            self.logger.error(f"Error processing detections from {camera_name}: {e}")
    
    def _synchronized_frames_callback(self, synchronized_set: SynchronizedFrameSet):
        """
        Callback for synchronized frames from multiple cameras
        
        Args:
            synchronized_set: Synchronized frame set
        """
        if not self.system_active:
            return
        
        with self.processing_lock:
            try:
                start_time = time.time()
                
                # Perform spatial fusion
                fused_detections = self.spatial_fusion.fuse_synchronized_detections(synchronized_set)
                
                # Publish outputs
                if fused_detections:
                    self._publish_humans_list(fused_detections, synchronized_set.reference_timestamp)
                    
                    if self.fused_detections_publisher:
                        self._publish_fused_detections(fused_detections, synchronized_set.reference_timestamp)
                
                # Update performance statistics
                processing_time = (time.time() - start_time) * 1000  # Convert to ms
                self._update_performance_stats(processing_time)
                
                self.last_output_time = time.time()
                
            except Exception as e:
                self.logger.error(f"Error processing synchronized frames: {e}")
    
    def _publish_humans_list(self, fused_detections: List[FusedDetection], timestamp: float):
        """
        Publish HumansList message for clustering pipeline
        
        Args:
            fused_detections: List of fused detections
            timestamp: Reference timestamp
        """
        try:
            from manriix_perception.msg import HumansList, HumanTrackingData

            # Filter for humans only using pipeline-aware label checking
            # ZED built-in uses 'PERSON', YOLO uses 'person'
            if self.zed_subscriber:
                human_detections = [d for d in fused_detections
                                  if self.zed_subscriber.is_human_label(d.object_class)]
            else:
                # Fallback: case-insensitive PERSON check
                human_detections = [d for d in fused_detections
                                  if d.object_class.upper() == "PERSON"]

            if not human_detections:
                return
            
            # Create HumansList message
            humans_msg = HumansList()
            humans_msg.header = Header()
            humans_msg.header.stamp = self.get_clock().now().to_msg()
            humans_msg.header.frame_id = self.transform_config['target_frame']
            
            # Convert each fused detection to HumanTrackingData
            for detection in human_detections:
                human_data = HumanTrackingData()
                human_data.tracking_id = detection.tracking_id
                human_data.position = detection.position
                human_data.confidence = detection.confidence
                
                # Calculate distance from robot origin
                distance = (detection.position.x**2 + detection.position.y**2 + detection.position.z**2)**0.5
                human_data.distance = distance
                
                humans_msg.humans.append(human_data)
            
            # Publish to the SAME TOPIC the clustering system expects
            self.humans_publisher.publish(humans_msg)
            
            self.logger.debug(f"Published {len(human_detections)} humans to clustering pipeline")
            
        except Exception as e:
            self.logger.error(f"Error publishing HumansList: {e}")
    
    def _publish_fused_detections(self, fused_detections: List[FusedDetection], timestamp: float):
        """
        Publish fused detections for debugging
        
        Args:
            fused_detections: List of fused detections
            timestamp: Reference timestamp
        """
        try:
            from manriix_perception.msg import ZedDetectionArray, ZedDetection as ZedDetectionMsg
            
            # Create ZedDetectionArray message
            detections_msg = ZedDetectionArray()
            detections_msg.header = Header()
            detections_msg.header.stamp = self.get_clock().now().to_msg()
            detections_msg.header.frame_id = self.transform_config['target_frame']
            detections_msg.total_cameras_active = len(self.camera_config)
            
            # Convert each fused detection
            for detection in fused_detections:
                detection_msg = ZedDetectionMsg()
                detection_msg.header = detections_msg.header
                detection_msg.tracking_id = detection.tracking_id
                detection_msg.object_class = detection.object_class
                detection_msg.object_subclass = detection.object_subclass
                detection_msg.class_id = detection.class_id
                detection_msg.confidence = float(detection.confidence)
                detection_msg.position = detection.position

                # Ensure dimensions_3d is exactly 3 floats (ROS2 message requirement)
                dims = detection.dimensions_3d
                if dims and len(dims) >= 3:
                    detection_msg.dimensions_3d = [float(dims[0]), float(dims[1]), float(dims[2])]
                else:
                    detection_msg.dimensions_3d = [0.5, 1.7, 0.5]  # Default human dimensions

                # Ensure velocity is exactly 3 floats (ROS2 message requirement)
                vel = detection.velocity
                if vel and len(vel) >= 3:
                    detection_msg.velocity = [float(vel[0]), float(vel[1]), float(vel[2])]
                else:
                    detection_msg.velocity = [0.0, 0.0, 0.0]

                detection_msg.camera_name = ','.join(detection.supporting_cameras)
                detection_msg.camera_frame_id = self.transform_config['target_frame']

                detections_msg.detections.append(detection_msg)
            
            # Publish
            self.fused_detections_publisher.publish(detections_msg)
            
        except Exception as e:
            self.logger.error(f"Error publishing fused detections: {e}")
    
    def _update_performance_stats(self, processing_time: float):
        """Update performance statistics"""
        self.performance_stats['total_fusions_processed'] += 1
        
        # Update average processing time
        total_processed = self.performance_stats['total_fusions_processed']
        current_avg = self.performance_stats['average_processing_time']
        new_avg = ((current_avg * (total_processed - 1)) + processing_time) / total_processed
        self.performance_stats['average_processing_time'] = new_avg
        
        # Calculate output rate
        current_time = time.time()
        if self.last_output_time > 0:
            time_diff = current_time - self.last_output_time
            self.performance_stats['output_rate_hz'] = 1.0 / time_diff if time_diff > 0 else 0.0
    
    def _monitor_performance(self):
        """Monitor and log system performance"""
        if not self.system_active:
            return
        
        current_time = time.time()
        
        # Log performance every 30 seconds
        if current_time - self.performance_stats['last_performance_log'] > 30.0:
            self._log_performance_metrics()
            self.performance_stats['last_performance_log'] = current_time
        
        # Publish system status
        if self.status_publisher:
            self._publish_system_status()
        
        if self.fusion_diagnostics_publisher:
            self._publish_fusion_diagnostics()
    
    def _log_performance_metrics(self):
        """Log current performance metrics"""
        metrics = {
            'fusions_processed': self.performance_stats['total_fusions_processed'],
            'avg_processing_time_ms': self.performance_stats['average_processing_time'],
            'output_rate_hz': self.performance_stats['output_rate_hz']
        }
        
        LoggerSetup.log_performance_metrics(self.logger, metrics)
        
        # Log component statistics
        if self.zed_subscriber:
            detection_stats = self.zed_subscriber.get_detection_statistics()
            LoggerSetup.log_camera_status(self.logger, detection_stats)
        
        if self.spatial_fusion:
            fusion_stats = self.spatial_fusion.get_fusion_statistics()
            self.logger.info(f"Fusion efficiency: {fusion_stats.get('fusion_efficiency', 0):.2%}")
    
    def _publish_system_status(self):
        """Publish system status messages (both ROS2 native and web JSON)"""
        try:
            from manriix_perception.msg import MultiCameraStatus
            from builtin_interfaces.msg import Time

            # Get camera status and stats
            camera_status = {}
            detection_stats = {}
            if self.zed_subscriber:
                camera_status = self.zed_subscriber.get_camera_status()
                detection_stats = self.zed_subscriber.get_detection_statistics()

            # Build MultiCameraStatus message for ROS2 native consumers
            if self.status_publisher:
                status_msg = MultiCameraStatus()
                status_msg.header.stamp = self.get_clock().now().to_msg()
                status_msg.header.frame_id = 'base_link'

                # Populate camera arrays
                for cam_name, is_active in camera_status.items():
                    status_msg.camera_names.append(cam_name)
                    status_msg.camera_active.append(is_active)

                    stats = detection_stats.get(cam_name, {})
                    status_msg.detection_counts.append(int(stats.get('total_detections', 0)))
                    status_msg.average_confidence.append(float(stats.get('average_confidence', 0.0)))

                    # Last detection time
                    last_time = stats.get('last_detection_time')
                    time_msg = Time()
                    if last_time:
                        time_msg.sec = int(last_time)
                        time_msg.nanosec = int((last_time % 1) * 1e9)
                    status_msg.last_detection_time.append(time_msg)

                # System performance
                status_msg.fusion_rate_hz = float(self.performance_stats.get('output_rate_hz', 0.0))
                status_msg.output_rate_hz = float(self.performance_stats.get('output_rate_hz', 0.0))
                status_msg.total_cpu_usage = 0.0
                status_msg.total_memory_usage_mb = 0.0
                status_msg.total_errors = 0
                status_msg.recent_errors = []

                self.status_publisher.publish(status_msg)

            # Publish JSON for web interface on /recovery/system_health
            web_status = {
                'timestamp': time.time(),
                'node_status': {
                    'zed_fusion_node': 'active' if self.system_active else 'idle',
                    # Other components not implemented - show as idle
                    'human_clustering_node': 'idle',
                    'poi_manager': 'idle',
                    'mission_controller': 'idle',
                    'photo_intelligence': 'idle',
                    'recovery_manager': 'idle'
                },
                'cameras': camera_status,
                'cpu_usage': 0.0,
                'memory_usage': 0.0,
                'disk_usage': 0.0
            }

            web_msg = String()
            web_msg.data = json.dumps(web_status)
            self.web_system_health_pub.publish(web_msg)

        except Exception as e:
            self.logger.error(f"Error publishing system status: {e}")
    
    def _publish_fusion_diagnostics(self):
        """Publish fusion diagnostics message"""
        # Implementation depends on message definition
        pass

    def _publish_web_camera_detections(self):
        """Publish camera detection data as JSON for web interface"""
        if not self.system_active or not self.zed_subscriber:
            return

        try:
            # Get detection data from ZED subscriber
            camera_data = self.zed_subscriber.get_camera_detections_for_web()
            detection_stats = self.zed_subscriber.get_detection_statistics()

            # Build JSON payload with per-camera detection info
            web_data = {
                'timestamp': time.time(),
                'cameras': {}
            }

            for camera_name in self.camera_config.keys():
                if not self.camera_config[camera_name].get('enabled', True):
                    continue

                # Get detection data for this camera
                cam_detections = camera_data.get(camera_name, {'count': 0, 'detections': []})
                cam_stats = detection_stats.get(camera_name, {})

                web_data['cameras'][camera_name] = {
                    'count': cam_detections['count'],
                    'detection_rate_hz': cam_stats.get('detection_rate_hz', 0.0),
                    'average_confidence': cam_stats.get('average_confidence', 0.0),
                    'active': cam_stats.get('active', False),
                    'detections': cam_detections['detections']
                }

            # Publish as JSON string
            msg = String()
            msg.data = json.dumps(web_data)
            self.web_detections_pub.publish(msg)

        except Exception as e:
            self.logger.error(f"Error publishing web camera detections: {e}")

    def shutdown_node(self):
        """Clean shutdown of the node"""
        self.logger.info("Shutting down ZED Fusion Node")
        
        self.system_active = False
        
        # Shutdown components
        if self.zed_subscriber:
            self.zed_subscriber.shutdown()
        
        if self.frame_synchronizer:
            self.frame_synchronizer.shutdown()
        
        if self.coordinate_transformer:
            self.coordinate_transformer.shutdown()
        
        self.logger.info("Shutdown completed")


def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)
    
    try:
        node = ZedFusionNode()
        
        # Use MultiThreadedExecutor for better performance
        from rclpy.executors import MultiThreadedExecutor
        executor = MultiThreadedExecutor(num_threads=4)
        
        try:
            rclpy.spin(node, executor=executor)
        except KeyboardInterrupt:
            pass
        finally:
            node.shutdown_node()
            node.destroy_node()
            
    except Exception as e:
        print(f"Failed to start ZED fusion node: {e}")
    finally:
        try:
            rclpy.shutdown()
        except:
            pass


if __name__ == '__main__':
    main()
