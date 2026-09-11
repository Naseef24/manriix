#!/usr/bin/env python3
"""
ZED Subscriber Core Module

Handles subscription to ZED ObjectsStamped messages from multiple cameras
and initial processing of detection data.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.callback_groups import ReentrantCallbackGroup
from zed_msgs.msg import ObjectsStamped
from std_msgs.msg import Header
from geometry_msgs.msg import Point
import threading
import time
from typing import Dict, List, Callable, Optional, Any
from dataclasses import dataclass
from collections import deque


@dataclass
class ZedDetection:
    """Single ZED detection data structure - matches zed_msgs/Object"""
    # ZED message fields (matching zed_msgs/Object.msg)
    label: str                          # Object label (e.g. "PERSON")
    label_id: int                       # Object label ID
    sublabel: str                       # Object sublabel
    confidence: float                   # Object confidence level (0-100)
    position: Point                     # Object centroid position [x,y,z]
    position_covariance: List[float]    # Position covariance [6]
    velocity: List[float]               # Object velocity [vx,vy,vz]
    tracking_available: bool            # Tracking available flag
    tracking_state: int                 # 0=OFF, 1=OK, 2=SEARCHING, 3=TERMINATE
    action_state: int                   # 0=IDLE, 2=MOVING
    dimensions_3d: List[float]          # 3D dimensions [width, height, length]
    bounding_box_2d: List[List[float]]  # 2D bounding box corners [[x,y], ...]

    # Additional fields for our system
    camera_name: str                    # Source camera name
    camera_frame_id: str                # Camera frame ID
    timestamp: float                    # Detection timestamp
    header: Header                      # ROS header
    
    # Compatibility properties for existing code
    @property
    def tracking_id(self) -> int:
        """Compatibility property - use label_id as tracking ID"""
        return self.label_id
    
    @property
    def object_class(self) -> str:
        """Compatibility property"""
        return self.label
        
    @property
    def object_subclass(self) -> str:
        """Compatibility property"""
        return self.sublabel
        
    @property
    def class_id(self) -> int:
        """Compatibility property - map ZED label to standard class ID"""
        # Map ZED label strings to standard class IDs
        label_to_class_map = {
            'PERSON': 0,
            'VEHICLE': 1,
            'BAG': 2, 
            'ANIMAL': 3,
            'ELECTRONICS': 4,
            'FRUIT_VEGETABLE': 5,
            'SPORT': 6
        }
        return label_to_class_map.get(self.label, 0)  # Default to PERSON if unknown
    
    @property
    def subclass_id(self) -> int:
        """Compatibility property - use label_id for subclass ID"""
        return self.label_id 


class ZedSubscriber:
    """
    Core subscriber class for ZED ObjectsStamped messages
    
    Manages subscriptions to multiple ZED cameras and provides
    callbacks for processing detection data.
    """
    
    def __init__(self, node: Node, camera_config: Dict[str, Any], 
                 detection_config: Dict[str, Any]):
        """
        Initialize ZED subscriber
        
        Args:
            node: ROS2 node instance
            camera_config: Camera configuration dictionary
            detection_config: Detection filtering configuration
        """
        self.node = node
        self.camera_config = camera_config
        self.detection_config = detection_config
        self.logger = node.get_logger()

        # Thread-safe callback handling
        self.callback_group = ReentrantCallbackGroup()
        self.detection_callbacks: List[Callable] = []
        self.lock = threading.Lock()

        # QoS profile for sensor data
        self.qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )

        # Subscribers dictionary
        self.subscribers: Dict[str, Any] = {}

        # Detection statistics
        self.detection_stats: Dict[str, Dict[str, Any]] = {}

        # Recent detections buffer for performance monitoring
        self.recent_detections = deque(maxlen=1000)

        # Store latest detections per camera (for web interface)
        self.latest_camera_detections: Dict[str, List[ZedDetection]] = {}

        # =====================================================================
        # DETECTION PIPELINE CONFIGURATION
        # =====================================================================
        # Determine which pipeline to use: 'ZED_INBUILT' or 'YOLO'
        self.detection_pipeline = detection_config.get('detection_pipeline', 'ZED_INBUILT').upper()
        self.logger.info(f"Detection pipeline: {self.detection_pipeline}")

        # Get pipeline-specific settings
        if self.detection_pipeline == 'YOLO':
            pipeline_config = detection_config.get('yolo', {})
            self.human_label = pipeline_config.get('human_label', 'person')
        else:
            pipeline_config = detection_config.get('zed_inbuilt', {})
            self.human_label = pipeline_config.get('human_label', 'PERSON')

        # Enabled labels for this pipeline
        self.enabled_labels = pipeline_config.get('enabled_labels', [self.human_label])
        self.confidence_thresholds = pipeline_config.get('confidence_thresholds', {})
        self.default_confidence = pipeline_config.get('default_confidence', 50.0)

        # Common settings
        common_config = detection_config.get('common', {})
        self.min_detection_range = common_config.get('min_detection_range', 0.3)
        self.max_detection_range = common_config.get('max_detection_range', 10.0)
        self.accept_searching_state = common_config.get('accept_tracking_searching', True)
        self.min_object_size = common_config.get('min_object_size', 0.1)
        self.max_object_size = common_config.get('max_object_size', 3.0)

        self.logger.info(f"Enabled labels: {self.enabled_labels}")
        self.logger.info(f"Human label: {self.human_label}")

        # Initialize subscribers for active cameras
        self._initialize_subscribers()
        
        # Initialize statistics
        self._initialize_statistics()
        
        self.logger.info(f"ZED Subscriber initialized for {len(self.subscribers)} cameras")
    
    def _initialize_subscribers(self):
        """Initialize ROS2 subscribers for all enabled cameras"""
        for camera_name, camera_info in self.camera_config.items():
            if not camera_info.get('enabled', True):
                self.logger.info(f"Camera {camera_name} disabled, skipping subscription")
                continue
            
            topic = camera_info.get('topic')
            if not topic:
                self.logger.warning(f"No topic specified for camera {camera_name}")
                continue
            
            try:
                subscriber = self.node.create_subscription(
                    ObjectsStamped,
                    topic,
                    lambda msg, cam=camera_name: self._objects_callback(msg, cam),
                    self.qos_profile,
                    callback_group=self.callback_group
                )
                
                self.subscribers[camera_name] = subscriber
                self.logger.info(f"Subscribed to {topic} for camera {camera_name}")
                
            except Exception as e:
                self.logger.error(f"Failed to create subscriber for {camera_name}: {e}")
    
    def _initialize_statistics(self):
        """Initialize detection statistics tracking"""
        for camera_name in self.camera_config.keys():
            if self.camera_config[camera_name].get('enabled', True):
                self.detection_stats[camera_name] = {
                    'total_detections': 0,
                    'filtered_detections': 0,
                    'last_detection_time': None,
                    'average_confidence': 0.0,
                    'detection_rate_hz': 0.0,
                    'active': False
                }
    
    def _objects_callback(self, msg: ObjectsStamped, camera_name: str):
        """
        Process incoming ObjectsStamped messages

        Uses ZED's built-in tracking_state for stability instead of custom filtering.

        Args:
            msg: ObjectsStamped message
            camera_name: Name of the camera that produced the detection
        """
        try:
            current_time = time.time()

            # Update camera activity status
            with self.lock:
                if camera_name in self.detection_stats:
                    self.detection_stats[camera_name]['active'] = True
                    self.detection_stats[camera_name]['last_detection_time'] = current_time

            # Process each detected object
            processed_detections = []
            total_confidence = 0.0
            filtered_count = 0

            # Debug: Log all unique labels received (once per camera startup)
            if not hasattr(self, '_labels_logged'):
                self._labels_logged = {}  # camera -> set of labels seen

            if camera_name not in self._labels_logged:
                self._labels_logged[camera_name] = set()

            for obj in msg.objects:
                # Debug: Log new label types as they're first seen
                if obj.label not in self._labels_logged[camera_name]:
                    is_enabled = obj.label.lower() in [l.lower() for l in self.enabled_labels]
                    status = "ACCEPTED" if is_enabled else "FILTERED (not in enabled_labels)"
                    self.logger.info(f"[DEBUG] {camera_name}: New label '{obj.label}' detected - {status}")
                    self.logger.info(f"        Enabled labels: {self.enabled_labels}, Pipeline: {self.detection_pipeline}")
                    self._labels_logged[camera_name].add(obj.label)

                # Check tracking_state first (ZED's built-in tracking)
                # 0=OFF (invalid), 1=OK (valid), 2=SEARCHING (occluded), 3=TERMINATE (deleted)
                tracking_state = getattr(obj, 'tracking_state', 1)

                # Accept OK state, optionally SEARCHING
                if tracking_state == 0 or tracking_state == 3:
                    filtered_count += 1
                    continue

                if tracking_state == 2 and not self.accept_searching_state:
                    filtered_count += 1
                    continue

                # Apply additional filtering (confidence, range, class)
                if self._should_filter_detection(obj, camera_name):
                    filtered_count += 1
                    continue

                # Convert to ZedDetection format
                detection = self._convert_to_zed_detection(obj, msg.header, camera_name, current_time)
                if detection:
                    processed_detections.append(detection)
                    total_confidence += detection.confidence

            # Update statistics
            with self.lock:
                stats = self.detection_stats[camera_name]
                stats['total_detections'] += len(msg.objects)
                stats['filtered_detections'] += filtered_count

                if processed_detections:
                    stats['average_confidence'] = total_confidence / len(processed_detections)

                # Calculate detection rate
                self._update_detection_rate(camera_name, current_time)

            # Store for performance monitoring
            self.recent_detections.append({
                'camera': camera_name,
                'timestamp': current_time,
                'detection_count': len(processed_detections),
                'filtered_count': filtered_count
            })

            # Store latest detections per camera (for web interface)
            with self.lock:
                self.latest_camera_detections[camera_name] = processed_detections

            # Call registered callbacks with processed detections
            if processed_detections:
                self._notify_callbacks(processed_detections, camera_name)

        except Exception as e:
            self.logger.error(f"Error processing detections from {camera_name}: {e}")

    
    def _should_filter_detection(self, obj: Any, camera_name: str) -> bool:
        """
        Determine if a detection should be filtered out

        Uses pipeline-specific settings from detection_config.yaml.
        Supports both ZED built-in (uppercase labels) and YOLO (lowercase COCO labels).

        Args:
            obj: ZED object from ObjectsStamped message (zed_msgs/Object)
            camera_name: Name of source camera

        Returns:
            True if detection should be filtered out
        """
        # Get label (case-insensitive matching)
        obj_label = obj.label if obj.label else ''
        obj_label_lower = obj_label.lower()

        # Check object class filter using enabled_labels from config
        enabled_labels_lower = [l.lower() for l in self.enabled_labels]
        if obj_label_lower not in enabled_labels_lower:
            return True

        # Check confidence threshold (use per-class threshold or default)
        min_confidence = self.confidence_thresholds.get(obj_label, self.default_confidence)
        if obj.confidence < min_confidence:
            return True

        # Check detection range using float32[3] position array
        try:
            if len(obj.position) >= 3:
                distance = (obj.position[0]**2 + obj.position[1]**2 + obj.position[2]**2)**0.5
                if distance < self.min_detection_range or distance > self.max_detection_range:
                    return True
        except (AttributeError, IndexError):
            # If position array is invalid, skip range check
            pass

        # Check object size constraints
        try:
            if len(obj.dimensions_3d) >= 3:
                max_dimension = max(obj.dimensions_3d[:3])
                if max_dimension < self.min_object_size or max_dimension > self.max_object_size:
                    return True
        except (TypeError, AttributeError):
            pass  # dimensions_3d not available or invalid

        return False
    
    def _convert_to_zed_detection(self, obj: Any, header: Header,
                                 camera_name: str, timestamp: float) -> Optional[ZedDetection]:
        """
        Convert ZED object to internal ZedDetection format

        Args:
            obj: ZED object from ObjectsStamped message (zed_msgs/Object)
            header: Message header
            camera_name: Name of source camera
            timestamp: Detection timestamp

        Returns:
            ZedDetection instance or None if conversion fails
        """
        try:
            # Create position Point from obj.position (float32[3] array)
            position = Point()
            position.x = float(obj.position[0])
            position.y = float(obj.position[1])
            position.z = float(obj.position[2])

            # Extract position covariance (6 elements) - handle numpy arrays safely
            try:
                position_covariance = [float(x) for x in obj.position_covariance[:6]]
                if len(position_covariance) < 6:
                    position_covariance.extend([0.0] * (6 - len(position_covariance)))
            except (TypeError, IndexError):
                position_covariance = [0.0] * 6

            # Extract velocity (3 elements) - handle numpy arrays safely
            try:
                velocity = [float(x) for x in obj.velocity[:3]]
                if len(velocity) < 3:
                    velocity.extend([0.0] * (3 - len(velocity)))
            except (TypeError, IndexError):
                velocity = [0.0, 0.0, 0.0]

            # Extract 3D dimensions (3 elements) - handle numpy arrays safely
            try:
                dimensions_3d = [float(x) for x in obj.dimensions_3d[:3]]
                if len(dimensions_3d) < 3:
                    dimensions_3d.extend([0.5] * (3 - len(dimensions_3d)))
            except (TypeError, IndexError):
                dimensions_3d = [0.5, 1.7, 0.5]

            # Extract 2D bounding box corners (ZED format: bounding_box_2d.corners[i].kp = [x, y])
            # Corner order: [top-left, top-right, bottom-right, bottom-left]
            bounding_box_2d = []
            try:
                if hasattr(obj, 'bounding_box_2d') and hasattr(obj.bounding_box_2d, 'corners'):
                    corners = obj.bounding_box_2d.corners
                    for corner in corners:
                        if hasattr(corner, 'kp') and len(corner.kp) >= 2:
                            bounding_box_2d.append([float(corner.kp[0]), float(corner.kp[1])])

            except Exception as e:
                self.logger.warning(f"Bounding box extraction failed: {e}")

            # Get camera frame ID
            camera_frame_id = self.camera_config[camera_name].get('frame_id', f"{camera_name}_frame")

            # Create ZedDetection using actual ZED message fields
            return ZedDetection(
                # ZED message fields (direct mapping)
                label=obj.label,
                label_id=obj.label_id,
                sublabel=obj.sublabel,
                confidence=obj.confidence,
                position=position,
                position_covariance=position_covariance,
                velocity=velocity,
                tracking_available=obj.tracking_available,
                tracking_state=obj.tracking_state,
                action_state=obj.action_state,
                dimensions_3d=dimensions_3d,
                bounding_box_2d=bounding_box_2d,

                # Additional fields for our system
                camera_name=camera_name,
                camera_frame_id=camera_frame_id,
                timestamp=timestamp,
                header=header
            )
            
        except Exception as e:
            self.logger.warning(f"Failed to convert detection from {camera_name}: {e}")
            return None
    
    def _update_detection_rate(self, camera_name: str, current_time: float):
        """Update detection rate calculation for a camera"""
        # Simple rate calculation using recent detections
        camera_detections = [d for d in self.recent_detections 
                           if d['camera'] == camera_name and 
                           current_time - d['timestamp'] < 1.0]
        
        self.detection_stats[camera_name]['detection_rate_hz'] = len(camera_detections)
    
    def _notify_callbacks(self, detections: List[ZedDetection], camera_name: str):
        """
        Notify all registered callbacks of new detections
        
        Args:
            detections: List of processed detections
            camera_name: Source camera name
        """
        with self.lock:
            for callback in self.detection_callbacks:
                try:
                    callback(detections, camera_name)
                except Exception as e:
                    self.logger.error(f"Error in detection callback: {e}")
    
    def register_detection_callback(self, callback: Callable[[List[ZedDetection], str], None]):
        """
        Register a callback to receive detection data
        
        Args:
            callback: Function to call with (detections, camera_name)
        """
        with self.lock:
            self.detection_callbacks.append(callback)
        
        self.logger.info(f"Registered detection callback: {callback.__name__}")
    
    def get_detection_statistics(self) -> Dict[str, Any]:
        """
        Get current detection statistics
        
        Returns:
            Dictionary containing detection statistics for all cameras
        """
        with self.lock:
            return dict(self.detection_stats)
    
    def get_camera_status(self) -> Dict[str, bool]:
        """
        Get current camera activity status

        Returns:
            Dictionary mapping camera names to active status
        """
        current_time = time.time()
        timeout = 2.0  # Camera considered inactive after 2 seconds

        with self.lock:
            status = {}
            for camera_name, stats in self.detection_stats.items():
                last_time = stats.get('last_detection_time')
                if last_time is None:
                    status[camera_name] = False
                else:
                    status[camera_name] = (current_time - last_time) < timeout
            return status

    def get_camera_detections_for_web(self) -> Dict[str, Any]:
        """
        Get latest camera detections with bounding boxes for web interface

        Returns:
            Dictionary with camera detection data ready for JSON serialization
        """
        with self.lock:
            result = {}
            for camera_name, detections in self.latest_camera_detections.items():
                result[camera_name] = {
                    'count': len(detections),
                    'detections': [
                        {
                            'label': d.label,
                            'label_id': d.label_id,
                            'confidence': d.confidence,
                            'bbox': d.bounding_box_2d,  # [[x,y], [x,y], [x,y], [x,y]]
                            'position': {
                                'x': d.position.x,
                                'y': d.position.y,
                                'z': d.position.z
                            }
                        }
                        for d in detections
                    ]
                }
            return result

    def is_human_label(self, label: str) -> bool:
        """
        Check if a label represents a human detection.

        Works with both ZED built-in ('PERSON') and YOLO ('person') labels.

        Args:
            label: The detection label to check

        Returns:
            True if the label represents a human
        """
        if not label:
            return False
        return label.lower() == self.human_label.lower()

    def get_human_label(self) -> str:
        """
        Get the human label for the current detection pipeline.

        Returns:
            'PERSON' for ZED_INBUILT, 'person' for YOLO
        """
        return self.human_label

    def get_detection_pipeline(self) -> str:
        """
        Get the current detection pipeline type.

        Returns:
            'ZED_INBUILT' or 'YOLO'
        """
        return self.detection_pipeline

    def shutdown(self):
        """Clean shutdown of subscribers"""
        self.logger.info("Shutting down ZED subscribers")
        
        for camera_name, subscriber in self.subscribers.items():
            try:
                self.node.destroy_subscription(subscriber)
            except Exception as e:
                self.logger.warning(f"Error destroying subscriber for {camera_name}: {e}")
        
        self.subscribers.clear()
        
        with self.lock:
            self.detection_callbacks.clear()