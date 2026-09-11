#!/usr/bin/env python3
"""
Photo Capture Bridge Node

Lightweight ROS2 node that bridges between detection system and photo capture system.

Subscribes to (ROS):
  - /human_clustering/humans (HumansList) - tracking data from detection node
  - /zedx_{front,left,right}/zed_node/depth/depth_registered/compressedDepth - depth images
  - /zedx_{front,left,right}/zed_node/left/camera_info - camera calibration

Publishes to (MQTT):
  - /photo_capture/tracking_results - enhanced tracking data with depth_mean & camera_matrix

Adds:
  - depth_mean: Average depth in each person's bounding box
  - camera_matrix: Camera intrinsics for gimbal targeting
  - 3-camera synchronization (optional)

Does NOT:
  - Publish RGB images (unnecessary overhead)
  - Run YOLO/detection (already done by direct_zed_detection_node)
  - Modify detection results (just enriches them)

Author: Vision Team
Date: 2026-02-23
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
import paho.mqtt.client as mqtt
import json
import numpy as np
import cv2
from collections import deque, defaultdict
import time
from typing import Dict, List, Optional, Any, Tuple

# ROS message types
try:
    from manriix_perception.msg import HumansList, HumanTrackingData, ObjectsList, ObjectTrackingData
    from sensor_msgs.msg import CompressedImage, CameraInfo
    from geometry_msgs.msg import Point, Vector3
    MSGS_AVAILABLE = True
except ImportError:
    MSGS_AVAILABLE = False
    print("WARNING: ROS messages not available. Bridge will not work properly.")


class PhotoCaptureBridge(Node):
    """Bridge between ROS detection system and MQTT photo capture system"""

    def __init__(self):
        super().__init__('photo_capture_bridge')
        
        self.get_logger().info("=" * 60)
        self.get_logger().info("Starting Photo Capture Bridge Node")
        self.get_logger().info("=" * 60)

        # Declare parameters
        self._declare_parameters()
        
        # Get parameters
        self.mqtt_enabled = self.get_parameter('mqtt_enabled').value
        self.mqtt_host = self.get_parameter('mqtt_host').value
        self.mqtt_port = self.get_parameter('mqtt_port').value
        self.mqtt_topic = self.get_parameter('mqtt_topic').value
        self.publish_objects = self.get_parameter('publish_objects').value
        self.mqtt_objects_topic = self.get_parameter('mqtt_objects_topic').value
        self.sync_enabled = self.get_parameter('sync_enabled').value
        self.sync_window = self.get_parameter('sync_window').value
        self.cameras = self.get_parameter('cameras').value
        
        self.get_logger().info(f"MQTT: {'enabled' if self.mqtt_enabled else 'disabled'}")
        self.get_logger().info(f"  Host: {self.mqtt_host}:{self.mqtt_port}")
        self.get_logger().info(f"  Humans topic: {self.mqtt_topic}")
        if self.publish_objects:
            self.get_logger().info(f"  Objects topic: {self.mqtt_objects_topic}")
        self.get_logger().info(f"Sync: {'enabled' if self.sync_enabled else 'disabled'}")
        if self.sync_enabled:
            self.get_logger().info(f"  Window: {self.sync_window}s")

        # Initialize MQTT client
        self.mqtt_client = None
        if self.mqtt_enabled:
            self._setup_mqtt()

        # Data caches (thread-safe with ROS single-threaded executor)
        self.depth_cache: Dict[str, np.ndarray] = {}
        self.camera_info_cache: Dict[str, Dict] = {}
        self.detection_buffers: Dict[str, deque] = {
            cam: deque(maxlen=30) for cam in self.cameras
        }
        
        # Statistics
        self.stats = {
            'detections_received': 0,
            'detections_enhanced': 0,
            'mqtt_published': 0,
            'objects_received': 0,       # NEW: multi-class objects count
            'objects_enhanced': 0,       # NEW: multi-class objects processed
            'mqtt_objects_published': 0, # NEW: objects messages published
            'depth_cache_updates': 0,
            'last_publish_time': 0.0,
            'last_objects_publish_time': 0.0,  # NEW
        }

        # Setup ROS subscribers
        self._setup_subscribers()
        
        # Status timer
        self.status_timer = self.create_timer(10.0, self._status_callback)
        
        self.get_logger().info("Photo Capture Bridge initialized successfully")

    def _declare_parameters(self):
        """Declare ROS2 parameters"""
        self.declare_parameter('mqtt_enabled', True)
        self.declare_parameter('mqtt_host', 'localhost')
        self.declare_parameter('mqtt_port', 1883)
        self.declare_parameter('mqtt_topic', '/photo_capture/tracking_results')
        
        # Multi-class detection support (NEW)
        self.declare_parameter('publish_objects', True)  # Enable multi-class forwarding
        self.declare_parameter('mqtt_objects_topic', '/photo_capture/objects_tracking')
        
        # 3-camera synchronization
        self.declare_parameter('sync_enabled', True)
        self.declare_parameter('sync_window', 0.2)  # 200ms window
        
        # Cameras to track
        self.declare_parameter('cameras', ['front', 'left', 'right'])

    def _setup_mqtt(self):
        """Initialize MQTT client"""
        try:
            self.mqtt_client = mqtt.Client()
            self.mqtt_client.connect(self.mqtt_host, self.mqtt_port, keepalive=60)
            self.mqtt_client.loop_start()
            self.get_logger().info(f"✅ MQTT connected to {self.mqtt_host}:{self.mqtt_port}")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to MQTT broker: {e}")
            self.mqtt_enabled = False

    def _setup_subscribers(self):
        """Setup all ROS2 subscriptions"""
        
        # QoS profiles
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        # Subscribe to human detections (main data source)
        self.humans_sub = self.create_subscription(
            HumansList,
            '/human_clustering/humans',
            self._humans_callback,
            reliable_qos
        )
        self.get_logger().info("Subscribed: /human_clustering/humans")

        # Subscribe to multi-class object detections (NEW)
        if self.publish_objects:
            self.objects_sub = self.create_subscription(
                ObjectsList,
                '/object_tracking/objects',
                self._objects_callback,
                reliable_qos
            )
            self.get_logger().info("Subscribed: /object_tracking/objects")

        # Subscribe to depth images (for depth_mean calculation)
        for cam_name in self.cameras:
            topic = f'/zedx_{cam_name}/zed_node/depth/depth_registered/compressedDepth'
            self.create_subscription(
                CompressedImage,
                topic,
                lambda msg, cam=cam_name: self._depth_callback(msg, cam),
                sensor_qos
            )
            self.get_logger().info(f"Subscribed: {topic}")

        # Subscribe to camera info (for camera_matrix)
        for cam_name in self.cameras:
            topic = f'/zedx_{cam_name}/zed_node/left/camera_info'
            self.create_subscription(
                CameraInfo,
                topic,
                lambda msg, cam=cam_name: self._camera_info_callback(msg, cam),
                reliable_qos
            )
            self.get_logger().info(f"Subscribed: {topic}")

    # ==================== ROS CALLBACKS ====================

    def _humans_callback(self, msg: HumansList):
        """Main callback - receives tracking data from detection node"""
        if not msg.humans:
            return  # No detections
        
        self.stats['detections_received'] += len(msg.humans)
        
        # Group detections by camera
        detections_by_camera = self._group_by_camera(msg.humans)
        
        # Enhance each detection with depth_mean
        enhanced = self._add_depth_info(detections_by_camera)
        
        # Add camera_matrix to each detection
        enhanced = self._add_camera_matrix(enhanced)
        
        # Build MQTT message
        mqtt_msg = self._build_mqtt_message(enhanced, msg.header.stamp)
        
        # Publish to MQTT
        if self.mqtt_enabled and mqtt_msg:
            self._publish_mqtt(mqtt_msg)

    def _objects_callback(self, msg: ObjectsList):
        """Callback for multi-class object detections (NEW)"""
        if not msg.objects:
            return  # No detections
        
        self.stats['objects_received'] += len(msg.objects)
        
        # Group detections by camera
        objects_by_camera = self._group_objects_by_camera(msg.objects)
        
        # Enhance each detection with depth_mean
        enhanced = self._add_depth_info_objects(objects_by_camera)
        
        # Add camera_matrix to each detection
        enhanced = self._add_camera_matrix(enhanced)
        
        # Build MQTT message
        mqtt_msg = self._build_objects_mqtt_message(enhanced, msg.header.stamp)
        
        # Publish to MQTT
        if self.mqtt_enabled and mqtt_msg:
            self._publish_mqtt_objects(mqtt_msg)

    def _depth_callback(self, msg: CompressedImage, camera_name: str):
        """Receive and cache depth images"""
        try:
            # Decompress depth image (16UC1 PNG format, millimeters)
            np_arr = np.frombuffer(msg.data, np.uint8)
            depth_mm = cv2.imdecode(np_arr, cv2.IMREAD_UNCHANGED)
            
            if depth_mm is not None:
                # Convert mm to meters
                depth_m = depth_mm.astype(np.float32) / 1000.0
                
                # Cache for depth_mean calculation
                self.depth_cache[camera_name] = depth_m
                self.stats['depth_cache_updates'] += 1
        except Exception as e:
            self.get_logger().warning(
                f"Failed to decode depth image from {camera_name}: {e}",
                throttle_duration_sec=5.0
            )

    def _camera_info_callback(self, msg: CameraInfo, camera_name: str):
        """Receive and cache camera calibration"""
        if camera_name in self.camera_info_cache:
            return  # Already cached (doesn't change)
        
        # Extract calibration parameters
        calib = {
            'width': msg.width,
            'height': msg.height,
            'fx': msg.k[0],
            'fy': msg.k[4],
            'cx': msg.k[2],
            'cy': msg.k[5],
            'distortion_model': msg.distortion_model,
            'distortion_coeffs': list(msg.d) if len(msg.d) > 0 else [0.0] * 5,
        }
        
        self.camera_info_cache[camera_name] = calib
        self.get_logger().info(f"✅ Cached camera calibration for {camera_name}")

    # ==================== DATA PROCESSING ====================

    def _group_by_camera(self, humans: List[HumanTrackingData]) -> Dict[str, List]:
        """Group detections by camera name"""
        grouped = defaultdict(list)
        
        for human in humans:
            cam_name = human.camera_name
            
            # Convert to dict for easier manipulation
            detection = {
                'tracking_id': human.tracking_id,
                'position': {
                    'x': human.position.x,
                    'y': human.position.y,
                    'z': human.position.z,
                },
                'confidence': human.confidence,
                'distance': human.distance,
                'velocity': {
                    'x': human.velocity.x,
                    'y': human.velocity.y,
                    'z': human.velocity.z,
                },
                'velocity_magnitude': human.velocity_magnitude,
                'camera_name': cam_name,
            }
            
            grouped[cam_name].append(detection)
        
        return dict(grouped)

    def _group_objects_by_camera(self, objects: List[ObjectTrackingData]) -> Dict[str, List]:
        """Group multi-class object detections by camera name (NEW)"""
        grouped = defaultdict(list)
        
        for obj in objects:
            cam_name = obj.camera_name
            
            # Convert to dict for easier manipulation (includes class info)
            detection = {
                'tracking_id': obj.tracking_id,
                'class_id': obj.class_id,         # NEW: COCO class ID
                'class_name': obj.class_name,     # NEW: Human-readable name
                'position': {
                    'x': obj.position.x,
                    'y': obj.position.y,
                    'z': obj.position.z,
                },
                'confidence': obj.confidence,
                'distance': obj.distance,
                'velocity': {
                    'x': obj.velocity.x,
                    'y': obj.velocity.y,
                    'z': obj.velocity.z,
                },
                'velocity_magnitude': obj.velocity_magnitude,
                'camera_name': cam_name,
            }
            
            grouped[cam_name].append(detection)
        
        return dict(grouped)

    def _add_depth_info(self, detections_by_camera: Dict[str, List]) -> Dict[str, List]:
        """Add depth_mean to each detection using cached depth images"""
        enhanced = {}
        
        for cam_name, detections in detections_by_camera.items():
            enhanced_dets = []
            
            # Get cached depth image for this camera
            depth_img = self.depth_cache.get(cam_name)
            
            for det in detections:
                # Start with existing detection
                enhanced_det = det.copy()
                
                # Try to calculate depth_mean from bounding box
                # Note: HumanTrackingData doesn't include bbox_2d by default
                # We'll use the distance from detection as fallback
                depth_mean = det['distance']  # Fallback to distance
                
                # TODO: If you add bbox_2d to HumanTrackingData message,
                # uncomment this to calculate actual depth_mean:
                # if depth_img is not None and 'bbox_2d' in det:
                #     depth_mean = self._calculate_depth_mean(
                #         depth_img, det['bbox_2d']
                #     )
                
                enhanced_det['depth_mean'] = depth_mean
                enhanced_dets.append(enhanced_det)
                self.stats['detections_enhanced'] += 1
            
            enhanced[cam_name] = enhanced_dets
        
        return enhanced

    def _add_depth_info_objects(self, objects_by_camera: Dict[str, List]) -> Dict[str, List]:
        """Add depth_mean to multi-class objects using cached depth images (NEW)"""
        enhanced = {}
        
        for cam_name, objects in objects_by_camera.items():
            enhanced_objs = []
            
            # Get cached depth image for this camera
            depth_img = self.depth_cache.get(cam_name)
            
            for obj in objects:
                # Start with existing object
                enhanced_obj = obj.copy()
                
                # Use distance as depth_mean fallback
                depth_mean = obj['distance']
                
                # TODO: Calculate from bbox_2d if available
                # if depth_img is not None and 'bbox_2d' in obj:
                #     depth_mean = self._calculate_depth_mean(depth_img, obj['bbox_2d'])
                
                enhanced_obj['depth_mean'] = depth_mean
                enhanced_objs.append(enhanced_obj)
                self.stats['objects_enhanced'] += 1
            
            enhanced[cam_name] = enhanced_objs
        
        return enhanced

    def _calculate_depth_mean(self, depth_img: np.ndarray, bbox_2d: List[List[float]]) -> float:
        """Calculate average depth in bounding box"""
        try:
            if len(bbox_2d) < 4:
                return 0.0
            
            # Extract bbox coordinates
            x_coords = [pt[0] for pt in bbox_2d]
            y_coords = [pt[1] for pt in bbox_2d]
            
            x1 = int(max(0, min(x_coords)))
            y1 = int(max(0, min(y_coords)))
            x2 = int(min(depth_img.shape[1], max(x_coords)))
            y2 = int(min(depth_img.shape[0], max(y_coords)))
            
            if x2 <= x1 or y2 <= y1:
                return 0.0
            
            # Extract depth values in bbox
            bbox_depth = depth_img[y1:y2, x1:x2]
            
            # Filter valid depth values (0.3m to 10m)
            valid_depth = bbox_depth[(bbox_depth > 0.3) & (bbox_depth < 10.0)]
            
            if len(valid_depth) > 0:
                return float(np.mean(valid_depth))
            else:
                return 0.0
                
        except Exception as e:
            self.get_logger().warning(f"Depth calculation failed: {e}")
            return 0.0

    def _add_camera_matrix(self, detections_by_camera: Dict[str, List]) -> Dict[str, List]:
        """Add camera_matrix to each detection"""
        enhanced = {}
        
        for cam_name, detections in detections_by_camera.items():
            # Get cached camera calibration
            calib = self.camera_info_cache.get(cam_name)
            
            enhanced_dets = []
            for det in detections:
                enhanced_det = det.copy()
                enhanced_det['camera_matrix'] = calib  # Will be None if not cached yet
                enhanced_dets.append(enhanced_det)
            
            enhanced[cam_name] = enhanced_dets
        
        return enhanced

    def _build_mqtt_message(self, detections_by_camera: Dict[str, List], 
                           timestamp) -> Optional[Dict]:
        """Build final MQTT message"""
        if not detections_by_camera:
            return None
        
        # Convert ROS timestamp to float
        ros_time = timestamp.sec + timestamp.nanosec / 1e9
        
        msg = {
            'timestamp': ros_time,
            'cameras': detections_by_camera,
            'total_detections': sum(len(dets) for dets in detections_by_camera.values()),
        }
        
        return msg

    def _build_objects_mqtt_message(self, objects_by_camera: Dict[str, List], 
                                    timestamp) -> Optional[Dict]:
        """Build MQTT message for multi-class objects (NEW)"""
        if not objects_by_camera:
            return None
        
        # Convert ROS timestamp to float
        ros_time = timestamp.sec + timestamp.nanosec / 1e9
        
        msg = {
            'timestamp': ros_time,
            'cameras': objects_by_camera,
            'total_detections': sum(len(objs) for objs in objects_by_camera.values()),
        }
        
        return msg

    # ==================== MQTT PUBLISHING ====================

    def _publish_mqtt(self, msg: Dict):
        """Publish message to MQTT"""
        if not self.mqtt_client:
            return
        
        try:
            payload = json.dumps(msg)
            result = self.mqtt_client.publish(
                self.mqtt_topic,
                payload,
                qos=0  # Fire-and-forget for low latency
            )
            
            if result.rc == mqtt.MQTT_ERR_SUCCESS:
                self.stats['mqtt_published'] += 1
                self.stats['last_publish_time'] = time.time()
            else:
                self.get_logger().warning(
                    f"MQTT publish failed: {result.rc}",
                    throttle_duration_sec=5.0
                )
                
        except Exception as e:
            self.get_logger().error(
                f"MQTT publish exception: {e}",
                throttle_duration_sec=5.0
            )

    def _publish_mqtt_objects(self, msg: Dict):
        """Publish multi-class objects message to MQTT (NEW)"""
        if not self.mqtt_client:
            return
        
        try:
            payload = json.dumps(msg)
            result = self.mqtt_client.publish(
                self.mqtt_objects_topic,
                payload,
                qos=0  # Fire-and-forget for low latency
            )
            
            if result.rc == mqtt.MQTT_ERR_SUCCESS:
                self.stats['mqtt_objects_published'] += 1
                self.stats['last_objects_publish_time'] = time.time()
            else:
                self.get_logger().warning(
                    f"MQTT objects publish failed: {result.rc}",
                    throttle_duration_sec=5.0
                )
                
        except Exception as e:
            self.get_logger().error(
                f"MQTT objects publish exception: {e}",
                throttle_duration_sec=5.0
            )

    # ==================== STATUS ====================

    def _status_callback(self):
        """Periodic status logging"""
        age = time.time() - self.stats['last_publish_time'] if self.stats['last_publish_time'] > 0 else 0
        objects_age = time.time() - self.stats['last_objects_publish_time'] if self.stats['last_objects_publish_time'] > 0 else 0
        
        self.get_logger().info(
            f"[BRIDGE] "
            f"Humans: {self.stats['detections_received']} rx, {self.stats['mqtt_published']} pub ({age:.1f}s ago) | "
            f"Objects: {self.stats['objects_received']} rx, {self.stats['mqtt_objects_published']} pub ({objects_age:.1f}s ago) | "
            f"Depth updates: {self.stats['depth_cache_updates']} | "
            f"Cameras: {len(self.camera_info_cache)}/3"
        )

    def destroy_node(self):
        """Cleanup on shutdown"""
        self.get_logger().info("Shutting down Photo Capture Bridge...")
        
        if self.mqtt_client:
            self.mqtt_client.loop_stop()
            self.mqtt_client.disconnect()
        
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    if not MSGS_AVAILABLE:
        print("ERROR: Required ROS messages not available. Cannot start bridge.")
        return
    
    node = PhotoCaptureBridge()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
