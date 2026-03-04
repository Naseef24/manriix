#!/usr/bin/env python3
"""
Coordinate Transformer Module

Handles transformation of detections from camera coordinates to robot base_link frame.
"""

from typing import List, Dict, Any, Optional
import numpy as np
from geometry_msgs.msg import Point
from rclpy.node import Node
import logging

from manriix_perception.zed_subscriber import ZedDetection
from manriix_perception..utils.tf_helper import TFHelper, TransformationError


class CoordinateTransformer:
    """
    Transforms ZED detections from camera frames to robot base_link frame
    
    Handles coordinate system conversion for multi-camera fusion and
    ensures all detections are in a common reference frame.
    """
    
    def __init__(self, node: Node, transform_config: Dict[str, Any], 
                 camera_config: Dict[str, Any]):
        """
        Initialize coordinate transformer
        
        Args:
            node: ROS2 node instance
            transform_config: Transform configuration dictionary
            camera_config: Camera configuration dictionary
        """
        self.node = node
        self.logger = node.get_logger()
        self.transform_config = transform_config
        self.camera_config = camera_config
        
        # Initialize TF helper
        self.tf_helper = TFHelper(node, transform_config)
        
        # Load static transforms from camera configuration
        self.tf_helper.load_static_transforms_from_config(camera_config)
        
        # Target frame for all transformations
        self.target_frame = transform_config.get('target_frame', 'base_link')
        
        # Transform statistics
        self.transform_stats = {
            'successful_transforms': 0,
            'failed_transforms': 0,
            'average_transform_time': 0.0,
            'camera_transform_success': {}
        }
        
        # Initialize camera-specific stats
        for camera_name in camera_config.keys():
            if camera_config[camera_name].get('enabled', True):
                self.transform_stats['camera_transform_success'][camera_name] = {
                    'success': 0,
                    'failures': 0
                }
        
        self.logger.info(f"Coordinate transformer initialized, target frame: {self.target_frame}")
    
    def transform_detections(self, detections: List[ZedDetection]) -> List[ZedDetection]:
        """
        Transform a list of detections to the target frame
        
        Args:
            detections: List of ZED detections to transform
            
        Returns:
            List of transformed detections
        """
        if not detections:
            return []
        
        transformed_detections = []
        
        for detection in detections:
            try:
                transformed_detection = self.transform_single_detection(detection)
                if transformed_detection:
                    transformed_detections.append(transformed_detection)
            except Exception as e:
                self.logger.warning(f"Failed to transform detection {detection.tracking_id}: {e}")
                self._update_transform_stats(detection.camera_name, False)
        
        return transformed_detections
    
    def transform_single_detection(self, detection: ZedDetection) -> Optional[ZedDetection]:
        """
        Transform a single detection to the target frame
        
        Args:
            detection: ZED detection to transform
            
        Returns:
            Transformed detection or None if transformation fails
        """
        import time
        start_time = time.time()
        
        try:
            # Skip transformation if already in target frame
            if detection.camera_frame_id == self.target_frame:
                return detection
            
            # Transform position
            transformed_position = self.tf_helper.transform_point(
                detection.position, 
                detection.camera_frame_id, 
                self.target_frame
            )
            
            # Transform velocity if available
            transformed_velocity = self._transform_velocity(
                detection.velocity,
                detection.camera_frame_id,
                self.target_frame
            )
            
            # Create transformed detection using actual ZedDetection dataclass fields
            transformed_detection = ZedDetection(
                label=detection.label,
                label_id=detection.label_id,
                sublabel=detection.sublabel,
                confidence=detection.confidence,
                position=transformed_position,
                position_covariance=detection.position_covariance,
                velocity=transformed_velocity,
                tracking_available=detection.tracking_available,
                tracking_state=detection.tracking_state,
                action_state=detection.action_state,
                dimensions_3d=detection.dimensions_3d,
                bounding_box_2d=detection.bounding_box_2d,  # Pass through (2D image coords)
                camera_name=detection.camera_name,
                camera_frame_id=self.target_frame,  # Update frame ID
                timestamp=detection.timestamp,
                header=detection.header
            )
            
            # Update statistics
            transform_time = (time.time() - start_time) * 1000  # Convert to ms
            self._update_transform_stats(detection.camera_name, True, transform_time)
            
            return transformed_detection
            
        except TransformationError as e:
            self.logger.warning(f"Transform failed for detection from {detection.camera_name}: {e}")
            self._update_transform_stats(detection.camera_name, False)
            return None
        except Exception as e:
            self.logger.error(f"Unexpected error transforming detection: {e}")
            self._update_transform_stats(detection.camera_name, False)
            return None
    
    def _transform_velocity(self, velocity: List[float], source_frame: str, 
                          target_frame: str) -> List[float]:
        """
        Transform velocity vector from source to target frame
        
        Args:
            velocity: Velocity vector [vx, vy, vz]
            source_frame: Source frame ID
            target_frame: Target frame ID
            
        Returns:
            Transformed velocity vector
        """
        if len(velocity) < 3:
            return [0.0, 0.0, 0.0]
        
        if source_frame == target_frame:
            return velocity
        
        try:
            # Get transformation matrix
            transform_matrix = self.tf_helper.get_transform_matrix(source_frame, target_frame)
            
            # Extract rotation part (3x3)
            rotation_matrix = transform_matrix[0:3, 0:3]
            
            # Transform velocity (only rotation, no translation for vectors)
            velocity_array = np.array(velocity[:3])
            transformed_velocity = rotation_matrix @ velocity_array
            
            return transformed_velocity.tolist()
            
        except Exception as e:
            self.logger.warning(f"Failed to transform velocity: {e}")
            return [0.0, 0.0, 0.0]
    
    def validate_detection_position(self, detection: ZedDetection) -> bool:
        """
        Validate that detection position is reasonable
        
        Args:
            detection: Detection to validate
            
        Returns:
            True if position is valid
        """
        # Check for NaN or infinity
        if not all(np.isfinite([detection.position.x, detection.position.y, detection.position.z])):
            return False
        
        # Check reasonable range bounds
        max_range = self.transform_config.get('max_reasonable_range', 50.0)
        distance = self.tf_helper.calculate_distance_from_origin(detection.position)
        
        if distance > max_range:
            self.logger.debug(f"Detection {detection.tracking_id} at unreasonable distance: {distance:.2f}m")
            return False
        
        # Check minimum height (detections shouldn't be underground)
        min_height = self.transform_config.get('min_ground_height', -1.0)
        if detection.position.z < min_height:
            self.logger.debug(f"Detection {detection.tracking_id} below ground: z={detection.position.z:.2f}m")
            return False
        
        return True
    
    def get_camera_position_in_base_link(self, camera_name: str) -> Optional[Point]:
        """
        Get camera position in base_link frame
        
        Args:
            camera_name: Name of the camera
            
        Returns:
            Camera position as Point or None if not available
        """
        if camera_name not in self.camera_config:
            return None
        
        camera_config = self.camera_config[camera_name]
        camera_frame_id = camera_config.get('frame_id', f"{camera_name}_frame")
        
        try:
            # Transform origin point from camera frame to base_link
            origin = Point(x=0.0, y=0.0, z=0.0)
            camera_position = self.tf_helper.transform_point(
                origin, 
                camera_frame_id, 
                self.target_frame
            )
            return camera_position
            
        except TransformationError as e:
            self.logger.warning(f"Cannot get position for camera {camera_name}: {e}")
            return None
    
    def get_detection_range_from_camera(self, detection: ZedDetection) -> float:
        """
        Calculate distance from detection to its source camera
        
        Args:
            detection: ZED detection
            
        Returns:
            Distance in meters, or -1 if calculation fails
        """
        camera_position = self.get_camera_position_in_base_link(detection.camera_name)
        
        if camera_position is None:
            return -1.0
        
        return self.tf_helper.calculate_distance(detection.position, camera_position)
    
    def filter_detections_by_transform_quality(self, detections: List[ZedDetection],
                                             min_success_rate: float = 0.8) -> List[ZedDetection]:
        """
        Filter detections based on transformation quality
        
        Args:
            detections: List of detections to filter
            min_success_rate: Minimum required success rate for camera transforms
            
        Returns:
            Filtered list of detections
        """
        filtered_detections = []
        
        for detection in detections:
            camera_name = detection.camera_name
            
            # Check camera transform success rate
            if camera_name in self.transform_stats['camera_transform_success']:
                stats = self.transform_stats['camera_transform_success'][camera_name]
                total = stats['success'] + stats['failures']
                
                if total > 10:  # Only filter if we have enough samples
                    success_rate = stats['success'] / total
                    if success_rate < min_success_rate:
                        self.logger.debug(f"Filtering detection from {camera_name} due to low transform success rate: {success_rate:.2f}")
                        continue
            
            # Validate detection position
            if not self.validate_detection_position(detection):
                self.logger.debug(f"Filtering detection {detection.tracking_id} due to invalid position")
                continue
            
            filtered_detections.append(detection)
        
        return filtered_detections
    
    def _update_transform_stats(self, camera_name: str, success: bool, 
                              transform_time: float = 0.0):
        """
        Update transformation statistics
        
        Args:
            camera_name: Name of the camera
            success: Whether transformation was successful
            transform_time: Time taken for transformation in ms
        """
        if success:
            self.transform_stats['successful_transforms'] += 1
        else:
            self.transform_stats['failed_transforms'] += 1
        
        # Update camera-specific stats
        if camera_name in self.transform_stats['camera_transform_success']:
            if success:
                self.transform_stats['camera_transform_success'][camera_name]['success'] += 1
            else:
                self.transform_stats['camera_transform_success'][camera_name]['failures'] += 1
        
        # Update average transform time
        if success and transform_time > 0:
            total_successful = self.transform_stats['successful_transforms']
            current_avg = self.transform_stats['average_transform_time']
            new_avg = ((current_avg * (total_successful - 1)) + transform_time) / total_successful
            self.transform_stats['average_transform_time'] = new_avg
    
    def get_transform_statistics(self) -> Dict[str, Any]:
        """
        Get current transformation statistics
        
        Returns:
            Dictionary containing transformation statistics
        """
        return dict(self.transform_stats)
    
    def check_transform_health(self) -> Dict[str, Any]:
        """
        Check transformation system health
        
        Returns:
            Dictionary containing health information
        """
        health_info = {
            'overall_health': 'good',
            'issues': [],
            'camera_health': {}
        }
        
        # Check overall transform success rate
        total_transforms = (self.transform_stats['successful_transforms'] + 
                          self.transform_stats['failed_transforms'])
        
        if total_transforms > 0:
            success_rate = self.transform_stats['successful_transforms'] / total_transforms
            if success_rate < 0.9:
                health_info['overall_health'] = 'degraded'
                health_info['issues'].append(f"Low overall transform success rate: {success_rate:.2f}")
        
        # Check camera-specific health
        for camera_name, stats in self.transform_stats['camera_transform_success'].items():
            total = stats['success'] + stats['failures']
            if total > 0:
                camera_success_rate = stats['success'] / total
                health_info['camera_health'][camera_name] = {
                    'success_rate': camera_success_rate,
                    'total_attempts': total
                }
                
                if camera_success_rate < 0.8:
                    health_info['camera_health'][camera_name]['status'] = 'degraded'
                    health_info['issues'].append(
                        f"Camera {camera_name} low transform success: {camera_success_rate:.2f}"
                    )
                else:
                    health_info['camera_health'][camera_name]['status'] = 'good'
        
        # Check transform availability
        for camera_name, camera_config in self.camera_config.items():
            if not camera_config.get('enabled', True):
                continue
            
            camera_frame_id = camera_config.get('frame_id', f"{camera_name}_frame")
            
            if not self.tf_helper.is_transform_available(camera_frame_id, self.target_frame):
                health_info['camera_health'][camera_name] = {
                    'status': 'unavailable',
                    'error': 'Transform not available'
                }
                health_info['issues'].append(f"Transform unavailable for camera {camera_name}")
        
        if health_info['issues']:
            health_info['overall_health'] = 'degraded'
        
        return health_info
    
    def shutdown(self):
        """Clean shutdown of coordinate transformer"""
        self.logger.info("Shutting down coordinate transformer")
        self.tf_helper.shutdown()