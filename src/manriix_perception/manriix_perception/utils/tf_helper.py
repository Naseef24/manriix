#!/usr/bin/env python3
"""
TF2 Transform Helper Utilities

Provides utilities for coordinate transformation between camera frames and base_link.
"""

import rclpy
from rclpy.node import Node
from rclpy.time import Time, Duration
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import Point, PointStamped, TransformStamped
from tf2_msgs.msg import TFMessage
import numpy as np
from typing import Optional, Dict, Tuple, List
import threading
import time


class TransformationError(Exception):
    """Raised when coordinate transformation fails"""
    pass


class TFHelper:
    """
    Helper class for coordinate transformations using TF2
    
    Handles transformations between camera frames and robot base_link,
    with caching and fallback mechanisms for robust operation.
    """
    
    def __init__(self, node: Node, transform_config: Dict):
        """
        Initialize TF helper
        
        Args:
            node: ROS2 node instance
            transform_config: Transform configuration dictionary
        """
        self.node = node
        self.logger = node.get_logger()
        self.config = transform_config
        
        # TF2 setup
        self.tf_buffer = tf2_ros.Buffer(
            cache_time=Duration(seconds=transform_config.get('transform_cache_time', 1.0))
        )
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, node)
        
        # Transform configuration
        self.target_frame = transform_config.get('target_frame', 'base_link')
        self.use_tf2 = transform_config.get('use_tf2', True)
        self.tf_timeout = transform_config.get('tf_timeout', 0.1)
        
        # Transform cache for performance
        self.transform_cache: Dict[str, TransformStamped] = {}
        self.cache_timestamps: Dict[str, float] = {}
        self.cache_timeout = 0.5  # Cache valid for 0.5 seconds
        self.cache_lock = threading.Lock()
        
        # Static transforms as fallback
        self.static_transforms: Dict[str, Dict] = {}
        
        self.logger.info(f"TF Helper initialized, target frame: {self.target_frame}")
    
    def add_static_transform(self, source_frame: str, target_frame: str, 
                           translation: Tuple[float, float, float],
                           rotation: Tuple[float, float, float, float]):
        """
        Add a static transform as fallback when TF2 fails
        
        Args:
            source_frame: Source frame ID
            target_frame: Target frame ID  
            translation: Translation as (x, y, z)
            rotation: Rotation as quaternion (x, y, z, w)
        """
        key = f"{source_frame}_to_{target_frame}"
        self.static_transforms[key] = {
            'translation': translation,
            'rotation': rotation
        }
        
        self.logger.info(f"Added static transform: {source_frame} → {target_frame}")
    
    def load_static_transforms_from_config(self, camera_config: Dict):
        """
        Load static transforms from camera configuration
        
        Args:
            camera_config: Camera configuration dictionary
        """
        for camera_name, config in camera_config.items():
            if not config.get('enabled', True):
                continue
            
            source_frame = config.get('frame_id', f"{camera_name}_frame")
            position = config.get('position', [0, 0, 0])
            rotation_rpy = config.get('rotation', [0, 0, 0])
            
            # Convert RPY to quaternion
            quaternion = self._rpy_to_quaternion(*rotation_rpy)
            
            self.add_static_transform(
                source_frame, 
                self.target_frame,
                tuple(position),
                quaternion
            )
    
    def transform_point(self, point: Point, source_frame: str, 
                       target_frame: Optional[str] = None) -> Point:
        """
        Transform a point from source frame to target frame
        
        Args:
            point: Point to transform
            source_frame: Source coordinate frame
            target_frame: Target coordinate frame (default: self.target_frame)
            
        Returns:
            Transformed point
            
        Raises:
            TransformationError: If transformation fails
        """
        if target_frame is None:
            target_frame = self.target_frame
        
        if source_frame == target_frame:
            return point
        
        if not self.use_tf2:
            return self._transform_point_static(point, source_frame, target_frame)
        
        try:
            # Try TF2 first
            return self._transform_point_tf2(point, source_frame, target_frame)
        except Exception as e:
            self.logger.warning(f"TF2 transform failed: {e}, trying static fallback")
            return self._transform_point_static(point, source_frame, target_frame)
    
    def _transform_point_tf2(self, point: Point, source_frame: str, target_frame: str) -> Point:
        """Transform point using TF2"""
        # Check cache first
        transform = self._get_cached_transform(source_frame, target_frame)
        
        if transform is None:
            # Get transform from TF2
            try:
                transform = self.tf_buffer.lookup_transform(
                    target_frame,
                    source_frame,
                    Time(),
                    timeout=Duration(seconds=self.tf_timeout)
                )
                
                # Cache the transform
                self._cache_transform(source_frame, target_frame, transform)
                
            except Exception as e:
                raise TransformationError(f"TF2 lookup failed: {e}")
        
        # Create PointStamped
        point_stamped = PointStamped()
        point_stamped.header.frame_id = source_frame
        point_stamped.point = point
        
        # Transform
        transformed_point_stamped = tf2_geometry_msgs.do_transform_point(point_stamped, transform)
        
        return transformed_point_stamped.point
    
    def _transform_point_static(self, point: Point, source_frame: str, target_frame: str) -> Point:
        """Transform point using static transforms"""
        transform_key = f"{source_frame}_to_{target_frame}"
        
        if transform_key not in self.static_transforms:
            raise TransformationError(f"No static transform available: {transform_key}")
        
        static_tf = self.static_transforms[transform_key]
        
        # Extract transform data
        translation = static_tf['translation']
        rotation = static_tf['rotation']
        
        # Convert point to numpy array
        point_array = np.array([point.x, point.y, point.z, 1.0])
        
        # Create transformation matrix
        transform_matrix = self._create_transform_matrix(translation, rotation)
        
        # Apply transformation
        transformed = transform_matrix @ point_array
        
        # Create result point
        result = Point()
        result.x = float(transformed[0])
        result.y = float(transformed[1])
        result.z = float(transformed[2])
        
        return result
    
    def transform_point_list(self, points: List[Point], source_frame: str,
                           target_frame: Optional[str] = None) -> List[Point]:
        """
        Transform a list of points efficiently
        
        Args:
            points: List of points to transform
            source_frame: Source coordinate frame
            target_frame: Target coordinate frame
            
        Returns:
            List of transformed points
        """
        if target_frame is None:
            target_frame = self.target_frame
        
        if source_frame == target_frame:
            return points
        
        transformed_points = []
        for point in points:
            try:
                transformed_point = self.transform_point(point, source_frame, target_frame)
                transformed_points.append(transformed_point)
            except TransformationError as e:
                self.logger.warning(f"Failed to transform point: {e}")
                # Use original point as fallback
                transformed_points.append(point)
        
        return transformed_points
    
    def get_transform_matrix(self, source_frame: str, target_frame: str) -> np.ndarray:
        """
        Get transformation matrix from source to target frame
        
        Args:
            source_frame: Source frame
            target_frame: Target frame
            
        Returns:
            4x4 transformation matrix
        """
        if source_frame == target_frame:
            return np.eye(4)
        
        # Try to get transform
        try:
            if self.use_tf2:
                transform = self.tf_buffer.lookup_transform(
                    target_frame,
                    source_frame,
                    Time(),
                    timeout=Duration(seconds=self.tf_timeout)
                )
                
                # Extract translation and rotation
                t = transform.transform.translation
                r = transform.transform.rotation
                
                translation = (t.x, t.y, t.z)
                rotation = (r.x, r.y, r.z, r.w)
                
            else:
                # Use static transform
                transform_key = f"{source_frame}_to_{target_frame}"
                if transform_key not in self.static_transforms:
                    raise TransformationError(f"No transform available: {transform_key}")
                
                static_tf = self.static_transforms[transform_key]
                translation = static_tf['translation']
                rotation = static_tf['rotation']
            
            return self._create_transform_matrix(translation, rotation)
            
        except Exception as e:
            self.logger.error(f"Failed to get transform matrix: {e}")
            return np.eye(4)  # Return identity matrix as fallback
    
    def _get_cached_transform(self, source_frame: str, target_frame: str) -> Optional[TransformStamped]:
        """Get cached transform if still valid"""
        cache_key = f"{source_frame}_to_{target_frame}"
        current_time = time.time()
        
        with self.cache_lock:
            if cache_key in self.transform_cache:
                cache_time = self.cache_timestamps.get(cache_key, 0)
                if current_time - cache_time < self.cache_timeout:
                    return self.transform_cache[cache_key]
        
        return None
    
    def _cache_transform(self, source_frame: str, target_frame: str, transform: TransformStamped):
        """Cache a transform for reuse"""
        cache_key = f"{source_frame}_to_{target_frame}"
        current_time = time.time()
        
        with self.cache_lock:
            self.transform_cache[cache_key] = transform
            self.cache_timestamps[cache_key] = current_time
    
    def _create_transform_matrix(self, translation: Tuple[float, float, float],
                               rotation: Tuple[float, float, float, float]) -> np.ndarray:
        """
        Create 4x4 transformation matrix from translation and quaternion
        
        Args:
            translation: Translation (x, y, z)
            rotation: Quaternion (x, y, z, w)
            
        Returns:
            4x4 transformation matrix
        """
        # Extract components
        tx, ty, tz = translation
        qx, qy, qz, qw = rotation
        
        # Create rotation matrix from quaternion
        # Normalize quaternion
        norm = np.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
        if norm > 0:
            qx, qy, qz, qw = qx/norm, qy/norm, qz/norm, qw/norm
        
        # Rotation matrix
        R = np.array([
            [1 - 2*(qy*qy + qz*qz), 2*(qx*qy - qz*qw), 2*(qx*qz + qy*qw)],
            [2*(qx*qy + qz*qw), 1 - 2*(qx*qx + qz*qz), 2*(qy*qz - qx*qw)],
            [2*(qx*qz - qy*qw), 2*(qy*qz + qx*qw), 1 - 2*(qx*qx + qy*qy)]
        ])
        
        # Create 4x4 transformation matrix
        T = np.eye(4)
        T[0:3, 0:3] = R
        T[0:3, 3] = [tx, ty, tz]
        
        return T
    
    def _rpy_to_quaternion(self, roll: float, pitch: float, yaw: float) -> Tuple[float, float, float, float]:
        """
        Convert roll-pitch-yaw to quaternion
        
        Args:
            roll: Roll angle in radians
            pitch: Pitch angle in radians  
            yaw: Yaw angle in radians
            
        Returns:
            Quaternion as (x, y, z, w)
        """
        # Compute half angles
        cr = np.cos(roll * 0.5)
        sr = np.sin(roll * 0.5)
        cp = np.cos(pitch * 0.5)
        sp = np.sin(pitch * 0.5)
        cy = np.cos(yaw * 0.5)
        sy = np.sin(yaw * 0.5)
        
        # Compute quaternion components
        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
        
        return (qx, qy, qz, qw)
    
    def calculate_distance(self, point1: Point, point2: Point) -> float:
        """
        Calculate Euclidean distance between two points
        
        Args:
            point1: First point
            point2: Second point
            
        Returns:
            Distance in meters
        """
        dx = point1.x - point2.x
        dy = point1.y - point2.y
        dz = point1.z - point2.z
        return np.sqrt(dx*dx + dy*dy + dz*dz)
    
    def calculate_distance_from_origin(self, point: Point) -> float:
        """
        Calculate distance from origin (0,0,0)
        
        Args:
            point: Point to measure distance from origin
            
        Returns:
            Distance from origin in meters
        """
        return np.sqrt(point.x*point.x + point.y*point.y + point.z*point.z)
    
    def is_transform_available(self, source_frame: str, target_frame: str) -> bool:
        """
        Check if transform is available between frames
        
        Args:
            source_frame: Source frame
            target_frame: Target frame
            
        Returns:
            True if transform is available
        """
        if source_frame == target_frame:
            return True
        
        if self.use_tf2:
            try:
                self.tf_buffer.lookup_transform(
                    target_frame,
                    source_frame,
                    Time(),
                    timeout=Duration(seconds=0.01)  # Very short timeout for check
                )
                return True
            except:
                pass
        
        # Check static transforms
        transform_key = f"{source_frame}_to_{target_frame}"
        return transform_key in self.static_transforms
    
    def get_available_frames(self) -> List[str]:
        """
        Get list of available frames
        
        Returns:
            List of frame IDs
        """
        if self.use_tf2:
            try:
                # Get all frames from TF2
                frame_string = self.tf_buffer.all_frames_as_string()
                # Parse frame names from the string
                frames = []
                for line in frame_string.split('\n'):
                    if 'Frame' in line and 'exists' in line:
                        frame_name = line.split()[1]
                        if frame_name.startswith('/'):
                            frame_name = frame_name[1:]  # Remove leading slash
                        frames.append(frame_name)
                return frames
            except:
                pass
        
        # Return frames from static transforms
        frames = set()
        for key in self.static_transforms.keys():
            source, target = key.split('_to_')
            frames.add(source)
            frames.add(target)
        
        return list(frames)
    
    def shutdown(self):
        """Clean shutdown of TF helper"""
        self.logger.info("Shutting down TF helper")
        
        with self.cache_lock:
            self.transform_cache.clear()
            self.cache_timestamps.clear()
        
        # TF listener cleanup is handled automatically