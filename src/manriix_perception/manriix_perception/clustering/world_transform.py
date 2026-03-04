#!/usr/bin/env python3
"""
Optimized World Transform - Batch coordinate transformation with caching
"""

import rclpy
from rclpy.node import Node
import tf2_ros
from geometry_msgs.msg import PointStamped, Point, TransformStamped
import tf2_geometry_msgs  # Required to register PointStamped with TF2
from typing import Dict, Optional, Tuple, List
import numpy as np
import time


class WorldTransform:
    """Enhanced coordinate transformation handler with caching and batch support"""

    def __init__(self, config: Dict, node: Node = None):
        """Initialize transform handler with map frame reference"""
        self.config = config
        self.node = node

        # Frame IDs
        self.base_frame = config['transform']['base_frame']
        self.map_frame = config['transform']['map_frame']
        self.camera_frame = config['transform']['camera_frame']
        self.tf_timeout = min(config['transform']['tf_timeout'], 0.1)  # Cap at 100ms

        # Initialize TF2 listener
        self.tf_buffer = tf2_ros.Buffer()
        if self.node:
            self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self.node)

        # Transform caching
        self._cached_transforms: Dict[str, Tuple[TransformStamped, float]] = {}
        self._cache_timeout = 0.1  # 100ms cache validity

        # Wait for transforms
        self._wait_for_transforms()

        if self.node:
            self.node.get_logger().info("WorldTransform initialized")

    def _wait_for_transforms(self):
        """Wait for required transforms to become available"""
        frames = [self.base_frame, self.camera_frame]
        max_retries = 10
        retry_delay = 1.0

        if self.node:
            self.node.get_logger().info("Waiting for transforms to become available...")

        # Wait for map transform
        for retry in range(max_retries):
            try:
                if self.tf_buffer.can_transform(
                    self.map_frame,
                    self.base_frame,
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=5.0)
                ):
                    if self.node:
                        self.node.get_logger().info("Map transform is available")
                    break
            except Exception as e:
                if retry < max_retries - 1:
                    if self.node:
                        self.node.get_logger().warn(f"Waiting for map transform (attempt {retry + 1}/{max_retries})")
                    time.sleep(retry_delay)
                else:
                    if self.node:
                        self.node.get_logger().error("Failed to get map transform")

    def _get_cached_transform(self, source_frame: str, target_frame: str) -> Optional[TransformStamped]:
        """Get cached transform or lookup and cache if expired"""
        cache_key = f"{source_frame}->{target_frame}"
        current_time = time.time()

        # Check cache
        if cache_key in self._cached_transforms:
            cached_tf, cache_time = self._cached_transforms[cache_key]
            if current_time - cache_time < self._cache_timeout:
                return cached_tf

        # Lookup and cache
        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=self.tf_timeout)
            )
            self._cached_transforms[cache_key] = (transform, current_time)
            return transform
        except Exception as e:
            if self.node:
                self.node.get_logger().debug(f"TF lookup failed {cache_key}: {e}")
            return None

    def _apply_transform(self, point: np.ndarray, transform: TransformStamped) -> np.ndarray:
        """Apply transform to a 3D point using matrix math (fast)"""
        # Extract translation
        tx = transform.transform.translation.x
        ty = transform.transform.translation.y
        tz = transform.transform.translation.z

        # Extract rotation quaternion
        q = transform.transform.rotation
        qw, qx, qy, qz = q.w, q.x, q.y, q.z

        # Rotation matrix from quaternion (optimized)
        r00 = 1 - 2 * (qy * qy + qz * qz)
        r01 = 2 * (qx * qy - qz * qw)
        r02 = 2 * (qx * qz + qy * qw)
        r10 = 2 * (qx * qy + qz * qw)
        r11 = 1 - 2 * (qx * qx + qz * qz)
        r12 = 2 * (qy * qz - qx * qw)
        r20 = 2 * (qx * qz - qy * qw)
        r21 = 2 * (qy * qz + qx * qw)
        r22 = 1 - 2 * (qx * qx + qy * qy)

        # Apply rotation and translation
        x_new = r00 * point[0] + r01 * point[1] + r02 * point[2] + tx
        y_new = r10 * point[0] + r11 * point[1] + r12 * point[2] + ty
        z_new = r20 * point[0] + r21 * point[1] + r22 * point[2] + tz

        return np.array([x_new, y_new, z_new])

    def _apply_transform_batch(self, points: np.ndarray, transform: TransformStamped) -> np.ndarray:
        """Apply transform to multiple 3D points using vectorized operations (fastest)"""
        if len(points) == 0:
            return np.array([])

        # Extract translation
        t = np.array([
            transform.transform.translation.x,
            transform.transform.translation.y,
            transform.transform.translation.z
        ])

        # Extract rotation quaternion
        q = transform.transform.rotation
        qw, qx, qy, qz = q.w, q.x, q.y, q.z

        # Build rotation matrix from quaternion
        R = np.array([
            [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
            [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
            [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)]
        ])

        # Apply rotation and translation to all points (vectorized)
        # points: Nx3, R: 3x3, result = points @ R.T + t
        transformed = points @ R.T + t

        return transformed

    def get_robot_pose(self) -> Optional[Tuple[np.ndarray, float]]:
        """Get robot's current position and orientation in map frame"""
        try:
            transform = self._get_cached_transform(self.base_frame, self.map_frame)
            if transform is None:
                return None

            position = np.array([
                transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z
            ])

            q = transform.transform.rotation
            yaw = np.arctan2(2.0 * (q.w * q.z + q.x * q.y),
                           1.0 - 2.0 * (q.y * q.y + q.z * q.z))

            return position, yaw

        except Exception as e:
            if self.node and self.config.get('debug', {}).get('print_transforms', False):
                self.node.get_logger().warn(f"Failed to get robot pose: {e}", throttle_duration_sec=1.0)
            return None

    def transform_to_map(self, points: List[Tuple[float, float, float]],
                        frame_id: str) -> List[Tuple[float, float]]:
        """
        Transform a list of points from given frame to map frame
        OPTIMIZED: Uses cached transforms and batch matrix operations
        """
        if not points:
            return []

        try:
            # Convert to numpy array for batch processing
            points_array = np.array(points, dtype=np.float64)

            # Get transform from source frame to base_frame (if needed)
            if frame_id != self.base_frame:
                tf_to_base = self._get_cached_transform(frame_id, self.base_frame)
                if tf_to_base is None:
                    if self.node:
                        self.node.get_logger().debug(f"No transform from {frame_id} to {self.base_frame}")
                    return []
                points_array = self._apply_transform_batch(points_array, tf_to_base)

            # Get transform from base_frame to map_frame
            tf_to_map = self._get_cached_transform(self.base_frame, self.map_frame)
            if tf_to_map is None:
                if self.node:
                    self.node.get_logger().debug(f"No transform from {self.base_frame} to {self.map_frame}")
                return []
            points_array = self._apply_transform_batch(points_array, tf_to_map)

            # Return as list of (x, y) tuples
            return [(float(p[0]), float(p[1])) for p in points_array]

        except Exception as e:
            if self.node:
                self.node.get_logger().warn(f"Failed to transform points: {e}", throttle_duration_sec=5.0)
            return []

    def transform_point_to_map(self, x: float, y: float, z: float,
                               frame_id: str) -> Optional[Tuple[float, float]]:
        """Transform a single point to map frame (convenience method)"""
        result = self.transform_to_map([(x, y, z)], frame_id)
        return result[0] if result else None
