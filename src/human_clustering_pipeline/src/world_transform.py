#!/usr/bin/env python3

import rospy
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PointStamped, Point, TransformStamped
from typing import Dict, Optional, Tuple, List
import numpy as np

class WorldTransform:
    """Enhanced coordinate transformation handler with SLAM support"""
    
    def __init__(self, config: Dict):
        """Initialize transform handler with map frame reference"""
        self.config = config
        
        # Frame IDs
        self.base_frame = config['transform']['base_frame']
        self.map_frame = config['transform']['map_frame']
        self.camera_frame = config['transform']['camera_frame']
        self.tf_timeout = config['transform']['tf_timeout']
        
        # Initialize TF2 listener
        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(30.0))  # Increased buffer time
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # Wait for transforms to become available
        self._wait_for_transforms()
        
        rospy.loginfo("WorldTransform initialized")

    def _wait_for_transforms(self):
        """Wait for required transforms to become available"""
        frames = [self.base_frame, self.camera_frame]
        timeout = rospy.Duration(5.0)
        max_retries = 10
        retry_delay = rospy.Duration(1.0)
        
        rospy.loginfo("Waiting for transforms to become available...")
        
        # First, wait for map frame to appear
        for retry in range(max_retries):
            try:
                # Check if we can transform from map to base_link
                self.tf_buffer.can_transform(
                    self.map_frame,
                    self.base_frame,
                    rospy.Time(0),
                    timeout
                )
                rospy.loginfo("Map transform is available")
                break
            except (tf2_ros.LookupException,
                    tf2_ros.ConnectivityException,
                    tf2_ros.ExtrapolationException) as e:
                if retry < max_retries - 1:
                    rospy.logwarn(f"Waiting for map transform (attempt {retry + 1}/{max_retries})")
                    rospy.sleep(retry_delay)
                else:
                    rospy.logerr("Failed to get map transform. Is Gmapping running?")
        
        # Then check other frames
        for frame in frames:
            for retry in range(max_retries):
                try:
                    self.tf_buffer.can_transform(
                        self.base_frame,
                        frame,
                        rospy.Time(0),
                        timeout
                    )
                    rospy.loginfo(f"Transform for {frame} is available")
                    break
                except (tf2_ros.LookupException,
                        tf2_ros.ConnectivityException,
                        tf2_ros.ExtrapolationException) as e:
                    if retry < max_retries - 1:
                        rospy.logwarn(f"Waiting for {frame} transform (attempt {retry + 1}/{max_retries})")
                        rospy.sleep(retry_delay)
                    else:
                        rospy.logerr(f"Failed to get transform for {frame}")

    def get_robot_pose(self) -> Optional[Tuple[np.ndarray, float]]:
        """Get robot's current position and orientation in map frame"""
        try:
            # First try to get transform
            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.base_frame,
                rospy.Time(0),
                rospy.Duration(self.tf_timeout)
            )
            
            # Extract position
            position = np.array([
                transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z
            ])
            
            # Extract orientation (yaw only for 2D)
            q = transform.transform.rotation
            yaw = np.arctan2(2.0 * (q.w * q.z + q.x * q.y),
                           1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            
            # if self.config['debug']['print_transforms']:
            #     rospy.loginfo(
            #         f"Robot pose in map: position={position}, "
            #         f"yaw={np.degrees(yaw):.1f}°"
            #     )
            
            return position, yaw
            
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            if self.config['debug']['print_transforms']:
                rospy.logwarn_throttle(1.0, f"Failed to get robot pose: {e}")
            return None

    def transform_to_map(self, points: List[Tuple[float, float, float]], 
                        frame_id: str) -> List[Tuple[float, float]]:
        """Transform a list of points from given frame to map frame"""
        try:
            transformed_points = []
            
            for x, y, z in points:
                # Create point in source frame
                point = PointStamped()
                point.header.frame_id = frame_id
                point.header.stamp = rospy.Time(0)
                point.point = Point(x=x, y=y, z=z)
                
                try:
                    # First transform to base_link if not already there
                    if frame_id != self.base_frame:
                        point = self.tf_buffer.transform(
                            point,
                            self.base_frame,
                            rospy.Duration(self.tf_timeout)
                        )
                    
                    # Then transform to map
                    point_map = self.tf_buffer.transform(
                        point,
                        self.map_frame,
                        rospy.Duration(self.tf_timeout)
                    )
                    
                    # Store transformed point
                    transformed_points.append(
                        (point_map.point.x, point_map.point.y)
                    )
                    
                    # if self.config['debug']['print_transforms']:
                    #     rospy.loginfo(
                    #         f"Transformed point from {frame_id} "
                    #         f"({x:.2f}, {y:.2f}, {z:.2f}) to map "
                    #         f"({point_map.point.x:.2f}, {point_map.point.y:.2f})"
                    #     )
                        
                except (tf2_ros.LookupException,
                        tf2_ros.ConnectivityException,
                        tf2_ros.ExtrapolationException) as e:
                    rospy.logwarn_throttle(1.0, f"Failed to transform point: {e}")
                    continue
            
            return transformed_points
            
        except Exception as e:
            rospy.logwarn_throttle(1.0, f"Failed to transform points: {e}")
            return []