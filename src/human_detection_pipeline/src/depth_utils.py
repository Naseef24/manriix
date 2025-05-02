#!/usr/bin/env python3

import rospy 
from sensor_msgs.msg import Image as msg_Image
from sensor_msgs.msg import CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import sys
import os
import numpy as np
import pyrealsense2 as rs2

class DepthUtils:
    def __init__(self, config):
        """Initialize depth processing utilities"""
        self.bridge = CvBridge()
        self.intrinsics = None
        
        # Subscribe to camera info to get intrinsics
        # self.sub_info = rospy.Subscriber('/camera/aligned_depth_to_color/camera_info', 
        #                                CameraInfo, 
        #                                self.camera_info_callback)
        self.sub_info = rospy.Subscriber('/camera/depth/camera_info', 
                                       CameraInfo, 
                                       self.camera_info_callback)        
    def camera_info_callback(self, cameraInfo):
        """Get camera intrinsics from camera_info"""
        try:
            if self.intrinsics:
                return
                
            self.intrinsics = rs2.intrinsics()
            self.intrinsics.width = cameraInfo.width
            self.intrinsics.height = cameraInfo.height
            self.intrinsics.ppx = cameraInfo.K[2]    # PC X
            self.intrinsics.ppy = cameraInfo.K[5]    # PC Y
            self.intrinsics.fx = cameraInfo.K[0]     # FL X
            self.intrinsics.fy = cameraInfo.K[4]     # FL Y
            
            # Set distortion model
            if cameraInfo.distortion_model == 'plumb_bob':
                self.intrinsics.model = rs2.distortion.brown_conrady
            elif cameraInfo.distortion_model == 'equidistant':
                self.intrinsics.model = rs2.distortion.kannala_brandt4
                
            self.intrinsics.coeffs = [i for i in cameraInfo.D]
            rospy.loginfo("Camera intrinsics loaded successfully")
            
        except Exception as e:
            rospy.logerr(f"Error loading camera intrinsics: {e}")

    def calculate_position(self, depth_frame, bbox):
        """Calculate 3D position using RealSense's deproject function"""
        try:
            if self.intrinsics is None:
                rospy.logwarn("No camera intrinsics available yet")
                return None, (None, None, None)

            # Get center points
            center_x = int((bbox[0] + bbox[2]) / 2)
            center_y = int((bbox[1] + bbox[3]) / 2)

            # depth val of center point
            depth = depth_frame[center_y, center_x]
            
            if depth == 0:  
                return None, (None, None, None)

            # RealSense's deproject function for accurate 3D point calculation
            result = rs2.rs2_deproject_pixel_to_point(
                self.intrinsics, 
                [center_x, center_y], 
                depth
            )
            
            # Calculate distance
            distance = np.sqrt(result[0]**2 + result[1]**2 + result[2]**2)
            
            # convert to m
            result = [x/1000.0 for x in result]  # mm -> m
            distance = distance/1000.0

            return distance, tuple(result), center_x, center_y

        except Exception as e:
            rospy.logerr(f"Error calculating position: {e}")
            return None, (None, None, None)

    def process_detections(self, depth_frame, detections):
        """Process multiple detections"""
        processed_detections = []
        
        for det in detections:
            distance, position, center_x, center_y = self.calculate_position(depth_frame, det['bbox'])
            # print(f"XC ={center_x} | YC = = {center_y}")
            if distance is not None:
                det.update({
                    "distance": float(distance),
                    "position": {
                        "x": float(position[0]),
                        "y": float(position[1]),
                        "z": float(position[2])
                    },
                    "center_point":{
                        "center_x":center_x,
                        "center_y":center_y
                    }
                })
                processed_detections.append(det)
        
        return processed_detections