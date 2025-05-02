#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Int32

# The different ArUco dictionaries built into the OpenCV library. 
ARUCO_DICT = {
    "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
    "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
    "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
    "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
    "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
    "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
    "DICT_5X5_250": cv2.aruco.DICT_5X5_250,
    "DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
    "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
    "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
    "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
    "DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
    "DICT_7X7_50": cv2.aruco.DICT_7X7_50,
    "DICT_7X7_100": cv2.aruco.DICT_7X7_100,
    "DICT_7X7_250": cv2.aruco.DICT_7X7_250,
    "DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
    "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL
}

class ArucoNode:
    """
    ArUco marker detection node
    """
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('aruco_node', disable_signals=True)
        
        # Diagnostic logging
        rospy.loginfo("🚀 Initializing ArUco Marker Detector")
        
        # Get parameters with defaults
        aruco_dictionary_name = rospy.get_param('~aruco_dictionary_name', 'DICT_ARUCO_ORIGINAL')
        self.aruco_marker_side_length = rospy.get_param('~aruco_marker_side_length', 0.05)
        self.camera_calibration_parameters_filename = rospy.get_param(
            '~camera_calibration_parameters_filename', 
            '/home/nsf24/inventa_sim/src/inventa_dock/calibration_chessboard.yaml'
        )
        image_topic = rospy.get_param('~image_topic', '/camera2/color/image_raw2')
        # image_topic = rospy.get_param('~image_topic', '/camera2/depth/image_raw2')

        self.aruco_marker_name = rospy.get_param('~aruco_marker_name', 'aruco_marker')

        # Extensive parameter logging
        rospy.loginfo(f"📷 Image Topic: {image_topic}")
        rospy.loginfo(f"🏷️ ArUco Dictionary: {aruco_dictionary_name}")
        rospy.loginfo(f"📋 Calibration File: {self.camera_calibration_parameters_filename}")

        # Check ArUco dictionary
        if ARUCO_DICT.get(aruco_dictionary_name, None) is None:
            rospy.logerr(f"❌ ArUco tag of '{aruco_dictionary_name}' is not supported")
            raise ValueError(f"Unsupported ArUco dictionary: {aruco_dictionary_name}")

        # Load camera parameters
        try:
            cv_file = cv2.FileStorage(
                self.camera_calibration_parameters_filename, cv2.FILE_STORAGE_READ)
            self.mtx = cv_file.getNode('K').mat()
            self.dst = cv_file.getNode('D').mat()
            cv_file.release()
            rospy.loginfo("✅ Camera calibration loaded successfully")
        except Exception as e:
            rospy.logerr(f"❌ Error loading camera calibration: {e}")
            self.mtx = None
            self.dst = None

        # Load ArUco dictionary
        try:
            # Use getPredefinedDictionary for newer OpenCV versions
            self.this_aruco_dictionary = cv2.aruco.getPredefinedDictionary(ARUCO_DICT[aruco_dictionary_name])
            
            # Updated detector parameters creation
            self.this_aruco_parameters = cv2.aruco.DetectorParameters()
            rospy.loginfo("✅ ArUco dictionary initialized")
        except Exception as e:
            rospy.logerr(f"❌ Error initializing ArUco dictionary: {e}")
            raise
        
        # Create subscribers and publishers
        self.image_sub = rospy.Subscriber(
            image_topic, 
            Image, 
            self.listener_callback, 
            queue_size=1
        )
        
        # Publishers
        self.aruco_detected_pub = rospy.Publisher(
            'aruco_marker_detected', 
            Bool, 
            queue_size=10
        )
        
        self.aruco_offset_pub = rospy.Publisher(
            'aruco_marker_offset', 
            Int32, 
            queue_size=10
        )
        
        # Used to convert between ROS and OpenCV images
        self.bridge = CvBridge()
        
        # Initial offset
        self.offset_aruco_marker = 0

        rospy.loginfo("🎉 ArUco Marker Detector Initialized Successfully")

    def listener_callback(self, data):
        """
        Callback function for image processing
        """
        try:
            # Detailed image information logging
            rospy.loginfo(f"📸 Received Image - Encoding: {data.encoding}, "
                          f"Height: {data.height}, Width: {data.width}")

            # Check if we're dealing with depth image
            if '16' in data.encoding:  # For depth images like 16UC1
                try:
                    # Convert depth image to CV2 format with correct encoding
                    current_frame = self.bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")
                    
                    # Normalize depth image for visualization (0-255)
                    depth_array = np.array(current_frame, dtype=np.float32)
                    cv2.normalize(depth_array, depth_array, 0, 255, cv2.NORM_MINMAX)
                    current_frame = np.uint8(depth_array)
                    
                    rospy.loginfo("✅ Successfully converted depth image")
                except CvBridgeError as e:
                    rospy.logwarn(f"❌ Depth image conversion failed: {e}")
                    return
            else:
                # For RGB/regular images, try multiple encodings
                conversion_attempts = ['bgr8', 'mono8', 'rgb8']
                current_frame = None

                for encoding in conversion_attempts:
                    try:
                        current_frame = self.bridge.imgmsg_to_cv2(data, encoding)
                        rospy.loginfo(f"✅ Successfully converted image with {encoding} encoding")
                        break
                    except CvBridgeError as e:
                        rospy.logwarn(f"❌ Conversion failed with {encoding} encoding: {e}")

                if current_frame is None:
                    rospy.logerr("❌ Failed to convert image with any encoding")
                    return

            # Detect ArUco markers
            try:
                # Updated detection method without camera matrix
                (corners, marker_ids, rejected) = cv2.aruco.detectMarkers(
                    current_frame, 
                    self.this_aruco_dictionary, 
                    parameters=self.this_aruco_parameters
                )
            except Exception as detect_error:
                rospy.logerr(f"❌ ArUco marker detection error: {detect_error}")
                return
            
            # ArUco detected flag
            aruco_detected_flag = Bool()
            aruco_detected_flag.data = False
            
            # ArUco center offset message
            aruco_center_offset_msg = Int32()
            aruco_center_offset_msg.data = self.offset_aruco_marker
            
            image_width = current_frame.shape[1]

            # Check if markers are detected
            if marker_ids is not None and len(marker_ids) > 0:
                # ArUco marker has been detected
                aruco_detected_flag.data = True
                
                # Draw markers
                cv2.aruco.drawDetectedMarkers(current_frame, corners, marker_ids)

                # Calculate marker center
                M = cv2.moments(corners[0][0])
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
                
                # Calculate offset
                self.offset_aruco_marker = cX - int(image_width/2)
                aruco_center_offset_msg.data = self.offset_aruco_marker

                # Add text to image
                cv2.putText(current_frame, 
                    f"Center Offset: {self.offset_aruco_marker}", 
                    (cX - 40, cY - 40), 
                    cv2.FONT_HERSHEY_COMPLEX, 
                    0.7, (0, 255, 0), 2
                ) 

                rospy.loginfo(f"🎯 ArUco Marker Detected - Offset: {self.offset_aruco_marker}")

            # Publish detection status and offset
            self.aruco_detected_pub.publish(aruco_detected_flag)
            self.aruco_offset_pub.publish(aruco_center_offset_msg)
            
            # Display image
            cv2.imshow("camera", current_frame)
            cv2.waitKey(1)
        
        except Exception as e:
            rospy.logerr(f"❌ Unexpected error in ArUco processing: {e}")
            import traceback
            traceback.print_exc()

def main():
    try:
        # Create the ArUco node
        aruco_node = ArucoNode()
        
        # Spin
        rospy.spin()
    
    except rospy.ROSInterruptException:
        rospy.loginfo("Node interrupted")
    except Exception as e:
        rospy.logerr(f"❌ Unexpected error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()