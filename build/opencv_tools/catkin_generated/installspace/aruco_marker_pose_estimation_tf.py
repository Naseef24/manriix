#!/usr/bin/env python3

# Import the necessary ROS libraries
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import Image
import tf
from scipy.spatial.transform import Rotation as R

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
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('aruco_node', anonymous=True)

        # Declare parameters
        self.aruco_marker_side_length = rospy.get_param("~aruco_marker_side_length", 0.0785)
        # self.camera_calibration_parameters_filename = '/home/nsf24/inventa_sim/src/inventa_dock/calibration_chessboard.yaml'
        self.camera_calibration_parameters_filename = rospy.get_param("~camera_calibration_parameters_filename", "/home/nsf24/inventa_sim/src/opencv_tools/opencv_tools/calibration_chessboard.yaml")
        self.image_topic = rospy.get_param("~image_topic", "/video_frames")
        self.aruco_marker_name = rospy.get_param("~aruco_marker_name", "aruco_marker")
        self.aruco_dictionary_name = rospy.get_param("~aruco_dictionary_name", "DICT_ARUCO_ORIGINAL")

        # Load the camera parameters
        cv_file = cv2.FileStorage(self.camera_calibration_parameters_filename, cv2.FILE_STORAGE_READ)
        self.mtx = cv_file.getNode('K').mat()
        self.dst = cv_file.getNode('D').mat()
        cv_file.release()

        # Load the ArUco dictionary
        # self.this_aruco_dictionary = cv2.aruco.Dictionary_get(ARUCO_DICT[self.aruco_dictionary_name])
        self.this_aruco_dictionary = cv2.aruco.getPredefinedDictionary(ARUCO_DICT[self.aruco_dictionary_name])    
        self.this_aruco_parameters = cv2.aruco.DetectorParameters()

        # Initialize the ROS publisher for transforms
        self.tfbroadcaster = tf.TransformBroadcaster()

        # Create the ROS subscriber for video frames
        self.image_subscriber = rospy.Subscriber(self.image_topic, Image, self.listener_callback)

        # Initialize CvBridge for ROS to OpenCV conversion
        self.bridge = CvBridge()

    def listener_callback(self, data):
        # Convert ROS Image message to OpenCV image
        current_frame = self.bridge.imgmsg_to_cv2(data, "bgr8")

        # Use the frame_id from the image message if available, otherwise use 'camera_frame'
        camera_frame = data.header.frame_id if data.header.frame_id else 'camera_frame'

        # Detect ArUco markers in the video frame - without camera parameters
        (corners, marker_ids, rejected) = cv2.aruco.detectMarkers(current_frame, self.this_aruco_dictionary, parameters=self.this_aruco_parameters)

        # Check if any markers are detected
        if marker_ids is not None:
            # Draw markers and ids
            cv2.aruco.drawDetectedMarkers(current_frame, corners, marker_ids)

            # Estimate the pose of each marker - use camera parameters here
            rvecs, tvecs, obj_points = cv2.aruco.estimatePoseSingleMarkers(corners, self.aruco_marker_side_length, self.mtx, self.dst)
            
            # Process each detected marker
            for i, marker_id in enumerate(marker_ids):
                # Create the transform message
                t = TransformStamped()
                t.header.stamp = rospy.Time.now()
                # t.header.frame_id = 'camera_depth_frame'
                t.header.frame_id = 'camera_frame'  # Change from camera_depth_frame to camera_frame
                t.child_frame_id = self.aruco_marker_name

                # Store the translation data
                t.transform.translation.x = tvecs[i][0][0]
                t.transform.translation.y = tvecs[i][0][1]
                t.transform.translation.z = tvecs[i][0][2]

                # Calculate the rotation from rvec to quaternion
                rotation_matrix = np.eye(4)
                rotation_matrix[0:3, 0:3] = cv2.Rodrigues(np.array(rvecs[i][0]))[0]
                r = R.from_matrix(rotation_matrix[0:3, 0:3])
                quat = r.as_quat()

                # Quaternion rotation
                t.transform.rotation.x = quat[0]
                t.transform.rotation.y = quat[1]
                t.transform.rotation.z = quat[2]
                t.transform.rotation.w = quat[3]

                # Send the transform
                self.tfbroadcaster.sendTransform((t.transform.translation.x, t.transform.translation.y, t.transform.translation.z),
                                                  (t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w),
                                                  t.header.stamp, t.child_frame_id, t.header.frame_id)

                # Draw the coordinate axes
                # cv2.aruco.drawAxis(current_frame, self.mtx, self.dst, rvecs[i], tvecs[i], 0.05)
                cv2.drawFrameAxes(current_frame, self.mtx, self.dst, rvecs[i], tvecs[i], 0.05, 3)

        # Display the frame
        cv2.imshow("Camera", current_frame)
        cv2.waitKey(1)

if __name__ == '__main__':
    # Create the node
    aruco_node = ArucoNode()

    # Keep the node running
    rospy.spin()