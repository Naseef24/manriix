#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

def callback(data):
    rospy.loginfo('Receiving video frame')
    
    # Convert ROS Image message to OpenCV image
    frame = bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
    
    # Display image
    cv2.imshow("camera", frame)
    cv2.waitKey(1)

def main():
    global bridge
    bridge = CvBridge()

    rospy.init_node('image_subscriber', anonymous=True)

    rospy.Subscriber('video_frames', Image, callback)

    rospy.spin()

    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()