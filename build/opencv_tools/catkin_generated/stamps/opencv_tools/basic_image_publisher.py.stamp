#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

def main():
    # Initialize the ROS node
    rospy.init_node('image_publisher', anonymous=True)
    
    # Create a publisher
    pub = rospy.Publisher('video_frames', Image, queue_size=10)
    
    # Create a CvBridge object
    bridge = CvBridge()
    
    # Set up webcam
    cap = cv2.VideoCapture(0)
    
    rate = rospy.Rate(10)  # 10 Hz
    
    while not rospy.is_shutdown():
        ret, frame = cap.read()
        
        if ret:
            # Convert OpenCV image to ROS Image message
            img_msg = bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            img_msg.header.frame_id = 'camera_frame'  # Add this line
            img_msg.header.stamp = rospy.Time.now()   # Add this line
            # Publish the image
            pub.publish(img_msg)
            
            rospy.loginfo('Publishing video frame')
        
        rate.sleep()

    # Release the webcam
    cap.release()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass