#!/usr/bin/env python3 

"""
Description:
Publish the coordinate transformation between the base_link frame
and the aruco_marker frame.
The output is [x,y,yaw]. yaw is -pi to pi
-------
Subscription Topics:
/tf - geometry_msgs/TransformStamped[]
-------
Publishing Topics:
/base_link_to_aruco_marker_pose2d – std_msgs/Float64MultiArray
-------
"""

import rospy
import tf
import math
from std_msgs.msg import Float64MultiArray

class FrameListener:
    """
    Listens to coordinate transformations and 
    publishes the 2D pose at a specific time interval.
    """
    def __init__(self):
        # Initialize the node
        rospy.init_node('base_link_aruco_marker_frame_listener')
        
        # Get target frame parameter or use default
        self.target_frame = rospy.get_param('~target_frame', 'aruco_marker')
        self.reference_frame = 'base_link'
        
        # Set up TF listener
        self.tf_listener = tf.TransformListener()
        
        # Create publisher for 2D pose
        self.publisher_2d_pose = rospy.Publisher(
            '/base_link_to_aruco_marker_pose2d', 
            Float64MultiArray, 
            queue_size=1)
        
        # Current position and orientation of the target frame
        self.current_x = 0.0 
        self.current_y = 0.0  
        self.current_yaw = 0.0 
        
        # Set up timer for publishing pose
        rospy.Timer(rospy.Duration(0.1), self.on_timer)
        
        rospy.loginfo("Base Link to Aruco Marker transform publisher initialized")
        
    def on_timer(self, event):
        """
        Callback function called at specific time interval
        """
        try:
            # Look up the transform
            (trans, rot) = self.tf_listener.lookupTransform(
                self.reference_frame, 
                self.target_frame, 
                rospy.Time(0))
            
            # Extract position
            self.current_x = trans[0]
            self.current_y = trans[1]
            
            # Convert quaternion to Euler angles
            euler = tf.transformations.euler_from_quaternion(rot)
            self.current_yaw = euler[2]  # yaw is the rotation around z
            
            # Publish the 2D pose
            msg = Float64MultiArray()
            msg.data = [self.current_x, self.current_y, self.current_yaw]
            self.publisher_2d_pose.publish(msg)
            
            rospy.loginfo(f"📡 Aruco Marker Transform: x={self.current_x}, y={self.current_y}, yaw={self.current_yaw}")
            
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logwarn_throttle(5, f"❌ TF Exception: {e}")  # Only log once every 5 seconds

def main():
    """
    Main function
    """
    try:
        # Create the frame listener
        frame_listener = FrameListener()
        
        # Keep the node running
        rospy.spin()
        
    except rospy.ROSInterruptException:
        rospy.loginfo("Node interrupted")
    except Exception as e:
        rospy.logerr(f"❌ Unexpected error: {e}")
 
if __name__ == '__main__':
    main()