#!/usr/bin/env python3 
 
"""
Description:
Publish the coordinate transformation between the map frame
and the base_link frame.
The output is [x,y,yaw]. yaw is -pi to pi
-------
Subscription Topics:
/tf - tf/tfMessage
-------
Publishing Topics:
/map_to_base_link_pose2d – std_msgs/Float64MultiArray
-------
"""
 
import rospy
import tf
import math
from std_msgs.msg import Float64MultiArray

class FrameListener:
    """
    This class listens to coordinate transformations and 
    publishes the 2D pose at a specific time interval.
    """
    def __init__(self):
        """
        Class constructor to set up the node
        """
        # Initialize the node
        rospy.init_node('map_base_link_frame_listener')
        
        # Get target frame parameter or use default
        self.target_frame = rospy.get_param('~target_frame', 'base_footprint')
        self.reference_frame = 'map'
        
        # Set up TF listener
        self.tf_listener = tf.TransformListener()
        
        # Create publisher for 2D pose
        self.publisher_2d_pose = rospy.Publisher(
            '/map_to_base_link_pose2d', 
            Float64MultiArray, 
            queue_size=1)
        
        # Current position and orientation of the target frame with respect to the 
        # reference frame. x and y are in meters, and yaw is in radians.
        self.current_x = 0.0
        self.current_y = 0.0 
        self.current_yaw = 0.0
        
        # Set up timer for publishing pose
        rospy.Timer(rospy.Duration(0.1), self.publish_pose)
        
        rospy.loginfo("Map to base_link transform publisher initialized")
        
    def publish_pose(self, event):
        """
        Timer callback function. Gets the transform and publishes the 2D pose.
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
            
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logwarn_throttle(5, f"TF Error: {e}")  # Only log once every 5 seconds
 
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
        rospy.logerr(f"Unexpected error: {e}")
 
if __name__ == '__main__':
    main()