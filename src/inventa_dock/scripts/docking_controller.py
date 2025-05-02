#!/usr/bin/env python3
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from ar_track_alvar_msgs.msg import AlvarMarkers
import tf
import math

class DockingController:
    def __init__(self):
        rospy.init_node('inventa_docking_controller')
        
        # Move Base Action Client
        self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.move_base_client.wait_for_server()
        
        # TF Listener
        self.tf_listener = tf.TransformListener()
        
        # AR Marker Subscriber
        self.marker_sub = rospy.Subscriber('/ar_pose_marker', AlvarMarkers, self.marker_callback)
        
        # Docking Parameters
        self.approach_distance = rospy.get_param('~approach_distance', 0.5)
        self.alignment_tolerance = rospy.get_param('~alignment_tolerance', 0.1)
        
        # Docking State
        self.current_marker = None
        self.docking_in_progress = False

    def marker_callback(self, msg):
        if not self.docking_in_progress and msg.markers:
            # Use first detected marker
            self.current_marker = msg.markers[0]
            rospy.loginfo("AR Marker detected. Initiating docking procedure.")
            self.start_docking()
    
    def calculate_docking_pose(self):
        if not self.current_marker:
            rospy.logerr("No marker detected!")
            return None
        
        try:
            # Transform marker pose to map frame
            marker_frame = f'ar_marker_{self.current_marker.id}'
            self.tf_listener.waitForTransform('/map', marker_frame, rospy.Time(0), rospy.Duration(4.0))
            (trans, rot) = self.tf_listener.lookupTransform('/map', marker_frame, rospy.Time(0))
            
            # Calculate docking goal
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = 'map'
            goal.target_pose.header.stamp = rospy.Time.now()
            
            # Position slightly offset from marker
            goal.target_pose.pose.position.x = trans[0] - self.approach_distance
            goal.target_pose.pose.position.y = trans[1]
            goal.target_pose.pose.position.z = 0
            
            # Orientation towards marker (quaternion)
            goal.target_pose.pose.orientation.w = 1.0
            
            return goal
        
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logerr(f"TF Error: {e}")
            return None

    def start_docking(self):
        if self.docking_in_progress:
            return
        
        self.docking_in_progress = True
        rospy.loginfo("Starting docking procedure")
        
        # Calculate docking pose
        docking_goal = self.calculate_docking_pose()
        
        if docking_goal:
            # Send goal to move_base
            self.move_base_client.send_goal(docking_goal, done_cb=self.docking_done_cb)
            
            # Wait for result with timeout
            success = self.move_base_client.wait_for_result(rospy.Duration(60))
            
            if not success:
                rospy.logerr("Docking approach timed out")
                self.docking_in_progress = False

    def docking_done_cb(self, state, result):
        self.docking_in_progress = False
        
        if state == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo("Docking approach successful")
            # Additional fine-tuning or charging logic can be added here
        else:
            rospy.logerr("Docking approach failed")

    def run(self):
        rospy.loginfo("Docking Controller initialized. Waiting for AR markers...")
        rospy.spin()

if __name__ == '__main__':
    try:
        controller = DockingController()
        controller.run()
    except rospy.ROSInterruptException:
        rospy.logerr("Docking controller node terminated.")