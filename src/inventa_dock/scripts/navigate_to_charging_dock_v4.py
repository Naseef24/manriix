#!/usr/bin/env python3

"""
Description:
  Navigate to a charging dock once the battery gets low.
  We first navigate to a staging area in front of the docking station (~1.5 meters is good)
  We rotate around to search for the ArUco marker using the robot's front camera.
  Once the ArUco marker is detected, move towards it, making minor heading adjustments as necessary.
  Stop once the robot gets close enough to the charging dock or starts charging.
-------
Subscription Topics:
  Current battery state
  /battery_status - sensor_msgs/BatteryState
  
  A boolean variable that is True of ArUco marker detected, otherwise False
  /aruco_marker_detected – std_msgs/Bool
  
  The number of pixels offset of the ArUco marker from the center of the camera image
  /aruco_marker_offset - std_msgs/Int32
  
  LaserScan readings for object detection
  /scan - sensor_msgs/LaserScan
-------
Publishing Topics:
  Velocity command to navigate to the charging dock.
  /cmd_vel - geometry_msgs/Twist
-------
"""

import math
import time
import rospy
import actionlib
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import BatteryState, LaserScan
from std_msgs.msg import Bool, Int32
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus

# Global variables
current_x = 0.0
current_y = 0.0
current_yaw_angle = 0.0

this_battery_state = BatteryState()
prev_battery_state = BatteryState()

low_battery = False
low_battery_min_threshold = 0.25

aruco_marker_detected = False
aruco_center_offset = 0
obstacle_distance_front = 999999.9

class ConnectToChargingDockNavigator:
    """
    Navigates and connects to the charging dock
    """     
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('connect_to_charging_dock_navigator')
        
        # Create a publisher for velocity commands
        self.publisher_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        # Initialize move_base action client
        self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        self.move_base_client.wait_for_server()

        # Declare linear and angular velocities
        self.linear_velocity = -0.1 #0.08  # meters per second
        self.angular_velocity = 0.1 #0.03 # radians per second
        
        # Keep track of which goal we're headed towards
        self.goal_idx = 0
        
        # Declare obstacle tolerance 
        self.obstacle_tolerance = 0.5 #0.22

        # Center offset tolerance in pixels
        self.center_offset_tolerance = 20 #10
        
        # Undocking distance
        self.undocking_distance = 0.50
        
        # Timer for navigation check
        rospy.Timer(rospy.Duration(0.1), self.navigate_to_dock_staging_area)
        
    def navigate_to_dock_staging_area(self, event=None):
        """
        Navigate from somewhere in the environment to a staging area near
        the charging dock.
        """    
        global low_battery
        
        # If we have enough battery, don't navigate to the charging dock.
        if not low_battery:
            return None 
        
        rospy.loginfo('🔋 Low battery. Navigating to the charging dock...')
        
        # Prepare staging area pose
        staging_area_pose = PoseStamped()
        staging_area_pose.header.frame_id = 'map'
        staging_area_pose.header.stamp = rospy.Time.now()
        staging_area_pose.pose.position.x = -1.0
        staging_area_pose.pose.position.y = 0.4 #2.5
        staging_area_pose.pose.position.z = 0.25
        
        # x = 0.0
        # y = 0.0
        # z = 1.0 * sin(π/2) = 1.0
        # w = cos(π/2) = 0.0          #'w' encodes the cosine of half the rotation angle  
        
        staging_area_pose.pose.orientation.x = 0.0
        staging_area_pose.pose.orientation.y = 0.0
        staging_area_pose.pose.orientation.z = 1.0
        staging_area_pose.pose.orientation.w = 0.0        

        # Send goal to move_base
        move_base_goal = MoveBaseGoal()
        move_base_goal.target_pose = staging_area_pose
        
        self.move_base_client.send_goal(move_base_goal)
        
        # Wait for result (5 minutes)
        if self.move_base_client.wait_for_result(rospy.Duration(300)):
            result_state = self.move_base_client.get_state()
            
            if result_state == GoalStatus.SUCCEEDED:
                rospy.loginfo('✅ Successfully reached charging dock staging area...')
                low_battery = False
                self.connect_to_dock()
            else:
                rospy.logerr('❌ Failed to reach staging area.')
        
    def connect_to_dock(self): 
        """
        Go to the charging dock.
        """ 
        
        # While the battery is not charging
        while this_battery_state.power_supply_status != 1:
            if self.goal_idx == 0:
                self.search_for_aruco_marker()
                rospy.loginfo('Searching for the ArUco marker...')
            elif self.goal_idx == 1:
                self.navigate_to_aruco_marker()
                rospy.loginfo('Navigating to the ArUco marker...')
            else:
                # Stop the robot
                cmd_vel_msg = Twist()
                cmd_vel_msg.linear.x = 0.0
                cmd_vel_msg.angular.z = 0.0
                self.publisher_cmd_vel.publish(cmd_vel_msg)
                rospy.loginfo('Arrived at charging dock. Robot is idle...')
            
            rospy.sleep(0.02)
        
        rospy.loginfo('🔋 CHARGING...')
        rospy.loginfo('✅ Successfully connected to the charging dock!')
        
        # Stop the robot
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = 0.0
        cmd_vel_msg.angular.z = 0.0
        self.publisher_cmd_vel.publish(cmd_vel_msg)
        
        # Reset the node
        self.goal_idx = 0

        # While the battery is not full
        while this_battery_state.percentage != 1.0:      
            rospy.loginfo('CHARGING...')
        
        # Undock from the docking station
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = -self.linear_velocity
        self.publisher_cmd_vel.publish(cmd_vel_msg)
        while obstacle_distance_front < self.undocking_distance:
            rospy.loginfo('Undocking from the charging dock...')

        # Stop the robot
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = 0.0     
        self.publisher_cmd_vel.publish(cmd_vel_msg)
        rospy.loginfo('Ready for my next goal!')

    def search_for_aruco_marker(self):
        """
        Rotate around until the robot finds the charging dock
        """
        if not aruco_marker_detected:
            # Create a velocity message
            cmd_vel_msg = Twist()
            cmd_vel_msg.angular.z = -self.angular_velocity           
        
            # Publish the velocity message  
            self.publisher_cmd_vel.publish(cmd_vel_msg) 
        else: 
            self.goal_idx = 1
        
    def navigate_to_aruco_marker(self):
        """
        Go straight to the ArUco marker
        """
        # If we have detected the ArUco marker and there are no obstacles in the way
        if aruco_marker_detected and (obstacle_distance_front > self.obstacle_tolerance):
            self.adjust_heading()
        # If we have detected the ArUco marker and there are obstacles in the way, we have reached the charging dock
        elif aruco_marker_detected and (obstacle_distance_front <= self.obstacle_tolerance):
            self.goal_idx = 2
        # If we have not detected the ArUco marker, and there is an obstacle in the way at a close distance,
        # we have reached the charging dock
        elif not aruco_marker_detected and (obstacle_distance_front <= self.obstacle_tolerance):
            self.goal_idx = 2   
        # Search for charging dock  
        else:
            self.goal_idx = 0
      
    def adjust_heading(self):
        """
        Adjust heading to keep the Aruco marker centerpoint centered.
        """
        cmd_vel_msg = Twist()
        if aruco_center_offset < -self.center_offset_tolerance:
            # Turn left
            cmd_vel_msg.angular.z = self.angular_velocity   
        elif aruco_center_offset > self.center_offset_tolerance:  
            # Turn right       
            cmd_vel_msg.angular.z = -self.angular_velocity 
        else:
            # Go straight
            cmd_vel_msg.linear.x = self.linear_velocity 
        
        # Publish the velocity message  
        self.publisher_cmd_vel.publish(cmd_vel_msg)  

class BatteryStateSubscriber:
    """
    Subscriber node to the current battery state
    """     
    def __init__(self):
        # Create a subscriber for battery state
        self.subscription_battery_state = rospy.Subscriber(
            '/battery_status',
            BatteryState,
            self.get_battery_state,
            queue_size=10
        )
    
    def get_battery_state(self, msg):
        """
        Update the current battery state.
        """
        global this_battery_state
        global prev_battery_state
        global low_battery
        
        prev_battery_state = this_battery_state
        this_battery_state = msg
        
        # Check for low battery
        if (prev_battery_state.percentage >= low_battery_min_threshold and 
            this_battery_state.percentage < low_battery_min_threshold):
            low_battery = True
        
class ArucoMarkerSubscriber:
    """
    Subscriber node to help for the ArUco marker navigation routine.
    """     
    def __init__(self):
        # Create subscribers
        self.subscription_aruco_detected = rospy.Subscriber(
            '/aruco_marker_detected', 
            Bool, 
            self.get_aruco_detected,
            queue_size=1
        )

        self.subscription_center_offset = rospy.Subscriber(
            '/aruco_marker_offset', 
            Int32, 
            self.get_center_offset,
            queue_size=1
        )
        
        self.subscription_laser_scan = rospy.Subscriber(
            '/realsense_scan2', 
            LaserScan, 
            self.scan_callback,
            queue_size=1
        )

    def get_aruco_detected(self, msg):
        """
        Update if the ArUco marker has been detected or not
        """
        global aruco_marker_detected 
        aruco_marker_detected = msg.data
        
    def get_center_offset(self, msg):
        """
        Update the ArUco marker center offset
        """
        global aruco_center_offset
        aruco_center_offset = msg.data
            
    def scan_callback(self, msg):
        """
        Update obstacle distance.
        """
        global obstacle_distance_front
        obstacle_distance_front = msg.ranges[179]
        
def main():
    """
    Entry point for the program.
    """
    try:
        # Create the nodes
        connect_to_charging_dock_navigator = ConnectToChargingDockNavigator()
        battery_state_subscriber = BatteryStateSubscriber()
        aruco_marker_subscriber = ArucoMarkerSubscriber()
        
        # Spin
        rospy.spin()
        
    except rospy.ROSInterruptException:
        rospy.loginfo("Node interrupted")

if __name__ == '__main__':
    main()