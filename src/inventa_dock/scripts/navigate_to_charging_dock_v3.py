#!/usr/bin/env python3

"""
Description:
  Navigate to a charging dock once the battery gets low.
  This script works with the static transform publisher for the Aruco marker.
  We assume we have hardcoded the exact position and orientation of the ArUco marker. 
-------
Subscription Topics:
  Current battery state
  /battery_status - sensor_msgs/BatteryState
  
  2D Pose of the aruco_marker of the docking station the robot's base_link frame
  /base_link_to_aruco_marker_pose2d – std_msgs/Float64MultiArray
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
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Float64MultiArray
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus

# Holds the current pose of the aruco_marker
current_x = 0.0
current_y = 0.0
current_yaw_angle = 0.0

# Holds the current state of the battery
this_battery_state = BatteryState()
prev_battery_state = BatteryState()

# Flag for detecting the change in the battery state
low_battery = False
low_battery_min_threshold = 0.25

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
        # self.linear_velocity = 0.1  # meters per second
        # self.angular_velocity = 0.2  # radians per second
        self.linear_velocity = 0.08  # meters per second
        self.angular_velocity = 0.1 # radians per second        
        
        # Keep track of which goal we're headed towards
        self.goal_idx = 0

        # Holds the goal pose of the robot
        self.goal_x = 0.36
        self.goal_y = 0.0
        self.goal_yaw_angle = -1.5708

        # Declare distance metrics in meters
        self.distance_goal_tolerance = 0.05
        self.reached_distance_goal = False      

        # Declare angle metrics in radians
        self.yaw_goal_tolerance = 0.05
        self.reached_yaw_angle_goal = False 
        
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
        
        # Set the robot's staging area pose 
        staging_area_pose = PoseStamped()
        staging_area_pose.header.frame_id = 'map'
        staging_area_pose.header.stamp = rospy.Time.now()
        staging_area_pose.pose.position.x = 0.0
        staging_area_pose.pose.position.y = 1.0 #2.0
        staging_area_pose.pose.position.z = 0.25
        staging_area_pose.pose.orientation.w = 1.0

        # Send goal to move_base
        move_base_goal = MoveBaseGoal()
        move_base_goal.target_pose = staging_area_pose
        
        self.move_base_client.send_goal(move_base_goal)
        
        # Wait for result
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
        Navigate and connect to the charging dock.
        """ 
        
        # While the battery is not charging
        while this_battery_state.power_supply_status != 1:
            rospy.loginfo('❌ NOT CHARGING...')
            
            if self.goal_idx == 0:
                self.go_to_line()
                rospy.loginfo('1-🏁 Going to perpendicular line to ArUco marker...')
            elif self.goal_idx == 1:
                self.align_with_aruco_marker()
                rospy.loginfo('2-🏁 Aligning with the ArUco marker...')
            elif self.goal_idx == 2:
                self.go_to_aruco_marker()
                rospy.loginfo('3-🏁 Going to the ArUco marker...')
            else:
                # Stop the robot
                cmd_vel_msg = Twist()
                cmd_vel_msg.linear.x = 0.0
                cmd_vel_msg.angular.z = 0.0
                self.publisher_cmd_vel.publish(cmd_vel_msg)
                rospy.loginfo('Robot is idle...')
            
            rospy.sleep(0.02)
        
        rospy.loginfo('🔋 CHARGING...')
        rospy.loginfo('✅ Successfully connected to the charging dock!')
    
    def get_distance_to_goal(self):
        """
        Get the distance between the current x,y coordinate and the desired x,y coordinate
        The unit is meters.
        """
        distance_to_goal = math.sqrt(
            math.pow(self.goal_x - current_x, 2) + 
            math.pow(self.goal_y - current_y, 2)
        )
        return distance_to_goal
        
    def get_radians_to_goal(self):
        """
        Get the yaw goal angle error in radians
        """
        yaw_goal_angle_error = self.goal_yaw_angle - current_yaw_angle

        # Make sure the yaw angle error falls within -PI to PI range
        if yaw_goal_angle_error > math.pi:
            yaw_goal_angle_error = yaw_goal_angle_error - (2 * math.pi)
        if yaw_goal_angle_error < -math.pi:
            yaw_goal_angle_error = yaw_goal_angle_error + (2 * math.pi)
        
        return yaw_goal_angle_error
      
    def go_to_line(self):
        """
        Go to the line that is perpendicular to the ArUco marker
        """
        distance_to_goal = math.fabs(current_x)
        yaw_goal_error = current_yaw_angle
        
        # Create a velocity message
        cmd_vel_msg = Twist()

        # Orient towards the yaw goal angle
        if (math.fabs(yaw_goal_error) > self.yaw_goal_tolerance and not self.reached_yaw_angle_goal):
            cmd_vel_msg.angular.z = self.angular_velocity      
        
        # If we are not yet at the target x position, go there now.
        elif (distance_to_goal > self.distance_goal_tolerance):
            self.reached_yaw_angle_goal = True
            
            if current_x > 0:
                cmd_vel_msg.linear.x = self.linear_velocity
            else: 
                cmd_vel_msg.linear.x = -self.linear_velocity
        
        # Reached perpendicular line 
        else:
            # Go to the next goal
            self.goal_idx += 1    
            rospy.loginfo('Going to the ArUco marker...')
            self.reached_yaw_angle_goal = False

        # Publish the velocity message  
        self.publisher_cmd_vel.publish(cmd_vel_msg) 

    def align_with_aruco_marker(self):
        """
        Align with the ArUco marker
        """
        yaw_goal_error = self.get_radians_to_goal()
        
        # Create a velocity message
        cmd_vel_msg = Twist()

        # Orient towards the yaw goal angle
        if (math.fabs(yaw_goal_error) > self.yaw_goal_tolerance and not self.reached_yaw_angle_goal):
            cmd_vel_msg.angular.z = self.angular_velocity      
        
        else:
            # Go to the next goal
            self.goal_idx += 1    
            rospy.loginfo('Going to the ArUco marker...')

        # Publish the velocity message  
        self.publisher_cmd_vel.publish(cmd_vel_msg) 
            
    def go_to_aruco_marker(self):
        """
        Go straight to the ArUco marker
        """
        distance_to_goal = self.get_distance_to_goal()
        yaw_goal_error = self.get_radians_to_goal()
            
        cmd_vel_msg = Twist()
        
        # Follow the perpendicular line to the ArUco marker
        if (current_y > self.distance_goal_tolerance and not self.reached_distance_goal):
            cmd_vel_msg.angular.z = self.angular_velocity 
        elif (current_y < -self.distance_goal_tolerance and not self.reached_distance_goal):
            cmd_vel_msg.angular.z = -self.angular_velocity 
        elif (math.fabs(distance_to_goal) > self.distance_goal_tolerance and not self.reached_distance_goal):
            cmd_vel_msg.linear.x = self.linear_velocity        
        # Goal position and orientation achieved
        else:
            self.goal_idx += 1 
            rospy.loginfo('🎯 Arrived at the charging dock...')   
            self.reached_distance_goal = True

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

class PoseSubscriber:
    """
    Subscriber node to the current 2D pose of the robot
    """     
    def __init__(self):
        # Create a subscriber for robot pose
        self.subscription_pose = rospy.Subscriber(
            '/base_link_to_aruco_marker_pose2d', 
            Float64MultiArray, 
            self.get_pose,
            queue_size=1
        )
    
    def get_pose(self, msg):
        """
        Update the current 2D pose.
        """
        global current_x
        global current_y
        global current_yaw_angle
        
        current_2d_pose = msg.data
        current_x = current_2d_pose[0]
        current_y = current_2d_pose[1]
        current_yaw_angle = current_2d_pose[2]

def main():
    """
    Entry point for the program.
    """
    try:
        # Create the node objects
        connect_to_charging_dock_navigator = ConnectToChargingDockNavigator()
        battery_state_subscriber = BatteryStateSubscriber()
        pose_subscriber = PoseSubscriber()
        
        # Spin
        rospy.spin()
        
    except rospy.ROSInterruptException:
        rospy.loginfo("Node interrupted")

if __name__ == '__main__':
    main()