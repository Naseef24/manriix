#!/usr/bin/env python3

"""
Description:
  Navigate to a charging dock once the battery gets low.
-------
Subscription Topics:
  Current battery state
  /battery_status - sensor_msgs/BatteryState
  
  2D Pose of the base_link of the robot in the map frame
  /map_to_base_link_pose2d – std_msgs/Float64MultiArray
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

# Holds the current pose of the robot
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
        # Initialize move_base action client
        self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        self.move_base_client.wait_for_server()
        
        # Create a publisher for velocity commands
        self.publisher_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        # Holds the goal poses of the robot
        self.goal_x = [-1.0, -1.0, -1.0]
        self.goal_y = [1.0, 0.0, -1.0]
        self.goal_yaw_angle = [-1.5708, -1.5708, -1.5708]

        # Keep track of which goal we're headed towards
        self.goal_idx = 0

        # Declare linear and angular velocities
        self.linear_velocity = 0.15 #0.08  # meters per second
        self.angular_velocity = 0.2 #0.1   # radians per second

        # Declare distance metrics in meters
        self.distance_goal_tolerance = 0.05
        self.reached_distance_goal = False      

        # Declare angle metrics in radians
        self.heading_tolerance = 0.05
        self.yaw_goal_tolerance = 0.05

        # Timer for navigation check
        rospy.Timer(rospy.Duration(0.1), self.navigate_to_dock)
        
    def navigate_to_dock(self, event=None):
        global low_battery
        
        if not low_battery:
            return None 
        
        rospy.loginfo('🔜 Navigating to the charging dock...')
        
        # Set the robot's goal pose
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = rospy.Time.now()
        goal_pose.pose.position.x = 0.0
        goal_pose.pose.position.y = 1.0
        goal_pose.pose.position.z = 0.25
        goal_pose.pose.orientation.w = 1.0

        # Send goal to move_base
        move_base_goal = MoveBaseGoal()
        move_base_goal.target_pose = goal_pose
        
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
        # While the battery is not charging
        while this_battery_state.power_supply_status != 1:
            rospy.loginfo('❌ NOT CHARGING...')
            
            if self.goal_idx == 0:
                self.go_to_line()
                rospy.loginfo('1-🏁 Going to perpendicular line to ARTag...')
            elif self.goal_idx == 1:
                self.go_to_line()
                rospy.loginfo('2-🏁 Going to perpendicular line to ARTag...')
            elif self.goal_idx == 2:
                self.go_to_artag()
                rospy.loginfo('3-🏁 Going straight to ARTag...')
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
            math.pow(self.goal_x[self.goal_idx] - current_x, 2) + 
            math.pow(self.goal_y[self.goal_idx] - current_y, 2)
        )
        return distance_to_goal
        
    def get_heading_error(self):
        """
        Get the heading error in radians
        """
        delta_x = self.goal_x[self.goal_idx] - current_x
        delta_y = self.goal_y[self.goal_idx] - current_y
        desired_heading = math.atan2(delta_y, delta_x) 
        heading_error = desired_heading - current_yaw_angle
        
        # Make sure the heading error falls within -PI to PI range
        if heading_error > math.pi:
            heading_error = heading_error - (2 * math.pi)
        if heading_error < -math.pi:
            heading_error = heading_error + (2 * math.pi)
        
        return heading_error
      
    def get_radians_to_goal(self):
        """
        Get the yaw goal angle error in radians
        """
        yaw_goal_angle_error = self.goal_yaw_angle[self.goal_idx] - current_yaw_angle
        return yaw_goal_angle_error
      
    def go_to_line(self):
        """
        Go to the line that is perpendicular to the AR tag
        """
        distance_to_goal = self.get_distance_to_goal()
        heading_error = self.get_heading_error()
        yaw_goal_error = self.get_radians_to_goal()
        
        cmd_vel_msg = Twist()
        
        # If we are not yet at the position goal
        if (math.fabs(distance_to_goal) > self.distance_goal_tolerance and 
            not self.reached_distance_goal):
            
            # If the robot's heading is off, fix it
            if math.fabs(heading_error) > self.heading_tolerance:
                rospy.loginfo(str(heading_error))
                
                if heading_error > 0:
                    cmd_vel_msg.angular.z = self.angular_velocity
                else:
                    cmd_vel_msg.angular.z = -self.angular_velocity
            else:
                cmd_vel_msg.linear.x = self.linear_velocity
        
        # Orient towards the yaw goal angle
        elif math.fabs(yaw_goal_error) > self.yaw_goal_tolerance:
            if yaw_goal_error > 0:
                cmd_vel_msg.angular.z = self.angular_velocity
            else:
                cmd_vel_msg.angular.z = -self.angular_velocity
            
            self.reached_distance_goal = True
        
        # Goal achieved, go to the next goal  
        else:
            # Go to the next goal
            self.goal_idx += 1
            rospy.loginfo('🎯 Arrived at perpendicular line. Going straight to ARTag...')
            self.reached_distance_goal = False
        
        # Publish the velocity message  
        self.publisher_cmd_vel.publish(cmd_vel_msg)
    
    def go_to_artag(self):
        """
        Go straight to the AR tag
        """
        distance_to_goal = self.get_distance_to_goal()
        heading_error = self.get_heading_error()
        yaw_goal_error = self.get_radians_to_goal()
        
        cmd_vel_msg = Twist()
        
        # If we are not yet at the position goal
        if (math.fabs(distance_to_goal) > self.distance_goal_tolerance and 
            not self.reached_distance_goal):
            
            # If the robot's heading is off, fix it
            if math.fabs(heading_error) > self.heading_tolerance:
                if heading_error > 0:
                    cmd_vel_msg.angular.z = self.angular_velocity
                else:
                    cmd_vel_msg.angular.z = -self.angular_velocity
            else:
                cmd_vel_msg.linear.x = self.linear_velocity
        
        # Orient towards the yaw goal angle
        elif math.fabs(yaw_goal_error) > self.yaw_goal_tolerance:
            if yaw_goal_error > 0:
                cmd_vel_msg.angular.z = self.angular_velocity
            else:
                cmd_vel_msg.angular.z = -self.angular_velocity
            
            self.reached_distance_goal = True
        
        # Goal achieved, go to the next goal  
        else:
            # Go to the next goal
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
            '/map_to_base_link_pose2d', 
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
    # Initialize the ROS node
    rospy.init_node('charging_dock_navigator')
    
    # Create the node objects
    connect_to_charging_dock_navigator = ConnectToChargingDockNavigator()
    battery_state_subscriber = BatteryStateSubscriber()
    pose_subscriber = PoseSubscriber()
    
    # Spin
    rospy.spin()

if __name__ == '__main__':
    main()

