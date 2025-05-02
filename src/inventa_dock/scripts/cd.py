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
Author: Addison Sears-Collins
Website: AutomaticAddison.com
Date: November 26, 2021
"""

import math
import time
import rospy
import tf.transformations
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Float64MultiArray
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
import actionlib

# Holds the current pose of the robot
current_x = 0.0
current_y = 0.0 #2.0
current_yaw_angle = 0.0

# Holds the current state of the battery
this_battery_state = BatteryState()
prev_battery_state = BatteryState()

# Flag for detecting the change in the battery state
low_battery = False
low_battery_min_threshold = 0.25

class PoseListener:
    """
    Pose estimation listener using TF
    """
    def __init__(self):
        # TF listener for pose transformations
        self.tf_listener = tf.TransformListener()
        
        # Custom pose publisher
        self.custom_pose_pub = rospy.Publisher('/map_to_base_link_pose2d', Float64MultiArray, queue_size=1)
        
        # Timer for pose estimation
        self.pose_timer = rospy.Timer(rospy.Duration(0.1), self.estimate_pose)

    def estimate_pose(self, event=None):
        """
        Estimate and publish robot pose
        """
        global current_x, current_y, current_yaw_angle
        
        try:
            # Get the latest transform between map and base_footprint
            now = rospy.Time(0)
            self.tf_listener.waitForTransform('/map', '/base_footprint', now, rospy.Duration(1.0))
            (trans, rot) = self.tf_listener.lookupTransform('/map', '/base_footprint', now)
            
            # Extract position
            current_x = trans[0]
            current_y = trans[1]
            
            # Convert quaternion to euler angles
            euler = tf.transformations.euler_from_quaternion(rot)
            current_yaw_angle = euler[2]
            
            # Publish custom pose message
            pose_msg = Float64MultiArray()
            pose_msg.data = [current_x, current_y, current_yaw_angle]
            self.custom_pose_pub.publish(pose_msg)
            
            rospy.loginfo(f"Estimated Pose🏁: x={current_x}, y={current_y}, yaw={current_yaw_angle}")
        
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logerr(f"TF Exception: {e}")


class ConnectToChargingDockNavigator:
    """
    Navigates and connects to the charging dock
    """     
    def __init__(self):
        # Initialize move_base action client
        self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server⏰...")
        self.move_base_client.wait_for_server()
        
        # Create a publisher for velocity commands
        self.publisher_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        # Holds the goal poses of the robot
        # self.goal_x = [-1.0, -1.0, -1.0]
        # self.goal_y = [1.2, 0.8, 0.4]
        # self.goal_yaw_angle = [1.5708, 1.5708, 1.5708]
        self.goal_x = [-1.0, -1.0, -1.0]
        self.goal_y = [1.2, 0.8, 0.4]
        self.goal_yaw_angle = [-1.5708, -1.5708, -1.5708]    
                    
        # Keep track of which goal we're headed towards
        self.goal_idx = 0
        
        # Declare linear and angular velocities
        self.linear_velocity = 0.08  # meters per second
        self.angular_velocity = 0.1   # radians per second
        
        # Declare distance metrics in meters
        self.distance_goal_tolerance = 0.05
        self.reached_distance_goal = False     
        
        # Declare angle metrics in radians
        self.heading_tolerance = 0.05
        self.yaw_goal_tolerance = 0.05
        
        # Add a flag to track pose validity
        self.pose_valid = False
        
        # Create a timer to check pose
        self.pose_check_timer = rospy.Timer(rospy.Duration(1.0), self.check_pose)

    def check_pose(self, event=None):
        """
        Check if pose is valid and log diagnostics
        """
        # Check if pose is effectively zero
        if (current_x == 0.0 and current_y == 0.0 and current_yaw_angle == 0.0):
            rospy.logerr("INVALID POSE:❌ Pose is zero!")
            self.pose_valid = False
            
            # Log additional diagnostic information
            rospy.logerr("Diagnostic Information:")
            rospy.logerr(f"Subscribed Topics:")
            try:
                topics = rospy.get_published_topics()
                for topic, type in topics:
                    if 'pose' in topic.lower() or 'map' in topic.lower():
                        rospy.logerr(f"Potential Pose Topic: {topic} (Type: {type})")
            except Exception as e:
                rospy.logerr(f"❌Error listing topics: {e}")
        else:
            self.pose_valid = True
            rospy.loginfo("🚀Pose is valid!")

    def navigate_to_dock(self):
        """
        Navigate to charging dock
        """
        global low_battery
        
        if not low_battery:
            return None
        
        rospy.loginfo('🔜Navigating to the charging dock...')
        
        # First, navigate to staging area
        staging_goal = PoseStamped()
        staging_goal.header.frame_id = 'map'
        staging_goal.header.stamp = rospy.Time.now()
        staging_goal.pose.position.x = 0.0
        staging_goal.pose.position.y = 1.2
        staging_goal.pose.position.z = 0.25
        staging_goal.pose.orientation.w = 1.0
        
        # Send goal to move_base
        move_base_goal = MoveBaseGoal()
        move_base_goal.target_pose = staging_goal
        
        self.move_base_client.send_goal(move_base_goal)
        
        # Wait for result
        if self.move_base_client.wait_for_result(rospy.Duration(300)):
            result_state = self.move_base_client.get_state()
            
            if result_state == GoalStatus.SUCCEEDED:
                rospy.loginfo('Successfully reached charging dock staging area ✅...')
                low_battery = False
                self.connect_to_dock()
            else:
                rospy.logerr('❌Failed to reach staging area.')
    
    def connect_to_dock(self):
        """
        Connect to the charging dock
        """
        # Reinitialize goal index to ensure we start from the first docking point
        self.goal_idx = 0
        self.reached_distance_goal = False

        # Create a publisher for move_base simple goal (for visualization)
        goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1, latch=True)

        # While the battery is not charging
        while this_battery_state.power_supply_status != 1 and self.goal_idx < len(self.goal_x):
            # Extensive diagnostic logging
            rospy.loginfo(f'Current Goal Index 🎯: {self.goal_idx}')
            rospy.loginfo(f'Current Pose 🚢: x={current_x}, y={current_y}, yaw={current_yaw_angle}')
            rospy.loginfo(f'Goal Pose 🏁: x={self.goal_x[self.goal_idx]}, y={self.goal_y[self.goal_idx]}, yaw={self.goal_yaw_angle[self.goal_idx]}')
            
            # Prepare goal for visualization
            dock_goal = PoseStamped()
            dock_goal.header.frame_id = 'map'
            dock_goal.header.stamp = rospy.Time.now()
            dock_goal.pose.position.x = self.goal_x[self.goal_idx]
            dock_goal.pose.position.y = self.goal_y[self.goal_idx]
            
            # Create quaternion from yaw angle
            quaternion = tf.transformations.quaternion_from_euler(0, 0, self.goal_yaw_angle[self.goal_idx])
            dock_goal.pose.orientation.x = quaternion[0]
            dock_goal.pose.orientation.y = quaternion[1]
            dock_goal.pose.orientation.z = quaternion[2]
            dock_goal.pose.orientation.w = quaternion[3]
            
            # Publish goal for visualization
            goal_pub.publish(dock_goal)

            # Calculate errors
            distance_to_goal = self.get_distance_to_goal()
            heading_error = self.get_heading_error()
            yaw_goal_error = self.get_radians_to_goal()
            
            rospy.loginfo(f'Distance to Goal: {distance_to_goal}')
            rospy.loginfo(f'Heading Error: {heading_error}')
            rospy.loginfo(f'Yaw Goal Error: {yaw_goal_error}')

            # Navigation logic
            if self.goal_idx < len(self.goal_x):
                if self.goal_idx == 0 or self.goal_idx == 1:
                    rospy.loginfo('1-🔜Attempting to go to perpendicular line to ARTag...')
                    self.go_to_line()
                elif self.goal_idx == 2:
                    rospy.loginfo('2-🔜Attempting to go straight to ARTag...')
                    self.go_to_artag()
            
            rospy.sleep(0.1)  # Increased sleep time for better diagnostics
        
        rospy.loginfo('CHARGING...')
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
        
        # Calculate the desired heading using atan2
        desired_heading = math.atan2(delta_y, delta_x)
        
        # Calculate heading error
        heading_error = desired_heading - current_yaw_angle
        
        # More robust normalization
        heading_error = (heading_error + math.pi) % (2 * math.pi) - math.pi
        
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
        
        rospy.loginfo(f"Distance Tolerance: {self.distance_goal_tolerance}")
        rospy.loginfo(f"Heading Tolerance: {self.heading_tolerance}")
        rospy.loginfo(f"Yaw Goal Tolerance: {self.yaw_goal_tolerance}")
        
        # If we are not yet at the position goal
        if (math.fabs(distance_to_goal) > self.distance_goal_tolerance and 
            not self.reached_distance_goal):
            
            # If the robot's heading is off, fix it
            if math.fabs(heading_error) > self.heading_tolerance:
                rospy.loginfo(f"Correcting Heading. Error: {heading_error}")
                
                if heading_error > 0:
                    cmd_vel_msg.angular.z = self.angular_velocity
                else:
                    cmd_vel_msg.angular.z = -self.angular_velocity
            else:
                rospy.loginfo("Moving forward")
                cmd_vel_msg.linear.x = self.linear_velocity
        
        # Orient towards the yaw goal angle
        elif math.fabs(yaw_goal_error) > self.yaw_goal_tolerance:
            rospy.loginfo(f"Orienting. Yaw Error: {yaw_goal_error}")
            if yaw_goal_error > 0:
                cmd_vel_msg.angular.z = self.angular_velocity
            else:
                cmd_vel_msg.angular.z = -self.angular_velocity
            
            self.reached_distance_goal = True
        
        # Goal achieved, go to the next goal  
        else:
            # Go to the next goal
            self.goal_idx += 1
            rospy.loginfo(f'Arrived at point {self.goal_idx-1}. Moving to next point.')
            rospy.loginfo(f'Next Goal: x={self.goal_x[self.goal_idx]}, y={self.goal_y[self.goal_idx]}')
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
            rospy.loginfo('Arrived at the charging dock...')
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
        
        # Log battery state for diagnostics
        rospy.loginfo(f"Battery Status: Percentage={this_battery_state.percentage}, Status={this_battery_state.power_supply_status}")

def main():
    """
    Entry point for the program.
    """
    # Initialize the ROS node
    rospy.init_node('charging_dock_navigator')
    
    # Initialize pose listener first
    pose_listener = PoseListener()
    
    # Create the node objects
    connect_to_charging_dock_navigator = ConnectToChargingDockNavigator()
    battery_state_subscriber = BatteryStateSubscriber()
    
    # Timer for navigation
    timer = rospy.Timer(rospy.Duration(0.1), 
                        lambda event: connect_to_charging_dock_navigator.navigate_to_dock())
    
    # Spin
    rospy.spin()

if __name__ == '__main__':
    main()
# ==================================Tested Working===========================================    
# #!/usr/bin/env python3

# """
# Specialized Charging Dock Navigator
# - Rotates robot to align charging point with dock
# """

# import rospy
# import math
# import actionlib
# import tf.transformations
# from geometry_msgs.msg import PoseStamped, Twist, Quaternion
# from sensor_msgs.msg import BatteryState
# from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
# from actionlib_msgs.msg import GoalStatus
# from std_msgs.msg import Float64MultiArray
# from std_srvs.srv import Empty

# class ChargingDockNavigator:
#     def __init__(self):
#         rospy.init_node('charging_dock_navigator')
        
#         # Velocity command publisher
#         self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
#         # RViz goal visualization publisher
#         self.goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1, latch=True)
        
#         # Move base action client
#         self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
#         rospy.loginfo("Waiting for move_base action server...")
#         self.move_base_client.wait_for_server()
        
#         # Service for clearing costmaps
#         self.clear_costmaps = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)
        
#         # Battery state subscriber
#         rospy.Subscriber('/battery_status', BatteryState, self.battery_callback, queue_size=10)
        
#         # Pose subscriber
#         rospy.Subscriber('/map_to_base_link_pose2d', Float64MultiArray, self.pose_callback)
        
#         # Docking parameters
#         self.LOW_BATTERY_THRESHOLD = 0.25
#         self.DOCK_X = 0.0  # X coordinate of charging stage
#         self.DOCK_Y = 0.0  # Y coordinate of charging stage

#         # Track the last known charging state
#         self.is_currently_charging = False
        
#         # Add this line to initialize is_navigating_to_dock
#         self.is_navigating_to_dock = False
        
#         # Last battery message for reference
#         self.last_battery_msg = None     
        
#         # Current robot state
#         self.current_x = 0.0
#         self.current_y = 0.0
#         self.current_yaw = 0.0
        
#         # Docking waypoints
#         self.goal_x = [-1.0, -1.0, -1.0]  # X coordinates of waypoints
#         self.goal_y = [1.4, 1.0, 0.5]  # Y coordinates of waypoints
#         self.goal_yaw_angle = [1.5708, 1.5708, 1.5708]  # Orientation at each waypoint
#         # Track navigation completion
#         self.navigation_completed = False            
#         rospy.loginfo("Charging Dock Navigator Initialized")
    
#     def publish_goal_to_rviz(self, x, y, yaw=None):
#         """
#         Publish goal pose to RViz for visualization
#         """
#         goal_pose = PoseStamped()
#         goal_pose.header.frame_id = 'map'
#         goal_pose.header.stamp = rospy.Time.now()
#         goal_pose.pose.position.x = x
#         goal_pose.pose.position.y = y
        
#         # If yaw is provided, convert to quaternion
#         if yaw is not None:
#             # Convert yaw to quaternion
#             quaternion = tf.transformations.quaternion_from_euler(0, 0, yaw)
#             goal_pose.pose.orientation = Quaternion(*quaternion)
#         else:
#             # Default orientation if no yaw provided
#             goal_pose.pose.orientation.w = 1.0
        
#         # Publish to RViz
#         self.goal_pub.publish(goal_pose)
#         rospy.loginfo(f"Published goal to RViz: {x}, {y}")
    
#     def pose_callback(self, msg):
#         """Update current robot pose"""
#         if len(msg.data) >= 3:
#             self.current_x = msg.data[0]
#             self.current_y = msg.data[1]
#             self.current_yaw = msg.data[2]
    
#     def battery_callback(self, msg):
#         """
#         Manage battery states and charging navigation
#         """
#         # Store the last battery message
#         self.last_battery_msg = msg
        
#         # Detailed logging for debugging
#         rospy.loginfo(f"Battery Status: "
#                     f"Voltage={msg.voltage}, "
#                     f"Percentage={msg.percentage * 100:.2f}%, "
#                     f"Power Supply Status={msg.power_supply_status}")
        
#         # Conditions for initiating charging dock navigation
#         if (msg.percentage <= self.LOW_BATTERY_THRESHOLD and 
#             msg.power_supply_status == BatteryState.POWER_SUPPLY_STATUS_NOT_CHARGING and
#             not self.is_navigating_to_dock and 
#             not self.navigation_completed):
            
#             rospy.logwarn(f"Low battery detected! "
#                         f"Battery at {msg.percentage * 100:.2f}%. "
#                         "Initiating charging dock approach.")
            
#             # Set navigation flags
#             self.is_navigating_to_dock = True
#             self.navigation_completed = False
            
#             # Call navigation method
#             self.navigate_to_charging_dock()
        
#         # Charging state handling
#         elif msg.power_supply_status == BatteryState.POWER_SUPPLY_STATUS_CHARGING:
#             rospy.loginfo("🔋 Charging connection established!")
#             self.is_currently_charging = True
#             self.is_navigating_to_dock = False
#             self.navigation_completed = True

#     def is_charging(self):
#         """
#         Check if the robot is charging
#         """
#         # Check if we have a recent battery message
#         if self.last_battery_msg is not None:
#             # Return True if power supply status is charging
#             return (self.last_battery_msg.power_supply_status == 
#                     BatteryState.POWER_SUPPLY_STATUS_CHARGING)
        
#         return False   
     
#     def navigate_to_charging_dock(self):
#         """
#         Specialized navigation to charging dock
#         1. Navigate to staging area
#         2. Navigate through docking waypoints
#         3. Final approach and docking
#         """
#         try:
#             # Clear costmaps
#             try:
#                 self.clear_costmaps()
#             except rospy.ServiceException as e:
#                 rospy.logerr(f"Could not clear costmaps: {e}")
            
#             # First, navigate to staging area
#             staging_goal = PoseStamped()
#             staging_goal.header.frame_id = 'map'
#             staging_goal.header.stamp = rospy.Time.now()
#             staging_goal.pose.position.x = 0.0
#             staging_goal.pose.position.y = 0.0
#             staging_goal.pose.position.z = 0.0
            
#             # Default orientation (facing forward)
#             staging_goal.pose.orientation.w = 1.0
            
#             # Publish staging area goal to RViz
#             self.publish_goal_to_rviz(0.0, 0.0, 0.0)
            
#             # Send staging area goal to move_base
#             move_base_goal = MoveBaseGoal()
#             move_base_goal.target_pose = staging_goal
            
#             rospy.loginfo("🚀 Initiating charging dock approach")
#             rospy.loginfo("🏁 Navigating to staging area at (0, 0, 0)")
            
#             self.move_base_client.send_goal(move_base_goal)
            
#             # Wait for staging area navigation result
#             if self.move_base_client.wait_for_result(rospy.Duration(300)):
#                 result_state = self.move_base_client.get_state()
                
#                 if result_state == GoalStatus.SUCCEEDED:
#                     rospy.loginfo("✅ Successfully reached staging area!")
#                     rospy.loginfo("🔜 Preparing to proceed to docking waypoints...")
                    
#                     # Small pause to simulate preparation
#                     rospy.sleep(2.0)
                    
#                     rospy.loginfo("🚢 Initiating precise docking sequence...")
#                 else:
#                     rospy.logerr(f"❌ Failed to reach staging area. Navigation state: {result_state}")
#                     self.is_navigating_to_dock = False
#                     return
#             else:
#                 rospy.logerr("⏰ Staging area navigation timed out!")
#                 self.is_navigating_to_dock = False
#                 return
            
#             # After reaching staging area, proceed with docking waypoints
#             for idx in range(len(self.goal_x)):
#                 # Prepare goal pose
#                 goal_pose = PoseStamped()
#                 goal_pose.header.frame_id = 'map'
#                 goal_pose.header.stamp = rospy.Time.now()
#                 goal_pose.pose.position.x = self.goal_x[idx]
#                 goal_pose.pose.position.y = self.goal_y[idx]
                
#                 # Convert yaw to quaternion
#                 quaternion = tf.transformations.quaternion_from_euler(0, 0, self.goal_yaw_angle[idx])
#                 goal_pose.pose.orientation = Quaternion(*quaternion)
                
#                 # Publish to RViz for visualization
#                 self.publish_goal_to_rviz(self.goal_x[idx], self.goal_y[idx], self.goal_yaw_angle[idx])
                
#                 # Send navigation goal
#                 move_base_goal = MoveBaseGoal()
#                 move_base_goal.target_pose = goal_pose
                
#                 rospy.loginfo(f"🎯 Navigating to waypoint {idx+1}: ({self.goal_x[idx]}, {self.goal_y[idx]})")
                
#                 # Send goal to move_base
#                 self.move_base_client.send_goal(move_base_goal)
                
#                 # Wait for result
#                 if self.move_base_client.wait_for_result(rospy.Duration(300)):
#                     result_state = self.move_base_client.get_state()
                    
#                     if result_state == GoalStatus.SUCCEEDED:
#                         rospy.loginfo(f"✅ Successfully reached waypoint {idx+1}")
#                     else:
#                         rospy.logerr(f"❌ Navigation to waypoint {idx+1} failed with state: {result_state}")
#                         self.is_navigating_to_dock = False
#                         break
#                 else:
#                     rospy.logerr(f"⏰ Navigation to waypoint {idx+1} timed out!")
#                     self.is_navigating_to_dock = False                
#                     break
            
#             # Final docking approach
#             self.final_docking_approach()
#             # At the end of successful navigation
#             self.navigation_completed = True
#             self.is_navigating_to_dock = False
        
#         except Exception as e:
#             rospy.logerr(f"❌ Docking navigation error: {e}")
#             self.is_navigating_to_dock = False
#             self.navigation_completed = False

#     def final_docking_approach(self):
#         """
#         Final approach to charging dock
#         """
#         rospy.loginfo("Starting final docking approach")
        
#         # Prepare velocity message
#         cmd_vel = Twist()
#         cmd_vel.linear.x = 0.1  # Slow forward movement
        
#         # Attempt docking with timeout
#         start_time = rospy.Time.now()
#         dock_timeout = rospy.Duration(30)  # 30 seconds timeout
        
#         while not rospy.is_shutdown():
#             # Check for charging connection or timeout
#             if (rospy.Time.now() - start_time) > dock_timeout:
#                 rospy.logerr("Docking timeout reached!")
#                 self.is_navigating_to_dock = False
#                 self.navigation_completed = False
#                 break
            
#             # Publish slow movement
#             self.cmd_vel_pub.publish(cmd_vel)
#             rospy.sleep(0.1)
            
#             # Wait for charging state to be activated
#             if (self.last_battery_msg and 
#                 self.last_battery_msg.power_supply_status == BatteryState.POWER_SUPPLY_STATUS_CHARGING):
#                 rospy.loginfo("🔋 Successfully docked and charging!")
                
#                 # Stop the robot
#                 stop_cmd = Twist()
#                 self.cmd_vel_pub.publish(stop_cmd)
                
#                 # Reset navigation state
#                 self.is_navigating_to_dock = False
#                 self.navigation_completed = True
#                 self.is_currently_charging = True
#                 break
        
#         # Stop the robot if not already stopped
#         stop_cmd = Twist()
#         self.cmd_vel_pub.publish(stop_cmd)
        
# def main():
#     try:
#         navigator = ChargingDockNavigator()
#         rospy.spin()
#     except rospy.ROSException as e:
#         rospy.logerr(f"ROS Exception: {e}")

# if __name__ == '__main__':
#     main()
