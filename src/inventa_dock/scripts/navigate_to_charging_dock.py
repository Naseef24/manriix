#! /usr/bin/env python3
 
"""
Description:
  Navigate to a charging dock once the battery gets low.
-------
Subscription Topics:
  Current battery state
  /battery_status - sensor_msgs/BatteryState
-------
Publishing Topics:
  Velocity command to navigate to the charging dock.
  /cmd_vel - geometry_msgs/Twist
-------
Author: Addison Sears-Collins (original)
Website: AutomaticAddison.com
Date: Original: November 16, 2021, Converted to ROS 1: April 17, 2025
"""
 
import time  # Time library
import rospy  # Python client library for ROS
import actionlib  # For action client
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
# modification1:
# from geometry_msgs.msg import Pose, Point, Quaternion
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped

from geometry_msgs.msg import Twist  # Velocity command
from sensor_msgs.msg import BatteryState  # Battery status
 
# Holds the current state of the battery
this_battery_state = BatteryState()
prev_battery_state = BatteryState()
 
# Flag for detecting the change in the battery state
low_battery = False
low_battery_min_threshold = 0.25

class BatteryStateSubscriber():
    
    def get_battery_state(self, msg):
        """
        Update the current battery state.
        """
        global this_battery_state
        global prev_battery_state
        global low_battery
        prev_battery_state = this_battery_state
        this_battery_state = msg

# modification2:
        # Log battery state periodically (every ~10 seconds)
        if int(rospy.get_time()) % 10 == 0:
            rospy.loginfo(f"Battery level: {msg.percentage * 100:.1f}% (Status: {msg.power_supply_status})")
                   
        # Check for low battery
        if prev_battery_state.percentage >= low_battery_min_threshold and this_battery_state.percentage < low_battery_min_threshold:
            low_battery = True    
# modification3:
            rospy.logwarn(f"Battery dropped below threshold: {msg.percentage * 100:.1f}%")
                
    """
    Subscriber node to the current battery state
    """     
    def __init__(self):
        # Initialize the node
        # rospy.init_node('battery_state_subscriber', anonymous=True)
     
        # Create a subscriber 
        # This node subscribes to messages of type
        # sensor_msgs/BatteryState
        self.subscription_battery_state = rospy.Subscriber(
            '/battery_status',
            BatteryState,
            self.get_battery_state,
            queue_size=10)
          
class ConnectToChargingDockNavigator():


    def navigate_to_dock_callback(self, event):
        global low_battery
        
        if low_battery == False:
            return None
        
        self.is_navigating = True
        rospy.loginfo('Navigating to the charging dock...')

        # Create the goal pose
        goal_pose = PoseStamped()

        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = rospy.Time.now()
        goal_pose.pose.position.x = 0.0
        goal_pose.pose.position.y = 2.0
        goal_pose.pose.position.z = 0.25
        goal_pose.pose.orientation.x = 0.0
        goal_pose.pose.orientation.y = 0.0
        goal_pose.pose.orientation.z = 0.0
        goal_pose.pose.orientation.w = 1.0

        # Publish the goal pose for visualization in RViz
        self.goal_pub.publish(goal_pose)
        rospy.loginfo(f"Publishing goal to RViz: ({goal_pose.pose.position.x}, {goal_pose.pose.position.y})") 

        # Set up the goal for move_base
        goal = MoveBaseGoal()
        goal.target_pose = goal_pose
        
        # Tracking variables for feedback
        start_time = rospy.Time.now()
        feedback_interval = rospy.Duration(5)  # Provide feedback every 5 seconds
        last_feedback_time = start_time
        max_navigation_time = rospy.Duration(600)  # 10-minute timeout
        
        try:
            # Send the goal to the action server
            self.move_base_client.send_goal(goal)
            rospy.loginfo("Goal sent to move_base, waiting for result...")
            
            # Continuous feedback and checking loop
            rate = rospy.Rate(10)  # 10 Hz
            while not rospy.is_shutdown():
                # Check if goal is done
                if self.move_base_client.wait_for_result(rospy.Duration(1.0)):
                    break
                
                # Periodic feedback
                current_time = rospy.Time.now()
                if current_time - last_feedback_time >= feedback_interval:
                    # Get and log feedback
                    state = self.move_base_client.get_state()
                    feedback = {
                        GoalStatus.PENDING: "Pending",
                        GoalStatus.ACTIVE: "Active",
                        GoalStatus.PREEMPTED: "Preempted",
                        GoalStatus.SUCCEEDED: "Succeeded",
                        GoalStatus.ABORTED: "Aborted"
                    }.get(state, "Unknown")
                    
                    rospy.loginfo(f"Navigation status: {feedback}")
                    last_feedback_time = current_time
                
                # Check for timeout
                if current_time - start_time > max_navigation_time:
                    rospy.logwarn("Navigation timeout reached")
                    self.move_base_client.cancel_goal()
                    self.is_navigating = False
                    return
                
                rate.sleep()
            
            # Check final status
            status = self.move_base_client.get_state()
            status_text = {
                GoalStatus.PENDING: "PENDING",
                GoalStatus.ACTIVE: "ACTIVE",
                GoalStatus.PREEMPTED: "PREEMPTED",
                GoalStatus.SUCCEEDED: "SUCCEEDED",
                GoalStatus.ABORTED: "ABORTED",
                GoalStatus.REJECTED: "REJECTED",
                GoalStatus.PREEMPTING: "PREEMPTING",
                GoalStatus.RECALLING: "RECALLING",
                GoalStatus.RECALLED: "RECALLED",
                GoalStatus.LOST: "LOST"
            }.get(status, "UNKNOWN")
            
            if status == GoalStatus.SUCCEEDED:
                rospy.loginfo('Successfully reached charging dock staging area...')
                rospy.loginfo('Preparing to connect to dock...')
                low_battery = False
                self.connect_to_dock()
            else:
                rospy.loginfo(f'Navigation failed with status: {status_text}')
                self.is_navigating = False
            
        except Exception as e:
            rospy.logerr(f"Error during navigation: {e}")
            self.is_navigating = False
                     
    def connect_to_dock(self):
        
        # # While the battery is not charging
        # while this_battery_state.power_supply_status != 1:
        #     # Publish the current battery state
        #     rospy.loginfo('NOT CHARGING...')
        
        #     # Send the velocity command to the robot by publishing to the topic
        #     cmd_vel_msg = Twist()
        #     cmd_vel_msg.linear.x = self.linear_velocity
        #     cmd_vel_msg.angular.z = self.angular_velocity
        #     self.publisher_cmd_vel.publish(cmd_vel_msg)      
        #     time.sleep(0.1)
     
        # # Stop the robot
        # cmd_vel_msg = Twist()
        # cmd_vel_msg.linear.x = 0.0
        # cmd_vel_msg.angular.z = 0.0
        # self.publisher_cmd_vel.publish(cmd_vel_msg)
     
        # rospy.loginfo('CHARGING...')
        # rospy.loginfo('Successfully connected to the charging dock!')
        
        # Modification:        
        """
        Final approach to connect with the charging dock
        Rotates the robot to find and connect with dock
        """
        rospy.loginfo("Starting final docking approach (rotating to find dock)") 

        # Start rotation until charging begins or timeout
        rate = rospy.Rate(10)  # 10 Hz
        start_time = rospy.Time.now()
        timeout = rospy.Duration(60)  # 60-second timeout
        
        try:
            # While the battery is not charging
            while this_battery_state.power_supply_status != 1 and not rospy.is_shutdown():
                # Check for timeout
                if (rospy.Time.now() - start_time) > timeout:
                    rospy.logwarn("Docking approach timed out")
                    self.stop_robot()
                    self.is_navigating = False
                    return
                
                # Publish status
                rospy.loginfo_throttle(5, 'Rotating to find charging dock...')
            
                # Send the velocity command to the robot by publishing to the topic
                cmd_vel_msg = Twist()
                cmd_vel_msg.linear.x = self.linear_velocity
                cmd_vel_msg.angular.z = self.angular_velocity
                self.publisher_cmd_vel.publish(cmd_vel_msg)      
                rate.sleep()
        
        except Exception as e:
            rospy.logerr(f"Error during docking approach: {e}")
        finally:
            # Stop the robot
            self.stop_robot()
            self.is_navigating = False
            
            if this_battery_state.power_supply_status == 1:
                rospy.loginfo('CHARGING DETECTED! Successfully connected to the charging dock!')
            else:
                rospy.logwarn('Docking approach completed but not charging')        
# Additional:  
    def stop_robot(self):
        """
        Stop the robot's movement
        """
        cmd_vel_msg = Twist()
        # Send stop command multiple times to ensure it's received
        for _ in range(5):
            self.publisher_cmd_vel.publish(cmd_vel_msg)
            rospy.sleep(0.1)
        rospy.loginfo("Robot stopped")      
        
    """
    Navigates and connects to the charging dock
    """     
    def __init__(self):
        # Initialize the node
        # rospy.init_node('connect_to_charging_dock_navigator')
        
        # Create a publisher
        # This node publishes the desired linear and angular velocity of the robot
        
                        # The Twist message has two main components:

                        # Linear velocity - Represents motion in straight lines

                        # linear.x: Forward/backward movement (positive values move forward, negative values move backward)
                        # linear.y: Left/right movement (used for holonomic robots that can move sideways)
                        # linear.z: Up/down movement (used for aerial vehicles like drones)


                        # Angular velocity - Represents rotational motion

                        # angular.x: Roll (rotation around the x-axis)
                        # angular.y: Pitch (rotation around the y-axis)
                        # angular.z: Yaw (rotation around the z-axis) - This is the most commonly used for ground robots, controlling turning   
        
        self.publisher_cmd_vel = rospy.Publisher(
            '/cmd_vel',
            Twist,
            queue_size=10)

# modification:
        # Publisher for visualization in RViz
        self.goal_pub = rospy.Publisher(
            '/move_base_simple/goal', 
            PoseStamped, 
            queue_size=1, 
            latch=True)     
    
        # Set up the move_base action client
        self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        self.move_base_client.wait_for_server()
        rospy.loginfo("Connected to move_base action server")
        
        # Declare velocities
        self.linear_velocity = 0.0
        self.angular_velocity = 0.15
        
# modification:
        # Navigation state tracking
        self.is_navigating = False
                
        # Set up timer
        rospy.Timer(rospy.Duration(0.1), self.navigate_to_dock_callback)
         

def main():
    """
    Entry point for the program.
    """
    try:
        # Initialize node ONCE for the entire process
        rospy.init_node('charging_dock_navigator')
        rospy.loginfo("Charging dock navigator initialized")        
        # Create the node objects
        navigator = ConnectToChargingDockNavigator()
        battery_subscriber = BatteryStateSubscriber()
        
        # Keep the program running
        rospy.spin()
        
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation node terminated.")
 
if __name__ == '__main__':
    main()
    
