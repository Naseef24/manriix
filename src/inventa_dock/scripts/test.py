    
#!/usr/bin/env python3

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
Requirements:
  - Subscribe to battery status
  - When low battery level detected, navigate to docking stage
  - Once at staging area, perform docking approach
  - Detect when charging begins and stop movement
"""

import time
import rospy
import actionlib
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import BatteryState
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from std_srvs.srv import Empty

# Constants
LOW_BATTERY_THRESHOLD = 0.25  # 25% battery level
STAGING_AREA_X = -1.0 #0.0  # X coordinate of staging area near dock
STAGING_AREA_Y = 0.5 #2.0  # Y coordinate of staging area near dock
STAGING_AREA_Z = 0.0  # Z coordinate of staging area near dock

class ChargingDockNavigator:
    """
    Monitors battery and navigates to charging dock when needed
    """
    def __init__(self):
        # Initialize node
        rospy.init_node('charging_dock_navigator')
        
        # Battery state tracking
        self.current_battery = BatteryState()
        self.prev_battery = BatteryState()
        self.low_battery_detected = False
        self.is_navigating = False
        
        # Publisher for velocity commands
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        # Publisher for visualization in RViz
        self.goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1, latch=True)
        
        # Action client for navigation
        rospy.loginfo("Connecting to move_base action server...")
        self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.move_base_client.wait_for_server()
        rospy.loginfo("Connected to move_base action server")
        
        # Try to set up costmap clearing service
        try:
            rospy.wait_for_service('/move_base/clear_costmaps', timeout=5.0)
            self.clear_costmaps = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)
            rospy.loginfo("Connected to clear_costmaps service")
        except rospy.ROSException:
            rospy.logwarn("clear_costmaps service not available")
            self.clear_costmaps = None
        
        # Subscribe to battery status
        rospy.Subscriber('/battery_status', BatteryState, self.battery_callback, queue_size=10)
        
        # Docking approach parameters
        self.linear_velocity = 0.0   # Initial forward speed (adjusted later)
        self.angular_velocity = 0.15  # Rotation speed for docking approach
        
        rospy.loginfo("Charging Dock Navigator initialized")
        rospy.loginfo(f"Low battery threshold: {LOW_BATTERY_THRESHOLD * 100}%")
        rospy.loginfo(f"Staging area coordinates: ({STAGING_AREA_X}, {STAGING_AREA_Y})")
        
    def battery_callback(self, msg):
        """
        Process battery state updates and trigger navigation when needed
        """
        # Update battery state
        self.prev_battery = self.current_battery
        self.current_battery = msg
        
        # Log battery state periodically (every ~10 seconds)
        if int(rospy.get_time()) % 10 == 0:
            rospy.loginfo(f"Battery level: {msg.percentage * 100:.1f}% (Status: {msg.power_supply_status})")
        
        # Check for low battery threshold crossing
        if (self.prev_battery.percentage >= LOW_BATTERY_THRESHOLD and 
            msg.percentage < LOW_BATTERY_THRESHOLD):
            rospy.logwarn(f"Battery dropped below threshold: {msg.percentage * 100:.1f}%")
            self.low_battery_detected = True
            
        # If battery is low and we're not already navigating, start navigation
        if self.low_battery_detected and not self.is_navigating:
            rospy.loginfo("Low battery detected. Initiating navigation to charging dock")
            self.is_navigating = True
            self.navigate_to_dock()
    
    def navigate_to_dock(self):
        """
        Navigate to the staging area near the charging dock
        """
        try:
            rospy.loginfo("Navigating to charging dock staging area...")
            
            # Clear costmaps if the service is available
            if self.clear_costmaps:
                try:
                    self.clear_costmaps()
                    rospy.loginfo("Costmaps cleared")
                except rospy.ServiceException as e:
                    rospy.logwarn(f"Failed to clear costmaps: {e}")
            
            # Create goal pose for the staging area
            goal_pose = PoseStamped()
            goal_pose.header.frame_id = 'map'
            goal_pose.header.stamp = rospy.Time.now()
            
            # Set position
            goal_pose.pose.position.x = STAGING_AREA_X
            goal_pose.pose.position.y = STAGING_AREA_Y
            goal_pose.pose.position.z = STAGING_AREA_Z
            
            # Set orientation (facing forward)
            goal_pose.pose.orientation.x = 0.0
            goal_pose.pose.orientation.y = 0.0
            goal_pose.pose.orientation.z = 0.0
            goal_pose.pose.orientation.w = 1.0
            
            # Publish goal for visualization in RViz
            self.goal_pub.publish(goal_pose)
            
            # Create move_base goal
            move_base_goal = MoveBaseGoal()
            move_base_goal.target_pose = goal_pose
            
            # Send the goal
            rospy.loginfo(f"Sending goal to move_base: ({STAGING_AREA_X}, {STAGING_AREA_Y})")
            self.move_base_client.send_goal(move_base_goal)
            
            # Wait for result with timeout (10 minutes)
            success = self.move_base_client.wait_for_result(rospy.Duration(600))
            
            if success:
                state = self.move_base_client.get_state()
                if state == GoalStatus.SUCCEEDED:
                    rospy.loginfo("Successfully reached charging dock staging area")
                    # Start docking procedure
                    self.connect_to_dock()
                else:
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
                    }.get(state, "UNKNOWN")
                    rospy.logwarn(f"Navigation failed with status: {status_text}")
                    self.is_navigating = False
            else:
                rospy.logwarn("Navigation timed out")
                self.is_navigating = False
                
        except Exception as e:
            rospy.logerr(f"Error during navigation: {e}")
            self.is_navigating = False
    
    def connect_to_dock(self):
        """
        Final approach to connect with the charging dock
        Rotates the robot to find and connect with dock
        """
        rospy.loginfo("Starting final docking approach (rotating to find dock)")
        
        # Create Twist message for rotation
        cmd_vel = Twist()
        cmd_vel.linear.x = self.linear_velocity
        cmd_vel.angular.z = self.angular_velocity
        
        # Start rotation until charging begins or timeout
        rate = rospy.Rate(10)  # 10 Hz
        start_time = rospy.Time.now()
        timeout = rospy.Duration(60)  # 60-second timeout
        
        try:
            while not rospy.is_shutdown():
                # Check if we're now charging
                if self.current_battery.power_supply_status == 1:  # 1 = CHARGING
                    rospy.loginfo("CHARGING DETECTED! Successfully connected to dock")
                    # Stop the robot
                    self.stop_robot()
                    self.low_battery_detected = False
                    self.is_navigating = False
                    return
                
                # Check for timeout
                if (rospy.Time.now() - start_time) > timeout:
                    rospy.logwarn("Docking approach timed out")
                    self.stop_robot()
                    self.is_navigating = False
                    return
                
                # Keep rotating
                rospy.loginfo_throttle(5, "Rotating to find charging dock...")
                self.cmd_vel_pub.publish(cmd_vel)
                rate.sleep()
                
        except Exception as e:
            rospy.logerr(f"Error during docking approach: {e}")
        finally:
            self.stop_robot()
            self.is_navigating = False
    
    def stop_robot(self):
        """
        Stop the robot's movement
        """
        cmd_vel = Twist()
        # Send stop command multiple times to ensure it's received
        for _ in range(5):
            self.cmd_vel_pub.publish(cmd_vel)
            rospy.sleep(0.1)
        rospy.loginfo("Robot stopped")

def main():
    """
    Main function
    """
    try:
        navigator = ChargingDockNavigator()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Node interrupted")
    except Exception as e:
        rospy.logerr(f"Unexpected error: {e}")

if __name__ == '__main__':
    main()