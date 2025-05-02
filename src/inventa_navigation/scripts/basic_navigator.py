#!/usr/bin/env python3

# Copyright 2021 Samsung Research America
# Modified for ROS1 Noetic

import time
from enum import Enum

import rospy
import actionlib
from std_srvs.srv import Empty
from nav_msgs.srv import GetMap, GetPlan, GetPlanRequest, GetPlanResponse
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Path, OccupancyGrid
from actionlib_msgs.msg import GoalStatus


class NavigationResult(Enum):
    UNKNOWN = 0
    SUCCEEDED = 1
    CANCELED = 2
    FAILED = 3 


class BasicNavigator:
    def __init__(self, initialize_node=True):
        self.node_name = 'basic_navigator'
        if initialize_node:
            rospy.init_node(self.node_name)
        
        self.initial_pose = PoseStamped()
        self.initial_pose.header.frame_id = 'map'
        
        self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.move_base_client.wait_for_server()
        
        self.goal_handle = None
        self.feedback = None
        self.status = None
        self.result = None
        self.is_active = False
        
        # Subscribe to the amcl pose topic to get the robot's localization
        self.initial_pose_received = False
        self.localization_pose_sub = rospy.Subscriber('amcl_pose', 
                                                     PoseWithCovarianceStamped,
                                                     self._amclPoseCallback)
        
        # Publisher for initialpose
        self.initial_pose_pub = rospy.Publisher('initialpose', 
                                               PoseWithCovarianceStamped,
                                               queue_size=10)
        
        # Service clients
        # For map swapping, we'll use a different approach than ROS2
        self.clear_costmap_global_srv = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)
        
        rospy.loginfo("BasicNavigator initialized")

    def setInitialPose(self, initial_pose):
        self.initial_pose_received = False
        self.initial_pose = initial_pose
        self._setInitialPose()

    def goToPose(self, pose):
        """Send a goal to the move_base action server"""
        self.debug("Sending goal to move_base")
        
        # Create a MoveBaseGoal
        goal = MoveBaseGoal()
        goal.target_pose = pose
        
        # Send the goal to the action server
        self.info('Navigating to goal: ' + str(pose.pose.position.x) + ' ' +
                 str(pose.pose.position.y) + '...')
        
        self.move_base_client.send_goal(goal, 
                                       done_cb=self._doneCallback,
                                       active_cb=self._activeCallback,
                                       feedback_cb=self._feedbackCallback)
        
        self.is_active = True
        return True

    def followWaypoints(self, poses):
        """Sequentially follow a list of waypoints"""
        self.debug("Following waypoints")
        if not poses or len(poses) == 0:
            self.error("No waypoints provided")
            return False
            
        self.waypoints = poses
        self.current_waypoint_idx = 0
        
        # Start navigation to the first waypoint
        return self._navigateToNextWaypoint()
        
    def _navigateToNextWaypoint(self):
        if self.current_waypoint_idx < len(self.waypoints):
            waypoint = self.waypoints[self.current_waypoint_idx]
            self.info("Navigating to waypoint {}/{}".format(self.current_waypoint_idx+1, len(self.waypoints)))
            return self.goToPose(waypoint)
        else:
            self.info("All waypoints reached")
            return True

    def cancelNav(self):
        """Cancel the current navigation goal"""
        self.info('Canceling current goal.')
        self.move_base_client.cancel_goal()
        self.is_active = False
        return

    def isNavComplete(self):
        """Check if navigation has completed"""
        # If we're not currently navigating, return True
        if not self.is_active:
            return True
            
        # Check the state of the action client
        state = self.move_base_client.get_state()
        if state == GoalStatus.ACTIVE or state == GoalStatus.PENDING:
            return False
        else:
            self.is_active = False
            self.status = state
            if state == GoalStatus.SUCCEEDED:
                self.debug('Goal succeeded!')
                
                # If we're following waypoints, move to the next one
                if hasattr(self, 'waypoints') and self.current_waypoint_idx < len(self.waypoints) - 1:
                    self.current_waypoint_idx += 1
                    self._navigateToNextWaypoint()
                    return False
            else:
                self.debug('Goal failed with status code: {}'.format(state))
            
            return True

    def getFeedback(self):
        """Get the latest feedback from move_base"""
        return self.feedback

    def getResult(self):
        """Get the result of the navigation"""
        if self.status == GoalStatus.SUCCEEDED:
            return NavigationResult.SUCCEEDED
        elif self.status == GoalStatus.PREEMPTED:
            return NavigationResult.CANCELED
        elif self.status == GoalStatus.ABORTED:
            return NavigationResult.FAILED
        else:
            return NavigationResult.UNKNOWN

    def waitUntilNav2Active(self):
        """Wait until move_base is active"""
        self.info("Waiting for move_base to become active...")
        self.move_base_client.wait_for_server()
        self._waitForInitialPose()
        self.info('move_base is ready for use!')
        return

    def getPath(self, start, goal):
        """Get a path from start to goal using move_base's make_plan service"""
        try:
            make_plan = rospy.ServiceProxy('/move_base/make_plan', GetPlan)
            req = GetPlanRequest()
            req.start = start
            req.goal = goal
            req.tolerance = 0.5
            plan = make_plan(req)
            return plan.plan
        except rospy.ServiceException as e:
            self.error("Service call failed: {}".format(e))
            return None

    def changeMap(self, map_filepath):
        """Change the map used by the navigation stack
        
        In ROS1, changing maps typically involves restarting the map_server node.
        This implementation uses subprocess to restart the map_server with the new map.
        """
        import subprocess
        import os
        
        try:
            # First, try to kill any existing map_server node
            self.info("Stopping current map_server...")
            subprocess.call(["rosnode", "kill", "/map_server"])
            rospy.sleep(1.0)  # Give it time to shut down
            
            # Make sure the map file exists
            if not os.path.exists(map_filepath):
                self.error("Map file does not exist: {}".format(map_filepath))
                return
                
            # Start a new map_server with the specified map
            self.info("Starting map_server with map: {}".format(map_filepath))
            subprocess.Popen(["rosrun", "map_server", "map_server", map_filepath])
            
            # Wait for the map to be available
            rospy.sleep(2.0)
            self.info('Map change request completed successfully')
            
        except Exception as e:
            self.error(f"Failed to change map: {e}")
        return

    def clearAllCostmaps(self):
        """Clear all costmaps"""
        try:
            self.clear_costmap_global_srv()
            self.info("Costmaps cleared")
        except rospy.ServiceException as e:
            self.error(f"Service call failed: {e}")
        return

    def _waitForInitialPose(self):
        while not self.initial_pose_received and not rospy.is_shutdown():
            self.info('Setting initial pose')
            self._setInitialPose()
            self.info('Waiting for amcl_pose to be received')
            rospy.sleep(1.0)
        return

    def _amclPoseCallback(self, msg):
        """Callback for receiving the amcl pose"""
        self.debug('Received amcl pose')
        self.initial_pose_received = True
        return

    def _feedbackCallback(self, feedback):
        """Callback for receiving feedback from move_base"""
        self.debug('Received action feedback message')
        self.feedback = feedback
        return
        
    def _activeCallback(self):
        """Callback when goal becomes active"""
        self.debug('Goal is now active')
        return
        
    def _doneCallback(self, state, result):
        """Callback when goal is done"""
        self.status = state
        self.result = result
        self.is_active = False
        return

    def _setInitialPose(self):
        """Set the initial pose of the robot for AMCL"""
        msg = PoseWithCovarianceStamped()
        msg.pose.pose = self.initial_pose.pose
        msg.header.frame_id = self.initial_pose.header.frame_id
        msg.header.stamp = rospy.Time.now()
        self.info('Publishing Initial Pose')
        self.initial_pose_pub.publish(msg)
        return

    def info(self, msg):
        rospy.loginfo(msg)
        return

    def warn(self, msg):
        rospy.logwarn(msg)
        return

    def error(self, msg):
        rospy.logerr(msg)
        return

    def debug(self, msg):
        rospy.logdebug(msg)
        return


# Define a main function so the script can be run directly
def main():
    """Example usage of the BasicNavigator class"""
    try:
        # Create the navigator
        navigator = BasicNavigator()
        
        # Wait for navigation to become active
        navigator.waitUntilNav2Active()
        
        # Example: Set the robot's goal pose
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = "map"
        goal_pose.header.stamp = rospy.Time.now()
        # goal_pose.pose.position.x = 1.0
        # goal_pose.pose.position.y = 0.0
        # goal_pose.pose.position.z = 0.0
        # goal_pose.pose.orientation.w = 1.0
        goal_pose.pose.position.x = -1.0  # Update these coordinates to match your charging station
        goal_pose.pose.position.y = 0.27  # These values should match the position in your world file
        goal_pose.pose.position.z = 0.0
        goal_pose.pose.orientation.x = 0.0
        goal_pose.pose.orientation.y = 0.0
        goal_pose.pose.orientation.z = 0.707  # For a rotation of ~90 degrees (1.57 radians)
        goal_pose.pose.orientation.w = 0.707  
              
        # Go to the goal pose
        navigator.goToPose(goal_pose)
        
        # Wait for navigation to complete
        while not navigator.isNavComplete() and not rospy.is_shutdown():
            # Get feedback
            feedback = navigator.getFeedback()
            if feedback:
                # Process feedback
                pass
            rospy.sleep(0.1)
        
        # Get the result
        result = navigator.getResult()
        if result == NavigationResult.SUCCEEDED:
            navigator.info("Goal succeeded!")
        elif result == NavigationResult.CANCELED:
            navigator.info("Goal was canceled!")
        else:
            navigator.info("Goal failed!")
            
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()