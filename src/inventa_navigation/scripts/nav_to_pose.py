#!/usr/bin/env python3
# # for Python 3
# Copyright 2021 Samsung Research America
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Modified by AutomaticAddison.com
# Converted to ROS Noetic by Claude

import time
import rospy
import actionlib
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseResult, MoveBaseFeedback

'''
Simple navigation class for ROS1 (Noetic) that wraps Move Base functionality 
'''
class BasicNavigator:
    def __init__(self):
        # Initialize the move_base action client
        self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        self.move_base_client.wait_for_server()
        rospy.loginfo("Connected to move_base server")
        
        # Store the current navigation result
        self.nav_result = None
        self.is_nav_complete = False
        self.feedback = None

    def waitUntilNav2Active(self):
        # In ROS1, we wait for the move_base action server in the constructor
        # This method is just for API compatibility with the ROS2 version
        rospy.loginfo("Navigation stack is active")
        
    def goToPose(self, pose_stamped):
        # Convert PoseStamped to MoveBaseGoal
        goal = MoveBaseGoal()
        goal.target_pose = pose_stamped
        
        # Send the goal
        rospy.loginfo("Sending goal to move_base")
        self.is_nav_complete = False
        self.move_base_client.send_goal(
            goal,
            done_cb=self._done_callback,
            feedback_cb=self._feedback_callback
        )
    
    def _done_callback(self, status, result):
        self.is_nav_complete = True
        self.nav_result = status
    
    def _feedback_callback(self, feedback):
        self.feedback = feedback
    
    def isNavComplete(self):
        return self.is_nav_complete
    
    def getFeedback(self):
        return self.feedback
    
    def getResult(self):
        # Map actionlib states to the NavigationResult enum used in the original code
        if self.nav_result == actionlib.GoalStatus.SUCCEEDED:
            return NavigationResult.SUCCEEDED
        elif self.nav_result == actionlib.GoalStatus.PREEMPTED:
            return NavigationResult.CANCELED
        elif self.nav_result == actionlib.GoalStatus.ABORTED:
            return NavigationResult.FAILED
        else:
            return NavigationResult.UNKNOWN
            
    def cancelNav(self):
        self.move_base_client.cancel_goal()
        
    def lifecycleShutdown(self):
        # Not needed in ROS1 but kept for API compatibility
        pass

# Define NavigationResult enum for compatibility with original code
class NavigationResult:
    SUCCEEDED = 0
    CANCELED = 1
    FAILED = 2
    UNKNOWN = 3

'''
Navigates a robot from an initial pose to a goal pose.
'''
def main():
    # Initialize the ROS node
    rospy.init_node('basic_navigator')
    
    # Create the navigator object
    navigator = BasicNavigator()
    
    # Set the robot's initial pose if necessary
    # initial_pose = PoseStamped()
    # initial_pose.header.frame_id = 'map'
    # initial_pose.header.stamp = rospy.Time.now()
    # initial_pose.pose.position.x = 0.0
    # initial_pose.pose.position.y = 0.0
    # initial_pose.pose.position.z = 0.0
    # initial_pose.pose.orientation.x = 0.0
    # initial_pose.pose.orientation.y = 0.0
    # initial_pose.pose.orientation.z = 0.0
    # initial_pose.pose.orientation.w = 1.0
    # Publish initial pose to a topic in ROS1 instead of setting directly
    # initial_pose_pub = rospy.Publisher('initialpose', PoseWithCovarianceStamped, queue_size=1)
    # initial_pose_pub.publish(initial_pose_with_covariance)
    
    # Wait for the navigation stack to be ready
    navigator.waitUntilNav2Active()
    
    # Set the robot's goal pose
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = rospy.Time.now()
    goal_pose.pose.position.x = 20.0
    goal_pose.pose.position.y = -10.0
    goal_pose.pose.position.z = 0.0
    goal_pose.pose.orientation.x = 0.0
    goal_pose.pose.orientation.y = 0.0
    goal_pose.pose.orientation.z = 0.0
    goal_pose.pose.orientation.w = 1.0
    
    # Go to the goal pose
    navigator.goToPose(goal_pose)
    
    i = 0
    start_time = rospy.Time.now()
    
    # Keep doing stuff as long as the robot is moving towards the goal
    while not navigator.isNavComplete() and not rospy.is_shutdown():
        ################################################
        #
        # Implement some code here for your application!
        #
        ################################################
        
        # Do something with the feedback
        i = i + 1
        feedback = navigator.getFeedback()
        if feedback and i % 5 == 0:
            # NOTE: ROS1 move_base feedback doesn't have distance_remaining directly
            # We would need to calculate it from current position and goal
            if hasattr(feedback, 'base_position'):
                current_position = feedback.base_position
                rospy.loginfo('Current position: x={:.2f}, y={:.2f}'.format(
                    current_position.pose.position.x, 
                    current_position.pose.position.y))
            
            # Some navigation timeout to demo cancellation
            elapsed = rospy.Time.now() - start_time
            if elapsed.to_sec() > 600.0:
                navigator.cancelNav()
                
            # Some navigation request change to demo preemption
            if elapsed.to_sec() > 120.0:
                goal_pose.pose.position.x = -3.0
                navigator.goToPose(goal_pose)
                
        # Sleep to prevent CPU hogging
        rospy.sleep(0.1)
    
    # Do something depending on the return code
    result = navigator.getResult()
    if result == NavigationResult.SUCCEEDED:
        rospy.loginfo('Goal succeeded!')
    elif result == NavigationResult.CANCELED:
        rospy.loginfo('Goal was canceled!')
    elif result == NavigationResult.FAILED:
        rospy.loginfo('Goal failed!')
    else:
        rospy.loginfo('Goal has an invalid return status!')
    
    return 0
    
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass