#!/usr/bin/env python3
"""
Move Base Sequence Node - ROS2 Version
Subscribes to optimal_position and sends goals to Nav2
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration
import math

from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from tf_transformations import quaternion_from_euler

from manriix_perception.msg import OptimalPosition


class MoveBaseSeq(Node):
    def __init__(self):
        super().__init__('move_base_sequence')
        
        # Initialize variables to store the target position
        self.target_x = None
        self.target_y = None
        self.target_theta = None
        
        # Declare parameters
        self.declare_parameter('debug_mode', True)
        self.debug_mode = self.get_parameter('debug_mode').get_parameter_value().bool_value
        
        # Create subscriber for the optimal position
        self.position_sub = self.create_subscription(
            OptimalPosition,
            'human_clustering/optimal_position',
            self.position_callback,
            10
        )
        
        # Create Nav2 action client
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        self.get_logger().info("Waiting for navigate_to_pose action server...")
        
        if not self.nav_client.wait_for_server(timeout_sec=15.0):
            self.get_logger().error("Action server not available!")
            return
        
        self.get_logger().info("Connected to Nav2 action server")
        
        # Track goal state
        self.goal_handle = None
        self.navigating = False

    def position_callback(self, msg: OptimalPosition):
        """Callback function for the position subscriber"""
        self.target_x = msg.x
        self.target_y = msg.y
        self.target_theta = msg.orientation
        
        self.get_logger().info(
            f"Received new target position: x={self.target_x:.2f}, "
            f"y={self.target_y:.2f}, theta={math.degrees(self.target_theta):.1f}°"
        )
        
        # Only send new goal if not currently navigating
        if not self.navigating:
            self.send_goal()

    def send_goal(self):
        """Send a goal to the Nav2 action server"""
        if None in (self.target_x, self.target_y, self.target_theta):
            self.get_logger().warn("Target position not yet received completely!")
            return
        
        # Create goal message
        goal_msg = NavigateToPose.Goal()
        
        # Set the frame parameters
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        
        # Set position
        goal_msg.pose.pose.position.x = self.target_x
        goal_msg.pose.pose.position.y = self.target_y
        goal_msg.pose.pose.position.z = 0.0
        
        # Convert theta to quaternion
        q = quaternion_from_euler(0, 0, self.target_theta)
        goal_msg.pose.pose.orientation.x = q[0]
        goal_msg.pose.pose.orientation.y = q[1]
        goal_msg.pose.pose.orientation.z = q[2]
        goal_msg.pose.pose.orientation.w = q[3]
        
        self.get_logger().info("Sending goal pose:")
        self.get_logger().info(f"  Position: x={self.target_x:.2f}, y={self.target_y:.2f}")
        self.get_logger().info(f"  Orientation: theta={math.degrees(self.target_theta):.1f}°")
        
        self.navigating = True
        
        # Send goal asynchronously
        send_goal_future = self.nav_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """Callback when goal is accepted/rejected"""
        self.goal_handle = future.result()
        
        if not self.goal_handle.accepted:
            self.get_logger().error("Goal was rejected!")
            self.navigating = False
            return
        
        self.get_logger().info("Goal accepted, navigating...")
        
        # Get result asynchronously
        result_future = self.goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def feedback_callback(self, feedback_msg):
        """Callback for navigation feedback"""
        feedback = feedback_msg.feedback
        current_pose = feedback.current_pose.pose
        
        # Extract current position
        x = current_pose.position.x
        y = current_pose.position.y
        
        # Extract yaw from quaternion
        q = current_pose.orientation
        # Simplified yaw extraction (assuming mostly 2D motion)
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        if self.debug_mode:
            self.get_logger().info(
                f"Current position: x={x:.2f}, y={y:.2f}, theta={math.degrees(yaw):.1f}°"
            )

    def result_callback(self, future):
        """Callback when navigation completes"""
        result = future.result()
        status = result.status
        
        self.navigating = False
        
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("Goal reached successfully!")
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().error("Goal was aborted!")
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warn("Goal was canceled!")
        else:
            self.get_logger().warn(f"Goal finished with status: {status}")

    def cancel_goal(self):
        """Cancel the current navigation goal"""
        if self.goal_handle is not None:
            self.get_logger().info("Canceling current goal...")
            cancel_future = self.goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(self.cancel_callback)

    def cancel_callback(self, future):
        """Callback when cancel request completes"""
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            self.get_logger().info("Goal successfully canceled")
        else:
            self.get_logger().warn("Goal cancel request failed")
        self.navigating = False


def main(args=None):
    rclpy.init(args=args)
    
    node = MoveBaseSeq()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Navigation node shutting down...")
        node.cancel_goal()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()