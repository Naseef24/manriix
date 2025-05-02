#!/usr/bin/env python3

import rospy
import math
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, Point, Quaternion
from tf.transformations import quaternion_from_euler
from human_clustering_pipeline.msg import OptimalPosition

class MoveBaseSeq():
    def __init__(self):
        rospy.init_node('move_base_sequence')
        
        # Initialize variables to store the target position
        self.target_x = None
        self.target_y = None
        self.target_theta = None
        
        # Create subscriber for the optimal position
        self.position_sub = rospy.Subscriber(
            "optimal_position",
            OptimalPosition,
            self.position_callback,
            queue_size=1
        )
        
        # Create action client
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        wait = self.client.wait_for_server(rospy.Duration(15.0))
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
            return
        
        rospy.loginfo("Connected to move base server")
        rospy.spin()

    def position_callback(self, msg):
        """Callback function for the position subscriber"""
        self.target_x = msg.x
        self.target_y = msg.y
        self.target_theta = msg.orientation
        
        rospy.loginfo(f"Received new target position: x={self.target_x}, y={self.target_y}, theta={math.degrees(self.target_theta)}°")
        
        # Call move_base when we receive a new position
        self.movebase_client()

    def movebase_client(self):
        """Send a goal to the move_base action server"""
        if None in (self.target_x, self.target_y, self.target_theta):
            rospy.logwarn("Target position not yet received completely!")
            return
            
        goal = MoveBaseGoal()
        
        # Set the frame parameters
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        
        # Set position
        goal.target_pose.pose.position.x = self.target_x
        goal.target_pose.pose.position.y = self.target_y
        goal.target_pose.pose.position.z = 0.0
        
        # Convert theta to quaternion
        q = quaternion_from_euler(0, 0, self.target_theta)
        goal.target_pose.pose.orientation.x = q[0]
        goal.target_pose.pose.orientation.y = q[1]
        goal.target_pose.pose.orientation.z = q[2]
        goal.target_pose.pose.orientation.w = q[3]
        
        rospy.loginfo("Sending goal pose:")
        rospy.loginfo(f"Position: x={self.target_x}, y={self.target_y}")
        # rospy.loginfo(f"Orientation: theta={math.degrees(self.target_theta)}°")
        rospy.loginfo(f"Orientation: theta={self.target_theta}°")
        
        self.client.send_goal(goal,
                            done_cb=self.done_cb,
                            active_cb=self.active_cb,
                            feedback_cb=self.feedback_cb)

    def active_cb(self):
        rospy.loginfo("Goal is now being processed by the Action Server...")

    def feedback_cb(self, feedback):
        current_position = feedback.base_position.pose
        quaternion = (
            current_position.orientation.x,
            current_position.orientation.y,
            current_position.orientation.z,
            current_position.orientation.w
        )
        current_theta = math.degrees(float(quaternion[2]))
        
        rospy.loginfo(f"Current robot position: x={current_position.position.x:.2f}, "
                     f"y={current_position.position.y:.2f}, theta={current_theta:.2f}°")

    def done_cb(self, status, result):
        if status == GoalStatus.SUCCEEDED:
            rospy.loginfo("Goal reached successfully!")
        elif status == GoalStatus.ABORTED:
            rospy.logerr("Goal was aborted!")
        elif status == GoalStatus.REJECTED:
            rospy.logerr("Goal was rejected!")
        else:
            rospy.logwarn(f"Goal status is: {status}")

if __name__ == '__main__':
    try:
        MoveBaseSeq()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation finished.")





# #!/usr/bin/env python3

# import rospy
# import math
# import actionlib
# from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
# from actionlib_msgs.msg import GoalStatus
# from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped
# from tf.transformations import euler_from_quaternion, quaternion_from_euler

# class MoveBaseSeq():
#     def __init__(self):
#         rospy.init_node('move_base_sequence')
        
#         # Initialize variables to store the target position
#         self.target_x = None
#         self.target_y = None
#         self.target_theta = None
        
#         # Create subscriber for the optimal position
#         self.position_sub = rospy.Subscriber(
#             "optimal_position",
#             PoseStamped,
#             self.position_callback,
#             queue_size=1
#         )
        
#         # Create action client
#         self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
#         rospy.loginfo("Waiting for move_base action server...")
#         wait = self.client.wait_for_server(rospy.Duration(15.0))
#         if not wait:
#             rospy.logerr("Action server not available!")
#             rospy.signal_shutdown("Action server not available!")
#             return
        
#         rospy.loginfo("Connected to move base server")
#         rospy.spin()

#     def position_callback(self, msg):
#         """Callback function for the position subscriber"""
#         self.target_x = msg.pose.position.x
#         self.target_y = msg.pose.position.y
        
#         # Extract theta from quaternion
#         quaternion = (
#             msg.pose.orientation.x,
#             msg.pose.orientation.y,
#             msg.pose.orientation.z,
#             msg.pose.orientation.w
#         )
#         # Convert quaternion to euler angles
#         euler = euler_from_quaternion(quaternion)
#         self.target_theta = euler[2]  # yaw angle
        
#         rospy.loginfo(f"Received new target position: x={self.target_x}, y={self.target_y}, theta={math.degrees(self.target_theta)}°")
        
#         # Call move_base when we receive a new position
#         self.movebase_client()

#     def movebase_client(self):
#         """Send a goal to the move_base action server"""
#         if None in (self.target_x, self.target_y, self.target_theta):
#             rospy.logwarn("Target position not yet received completely!")
#             return
            
#         goal = MoveBaseGoal()
        
#         # Set the frame parameters
#         goal.target_pose.header.frame_id = "map"
#         goal.target_pose.header.stamp = rospy.Time.now()
        
#         # Set position
#         goal.target_pose.pose.position.x = self.target_x
#         goal.target_pose.pose.position.y = self.target_y
#         goal.target_pose.pose.position.z = 0.0
        
#         # Convert theta to quaternion
#         q = quaternion_from_euler(0, 0, self.target_theta)
#         goal.target_pose.pose.orientation.x = q[0]
#         goal.target_pose.pose.orientation.y = q[1]
#         goal.target_pose.pose.orientation.z = q[2]
#         goal.target_pose.pose.orientation.w = q[3]
        
#         rospy.loginfo("Sending goal pose:")
#         rospy.loginfo(f"Position: x={self.target_x}, y={self.target_y}")
#         rospy.loginfo(f"Orientation: theta={math.degrees(self.target_theta)}°")
        
#         self.client.send_goal(goal,
#                             done_cb=self.done_cb,
#                             active_cb=self.active_cb,
#                             feedback_cb=self.feedback_cb)

#     def active_cb(self):
#         rospy.loginfo("Goal is now being processed by the Action Server...")

#     def feedback_cb(self, feedback):
#         # Convert current position for more readable feedback
#         current_position = feedback.base_position.pose
#         quaternion = (
#             current_position.orientation.x,
#             current_position.orientation.y,
#             current_position.orientation.z,
#             current_position.orientation.w
#         )
#         euler = euler_from_quaternion(quaternion)
#         current_theta = math.degrees(euler[2])
        
#         rospy.loginfo(f"Current robot position: x={current_position.position.x:.2f}, "
#                      f"y={current_position.position.y:.2f}, theta={current_theta:.2f}°")

#     def done_cb(self, status, result):
#         if status == GoalStatus.SUCCEEDED:
#             rospy.loginfo("Goal reached successfully!")
#         elif status == GoalStatus.ABORTED:
#             rospy.logerr("Goal was aborted!")
#         elif status == GoalStatus.REJECTED:
#             rospy.logerr("Goal was rejected!")
#         else:
#             rospy.logwarn(f"Goal status is: {status}")

# if __name__ == '__main__':
#     try:
#         MoveBaseSeq()
#     except rospy.ROSInterruptException:
#         rospy.loginfo("Navigation finished.")
# # --------------------------------------------

# # #!/usr/bin/env python3

# # import rospy
# # import math
# # import actionlib
# # from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
# # from actionlib_msgs.msg import GoalStatus
# # from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped
# # from tf.transformations import quaternion_from_euler

# # class MoveBaseSeq():
# #     def __init__(self):
# #         rospy.init_node('move_base_sequence')
        
# #         # Initialize variables to store the target position
# #         self.target_x = None
# #         self.target_y = None
# #         self.target_theta = None
        
# #         # Create subscriber for the optimal position
# #         # Assuming the optimal position is published as a PoseStamped message
# #         # Replace "optimal_position" with your actual topic name
# #         self.position_sub = rospy.Subscriber(
# #             "optimal_position",
# #             PoseStamped,
# #             self.position_callback,
# #             queue_size=1
# #         )
        
# #         # Create action client
# #         self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
# #         rospy.loginfo("Waiting for move_base action server...")
# #         wait = self.client.wait_for_server(rospy.Duration(15.0))
# #         if not wait:
# #             rospy.logerr("Action server not available!")
# #             rospy.signal_shutdown("Action server not available!")
# #             return
        
# #         rospy.loginfo("Connected to move base server")
        
# #         # Instead of immediately calling movebase_client, 
# #         # wait for position updates
# #         rospy.spin()

# #     def position_callback(self, msg):
# #         """Callback function for the position subscriber"""
# #         self.target_x = msg.pose.position.x
# #         self.target_y = msg.pose.position.y
        
# #         # Extract theta from quaternion
# #         # You might need to adjust this based on your message format
# #         quaternion = (
# #             msg.pose.orientation.x,
# #             msg.pose.orientation.y,
# #             msg.pose.orientation.z,
# #             msg.pose.orientation.w
# #         )
# #         # If you're receiving euler angles directly, you won't need this conversion
# #         # self.target_theta = euler from quaternion conversion here if needed
        
# #         rospy.loginfo(f"Received new target position: x={self.target_x}, y={self.target_y}")
        
# #         # Call move_base when we receive a new position
# #         self.movebase_client()

# #     def movebase_client(self):
# #         """Send a goal to the move_base action server"""
# #         if None in (self.target_x, self.target_y):
# #             rospy.logwarn("Target position not yet received!")
# #             return
            
# #         goal = MoveBaseGoal()
        
# #         # Set the frame parameters
# #         goal.target_pose.header.frame_id = "map"  # Changed to map frame
# #         goal.target_pose.header.stamp = rospy.Time.now()
        
# #         # Set position
# #         goal.target_pose.pose.position.x = self.target_x
# #         goal.target_pose.pose.position.y = self.target_y
# #         goal.target_pose.pose.position.z = 0.0
        
# #         # Set orientation
# #         # If you have theta as euler angle:
# #         # q = quaternion_from_euler(0, 0, self.target_theta)
# #         # goal.target_pose.pose.orientation = Quaternion(*q)
        
# #         # For now, keeping orientation unchanged
# #         goal.target_pose.pose.orientation.x = 0.0
# #         goal.target_pose.pose.orientation.y = 0.0
# #         goal.target_pose.pose.orientation.z = 0.0
# #         goal.target_pose.pose.orientation.w = 1.0
        
# #         rospy.loginfo("Sending goal pose")
# #         rospy.loginfo(str(goal.target_pose.pose))
        
# #         self.client.send_goal(goal,
# #                             done_cb=self.done_cb,
# #                             active_cb=self.active_cb,
# #                             feedback_cb=self.feedback_cb)

# #     def active_cb(self):
# #         rospy.loginfo("Goal is now being processed by the Action Server...")

# #     def feedback_cb(self, feedback):
# #         rospy.loginfo("Current robot location: " + str(feedback.base_position))

# #     def done_cb(self, status, result):
# #         if status == GoalStatus.SUCCEEDED:
# #             rospy.loginfo("Goal succeeded!")
# #         elif status == GoalStatus.ABORTED:
# #             rospy.logerr("Goal was aborted!")
# #         elif status == GoalStatus.REJECTED:
# #             rospy.logerr("Goal was rejected!")
# #         else:
# #             rospy.logwarn(f"Goal status is: {status}")
        
# #         # Don't shut down the node after reaching the goal
# #         # We want to keep listening for new positions
# #         # rospy.signal_shutdown("Goal finished")

# # if __name__ == '__main__':
# #     try:
# #         MoveBaseSeq()
# #     except rospy.ROSInterruptException:
# #         rospy.loginfo("Navigation finished.")
#         # ----------------------------------------------------
        
# # #!/usr/bin/env python3

# # import rospy
# # import math
# # import actionlib
# # from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
# # from actionlib_msgs.msg import GoalStatus
# # from geometry_msgs.msg import Pose, Point, Quaternion
# # from tf.transformations import quaternion_from_euler

# # class MoveBaseSeq():
# #     def __init__(self):
# #         rospy.init_node('move_base_sequence')
        
# #         try:
# #             self.points_seq = rospy.get_param('move_base_seq/p_seq')
# #             self.yaweulerangles_seq = rospy.get_param('move_base_seq/yea_seq')
# #         except KeyError as e:
# #             rospy.logerr(f'Parameters not set! Error: {e}')
# #             rospy.signal_shutdown('Parameters not set')
# #             return

# #         self.goal_cnt = 0
        
# #         # Create action client
# #         self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
# #         rospy.loginfo("Waiting for move_base action server...")
# #         wait = self.client.wait_for_server(rospy.Duration(15.0))
# #         if not wait:
# #             rospy.logerr("Action server not available!")
# #             rospy.signal_shutdown("Action server not available!")
# #             return
        
# #         rospy.loginfo("Connected to move base server")
# #         self.movebase_client()

# #     def movebase_client(self):
# #         goal = MoveBaseGoal()
        
# #         # Set the frame explicitly to base_link for relative motion
# #         goal.target_pose.header.frame_id = "base_link"  # Changed from "map" to "base_link"
# #         goal.target_pose.header.stamp = rospy.Time.now()
        
# #         # Set position (0.5m forward of current position)
# #         goal.target_pose.pose.position.x = self.points_seq[0]
# #         goal.target_pose.pose.position.y = self.points_seq[1]
# #         goal.target_pose.pose.position.z = 0.0
        
# #         # Set orientation (unchanged from current)
# #         goal.target_pose.pose.orientation.x = 0.0
# #         goal.target_pose.pose.orientation.y = 0.0
# #         goal.target_pose.pose.orientation.z = 0.0
# #         goal.target_pose.pose.orientation.w = 1.0
        
# #         rospy.loginfo("Sending goal pose")
# #         rospy.loginfo(str(goal.target_pose.pose))
        
# #         self.client.send_goal(goal,
# #                             done_cb=self.done_cb,
# #                             active_cb=self.active_cb,
# #                             feedback_cb=self.feedback_cb)
# #         rospy.spin()

# #     def active_cb(self):
# #         rospy.loginfo("Goal is now being processed by the Action Server...")

# #     def feedback_cb(self, feedback):
# #         rospy.loginfo("Current robot location: " + str(feedback.base_position))

# #     def done_cb(self, status, result):
# #         if status == GoalStatus.SUCCEEDED:
# #             rospy.loginfo("Goal succeeded!")
# #         elif status == GoalStatus.ABORTED:
# #             rospy.logerr("Goal was aborted!")
# #         elif status == GoalStatus.REJECTED:
# #             rospy.logerr("Goal was rejected!")
# #         else:
# #             rospy.logwarn(f"Goal status is: {status}")
        
# #         rospy.signal_shutdown("Goal finished")

# # if __name__ == '__main__':
# #     try:
# #         MoveBaseSeq()
# #     except rospy.ROSInterruptException:
# #         rospy.loginfo("Navigation finished.")