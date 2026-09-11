#!/usr/bin/env python3

# import asyncio

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.action import ActionServer, ActionClient
from rclpy.action import GoalResponse, CancelResponse

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger
from nav2_msgs.action import NavigateToPose

from tf2_ros import Buffer, TransformListener, TransformException

from rclpy.qos import (
    QoSProfile,
    DurabilityPolicy,
    ReliabilityPolicy,
)

from manriix_interfaces.action import GoToDockingStage


class DockingStageManager(Node):

    def __init__(self):
        super().__init__('docking_stage_manager')

        # ============================================================
        # Parameters
        # ============================================================

        self.declare_parameter('global_frame', 'map')
        self.declare_parameter(
            'robot_base_frame',
            'base_footprint'
        )
        self.declare_parameter('tf_timeout_sec', 2.0)
        self.declare_parameter(
            'navigate_action_name',
            '/navigate_to_pose'
        )

        self.global_frame = (
            self.get_parameter('global_frame')
            .get_parameter_value()
            .string_value
        )

        self.robot_base_frame = (
            self.get_parameter('robot_base_frame')
            .get_parameter_value()
            .string_value
        )

        self.tf_timeout_sec = (
            self.get_parameter('tf_timeout_sec')
            .get_parameter_value()
            .double_value
        )

        self.navigate_action_name = (
            self.get_parameter('navigate_action_name')
            .get_parameter_value()
            .string_value
        )

        # ============================================================
        # TF
        # ============================================================

        self.tf_buffer = Buffer()

        self.tf_listener = TransformListener(
            self.tf_buffer,
            self
        )

        # ============================================================
        # Docking stage
        # ============================================================

        self.docking_stage_pose = None
        self.docking_stage_valid = False

        # ============================================================
        # Navigation state
        # ============================================================

        self.navigation_active = False
        self.current_nav_goal_handle = None

        # ============================================================
        # Docking stage debug publisher
        # ============================================================

        pose_qos = QoSProfile(depth=1)
        pose_qos.reliability = ReliabilityPolicy.RELIABLE
        pose_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.stage_pose_pub = self.create_publisher(
            PoseStamped,
            '/docking_stage_pose',
            pose_qos
        )

        # ============================================================
        # SET docking stage service
        # ============================================================

        self.set_stage_service = self.create_service(
            Trigger,
            '/set_docking_stage',
            self.set_docking_stage_callback
        )

        # ============================================================
        # Nav2 client
        # ============================================================

        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            self.navigate_action_name
        )

        # ============================================================
        # MANRIIX action server
        # ============================================================

        self.go_stage_action_server = ActionServer(
            self,
            GoToDockingStage,
            '/go_to_docking_stage',
            execute_callback=self.execute_go_to_stage,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )

        # Feedback logging throttle
        self.last_logged_distance = None

        self.get_logger().info(
            '=============================================='
        )
        self.get_logger().info(
            'MANRIIX Docking Stage Manager started'
        )
        self.get_logger().info(
            f'Global frame:       {self.global_frame}'
        )
        self.get_logger().info(
            f'Robot base frame:   {self.robot_base_frame}'
        )
        self.get_logger().info(
            f'Nav2 action:        {self.navigate_action_name}'
        )
        self.get_logger().info(
            'Service:            /set_docking_stage'
        )
        self.get_logger().info(
            'Action:             /go_to_docking_stage'
        )
        self.get_logger().info(
            'Debug topic:        /docking_stage_pose'
        )
        self.get_logger().info(
            '=============================================='
        )

    # ================================================================
    # SET DOCKING STAGE
    # ================================================================

    def set_docking_stage_callback(self, request, response):

        if self.navigation_active:
            response.success = False
            response.message = (
                'Cannot overwrite docking stage while '
                'return navigation is active.'
            )
            return response

        self.get_logger().info(
            'Docking-stage capture requested.'
        )

        try:

            transform = self.tf_buffer.lookup_transform(
                self.global_frame,
                self.robot_base_frame,
                rclpy.time.Time(),
                timeout=Duration(
                    seconds=self.tf_timeout_sec
                )
            )

        except TransformException as ex:

            response.success = False

            response.message = (
                f'Failed to obtain TF '
                f'{self.global_frame} -> '
                f'{self.robot_base_frame}: {ex}'
            )

            self.get_logger().error(
                response.message
            )

            return response

        pose = PoseStamped()

        pose.header.stamp = (
            self.get_clock().now().to_msg()
        )

        pose.header.frame_id = self.global_frame

        pose.pose.position.x = (
            transform.transform.translation.x
        )

        pose.pose.position.y = (
            transform.transform.translation.y
        )

        pose.pose.position.z = 0.0

        pose.pose.orientation.x = (
            transform.transform.rotation.x
        )

        pose.pose.orientation.y = (
            transform.transform.rotation.y
        )

        pose.pose.orientation.z = (
            transform.transform.rotation.z
        )

        pose.pose.orientation.w = (
            transform.transform.rotation.w
        )

        self.docking_stage_pose = pose
        self.docking_stage_valid = True

        self.stage_pose_pub.publish(pose)

        response.success = True

        response.message = (
            'Docking stage captured successfully: '
            f'frame={pose.header.frame_id}, '
            f'x={pose.pose.position.x:.3f}, '
            f'y={pose.pose.position.y:.3f}, '
            f'qz={pose.pose.orientation.z:.6f}, '
            f'qw={pose.pose.orientation.w:.6f}'
        )

        self.get_logger().info(
            '=============================================='
        )
        self.get_logger().info(
            'DOCKING STAGE CAPTURED'
        )
        self.get_logger().info(
            f'Frame: {pose.header.frame_id}'
        )
        self.get_logger().info(
            f'X:     {pose.pose.position.x:.3f} m'
        )
        self.get_logger().info(
            f'Y:     {pose.pose.position.y:.3f} m'
        )
        self.get_logger().info(
            '=============================================='
        )

        return response

    # ================================================================
    # ACTION GOAL VALIDATION
    # ================================================================

    def goal_callback(self, goal_request):

        self.get_logger().info(
            'GoToDockingStage goal requested | '
            f'reason="{goal_request.reason}"'
        )

        # Stage must exist.
        if not self.docking_stage_valid:

            self.get_logger().warn(
                'Rejecting GoToDockingStage: '
                'docking stage is not configured.'
            )

            return GoalResponse.REJECT

        # Only one docking-stage return at once.
        if self.navigation_active:

            self.get_logger().warn(
                'Rejecting GoToDockingStage: '
                'navigation already active.'
            )

            return GoalResponse.REJECT

        self.get_logger().info(
            'GoToDockingStage goal accepted.'
        )

        return GoalResponse.ACCEPT

    # ================================================================
    # ACTION CANCELLATION
    # ================================================================

    # def cancel_callback(self, goal_handle):

    #     self.get_logger().warn(
    #         'GoToDockingStage cancellation requested.'
    #     )

    #     return CancelResponse.ACCEPT

    def cancel_callback(self, goal_handle):

        self.get_logger().warn(
            'GoToDockingStage cancellation requested.'
        )

        # If the forwarded Nav2 goal already exists,
        # immediately request cancellation there too.
        if self.current_nav_goal_handle is not None:

            self.get_logger().warn(
                'Forwarding cancellation request to Nav2.'
            )

            self.current_nav_goal_handle.cancel_goal_async()

        return CancelResponse.ACCEPT
    
    # ================================================================
    # NAV2 FEEDBACK
    # ================================================================

    def nav_feedback_callback(
        self,
        feedback_msg,
        manriix_goal_handle
    ):

        feedback = feedback_msg.feedback

        manriix_feedback = (
            GoToDockingStage.Feedback()
        )

        manriix_feedback.distance_remaining = float(
            feedback.distance_remaining
        )

        manriix_feedback.state = 'navigating'

        manriix_goal_handle.publish_feedback(
            manriix_feedback
        )

        # Don't flood the terminal like Step 1C.
        distance = feedback.distance_remaining

        if (
            self.last_logged_distance is None
            or abs(
                distance -
                self.last_logged_distance
            ) >= 0.10
        ):

            self.get_logger().info(
                'Returning to docking stage | '
                f'distance remaining: '
                f'{distance:.2f} m'
            )

            self.last_logged_distance = distance

    # ================================================================
    # EXECUTE GoToDockingStage
    # ================================================================

    async def execute_go_to_stage(
        self,
        goal_handle
    ):

        reason = goal_handle.request.reason

        self.navigation_active = True
        self.last_logged_distance = None

        result = GoToDockingStage.Result()

        self.get_logger().info(
            '=============================================='
        )

        self.get_logger().info(
            'RETURN TO DOCKING STAGE STARTED'
        )

        self.get_logger().info(
            f'Reason: {reason}'
        )

        self.get_logger().info(
            '=============================================='
        )

        # ------------------------------------------------------------
        # Check Nav2
        # ------------------------------------------------------------

        feedback = GoToDockingStage.Feedback()
        feedback.distance_remaining = -1.0
        feedback.state = 'waiting_for_nav2'

        goal_handle.publish_feedback(feedback)

        nav2_available = (
            self.nav_client.wait_for_server(
                timeout_sec=2.0
            )
        )

        if not nav2_available:

            self.navigation_active = False

            result.success = False
            result.message = (
                'Nav2 /navigate_to_pose '
                'action server is unavailable.'
            )

            self.get_logger().error(
                result.message
            )

            goal_handle.abort()

            return result

        # ------------------------------------------------------------
        # Construct Nav2 goal
        # ------------------------------------------------------------

        nav_goal = NavigateToPose.Goal()

        nav_goal.pose = PoseStamped()

        nav_goal.pose.header.stamp = (
            self.get_clock().now().to_msg()
        )

        nav_goal.pose.header.frame_id = (
            self.docking_stage_pose.header.frame_id
        )

        nav_goal.pose.pose = (
            self.docking_stage_pose.pose
        )

        feedback.state = 'goal_sent'
        goal_handle.publish_feedback(feedback)

        # ------------------------------------------------------------
        # Send goal
        # ------------------------------------------------------------

        send_goal_future = (
            self.nav_client.send_goal_async(
                nav_goal,
                feedback_callback=lambda msg:
                    self.nav_feedback_callback(
                        msg,
                        goal_handle
                    )
            )
        )

        nav_goal_handle = await send_goal_future

        if not nav_goal_handle.accepted:

            self.navigation_active = False

            result.success = False
            result.message = (
                'Nav2 rejected the docking-stage goal.'
            )

            self.get_logger().error(
                result.message
            )

            goal_handle.abort()

            return result

        self.current_nav_goal_handle = (
            nav_goal_handle
        )

        self.get_logger().info(
            'Nav2 accepted docking-stage goal.'
        )

        feedback.state = 'navigating'
        goal_handle.publish_feedback(feedback)

        # nav_result_future = (
        #     nav_goal_handle.get_result_async()
        # )

        # # ------------------------------------------------------------
        # # Wait for result while monitoring cancellation
        # # ------------------------------------------------------------

        # while not nav_result_future.done():

        #     if goal_handle.is_cancel_requested:

        #         self.get_logger().warn(
        #             'Cancelling Nav2 docking-stage goal...'
        #         )

        #         feedback.state = 'cancelling'
        #         goal_handle.publish_feedback(
        #             feedback
        #         )

        #         cancel_future = (
        #             nav_goal_handle.cancel_goal_async()
        #         )

        #         await cancel_future

        #         self.navigation_active = False
        #         self.current_nav_goal_handle = None

        #         goal_handle.canceled()

        #         result.success = False
        #         result.message = (
        #             'Return to docking stage cancelled.'
        #         )

        #         self.get_logger().warn(
        #             result.message
        #         )

        #         return result

        #     await asyncio.sleep(0.1)

        # # ------------------------------------------------------------
        # # Nav2 finished
        # # ------------------------------------------------------------

        # nav_result = nav_result_future.result()

        nav_result_future = (
            nav_goal_handle.get_result_async()
        )

        # ------------------------------------------------------------
        # Wait directly for Nav2.
        #
        # This is an rclpy Future, so we await the ROS future itself.
        # Do NOT use asyncio.sleep() here.
        # ------------------------------------------------------------

        # nav_result = await nav_result_future

        try:

            nav_result = await nav_result_future

        except Exception as ex:

            self.get_logger().error(
                f'Exception while waiting for Nav2 result: {ex}'
            )

            # Do not leave a Nav2 goal running if our wrapper fails.
            if self.current_nav_goal_handle is not None:

                self.get_logger().warn(
                    'Cancelling Nav2 goal because '
                    'GoToDockingStage encountered an exception.'
                )

                self.current_nav_goal_handle.cancel_goal_async()

            self.navigation_active = False
            self.current_nav_goal_handle = None

            goal_handle.abort()

            result.success = False
            result.message = (
                f'GoToDockingStage internal error: {ex}'
            )

            return result

        nav_status = nav_result.status

        self.navigation_active = False
        self.current_nav_goal_handle = None

        # STATUS_SUCCEEDED
        if nav_status == GoalStatus.STATUS_SUCCEEDED:

            goal_handle.succeed()

            result.success = True

            result.message = (
                'Docking stage reached successfully.'
            )

            self.get_logger().info(
                '=============================================='
            )

            self.get_logger().info(
                'DOCKING STAGE REACHED SUCCESSFULLY'
            )

            self.get_logger().info(
                '=============================================='
            )

            return result

        # STATUS_CANCELED
        # if nav_status == GoalStatus.STATUS_CANCELED:

        #     goal_handle.canceled()

        #     result.success = False

        #     result.message = (
        #         'Nav2 docking-stage navigation '
        #         'was cancelled.'
        #     )

        #     self.get_logger().warn(
        #         result.message
        #     )

        #     return result

        if (
            nav_status == GoalStatus.STATUS_CANCELED
            or goal_handle.is_cancel_requested
        ):

            goal_handle.canceled()

            result.success = False
            result.message = (
                'Return to docking stage cancelled.'
            )

            self.get_logger().warn(
                result.message
            )

            return result
        
        # Everything else is treated as failure.
        goal_handle.abort()

        result.success = False

        result.message = (
            'Nav2 failed to reach docking stage. '
            f'Nav2 status={nav_status}'
        )

        self.get_logger().error(
            result.message
        )

        return result


def main(args=None):

    rclpy.init(args=args)

    node = DockingStageManager()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    finally:

        node.go_stage_action_server.destroy()

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()