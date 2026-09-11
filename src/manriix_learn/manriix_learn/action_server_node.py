#!/usr/bin/env python3
"""action_server_node.py - A simple ROS 2 Action Server node that implements a Fibonacci action
    action_server_node.py — CountSession action server

    Runs a counting session for a requested duration.
    Publishes feedback every 0.5s.
"""

import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from manriix_learn.action import CountSession   

class C:
    RESET    = '\033[0m'
    BOLD     = '\033[1m'
    # Action states
    WAITING  = '\033[90m'    # dark grey
    GOAL     = '\033[34m'    # blue
    ACCEPTED = '\033[32m'    # green
    FEEDBACK = '\033[36m'    # cyan
    RESULT   = '\033[35m'    # magenta
    CANCEL   = '\033[33m'    # yellow
    ERROR    = '\033[31m'    # red

class ActionServerNode(Node):
    def __init__(self):
        super().__init__('manriix_learn_action_server')
        self._action_cb_group = ReentrantCallbackGroup()
        self._action_server = ActionServer(
            self,
            CountSession,
            'count_session',
            execute_callback=self._execute_callback,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=self._action_cb_group
        )
        self.get_logger().info('Action Server started - waiting for goals...')  


    def _goal_callback(self, goal_request): 
        """Accept or reject incoming goal requests."""
        # self.get_logger().info(
        #     f'Goal recieved: target={goal_request.target_count} duration={goal_request.session_duration}s')
        self.get_logger().info(
            f'{C.GOAL}{C.BOLD}[GOAL RECEIVED]{C.RESET} '
            f'target={goal_request.target_count} '
            f'duration={goal_request.session_duration}s')
        return GoalResponse.ACCEPT
    
    def _cancel_callback(self, goal_handle):
        """Handle cancellation requests."""
        # self.get_logger().info('Cancel request received')
        self.get_logger().info(
            f'{C.CANCEL}{C.BOLD}[CANCELLED]{C.RESET} ')
        return CancelResponse.ACCEPT
    
    def _execute_callback(self, goal_handle):
        """Main session execution - runs in its own thread."""
        # self.get_logger().info('Executing goal')
        target = goal_handle.request.target_count
        duration = goal_handle.request.session_duration
        start_time = time.time()
        count = 0               

        self.get_logger().info(
            f'{C.ACCEPTED}{C.BOLD}[EXECUTING]{C.RESET} '
            f'session started — target={target} duration={duration}s')

        feedback = CountSession.Feedback()  
        result = CountSession.Result()
                
        #run until duration expires or cancelled
        while time.time() - start_time < duration:

           #check if client request cancellation
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.final_count = count
                result.completed = False
                result.stop_reason = 'Cancelled by client'
                self.get_logger().info(f'Goal cancelled at count={count}')
                return result

            count += 1
            elapsed = time.time() - start_time
            remaining = duration - elapsed

            #publish feedback every 0.5s
            feedback.current_count = count
            feedback.elapsed_time = float(elapsed)
            feedback.remaining_time = float(remaining)
            goal_handle.publish_feedback(feedback)

            # self.get_logger().info(f'count={count} elapsed={elapsed:.1f}s remaining={remaining:.1f}s') 
            self.get_logger().info(
                f'{C.FEEDBACK}  ↳ feedback{C.RESET} '
                f'count={C.BOLD}{count}{C.RESET} '
                f'elapsed={elapsed:.1f}s remaining={remaining:.1f}s')
            time.sleep(0.5)  #simulate work

        #session complete
        goal_handle.succeed()
        result.final_count = count
        result.completed = True
        result.stop_reason = 'Session completed successfully'
        # self.get_logger().info(f'Session completed: count={count} duration={duration}s')
        self.get_logger().info(
            f'{C.RESULT}{C.BOLD}[RESULT]{C.RESET} '
            f'final_count={count} duration={duration}s — SUCCEEDED')
        return result

def main(args=None):
    rclpy.init(args=args)
    node = ActionServerNode()
    from rclpy.executors import MultiThreadedExecutor
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:        
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
