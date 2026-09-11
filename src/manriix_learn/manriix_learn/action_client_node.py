#!/usr/bin/env python3
"""action client_node.py - A simple ROS 2 Action Client node that sends a goal to the CountSession action server and logs feedback and result.
"""
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from manriix_learn.action import CountSession

class C:
    RESET    = '\033[0m'
    BOLD     = '\033[1m'
    SENDING  = '\033[34m'    # blue
    ACCEPTED = '\033[32m'    # green
    FEEDBACK = '\033[36m'    # cyan
    RESULT   = '\033[35m'    # magenta
    ERROR    = '\033[31m'    # red

class ActionClientNode(Node):
    def __init__(self):
        super().__init__('manriix_learn_action_client')
        self._client = ActionClient(self, CountSession, 'count_session')
        self.timer = self.create_timer(5.0, self.send_goal)  # send a goal every 5 seconds
        self.goal_sent = False
        # self.get_logger().info('ActionClient started - sending goal in 5 seconds...')
        self.get_logger().info(
            f'{C.SENDING}{C.BOLD}[ACTION CLIENT]{C.RESET} '
            f'started — sending goal in 5s...')

    def send_goal(self):
        if self.goal_sent:
            return  # only send one goal for this example
        self.goal_sent = True    
        self.timer.cancel()  # stop the timer after sending the first goal

        if not self._client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Action server not available - exiting')
            return

        goal = CountSession.Goal()
        goal.target_count = 20
        goal.session_duration = 5.0  # seconds    
        self.get_logger().info(f'Sending goal ....')
        future = self._client.send_goal_async(goal, feedback_callback=self._feedback_callback)
        future.add_done_callback(self._goal_response_callback)

    def _feedback_callback(self, feedback_msg):
        fb = feedback_msg.feedback    
        # self.get_logger().info(f'Feedback: count={fb.current_count} '
        #                        f'remaining={fb.remaining_time:.1f}s')
        self.get_logger().info(
            f'{C.FEEDBACK}  ↳ feedback{C.RESET} '
            f'count={C.BOLD}{fb.current_count}{C.RESET} '
            f'remaining={fb.remaining_time:.1f}s')
        
    def _goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        # self.get_logger().info('Goal accepted - waiting for result')
        self.get_logger().info(
            f'{C.ACCEPTED}{C.BOLD}[ACCEPTED]{C.RESET} '
            f'waiting for result...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_callback)

    def _result_callback(self, future):
        result = future.result().result
        # self.get_logger().info(f'Goal result: final_count={result.final_count}'
        #                        f'completed={result.completed} '
        #                        f'reason={result.stop_reason}')   
        self.get_logger().info(
            f'{C.RESULT}{C.BOLD}[RESULT]{C.RESET} '
            f'final={result.final_count} '
            f'completed={result.completed} '
            f'reason={result.stop_reason}')  

def main(args=None):
    rclpy.init(args=args)
    node = ActionClientNode()
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

