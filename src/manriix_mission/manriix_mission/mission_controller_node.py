#!/usr/bin/env python3
"""
Manriix Mission Controller Node v2 - Smooth Photographer Edition

Key Behaviors:
1. Never abrupt stops - always smooth transitions
2. Complete nearby tasks - if < 1.5m from target, finish it
3. Humans need high score - threshold 0.7 to interrupt
4. Stability check - humans must be stable before switching
5. Queue system - remember POIs, don't forget them
6. Scenic can wait - it's not going anywhere

States:
- IDLE: Waiting for START command
- EXPLORING: Following exploration goals
- EVALUATING: New POI detected, deciding whether to switch
- NAVIGATING_TO_POI: Going to a POI for photo
- WAITING_FOR_PHOTO: Timer-based photo capture
- TRANSITIONING: Smooth transition between targets
- PAUSED: Mission paused
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from std_msgs.msg import String, Bool, Float32, Int32
from geometry_msgs.msg import PoseStamped, Point, Twist
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus

from enum import Enum
import math
import time
from typing import Optional
from dataclasses import dataclass


class MissionState(Enum):
    IDLE = "IDLE"
    EXPLORING = "EXPLORING"
    EVALUATING = "EVALUATING"           # NEW: Deciding whether to switch
    NAVIGATING_TO_POI = "NAVIGATING_TO_POI"
    WAITING_FOR_PHOTO = "WAITING_FOR_PHOTO"
    TRANSITIONING = "TRANSITIONING"      # NEW: Smooth transition
    PAUSED = "PAUSED"


@dataclass
class NavigationTarget:
    """Represents a navigation target with metadata"""
    pose: PoseStamped
    target_type: str  # 'exploration', 'human_poi', 'scenic_poi', 'queued_poi'
    score: float = 0.0
    set_time: float = 0.0
    
    def age(self) -> float:
        return time.time() - self.set_time


class MissionControllerNode(Node):
    def __init__(self):
        super().__init__('mission_controller_node')
        
        # Callback group for concurrent callbacks
        self.cb_group = ReentrantCallbackGroup()
        
        # ═══════════════════════════════════════════════════════════════════
        # PARAMETERS
        # ═══════════════════════════════════════════════════════════════════
        
        self.declare_parameter('photo_duration', 60.0)           # Time at POI
        self.declare_parameter('nav_timeout', 120.0)             # Navigation timeout
        self.declare_parameter('poi_arrival_threshold', 1.0)     # Distance = arrived
        self.declare_parameter('completion_threshold', 1.5)      # Don't interrupt if closer
        self.declare_parameter('transition_speed_factor', 0.5)   # Slow down during transition
        self.declare_parameter('evaluation_time', 1.0)           # Time to evaluate new POI
        
        self.photo_duration = self.get_parameter('photo_duration').value
        self.nav_timeout = self.get_parameter('nav_timeout').value
        self.poi_arrival_threshold = self.get_parameter('poi_arrival_threshold').value
        self.completion_threshold = self.get_parameter('completion_threshold').value
        self.transition_speed_factor = self.get_parameter('transition_speed_factor').value
        self.evaluation_time = self.get_parameter('evaluation_time').value
        
        # ═══════════════════════════════════════════════════════════════════
        # STATE
        # ═══════════════════════════════════════════════════════════════════
        
        self.state = MissionState.IDLE
        self.current_target: Optional[NavigationTarget] = None
        self.pending_target: Optional[NavigationTarget] = None  # NEW: For smooth transition
        self.photo_start_time: Optional[float] = None
        self.nav_start_time: Optional[float] = None
        self.evaluation_start_time: Optional[float] = None
        self.last_poi_position: Optional[Point] = None
        self.paused_state: Optional[MissionState] = None
        
        # POI tracking
        self.current_poi_score: float = 0.0
        self.poi_interrupt_ok: bool = False
        self.poi_queue_size: int = 0
        
        # ═══════════════════════════════════════════════════════════════════
        # NAVIGATION ACTION CLIENT
        # ═══════════════════════════════════════════════════════════════════
        
        self.nav_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose',
            callback_group=self.cb_group
        )
        self.current_nav_goal_handle = None
        
        # ═══════════════════════════════════════════════════════════════════
        # PUBLISHERS
        # ═══════════════════════════════════════════════════════════════════
        
        self.state_pub = self.create_publisher(String, '/mission/state', 10)
        self.photo_command_pub = self.create_publisher(String, '/photo/command', 10)
        self.current_target_pub = self.create_publisher(PoseStamped, '/mission/current_target', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel_smooth', 10)  # For speed control
        
        # ═══════════════════════════════════════════════════════════════════
        # SUBSCRIBERS
        # ═══════════════════════════════════════════════════════════════════
        
        self.create_subscription(
            String, '/mission/command',
            self.command_callback, 10,
            callback_group=self.cb_group
        )
        
        # POI Manager inputs
        self.create_subscription(
            PoseStamped, '/poi/target',
            self.poi_target_callback, 10,
            callback_group=self.cb_group
        )
        
        self.create_subscription(
            Bool, '/poi/should_capture',
            self.should_capture_callback, 10,
            callback_group=self.cb_group
        )
        
        self.create_subscription(
            Bool, '/poi/interrupt_ok',
            self.interrupt_ok_callback, 10,
            callback_group=self.cb_group
        )
        
        self.create_subscription(
            Float32, '/poi/score',
            self.poi_score_callback, 10,
            callback_group=self.cb_group
        )
        
        self.create_subscription(
            String, '/poi/type',
            self.poi_type_callback, 10,
            callback_group=self.cb_group
        )
        
        self.create_subscription(
            Int32, '/poi/queue_size',
            self.queue_size_callback, 10,
            callback_group=self.cb_group
        )
        
        # Exploration inputs
        self.create_subscription(
            PoseStamped, '/exploration/goal',
            self.exploration_goal_callback, 10,
            callback_group=self.cb_group
        )
        
        # ═══════════════════════════════════════════════════════════════════
        # TIMERS
        # ═══════════════════════════════════════════════════════════════════
        
        self.create_timer(0.5, self.state_machine_tick, callback_group=self.cb_group)
        self.create_timer(1.0, self.publish_state, callback_group=self.cb_group)
        self.create_timer(0.2, self.publish_current_target, callback_group=self.cb_group)
        
        self.get_logger().info("="*60)
        self.get_logger().info("Mission Controller v2 - Smooth Photographer Edition")
        self.get_logger().info("="*60)
        self.get_logger().info(f"  Photo duration: {self.photo_duration}s")
        self.get_logger().info(f"  Completion threshold: {self.completion_threshold}m")
        self.get_logger().info(f"  Transition speed: {self.transition_speed_factor*100:.0f}%")
        self.get_logger().info("="*60)
        self.get_logger().info("Waiting for START command...")
        
    # ═══════════════════════════════════════════════════════════════════════
    # PUBLISHERS
    # ═══════════════════════════════════════════════════════════════════════
        
    def publish_state(self):
        """Publish current state"""
        msg = String()
        msg.data = self.state.value
        self.state_pub.publish(msg)
        
    def publish_current_target(self):
        """Publish current navigation target for POI manager awareness"""
        if self.current_target is not None:
            self.current_target_pub.publish(self.current_target.pose)
            
    # ═══════════════════════════════════════════════════════════════════════
    # COMMAND CALLBACKS
    # ═══════════════════════════════════════════════════════════════════════
        
    def command_callback(self, msg):
        """Handle mission commands: START, STOP, PAUSE"""
        cmd = msg.data.upper().strip()
        self.get_logger().info(f"📥 Command received: {cmd}")
        
        if cmd == "START":
            if self.state == MissionState.IDLE:
                self.get_logger().info("🚀 Starting mission - EXPLORING")
                self.state = MissionState.EXPLORING
            elif self.state == MissionState.PAUSED:
                self.get_logger().info(f"▶ Resuming from PAUSED to {self.paused_state.value}")
                self.state = self.paused_state
                self.paused_state = None
            else:
                self.get_logger().info(f"Already running in {self.state.value}")
                
        elif cmd == "STOP":
            self.get_logger().info("🛑 Stopping mission")
            self._cancel_navigation()
            self._stop_photo()
            self.state = MissionState.IDLE
            self.current_target = None
            self.pending_target = None
            self.paused_state = None
            
        elif cmd == "PAUSE":
            if self.state not in [MissionState.IDLE, MissionState.PAUSED]:
                self.get_logger().info(f"⏸ Pausing from {self.state.value}")
                self.paused_state = self.state
                self._cancel_navigation()
                self.state = MissionState.PAUSED
                
    # ═══════════════════════════════════════════════════════════════════════
    # POI CALLBACKS
    # ═══════════════════════════════════════════════════════════════════════
    
    def poi_target_callback(self, msg: PoseStamped):
        """Receive POI target from POI manager"""
        # Only process in relevant states
        if self.state not in [MissionState.EXPLORING, MissionState.NAVIGATING_TO_POI, 
                              MissionState.EVALUATING]:
            return
            
        # Check if this is a new POI (not too close to last one)
        if self.last_poi_position is not None:
            dx = msg.pose.position.x - self.last_poi_position.x
            dy = msg.pose.position.y - self.last_poi_position.y
            dist = math.sqrt(dx*dx + dy*dy)
            if dist < 2.0:  # Too close to last POI
                return
                
        # Create pending target (don't switch immediately)
        self.pending_target = NavigationTarget(
            pose=msg,
            target_type='human_poi',  # Will be updated by type callback
            score=self.current_poi_score,
            set_time=time.time()
        )
        
        # Enter evaluation state if exploring
        if self.state == MissionState.EXPLORING:
            self.get_logger().info(f"📍 New POI detected, evaluating...")
            self.state = MissionState.EVALUATING
            self.evaluation_start_time = time.time()
            
        # If already navigating to POI, let interrupt_ok decide
        elif self.state == MissionState.NAVIGATING_TO_POI and self.poi_interrupt_ok:
            self.get_logger().info(f"🔄 Better POI found, transitioning smoothly...")
            self.state = MissionState.TRANSITIONING
            
    def should_capture_callback(self, msg: Bool):
        """POI manager says we should capture"""
        pass  # Handled by score and interrupt_ok
        
    def interrupt_ok_callback(self, msg: Bool):
        """POI manager says it's OK to interrupt current target"""
        self.poi_interrupt_ok = msg.data
        
    def poi_score_callback(self, msg: Float32):
        """Track current POI score"""
        self.current_poi_score = msg.data
        if self.pending_target:
            self.pending_target.score = msg.data
        
    def poi_type_callback(self, msg: String):
        """Track POI type"""
        if self.pending_target:
            self.pending_target.target_type = f"{msg.data}_poi"
            
    def queue_size_callback(self, msg: Int32):
        """Track queue size"""
        self.poi_queue_size = msg.data
        
    # ═══════════════════════════════════════════════════════════════════════
    # EXPLORATION CALLBACKS
    # ═══════════════════════════════════════════════════════════════════════
            
    def exploration_goal_callback(self, msg: PoseStamped):
        """Receive exploration goal when no POI available"""
        if self.state != MissionState.EXPLORING:
            return
            
        # Only use exploration goal if we don't have a current target
        if self.current_target is None:
            self.get_logger().info(
                f"🔍 Exploration goal: ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})"
            )
            self.current_target = NavigationTarget(
                pose=msg,
                target_type='exploration',
                score=0.0,
                set_time=time.time()
            )
            self._send_nav_goal(msg)
            
    # ═══════════════════════════════════════════════════════════════════════
    # NAVIGATION
    # ═══════════════════════════════════════════════════════════════════════
            
    def _send_nav_goal(self, pose_stamped: PoseStamped, slow: bool = False):
        """Send navigation goal"""
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Nav2 action server not available!")
            self.state = MissionState.EXPLORING
            self.current_target = None
            return
            
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose_stamped
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        
        self.nav_start_time = time.time()
        
        self.get_logger().info(f"📤 Sending nav goal... {'(slow)' if slow else ''}")
        
        send_goal_future = self.nav_client.send_goal_async(
            goal_msg,
            feedback_callback=self._nav_feedback_callback
        )
        send_goal_future.add_done_callback(self._nav_goal_response_callback)
        
    def _nav_goal_response_callback(self, future):
        """Handle navigation goal acceptance"""
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().warn("Navigation goal rejected!")
            self.state = MissionState.EXPLORING
            self.current_target = None
            return
            
        self.get_logger().info("Navigation goal accepted")
        self.current_nav_goal_handle = goal_handle
        
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._nav_result_callback)
        
    def _nav_feedback_callback(self, feedback_msg):
        """Navigation feedback - could log distance remaining"""
        pass
        
    def _nav_result_callback(self, future):
        """Handle navigation result"""
        result = future.result()
        status = result.status
        
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("✅ Navigation succeeded!")
            self._on_navigation_complete()
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().info("Navigation canceled (expected for transitions)")
            # Don't change state here - let the state machine handle it
        else:
            self.get_logger().warn(f"Navigation failed with status: {status}")
            self._on_navigation_failed()
            
        self.current_nav_goal_handle = None
        
    def _on_navigation_complete(self):
        """Called when robot arrives at goal"""
        if self.state == MissionState.NAVIGATING_TO_POI:
            # Save this POI position
            if self.current_target is not None:
                self.last_poi_position = self.current_target.pose.pose.position
            
            self.get_logger().info("="*60)
            self.get_logger().info("📸 ARRIVED AT POI - Starting photo sequence")
            self.get_logger().info(f"   Will wait {self.photo_duration} seconds")
            self.get_logger().info("="*60)
            
            # Publish START command to R6
            self._start_photo()
            
            # Enter waiting state
            self.state = MissionState.WAITING_FOR_PHOTO
            self.photo_start_time = time.time()
            
        elif self.state == MissionState.TRANSITIONING:
            # Completed smooth transition to new target
            self.get_logger().info("Transition complete, now at new POI")
            if self.current_target is not None:
                self.last_poi_position = self.current_target.pose.pose.position
            self._start_photo()
            self.state = MissionState.WAITING_FOR_PHOTO
            self.photo_start_time = time.time()
            
        else:
            # Exploration goal complete
            self.get_logger().info("Exploration goal reached")
            self.current_target = None
            self.state = MissionState.EXPLORING
            
    def _on_navigation_failed(self):
        """Called when navigation fails"""
        self.get_logger().warn("Navigation failed, returning to exploring")
        self.current_target = None
        self.state = MissionState.EXPLORING
        
    def _cancel_navigation(self):
        """Cancel current navigation"""
        if self.current_nav_goal_handle is not None:
            self.get_logger().info("Canceling navigation...")
            self.current_nav_goal_handle.cancel_goal_async()
            self.current_nav_goal_handle = None
            
    # ═══════════════════════════════════════════════════════════════════════
    # PHOTO CONTROL
    # ═══════════════════════════════════════════════════════════════════════
            
    def _start_photo(self):
        """Start photo capture"""
        msg = String()
        msg.data = "START"
        self.photo_command_pub.publish(msg)
        self.get_logger().info("📤 Published START to /photo/command")
        
    def _stop_photo(self):
        """Stop photo capture"""
        msg = String()
        msg.data = "STOP"
        self.photo_command_pub.publish(msg)
        self.get_logger().info("📤 Published STOP to /photo/command")
            
    # ═══════════════════════════════════════════════════════════════════════
    # STATE MACHINE
    # ═══════════════════════════════════════════════════════════════════════
            
    def state_machine_tick(self):
        """Main state machine tick - Smooth Photographer Logic"""
        current_time = time.time()
        
        # ─────────────────────────────────────────────────────────────────
        # EVALUATING: Decide whether to switch to new POI
        # ─────────────────────────────────────────────────────────────────
        if self.state == MissionState.EVALUATING:
            if self.evaluation_start_time is None:
                self.evaluation_start_time = current_time
                
            elapsed = current_time - self.evaluation_start_time
            
            # Wait for evaluation time (let POI stabilize)
            if elapsed >= self.evaluation_time:
                if self.pending_target and self.pending_target.score >= 0.6:
                    # Accept the POI
                    self.get_logger().info(
                        f"✓ POI accepted (score: {self.pending_target.score:.2f}) - navigating"
                    )
                    self.current_target = self.pending_target
                    self.pending_target = None
                    self.state = MissionState.NAVIGATING_TO_POI
                    self._send_nav_goal(self.current_target.pose)
                else:
                    # Reject - return to exploring
                    self.get_logger().info(
                        f"✗ POI rejected (score: {self.pending_target.score if self.pending_target else 0:.2f}) - continuing exploration"
                    )
                    self.pending_target = None
                    self.state = MissionState.EXPLORING
                    
                self.evaluation_start_time = None
                
        # ─────────────────────────────────────────────────────────────────
        # TRANSITIONING: Smooth switch to new POI
        # ─────────────────────────────────────────────────────────────────
        elif self.state == MissionState.TRANSITIONING:
            if self.pending_target is not None:
                # Cancel current navigation gracefully
                self._cancel_navigation()
                
                # Switch to new target
                self.get_logger().info(
                    f"🔄 Transitioning to better POI (score: {self.pending_target.score:.2f})"
                )
                self.current_target = self.pending_target
                self.pending_target = None
                
                # Send new goal
                self.state = MissionState.NAVIGATING_TO_POI
                self._send_nav_goal(self.current_target.pose)
            else:
                # No pending target, return to previous state
                self.state = MissionState.EXPLORING
                
        # ─────────────────────────────────────────────────────────────────
        # WAITING_FOR_PHOTO: Timer-based photo capture
        # ─────────────────────────────────────────────────────────────────
        elif self.state == MissionState.WAITING_FOR_PHOTO:
            if self.photo_start_time is not None:
                elapsed = current_time - self.photo_start_time
                remaining = self.photo_duration - elapsed
                
                # Log progress every 10 seconds
                if int(elapsed) % 10 == 0 and int(elapsed) > 0 and int(elapsed) != int(elapsed - 0.5):
                    self.get_logger().info(f"⏳ Photo: {remaining:.0f}s remaining...")
                
                # Check if time is up
                if elapsed >= self.photo_duration:
                    self.get_logger().info("="*60)
                    self.get_logger().info("✅ Photo complete!")
                    self.get_logger().info("="*60)
                    
                    self._stop_photo()
                    
                    # Check if there's a queued POI to go to next
                    if self.poi_queue_size > 0:
                        self.get_logger().info(f"📋 {self.poi_queue_size} POIs queued, checking...")
                    
                    # Return to exploring (POI manager will send next target if queued)
                    self.photo_start_time = None
                    self.current_target = None
                    self.state = MissionState.EXPLORING
                    self.get_logger().info("🔍 Returning to EXPLORING")
                    
        # ─────────────────────────────────────────────────────────────────
        # NAVIGATING_TO_POI: Check for timeout
        # ─────────────────────────────────────────────────────────────────
        elif self.state == MissionState.NAVIGATING_TO_POI:
            if self.nav_start_time is not None:
                elapsed = current_time - self.nav_start_time
                if elapsed > self.nav_timeout:
                    self.get_logger().warn(f"Navigation timeout ({self.nav_timeout}s)!")
                    self._cancel_navigation()
                    self.current_target = None
                    self.state = MissionState.EXPLORING
                    
        # ─────────────────────────────────────────────────────────────────
        # EXPLORING: Just monitoring, exploration node handles goals
        # ─────────────────────────────────────────────────────────────────
        elif self.state == MissionState.EXPLORING:
            # Exploration goals come from exploration_goal_callback
            # POI targets come from poi_target_callback
            pass


def main(args=None):
    rclpy.init(args=args)
    
    node = MissionControllerNode()
    
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


# #!/usr/bin/env python3
# """
# Manriix Mission Controller Node
# Coordinates the autonomous photography mission

# R6 Camera Mode: Timer-based (no external node required)
# - Publishes START when arriving at POI
# - Waits for configured time (default 60 seconds)
# - Publishes STOP
# - Continues exploring
# """

# import rclpy
# from rclpy.node import Node
# from rclpy.action import ActionClient
# from rclpy.callback_groups import ReentrantCallbackGroup
# from rclpy.executors import MultiThreadedExecutor

# from std_msgs.msg import String, Bool, Float32
# from geometry_msgs.msg import PoseStamped, Point
# from nav2_msgs.action import NavigateToPose
# from action_msgs.msg import GoalStatus

# from enum import Enum
# import math
# import time


# class MissionState(Enum):
#     IDLE = "IDLE"
#     EXPLORING = "EXPLORING"
#     NAVIGATING_TO_POI = "NAVIGATING_TO_POI"
#     WAITING_FOR_PHOTO = "WAITING_FOR_PHOTO"  # Now just a timer wait
#     PAUSED = "PAUSED"


# class MissionControllerNode(Node):
#     def __init__(self):
#         super().__init__('mission_controller_node')
        
#         # Callback group for concurrent callbacks
#         self.cb_group = ReentrantCallbackGroup()
        
#         # Parameters
#         self.declare_parameter('photo_duration', 60.0)  # Time to wait at POI (seconds)
#         self.declare_parameter('nav_timeout', 120.0)    # Navigation timeout
#         self.declare_parameter('poi_arrival_threshold', 1.0)  # Distance to consider "arrived"
        
#         self.photo_duration = self.get_parameter('photo_duration').value
#         self.nav_timeout = self.get_parameter('nav_timeout').value
#         self.poi_arrival_threshold = self.get_parameter('poi_arrival_threshold').value
        
#         # State
#         self.state = MissionState.IDLE
#         self.current_goal = None
#         self.photo_start_time = None
#         self.nav_start_time = None
#         self.last_poi_position = None
#         self.paused_state = None
        
#         # Navigation action client
#         self.nav_client = ActionClient(
#             self, NavigateToPose, 'navigate_to_pose',
#             callback_group=self.cb_group
#         )
#         self.current_nav_goal_handle = None
        
#         # Publishers
#         self.state_pub = self.create_publisher(String, '/mission/state', 10)
#         self.photo_command_pub = self.create_publisher(String, '/photo/command', 10)
        
#         # Subscribers
#         self.create_subscription(
#             String, '/mission/command',
#             self.command_callback, 10,
#             callback_group=self.cb_group
#         )
        
#         self.create_subscription(
#             PoseStamped, '/poi/target',
#             self.poi_target_callback, 10,
#             callback_group=self.cb_group
#         )
        
#         self.create_subscription(
#             Bool, '/poi/should_capture',
#             self.should_capture_callback, 10,
#             callback_group=self.cb_group
#         )
        
#         self.create_subscription(
#             PoseStamped, '/exploration/goal',
#             self.exploration_goal_callback, 10,
#             callback_group=self.cb_group
#         )
        
#         # Timers
#         self.create_timer(0.5, self.state_machine_tick, callback_group=self.cb_group)
#         self.create_timer(1.0, self.publish_state, callback_group=self.cb_group)
        
#         self.get_logger().info("="*50)
#         self.get_logger().info("Mission Controller Started")
#         self.get_logger().info(f"  Photo duration: {self.photo_duration}s")
#         self.get_logger().info(f"  Nav timeout: {self.nav_timeout}s")
#         self.get_logger().info("="*50)
#         self.get_logger().info("Waiting for START command...")
        
#     def publish_state(self):
#         """Publish current state"""
#         msg = String()
#         msg.data = self.state.value
#         self.state_pub.publish(msg)
        
#     def command_callback(self, msg):
#         """Handle mission commands: START, STOP, PAUSE"""
#         cmd = msg.data.upper().strip()
#         self.get_logger().info(f"Received command: {cmd}")
        
#         if cmd == "START":
#             if self.state == MissionState.IDLE:
#                 self.get_logger().info("🚀 Starting mission - EXPLORING")
#                 self.state = MissionState.EXPLORING
#             elif self.state == MissionState.PAUSED:
#                 self.get_logger().info(f"▶ Resuming from PAUSED to {self.paused_state.value}")
#                 self.state = self.paused_state
#                 self.paused_state = None
#             else:
#                 self.get_logger().info(f"Already running in {self.state.value}")
                
#         elif cmd == "STOP":
#             self.get_logger().info("🛑 Stopping mission")
#             self._cancel_navigation()
#             self.state = MissionState.IDLE
#             self.current_goal = None
#             self.paused_state = None
            
#         elif cmd == "PAUSE":
#             if self.state not in [MissionState.IDLE, MissionState.PAUSED]:
#                 self.get_logger().info(f"⏸ Pausing from {self.state.value}")
#                 self.paused_state = self.state
#                 self._cancel_navigation()
#                 self.state = MissionState.PAUSED
                
#     def poi_target_callback(self, msg):
#         """Receive POI target from POI manager"""
#         if self.state != MissionState.EXPLORING:
#             return
            
#         # Check if this is a new POI (not too close to last one)
#         if self.last_poi_position is not None:
#             dx = msg.pose.position.x - self.last_poi_position.x
#             dy = msg.pose.position.y - self.last_poi_position.y
#             dist = math.sqrt(dx*dx + dy*dy)
#             if dist < 2.0:  # Too close to last POI
#                 return
                
#         self.get_logger().info(f"📍 New POI target: ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})")
#         self.current_goal = msg
#         self.state = MissionState.NAVIGATING_TO_POI
#         self._send_nav_goal(msg)
        
#     def should_capture_callback(self, msg):
#         """POI manager says we should capture"""
#         if msg.data and self.state == MissionState.EXPLORING:
#             self.get_logger().debug("POI manager indicates capture opportunity")
            
#     def exploration_goal_callback(self, msg):
#         """Receive exploration goal when no POI available"""
#         if self.state != MissionState.EXPLORING:
#             return
            
#         # Only use exploration goal if we don't have a POI
#         if self.current_goal is None:
#             self.get_logger().info(f"🔍 Exploration goal: ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})")
#             self.current_goal = msg
#             self._send_nav_goal(msg)
            
#     def _send_nav_goal(self, pose_stamped):
#         """Send navigation goal"""
#         if not self.nav_client.wait_for_server(timeout_sec=5.0):
#             self.get_logger().error("Nav2 action server not available!")
#             self.state = MissionState.EXPLORING
#             self.current_goal = None
#             return
            
#         goal_msg = NavigateToPose.Goal()
#         goal_msg.pose = pose_stamped
#         goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        
#         self.nav_start_time = time.time()
        
#         self.get_logger().info(f"📤 Sending nav goal...")
        
#         send_goal_future = self.nav_client.send_goal_async(
#             goal_msg,
#             feedback_callback=self._nav_feedback_callback
#         )
#         send_goal_future.add_done_callback(self._nav_goal_response_callback)
        
#     def _nav_goal_response_callback(self, future):
#         """Handle navigation goal acceptance"""
#         goal_handle = future.result()
        
#         if not goal_handle.accepted:
#             self.get_logger().warn("Navigation goal rejected!")
#             self.state = MissionState.EXPLORING
#             self.current_goal = None
#             return
            
#         self.get_logger().info("Navigation goal accepted")
#         self.current_nav_goal_handle = goal_handle
        
#         result_future = goal_handle.get_result_async()
#         result_future.add_done_callback(self._nav_result_callback)
        
#     def _nav_feedback_callback(self, feedback_msg):
#         """Navigation feedback"""
#         pass  # Could add distance remaining logging
        
#     def _nav_result_callback(self, future):
#         """Handle navigation result"""
#         result = future.result()
#         status = result.status
        
#         if status == GoalStatus.STATUS_SUCCEEDED:
#             self.get_logger().info("✅ Navigation succeeded!")
#             self._on_navigation_complete()
#         elif status == GoalStatus.STATUS_CANCELED:
#             self.get_logger().info("Navigation canceled")
#         else:
#             self.get_logger().warn(f"Navigation failed with status: {status}")
#             self._on_navigation_failed()
            
#         self.current_nav_goal_handle = None
        
#     def _on_navigation_complete(self):
#         """Called when robot arrives at goal"""
#         if self.state == MissionState.NAVIGATING_TO_POI:
#             # Save this POI position
#             if self.current_goal is not None:
#                 self.last_poi_position = self.current_goal.pose.position
            
#             self.get_logger().info("="*50)
#             self.get_logger().info("📸 ARRIVED AT POI - Starting photo sequence")
#             self.get_logger().info(f"   Will wait {self.photo_duration} seconds")
#             self.get_logger().info("="*50)
            
#             # Publish START command
#             start_msg = String()
#             start_msg.data = "START"
#             self.photo_command_pub.publish(start_msg)
#             self.get_logger().info("📤 Published START to /photo/command")
            
#             # Enter waiting state
#             self.state = MissionState.WAITING_FOR_PHOTO
#             self.photo_start_time = time.time()
            
#         else:
#             # Exploration goal complete
#             self.get_logger().info("Exploration goal reached")
#             self.current_goal = None
#             self.state = MissionState.EXPLORING
            
#     def _on_navigation_failed(self):
#         """Called when navigation fails"""
#         self.get_logger().warn("Navigation failed, returning to exploring")
#         self.current_goal = None
#         self.state = MissionState.EXPLORING
        
#     def _cancel_navigation(self):
#         """Cancel current navigation"""
#         if self.current_nav_goal_handle is not None:
#             self.get_logger().info("Canceling navigation...")
#             self.current_nav_goal_handle.cancel_goal_async()
#             self.current_nav_goal_handle = None
            
#     def state_machine_tick(self):
#         """Main state machine tick"""
        
#         # Handle WAITING_FOR_PHOTO state (timer-based)
#         if self.state == MissionState.WAITING_FOR_PHOTO:
#             if self.photo_start_time is not None:
#                 elapsed = time.time() - self.photo_start_time
#                 remaining = self.photo_duration - elapsed
                
#                 # Log progress every 10 seconds
#                 if int(elapsed) % 10 == 0 and int(elapsed) > 0:
#                     self.get_logger().info(f"⏳ Photo wait: {remaining:.0f}s remaining...")
                
#                 # Check if time is up
#                 if elapsed >= self.photo_duration:
#                     self.get_logger().info("="*50)
#                     self.get_logger().info("✅ Photo duration complete!")
#                     self.get_logger().info("="*50)
                    
#                     # Publish STOP command
#                     stop_msg = String()
#                     stop_msg.data = "STOP"
#                     self.photo_command_pub.publish(stop_msg)
#                     self.get_logger().info("📤 Published STOP to /photo/command")
                    
#                     # Return to exploring
#                     self.photo_start_time = None
#                     self.current_goal = None
#                     self.state = MissionState.EXPLORING
#                     self.get_logger().info("🔍 Returning to EXPLORING state")
                    
#         # Handle navigation timeout
#         elif self.state == MissionState.NAVIGATING_TO_POI:
#             if self.nav_start_time is not None:
#                 elapsed = time.time() - self.nav_start_time
#                 if elapsed > self.nav_timeout:
#                     self.get_logger().warn(f"Navigation timeout ({self.nav_timeout}s)!")
#                     self._cancel_navigation()
#                     self.current_goal = None
#                     self.state = MissionState.EXPLORING


# def main(args=None):
#     rclpy.init(args=args)
    
#     node = MissionControllerNode()
    
#     executor = MultiThreadedExecutor()
#     executor.add_node(node)
    
#     try:
#         executor.spin()
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()


# if __name__ == '__main__':
#     main()
