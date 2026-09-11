#!/usr/bin/env python3

"""
MissionController - FIXED Version
==================================

FIXES APPLIED:
  [#1]  REMOVED direct /cmd_vel publishing. All velocity control goes through
        Nav2's action system. Emergency stops use Nav2 cancel + recovery manager.
        This is the professional Nav2 pattern — the action client owns velocity.

  [#7]  Fixed double-negative logic bug in handle_stability_state.

  [#8]  Added recovery coordination protocol:
        - Recovery state is now LOCKED: mission controller will not accept new
          targets or send new Nav2 goals while in RECOVERY state.
        - RecoveryManager must explicitly release the lock via "Level:0" status.
        - Prevents "Nav2 gets new goal while robot is mid-spin" scenario.
        - Added _recovery_lock flag and _is_recovery_action_active tracking.

  [General] Mission controller no longer fights RecoveryManager.
            Clear ownership: RecoveryManager commands → MissionController executes.
            MissionController waits for explicit "all clear" before resuming.
"""

from unittest import result

import rclpy
from rclpy.lifecycle import LifecycleNode, State, TransitionCallbackReturn
from rclpy.action import ActionClient
from manriix_perception.action import TakePhoto
from manriix_perception.mission.mission_bt import build_mission_tree
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile, ReliabilityPolicy
import numpy as np
from typing import Dict, List, Optional, Tuple
import time
import json
from tf_transformations import euler_from_quaternion, quaternion_from_euler  # ADD
from dataclasses import dataclass
from enum import Enum

from std_msgs.msg import String, Bool, Float32
from std_srvs.srv import SetBool
from geometry_msgs.msg import PoseStamped, Point, Twist
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from manriix_perception.msg import OptimalPosition, ClusterArray
from manriix_perception.utils.parameter_validator import ParameterValidator, ConfigValidator


class MissionState(Enum):
    IDLE = "idle"
    EVALUATING_TARGET = "evaluating_target"
    POSITIONING_OPTIMAL = "positioning_optimal"
    WAITING_FOR_STABILITY = "waiting_for_stability"
    PHOTO_SEQUENCE = "photo_sequence"
    MOVING_TO_NEXT = "moving_to_next"
    RECOVERY = "recovery"


@dataclass
class MissionTarget:
    """Target for mission execution"""
    position: np.ndarray
    orientation: float
    priority_score: float
    cluster_id: int
    stability_score: float = 0.0
    evaluation_start_time: float = 0.0


@dataclass
class MissionMetrics:
    """Mission execution metrics"""
    targets_evaluated: int = 0
    photos_completed: int = 0
    transitions_made: int = 0
    total_distance_traveled: float = 0.0
    average_photo_quality: float = 0.0
    mission_efficiency: float = 0.0


class MissionController(LifecycleNode):
    """
    Mission controller with professional Nav2 integration.
    
    KEY PRINCIPLE: This node NEVER publishes to /cmd_vel directly.
    All motion control goes through Nav2's action system.
    The collision monitor is the ONLY node that writes to /cmd_vel.
    
    Nav2 velocity chain (DO NOT BREAK):
        Controller → cmd_vel_smoothed → Collision Monitor → cmd_vel → Motors
    """
    
    def __init__(self):
        super().__init__('mission_controller')
        
    def on_configure(self, state: State) -> TransitionCallbackReturn:
        """Load config, create subs/pubs/service, wait for Nav2."""
        self.get_logger().info("Configuring MissionController...")
        try:
            self.cb_group = ReentrantCallbackGroup()

            validator = ParameterValidator(self)
            config = ConfigValidator.validate_mission_config(validator)
            self.evaluation_time = config['evaluation_time']
            self.stability_wait_time = config['stability_wait_time']
            self.arrival_threshold = config['arrival_threshold']

            self.transition_speed = validator.declare_and_validate(
                'transition_speed_factor', 0.7, min_val=0.1, max_val=1.0,
                description="Speed reduction during transitions")
            self.switch_threshold = validator.declare_and_validate(
                'target_switch_threshold', 0.15, min_val=0.05, max_val=0.5,
                description="Score difference to switch targets")
            self.max_evaluation_time = validator.declare_and_validate(
                'max_evaluation_time', 10.0, min_val=5.0, max_val=30.0,
                description="Maximum time to evaluate before committing")

            if not self.has_parameter('enable_autonomous_navigation'):
                self.declare_parameter('enable_autonomous_navigation', False)
            self.enable_autonomous_navigation = self.get_parameter(
                'enable_autonomous_navigation').get_parameter_value().bool_value

            if not self.enable_autonomous_navigation:
                self.get_logger().warn(
                    "\n" + "="*70 + "\n"
                    "AUTONOMOUS NAVIGATION DISABLED\n"
                    "   Robot will NOT move automatically.\n"
                    "   Set enable_autonomous_navigation:=true to enable.\n"
                    + "="*70)

            self.stability_threshold = validator.declare_and_validate(
                'stability_threshold', 0.05, min_val=0.01, max_val=0.2,
                description="Movement threshold for stability detection")
            self.replanning_interval = validator.declare_and_validate(
                'replanning_interval', 5.0, min_val=1.0, max_val=15.0,
                description="How often to replan routes")

            _nav_params = {
                'max_poi_retries': 5,
                'per_goal_timeout': 60.0,
                'failed_position_radius': 1.5,
                'failed_position_memory_time': 300.0,
                'progress_check_window': 15.0,
                'progress_min_distance': 0.5,
                'absolute_nav_timeout': 180.0,
            }
            for _k, _v in _nav_params.items():
                if not self.has_parameter(_k):
                    self.declare_parameter(_k, _v)

            self.max_poi_retries = self.get_parameter('max_poi_retries').value
            self.per_goal_timeout = self.get_parameter('per_goal_timeout').value
            self.failed_position_radius = self.get_parameter('failed_position_radius').value
            self.failed_position_memory_time = self.get_parameter('failed_position_memory_time').value
            self.progress_check_window = self.get_parameter('progress_check_window').value
            self.progress_min_distance = self.get_parameter('progress_min_distance').value
            self.absolute_nav_timeout = self.get_parameter('absolute_nav_timeout').value

            # State
            self.current_state = MissionState.IDLE
            self.current_target = None
            self.available_targets = []
            self.robot_position = None
            self.robot_orientation = None
            self.last_position = None
            self.position_history = []
            self._teleop_paused = False
            self._camera_active = False
            self._take_photo_goal_handle = None
            self._last_successful_position = None            
            self._failed_positions = []
            self._all_pois = []
            self._current_retry_count = 0
            self._retry_active = False
            self._nav_goal_start_distance = None
            self._last_nav_goal_time = 0.0
            self._recovery_cooldown_until = 0.0
            self._recovery_lock = False
            self._is_recovery_action_active = False
            self._pending_recovery_command = None
            self._scan_angles = []
            self._scan_index = 0
            self._scan_position = None
            self._scan_pause_timer = None
            self.state_start_time = time.time()
            self.last_replan_time = time.time()
            self.last_position_time = time.time()
            self.metrics = MissionMetrics()
            self.current_nav_goal_handle = None
            self.nav_start_time = None
            self.navigation_timeout = self.per_goal_timeout

            # QoS
            qos_reliable = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, depth=5)
            qos_best_effort = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=3)

            # Nav2 Action Client
            self.nav_client = ActionClient(
                self, NavigateToPose, 'navigate_to_pose',
                callback_group=self.cb_group)
            
            self.take_photo_client = ActionClient(
                self, TakePhoto, 'take_photo',
                callback_group=self.cb_group)

            if self.nav_client.wait_for_server(timeout_sec=3.0):
                self.get_logger().info("Nav2 action server connected")
            else:
                self.get_logger().warn("Nav2 not available yet — will retry on first goal")

            # Subscribers
            self.optimal_positions_sub = self.create_subscription(
                OptimalPosition, '/poi_manager/optimal_positions',
                self.optimal_positions_callback, qos_reliable, callback_group=self.cb_group)
            self.all_pois_sub = self.create_subscription(
                String, '/poi_manager/all_pois',
                self.all_pois_callback, qos_best_effort, callback_group=self.cb_group)
            self.exploration_goal_sub = self.create_subscription(
                PoseStamped, '/exploration/goal',
                self.exploration_goal_callback, qos_reliable, callback_group=self.cb_group)
            self.clusters_sub = self.create_subscription(
                ClusterArray, '/human_clustering/clusters',
                self.clusters_callback, qos_reliable, callback_group=self.cb_group)
            self.robot_pose_sub = self.create_subscription(
                Odometry, '/odometry/filtered',
                self.robot_pose_callback, qos_best_effort, callback_group=self.cb_group)
            # self.photo_status_sub = self.create_subscription(
            #     String, '/photo/intelligence/status',
            #     self.photo_status_callback, qos_reliable, callback_group=self.cb_group)
            # self.recovery_status_sub = self.create_subscription(
            #     String, '/recovery/status',
            #     self.recovery_status_callback, qos_reliable, callback_group=self.cb_group)
            # self.recovery_command_sub = self.create_subscription(
            #     String, '/recovery/command',
            #     self.recovery_command_callback, qos_reliable, callback_group=self.cb_group)
            self.photo_status_sub = self.create_subscription(
                String, '/photo/intelligence/status',
                self.photo_status_callback, qos_reliable, callback_group=self.cb_group)
            
            self.teleop_override_sub = self.create_subscription(
                Bool, '/teleop/override',
                self.teleop_override_callback, qos_reliable, callback_group=self.cb_group)
            self.photo_command_sub = self.create_subscription(
                String, '/photo/command',
                self.photo_command_callback, qos_reliable, callback_group=self.cb_group)

            # Publishers
            self.nav_goal_pub = self.create_publisher(
                PoseStamped, '/move_base_simple/goal', qos_reliable)
            self.photo_trigger_pub = self.create_publisher(
                Bool, '/photo/intelligence/trigger', qos_reliable)
            self.photo_command_pub = self.create_publisher(
                String, '/photo/command', qos_reliable)
            self.mission_status_pub = self.create_publisher(
                String, '/mission/status', qos_reliable)
            self.mission_metrics_pub = self.create_publisher(
                String, '/mission/metrics', qos_best_effort)
            # self.recovery_ack_pub = self.create_publisher(
            #     String, '/mission/recovery_ack', qos_reliable)

            # Service
            self.set_autonomous_srv = self.create_service(
                SetBool, '/mission_controller/set_autonomous',
                self._handle_set_autonomous, callback_group=self.cb_group)

            # Build Mission BT — replaces inner state machine
            self._mission_tree = build_mission_tree(self)
            self.get_logger().info("Mission BT built ✅")
            self.get_logger().info("MissionController configured (NO direct /cmd_vel — Nav2 owns velocity)")
            return TransitionCallbackReturn.SUCCESS

        except Exception as e:
            self.get_logger().error(f"Configuration failed: {e}")
            return TransitionCallbackReturn.FAILURE

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        """Start mission control loop."""
        self.get_logger().info("Activating MissionController...")
        try:
            self.control_timer = self.create_timer(0.2, self._bt_tick)
            self.metrics_timer = self.create_timer(2.0, self.publish_metrics)
            # One-time blocking wait — safe here, not inside BT tick
            if self.take_photo_client.wait_for_server(timeout_sec=5.0):
                self.get_logger().info("TakePhoto action server connected ✓")
            else:
                self.get_logger().warn("TakePhoto action server not found at activation")
            self.get_logger().info("MissionController active — mission control loop running")
            return TransitionCallbackReturn.SUCCESS
        except Exception as e:
            self.get_logger().error(f"Activation failed: {e}")
            return TransitionCallbackReturn.FAILURE

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        """Stop mission control loop and cancel navigation."""
        self.get_logger().info("Deactivating MissionController...")
        try:
            if hasattr(self, 'control_timer') and self.control_timer:
                self.control_timer.cancel()
                self.control_timer = None
            if hasattr(self, 'metrics_timer') and self.metrics_timer:
                self.metrics_timer.cancel()
                self.metrics_timer = None
            self._cancel_navigation()
            return TransitionCallbackReturn.SUCCESS
        except Exception as e:
            self.get_logger().error(f"Deactivation failed: {e}")
            return TransitionCallbackReturn.FAILURE

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        """Release all resources."""
        self.get_logger().info("Cleaning up MissionController...")
        try:
            self._cancel_navigation()
            self.current_target = None
            self.available_targets = []
            self._failed_positions = []
            self._all_pois = []
            return TransitionCallbackReturn.SUCCESS
        except Exception as e:
            self.get_logger().error(f"Cleanup failed: {e}")
            return TransitionCallbackReturn.FAILURE

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        """Final cleanup — stop camera and cancel nav."""
        self.get_logger().info("Shutting down MissionController...")
        try:
            self._cancel_navigation()
            if hasattr(self, '_take_photo_goal_handle') and self._take_photo_goal_handle:
                self._take_photo_goal_handle.cancel_goal_async()
                self._take_photo_goal_handle = None
            if hasattr(self, 'photo_command_pub'):
                stop_msg = String()
                stop_msg.data = "no"
                self.photo_command_pub.publish(stop_msg)
            if hasattr(self, 'control_timer') and self.control_timer:
                self.control_timer.cancel()
            if hasattr(self, 'metrics_timer') and self.metrics_timer:
                self.metrics_timer.cancel()
            return TransitionCallbackReturn.SUCCESS
        except Exception as e:
            self.get_logger().error(f"Shutdown failed: {e}")
            return TransitionCallbackReturn.FAILURE
        
        
    def _handle_set_autonomous(self, request, response):
        """
        Service handler — toggle autonomous navigation at runtime.

        Usage:
          ros2 service call /mission_controller/set_autonomous std_srvs/srv/SetBool "{data: true}"
          ros2 service call /mission_controller/set_autonomous std_srvs/srv/SetBool "{data: false}"
        """
        old = self.enable_autonomous_navigation
        self.enable_autonomous_navigation = request.data

        if request.data and not old:
            self.get_logger().warn("✅ Autonomous navigation ENABLED via service")
        elif not request.data and old:
            self.get_logger().warn("🛑 Autonomous navigation DISABLED via service")
            # Cancel any active navigation immediately
            if self.current_nav_goal_handle is not None:
                self._cancel_navigation()
            self.current_target = None
            self.transition_to_state(MissionState.IDLE)

        response.success = True
        response.message = f"Autonomous navigation set to {request.data}"
        return response

    # ========================================================================
    # CALLBACKS
    # ========================================================================
        
    def optimal_positions_callback(self, msg: OptimalPosition):
        """Receive optimal positions from POIManager"""
        try:
            target = MissionTarget(
                position=np.array([msg.x, msg.y]),
                orientation=msg.orientation,
                priority_score=msg.priority_score,
                cluster_id=0,
                stability_score=0.5
            )
            
            self.available_targets = [target]
            
            # [FIX #8] Do NOT accept new targets while recovery lock is held
            if self._recovery_lock:
                self.get_logger().debug("Recovery lock active — ignoring new POI target")
                return

            # [FIX #NAV] Don't interrupt active retry sequence
            if self._retry_active:
                self.get_logger().debug("POI retry active — queuing new optimal position")
                return

            # [TELEOP] Ignore POIs while manually paused
            if self._teleop_paused:
                return  

            if self.current_state == MissionState.IDLE and not self.current_target:
                if not self.enable_autonomous_navigation:
                    self.get_logger().debug("🛑 Navigation disabled — staying in IDLE")
                    return

                # If robot is already very close to the POI, skip navigation
                # and start the photo session in place
                if self.robot_position is not None:
                    travel_distance = np.linalg.norm(
                        target.position - self.robot_position
                    )
                    if travel_distance < 0.8:
                        self.get_logger().info(
                            f"POI {travel_distance:.2f}m away — already in position, "
                            f"starting session in place"
                        )
                        self.current_target = target
                        self.transition_to_stability_wait()
                        return

                self.start_target_evaluation(target)

            elif self.current_state in [MissionState.EVALUATING_TARGET, MissionState.POSITIONING_OPTIMAL]:
                if self.current_target and target.priority_score > self.current_target.priority_score + self.switch_threshold:
                    self.get_logger().info(
                        f"Better target found ({target.priority_score:.2f} vs "
                        f"{self.current_target.priority_score:.2f})")
                    self._cancel_navigation()
                    self.start_target_evaluation(target)
                
        except Exception as e:
            self.get_logger().error(f"Error processing optimal position: {e}")

    def all_pois_callback(self, msg: String):
        """
        [FIX #NAV] Receive ALL POIs from POI manager for retry list.
        POI manager publishes top 10 POIs sorted by priority as JSON.
        """
        try:
            pois_data = json.loads(msg.data)
            if not pois_data:
                return
            
            new_all_pois = []
            for poi_dict in pois_data:
                pos = poi_dict.get('position', [0, 0])
                target = MissionTarget(
                    position=np.array(pos),
                    orientation=poi_dict.get('orientation', 0.0),
                    priority_score=poi_dict.get('priority_score', 0.0),
                    cluster_id=poi_dict.get('cluster_id', 0),
                    stability_score=poi_dict.get('stability_score', 0.0)
                )
                new_all_pois.append(target)
            
            self._all_pois = new_all_pois
            
        except json.JSONDecodeError:
            pass
        except Exception as e:
            self.get_logger().error(f"Error processing all POIs: {e}")            
            
    def exploration_goal_callback(self, msg: PoseStamped):
        """Fallback to exploration goals when no POIs available"""
        try:
            # [FIX #8] Do NOT accept exploration goals during recovery
            if self._recovery_lock:
                self.get_logger().debug("Recovery lock active — ignoring exploration goal")
                return

            # [TELEOP] Ignore exploration goals while manually paused
            if self._teleop_paused:
                return  
                          
            if self.current_state == MissionState.IDLE and not self.current_target:
                self.get_logger().info("No POI targets — using exploration goal")
                
                # from tf_transformations import euler_from_quaternion
                q = msg.pose.orientation
                _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
                
                exploration_target = MissionTarget(
                    position=np.array([msg.pose.position.x, msg.pose.position.y]),
                    orientation=yaw,
                    priority_score=0.3,
                    cluster_id=-1,
                    stability_score=1.0
                )
                self.start_target_evaluation(exploration_target)
                
        except Exception as e:
            self.get_logger().error(f"Error processing exploration goal: {e}")
            
            
    def clusters_callback(self, msg: ClusterArray):
        """Monitor cluster changes for dynamic replanning"""
        try:
            current_time = time.time()
            if (current_time - self.last_replan_time) > self.replanning_interval:
                self.evaluate_replanning_need()
                self.last_replan_time = current_time
        except Exception as e:
            self.get_logger().error(f"Error processing clusters: {e}")
            
            
    def robot_pose_callback(self, msg: Odometry):
        """Track robot position"""
        try:
            self.robot_position = np.array([
                msg.pose.pose.position.x,
                msg.pose.pose.position.y
            ])
            
            # from tf_transformations import euler_from_quaternion
            q = msg.pose.pose.orientation
            _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
            self.robot_orientation = yaw
            
            current_time = time.time()
            self.position_history.append((current_time, self.robot_position.copy()))
            
            cutoff_time = current_time - 10.0
            self.position_history = [h for h in self.position_history if h[0] > cutoff_time]
            
            if self.last_position is not None:
                distance = np.linalg.norm(self.robot_position - self.last_position)
                self.metrics.total_distance_traveled += distance
                
            self.last_position = self.robot_position.copy()
            self.last_position_time = current_time
            
        except Exception as e:
            self.get_logger().error(f"Error processing robot pose: {e}")          

    def photo_status_callback(self, msg: String):
        try:
            if msg.data == "completed":
                # If action is in flight, result_callback handles everything
                # Only act here for RC-triggered sessions (no action goal)
                if self._take_photo_goal_handle is not None:
                    self.get_logger().debug("photo_status completed — action in flight, ignoring")
                    return
                self._camera_active = False
                self.metrics.photos_completed += 1
                self._reset_retry_state()
                if self.current_state == MissionState.PHOTO_SEQUENCE:
                    self.get_logger().info("Photo session completed (RC path)")
                    self.transition_to_next_target()
                else:
                    self.get_logger().info("Photo completed (already moved on)")
        except Exception as e:
            self.get_logger().error(f"Error processing photo status: {e}")

    def recovery_status_callback(self, msg: String):
        """
        [FIX #8] Recovery coordination protocol.
        
        RecoveryManager publishes status. Mission controller uses it to:
        - LOCK: Prevent new targets when recovery is active
        - UNLOCK: Resume normal operation when recovery completes
        - WAIT: Never send new Nav2 goals during active recovery
        """
        try:
            status = msg.data
            
            if "Level:8" in status:
                # Emergency stop — lock everything
                self.get_logger().error("EMERGENCY STOP from RecoveryManager")
                self._recovery_lock = True
                self._cancel_navigation()
                self.transition_to_state(MissionState.RECOVERY)
                self.current_target = None
                
            elif "Level:7" in status:
                # Safe parking — lock and let recovery manager drive
                self.get_logger().warn("SAFE PARKING from RecoveryManager")
                self._recovery_lock = True
                if self.current_state != MissionState.RECOVERY:
                    self._cancel_navigation()
                    self.transition_to_state(MissionState.RECOVERY)
                    
            elif "Level:0" in status:
                # Recovery complete — UNLOCK
                if self._recovery_lock:
                    self.get_logger().info("Recovery complete — releasing lock, resuming normal operation")
                    self._recovery_cooldown_until = time.time() + 10.0  # 10s cooldown
                    self._recovery_lock = False
                    self._is_recovery_action_active = False
                    self._pending_recovery_command = None
                    # [FIX #NAV] Clear retry state on recovery complete
                    self._reset_retry_state()                    
                    self.transition_to_state(MissionState.IDLE)
                    self.current_target = None                    
                    
            elif any(f"Level:{i}" in status for i in range(1, 7)):
                # Active recovery levels 1-6 — lock but don't cancel current nav
                # (recovery manager might be trying different strategies)
                self._recovery_lock = True
                if self.current_state != MissionState.RECOVERY:
                    self.transition_to_state(MissionState.RECOVERY)
                
        except Exception as e:
            self.get_logger().error(f"Error processing recovery status: {e}")
            
            
    def recovery_command_callback(self, msg: String):
        """
        [FIX #8] Handle recovery commands with proper sequencing.
        
        Commands are NOT executed if another recovery action is in progress.
        This prevents the "new goal while mid-spin" problem.
        """
        try:
            command = msg.data
            self.get_logger().info(f"Recovery command received: {command}")
            
            if not self.enable_autonomous_navigation:
                self.get_logger().warn(f"Navigation disabled — ignoring: {command}")
                self._ack_recovery_command(command, "ignored_nav_disabled")
                return
            
            # [FIX #8] If a recovery action is already running, DON'T start a new one
            if self._is_recovery_action_active:
                self.get_logger().warn(
                    f"Recovery action already active — queueing: {command}")
                self._pending_recovery_command = command
                self._ack_recovery_command(command, "queued")
                return
            
            # Cancel any existing navigation FIRST, then wait for cancellation
            # to complete before starting recovery action
            if self.current_nav_goal_handle is not None:
                self.get_logger().info("Cancelling current navigation before recovery action...")
                self._cancel_navigation()
                # Small delay to let Nav2 process the cancellation
                # The recovery action will be executed in the next control loop tick
                self._pending_recovery_command = command
                self._ack_recovery_command(command, "cancelling_nav_first")
                return
            
            # Execute immediately
            self._execute_recovery_command(command)
            
        except Exception as e:
            self.get_logger().error(f"Error handling recovery command: {e}")
    
    def teleop_override_callback(self, msg: Bool):
        """
        [TELEOP] Manual override toggle.
        
        True  = PAUSE: Stop everything, allow manual teleop
        False = RESUME: Clear state, let system pick up fresh POIs
        
        Usage:
          ros2 topic pub --once /teleop/override std_msgs/Bool "data: true"
          ros2 topic pub --once /teleop/override std_msgs/Bool "data: false"
        """
        try:

            if msg.data and not self._teleop_paused:
                # PAUSE
                self.get_logger().warn("🎮 TELEOP OVERRIDE — pausing autonomous mission")
                self._teleop_paused = True
                self._cancel_navigation()
                # Stop camera if active
                if self._camera_active:
                    if self._take_photo_goal_handle is not None:
                        self._take_photo_goal_handle.cancel_goal_async()
                        self._take_photo_goal_handle = None
                        self.get_logger().info("📷 TakePhoto action cancelled by teleop")
                    else:
                        stop_msg = String()
                        stop_msg.data = "no"
                        self.photo_command_pub.publish(stop_msg)
                    self._camera_active = False

                self._reset_retry_state()
                self._failed_positions = []
                self.current_target = None
                self.available_targets = []
                self.transition_to_state(MissionState.IDLE)
                self.get_logger().warn("🎮 Robot stopped — manual control active")

            elif not msg.data and self._teleop_paused:
                # RESUME
                self.get_logger().warn("🎮 TELEOP RESUME — returning to autonomous operation")
                self._teleop_paused = False
                self._reset_retry_state()
                self._failed_positions = []
                self.current_target = None
                self.available_targets = []
                self.transition_to_state(MissionState.IDLE)
                self.get_logger().warn("🎮 Autonomous mode active — waiting for fresh POIs")
                
        except Exception as e:
            self.get_logger().error(f"Error in teleop override: {e}")

    def photo_command_callback(self, msg: String):
        """
        Track camera active state from both RC manual and autonomous paths.

        Routing rules:
          YES + action in flight  → ignore (action owns the session)
          YES + no action         → RC direct path — block navigation
          NO  + action in flight  → cancel action (PIS stops camera + returns result)
          NO  + no action         → RC direct path — resume navigation
        """
        try:
            if msg.data == "yes":
                if self._take_photo_goal_handle is not None:
                    # Action already running — ignore duplicate yes
                    self.get_logger().debug(
                        "📷 Camera YES ignored — TakePhoto action in flight")
                    return
                # RC direct path — no action, manual session
                self._camera_active = True
                self.get_logger().info("📷 Camera ACTIVE (RC manual) — holding position")
                if self.current_nav_goal_handle is not None:
                    self._cancel_navigation()

            elif msg.data == "no":
                if self._take_photo_goal_handle is not None:
                    # Action in flight — cancel it cleanly
                    # PIS will stop Canon R6 and return interrupted=True result
                    # _take_photo_result_callback will clear _camera_active
                    self.get_logger().info(
                        "📷 Camera STOP (RC) — cancelling TakePhoto action")
                    self._take_photo_goal_handle.cancel_goal_async()
                    self._take_photo_goal_handle = None
                    return
                # RC direct path — no action, manual session ended
                self._camera_active = False
                self.get_logger().info("📷 Camera STOPPED (RC manual)")

        except Exception as e:
            self.get_logger().error(f"Error in photo command: {e}")

    def _execute_recovery_command(self, command: str):
        """Execute a recovery command with proper state tracking"""
        try:
            self._is_recovery_action_active = True
            self.transition_to_state(MissionState.RECOVERY)
            
            if command == "SPIN_RECOVERY":
                self._execute_orientation_scan()
            elif command.startswith("RELOCATE:"):
                coords = command.split(":")[1].split(",")
                x, y = float(coords[0]), float(coords[1])
                self._execute_relocation(x, y)
            elif command.startswith("SAFE_PARKING:"):
                coords = command.split(":")[1].split(",")
                x, y = float(coords[0]), float(coords[1])
                self._execute_relocation(x, y)  # Same Nav2 goal mechanism
            elif command == "EMERGENCY_STOP":
                self._execute_emergency_stop()

            elif command == "start_exploration":
                self._is_recovery_action_active = False  # Exploration is passive
                self.get_logger().info("Exploration mode requested")
            elif command == "reset_sensors":
                self.get_logger().info("Sensor reset requested — clearing nav state")
                self._cancel_navigation()
                self.position_history = []
                self._is_recovery_action_active = False
            else:
                self.get_logger().warn(f"Unknown recovery command: {command}")
                self._is_recovery_action_active = False                
            self._ack_recovery_command(command, "executing")
            
        except Exception as e:
            self.get_logger().error(f"Error executing recovery command: {e}")
            self._is_recovery_action_active = False
            
            
    def _ack_recovery_command(self, command: str, status: str):
        # """[FIX #8] Acknowledge recovery command to RecoveryManager"""
        # try:
        #     msg = String()
        #     msg.data = f"ACK:{command}:{status}"
        #     self.recovery_ack_pub.publish(msg)
        # except Exception as e:
        #     self.get_logger().error(f"Error sending recovery ack: {e}")
        """Recovery ACK — recovery_manager removed, no-op kept for compatibility."""
        pass

    # ========================================================================
    # [FIX #NAV] POI RETRY LOGIC
    # ========================================================================
    
    def _mark_position_failed(self, position: np.ndarray):
        """Mark a position as failed so we don't retry it."""
        self._failed_positions.append((position.copy(), time.time()))
        self.get_logger().info(
            f"📍 Marked position ({position[0]:.2f}, {position[1]:.2f}) as failed "
            f"(total failed: {len(self._failed_positions)})")
    
    def _is_position_failed(self, position: np.ndarray) -> bool:
        """Check if a position is too close to any recently failed position."""
        current_time = time.time()
        # Clean up expired failed positions
        self._failed_positions = [
            (pos, t) for pos, t in self._failed_positions
            if current_time - t < self.failed_position_memory_time
        ]
        # Check against all active failed positions
        for failed_pos, _ in self._failed_positions:
            distance = np.linalg.norm(position - failed_pos)
            if distance < self.failed_position_radius:
                return True
        return False
    
    def _get_next_viable_poi(self) -> Optional[MissionTarget]:
        """Get the next best POI that hasn't been tried yet."""
        for poi in self._all_pois:
            if not self._is_position_failed(poi.position):
                return poi
        # Also check available_targets
        for target in self.available_targets:
            if not self._is_position_failed(target.position):
                return target
        return None
    
    def _try_next_poi(self) -> bool:
        """
        Attempt to navigate to the next viable POI.
        Returns True if a new goal was sent, False if no POIs available.
        """
        self._current_retry_count += 1
        
        if self._current_retry_count > self.max_poi_retries:
            self.get_logger().warn(
                f"Max POI retries reached ({self.max_poi_retries})")
            return False
        
        next_poi = self._get_next_viable_poi()
        
        if next_poi is None:
            self.get_logger().warn("No viable POIs remaining")
            return False
        
        self.get_logger().info(
            f"🔄 Retry {self._current_retry_count}/{self.max_poi_retries}: "
            f"Trying POI at ({next_poi.position[0]:.2f}, {next_poi.position[1]:.2f}) "
            f"score={next_poi.priority_score:.2f}")
        
        self._retry_active = True
        self.current_target = next_poi
        self.current_target.evaluation_start_time = time.time()
        
        # Send Nav2 goal directly (skip evaluation for retries)
        self.transition_to_state(MissionState.POSITIONING_OPTIMAL)
        self._send_nav_goal(next_poi)
        return True
    
    def _handle_nav_failure(self):
        """
        Central handler for navigation failures.
        Mark current position as failed → try next POI → or RECOVERY.
        """
        # Mark current position as unreachable
        if self.current_target is not None:
            self._mark_position_failed(self.current_target.position)
        
        # Try the next POI
        if self._try_next_poi():
            return
        
        # No more POIs — NOW enter RECOVERY
        self.get_logger().warn(
            "⚠️ All POI alternatives exhausted — entering RECOVERY state")
        self._reset_retry_state()
        self.transition_to_state(MissionState.RECOVERY)
    
    def _reset_retry_state(self):
        """Reset the POI retry state for a fresh start."""
        self._retry_active = False
        self._current_retry_count = 0
        self._nav_goal_start_distance = None

    def _is_making_progress(self) -> bool:
        """
        Check if robot has been making meaningful progress toward the goal.
        
        Two checks:
        1. Has robot moved ≥ 0.5m closer in the last 15 seconds?
        2. Is robot significantly closer than when the goal was first sent?
        
        Both must be true. This prevents in-place rotation from
        falsely triggering "progress" via tiny odometry drift.
        """
        if not self.current_target or not self.position_history or self.robot_position is None:
            return False
        
        current_distance = np.linalg.norm(
            self.robot_position - self.current_target.position)
        
        # Check 1: Has robot moved closer in the last progress_check_window seconds?
        current_time = time.time()
        window = self.progress_check_window
        old_positions = [
            pos for t, pos in self.position_history
            if current_time - t >= window - 2.0
            and current_time - t <= window + 2.0
        ]
        
        if not old_positions:
            # No history far enough back
            # Instead of assuming progress, check against start distance
            if self._nav_goal_start_distance is not None:
                progress_from_start = self._nav_goal_start_distance - current_distance
                self.get_logger().debug(
                    f"Progress check (no window data): start={self._nav_goal_start_distance:.1f}m "
                    f"now={current_distance:.1f}m moved={progress_from_start:.2f}m")
                return progress_from_start > 1.0  # Must have moved at least 1m closer
            return False  # No start distance either — not progressing
        
        old_position = old_positions[0]
        old_distance = np.linalg.norm(old_position - self.current_target.position)
        recent_progress = old_distance - current_distance
        
        # Check 2: Is robot meaningfully closer than when goal was sent?
        overall_progress = 0.0
        if self._nav_goal_start_distance is not None:
            overall_progress = self._nav_goal_start_distance - current_distance
        
        self.get_logger().debug(
            f"Progress check: was {old_distance:.1f}m → now {current_distance:.1f}m "
            f"(recent={recent_progress:.2f}m, overall={overall_progress:.2f}m, "
            f"start={self._nav_goal_start_distance or 0:.1f}m)")
        
        # Both checks must pass:
        # - Recent: moved ≥ 0.5m closer in last 15s
        # - Overall: moved ≥ 1.0m closer than start
        has_recent_progress = recent_progress > self.progress_min_distance
        has_overall_progress = overall_progress > 1.0
        
        return has_recent_progress and has_overall_progress

    def _bt_tick(self):
        """Tick the Mission BT — replaces mission_control_loop() if/elif cascade."""
        try:
            # Guards handled inside EvaluateTarget node
            self._mission_tree.tick()
            self.publish_mission_status()
        except Exception as e:
            self.get_logger().error(f"BT tick error: {e}")    
    # ========================================================================
    # STATE MACHINE
    # ========================================================================
    
    def mission_control_loop(self):
        """Main mission control state machine"""
        try:
            current_time = time.time()
            elapsed = current_time - self.state_start_time
            
            # [FIX #8] Check for pending recovery commands (deferred execution)
            if (self._pending_recovery_command is not None and 
                not self._is_recovery_action_active and
                self.current_nav_goal_handle is None):
                cmd = self._pending_recovery_command
                self._pending_recovery_command = None
                self._execute_recovery_command(cmd)
                return
            
            if self.current_state == MissionState.IDLE:
                self.handle_idle_state()
            elif self.current_state == MissionState.EVALUATING_TARGET:
                self.handle_evaluating_state(elapsed)
            elif self.current_state == MissionState.POSITIONING_OPTIMAL:
                self.handle_positioning_state(elapsed)
            elif self.current_state == MissionState.WAITING_FOR_STABILITY:
                self.handle_stability_state(elapsed)
            elif self.current_state == MissionState.PHOTO_SEQUENCE:
                self.handle_photo_sequence_state(elapsed)
            elif self.current_state == MissionState.MOVING_TO_NEXT:
                self.handle_moving_to_next_state(elapsed)
            elif self.current_state == MissionState.RECOVERY:
                self.handle_recovery_state(elapsed)
                
            self.publish_mission_status()
            
        except Exception as e:
            self.get_logger().error(f"Error in mission control loop: {e}")
            
            
    def handle_idle_state(self):
        """Handle idle state"""
        # [FIX #8] Don't start evaluating if recovery lock is held
        if self._recovery_lock:
            return

        # [TELEOP] Don't start evaluating if manually paused
        if self._teleop_paused:
            return

        # [PHOTO] Don't start evaluating while camera is active
        if self._camera_active:
            return 

        # [FIX #COOLDOWN] Don't accept targets during recovery cooldown
        if time.time() < self._recovery_cooldown_until:
            return  
                                 
        if self.available_targets and not self.current_target:
            best_target = max(self.available_targets, key=lambda t: t.priority_score)
            self.start_target_evaluation(best_target)
            
            
    def handle_evaluating_state(self, elapsed: float):
        """Handle target evaluation state"""
        if not self.current_target:
            self.transition_to_state(MissionState.IDLE)
            return
            
        if elapsed >= self.evaluation_time:
            self.get_logger().info(
                f"Target evaluation complete — committing to "
                f"({self.current_target.position[0]:.2f}, {self.current_target.position[1]:.2f})")
            self.transition_to_positioning()
            
            
    def handle_positioning_state(self, elapsed: float):
        """Handle positioning/navigation state"""
        if not self.current_target or self.robot_position is None:
            self.transition_to_state(MissionState.RECOVERY)
            return
            
        distance = np.linalg.norm(self.robot_position - self.current_target.position)
        
        if distance <= self.arrival_threshold:
            self.get_logger().info(f"Arrival detected: {distance:.2f}m ≤ {self.arrival_threshold}m")
            if self.current_nav_goal_handle is not None:
                self._cancel_navigation()

            # [FIX #NAV] Reset retry state on arrival
            self._reset_retry_state()

            if self.current_target.cluster_id >= 0:
                self.transition_to_stability_wait()
            else:
                self.transition_to_state(MissionState.IDLE)
                self.current_target = None

        else:
            # [FIX #NAV] Smart timeout with progress checking
            if self.nav_start_time:
                nav_elapsed = time.time() - self.nav_start_time
                
                # Hard cap — cancel regardless of progress
                if nav_elapsed > self.absolute_nav_timeout:
                    self.get_logger().warn(
                        f"⏰ Absolute timeout ({self.absolute_nav_timeout:.0f}s) — "
                        f"trying next POI")
                    self._cancel_navigation()
                    self._handle_nav_failure()
                
                # Soft timeout — check if making progress
                elif nav_elapsed > self.per_goal_timeout:
                    if self._is_making_progress():
                        self.get_logger().info(
                            f"⏰ Timeout ({self.per_goal_timeout:.0f}s) but robot is "
                            f"making progress ({distance:.1f}m remaining) — extending")
                        self.nav_start_time = time.time()  # Reset timer
                    else:
                        self.get_logger().warn(
                            f"⏰ Timeout ({self.per_goal_timeout:.0f}s) and robot is "
                            f"NOT making progress ({distance:.1f}m remaining) — trying next POI")
                        self._cancel_navigation()
                        self._handle_nav_failure()                
                
    def handle_stability_state(self, elapsed: float):
        """Handle waiting for stability state"""
        # [FIX #7] Fixed double-negative logic bug
        if self.robot_position is None:
            self.transition_to_state(MissionState.RECOVERY)
            return
            
        is_stable = self.check_robot_stability()
        
        if is_stable and elapsed >= self.stability_wait_time:
            self.get_logger().info("Robot stable — starting photo sequence")
            self.transition_to_photo_sequence()
        elif not is_stable:
            self.state_start_time = time.time()
        elif elapsed > 30.0:
            self.get_logger().warn("Stability timeout — starting photo sequence anyway")
            self.transition_to_photo_sequence()

    def handle_photo_sequence_state(self, elapsed: float):
        """Handle active photo sequence"""
        if elapsed > 180.0:
            self.get_logger().warn("Photo sequence timeout — cancelling")
            if self._take_photo_goal_handle is not None:
                self._take_photo_goal_handle.cancel_goal_async()
                self._take_photo_goal_handle = None
            self._camera_active = False
            self.transition_to_next_target()            
            
    def handle_moving_to_next_state(self, elapsed: float):
        """Handle transition to next target"""
        if self.available_targets:
            next_target = max(self.available_targets, key=lambda t: t.priority_score)
            current_id = self.current_target.cluster_id if self.current_target else None
            if next_target.cluster_id != current_id:
                self.start_target_evaluation(next_target)
                return

        self.transition_to_state(MissionState.IDLE)
        self.current_target = None
        
        
    def handle_recovery_state(self, elapsed: float):
        """Handle recovery state — wait for RecoveryManager coordination"""
        # [FIX #8] Recovery is fully coordinated via recovery_status_callback
        # This handler only manages timeouts
        if elapsed > 300.0:
            self.get_logger().warn("Recovery timeout (5min) — forcing idle")
            self._recovery_lock = False
            self._is_recovery_action_active = False
            self.transition_to_state(MissionState.IDLE)
            self.current_target = None
            
            
    # ========================================================================
    # STATE TRANSITIONS
    # ========================================================================
            
    def start_target_evaluation(self, target: MissionTarget):
        """Start evaluating a target"""
        self.current_target = target
        self.current_target.evaluation_start_time = time.time()
        self.metrics.targets_evaluated += 1
        self.get_logger().info(
            f"Evaluating target ({target.position[0]:.2f}, {target.position[1]:.2f}) "
            f"score: {target.priority_score:.2f}")
        self.transition_to_state(MissionState.EVALUATING_TARGET)
        
        
    def transition_to_positioning(self):
        """Transition to positioning state"""
        self.transition_to_state(MissionState.POSITIONING_OPTIMAL)
        if self.current_target:
            self._send_nav_goal(self.current_target)
            
            
    def transition_to_stability_wait(self):
        """Transition to stability waiting state"""
        self.transition_to_state(MissionState.WAITING_FOR_STABILITY)
        self.get_logger().info("Waiting for stability (Nav2 goal reached, robot decelerating)")
        
        
    def transition_to_photo_sequence(self):
        """Transition to photo sequence — send TakePhoto action goal."""
        self.transition_to_state(MissionState.PHOTO_SEQUENCE)
        self._camera_active = True

        if not self.take_photo_client.server_is_ready():
            self.get_logger().warn("TakePhoto action server not ready — falling back to trigger")
            trigger_msg = Bool()
            trigger_msg.data = True
            self.photo_trigger_pub.publish(trigger_msg)
            return
        group_size = 0
        if self.current_intelligence if hasattr(self, 'current_intelligence') else False:
            pass
        
        goal = TakePhoto.Goal()
        goal.group_size = 0
        goal.requested_duration = 0.0  # 0 = auto from PIS intelligence

        self.get_logger().info("Sending TakePhoto action goal to PIS...")
        future = self.take_photo_client.send_goal_async(
            goal,
            feedback_callback=self._take_photo_feedback_callback)
        future.add_done_callback(self._take_photo_goal_response_callback)

    def _take_photo_goal_response_callback(self, future):
        """Handle TakePhoto goal acceptance."""
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().warn("TakePhoto goal REJECTED by PIS")
                self._camera_active = False
                self.transition_to_next_target()
                return
            self.get_logger().info("TakePhoto goal ACCEPTED")
            self._take_photo_goal_handle = goal_handle
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(self._take_photo_result_callback)
        except Exception as e:
            self.get_logger().error(f"Error in TakePhoto goal response: {e}")
            self._camera_active = False
            self.transition_to_next_target()

    def _take_photo_feedback_callback(self, feedback_msg):
        """Log TakePhoto feedback."""
        fb = feedback_msg.feedback
        self.get_logger().debug(
            f"TakePhoto: {fb.elapsed_time:.1f}s elapsed, "
            f"group={fb.current_group_size}, formation={fb.formation_type}")

    def _take_photo_result_callback(self, future):
        """Handle TakePhoto result — resume navigation."""
        try:
            result = future.result().result
            self._take_photo_goal_handle = None
            self._last_successful_position = None
            self._recently_photographed = {}  # pos_key → timestamp, 120s cooldown
            self._camera_active = False
            self.metrics.photos_completed += 1
            self._reset_retry_state()
            # Store last successful position for recovery L3
            if result.photos_taken > 0 and self.robot_position is not None:
                self._last_successful_position = self.robot_position.copy()
            # Store POI position (not robot position) for cooldown matching
            if result.photos_taken > 0 and self.current_target is not None:
                pos_key = (round(self.current_target.position[0], 1),
                           round(self.current_target.position[1], 1))
                self._recently_photographed[pos_key] = time.time()
                self.get_logger().info(
                    f"POI {pos_key} cooldown started — 120s")
            self.get_logger().info(
                f"TakePhoto result: photos={result.photos_taken}, "
                f"duration={result.actual_duration:.1f}s, "
                f"interrupted={result.interrupted}, reason={result.stop_reason}")
            if self.current_state == MissionState.PHOTO_SEQUENCE:
                self.transition_to_next_target()
        except Exception as e:
            self.get_logger().error(f"Error in TakePhoto result: {e}")
            self._camera_active = False
            self.transition_to_next_target()
        
        
    def transition_to_next_target(self):
        """Transition to moving to next target"""
        self.transition_to_state(MissionState.MOVING_TO_NEXT)
        self.metrics.transitions_made += 1
        
        
    def transition_to_state(self, new_state: MissionState):
        """Transition to new state"""
        old_state = self.current_state
        self.current_state = new_state
        self.state_start_time = time.time()
        self.get_logger().info(f"State: {old_state.value} → {new_state.value}")
        
        
    # ========================================================================
    # NAV2 INTEGRATION
    # ========================================================================
    
    def _send_nav_goal(self, target: MissionTarget):
        """Send navigation goal via Nav2 action client"""
        try:
            if not self.enable_autonomous_navigation:
                self.get_logger().debug("🛑 Navigation disabled — skipping goal")
                return
            
            # [FIX #RATE] Rate limit — minimum 2 seconds between Nav2 goals
            now = time.time()
            if now - self._last_nav_goal_time < 2.0:
                self.get_logger().debug("⏱️ Nav2 goal rate limited — skipping")
                return        
            # [FIX #8] Do not send goals during recovery lock
            if self._recovery_lock:
                self.get_logger().warn("Recovery lock active — cannot send Nav2 goal")
                return

            # [PHOTO] Do not send goals while camera is active
            if self._camera_active:
                self.get_logger().info("📷 Camera active — holding Nav2 goal")
                return
                            
            if not self.nav_client.wait_for_server(timeout_sec=1.0):
                self.get_logger().error("Nav2 not available!")
                self._handle_nav_failure()
                return
                
            goal_msg = NavigateToPose.Goal()
            goal_msg.pose.header.frame_id = 'map'
            goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
            goal_msg.pose.pose.position.x = float(target.position[0])
            goal_msg.pose.pose.position.y = float(target.position[1])
            goal_msg.pose.pose.position.z = 0.0
            
            # from tf_transformations import quaternion_from_euler
            quat = quaternion_from_euler(0, 0, target.orientation)
            goal_msg.pose.pose.orientation.x = quat[0]
            goal_msg.pose.pose.orientation.y = quat[1]
            goal_msg.pose.pose.orientation.z = quat[2]
            goal_msg.pose.pose.orientation.w = quat[3]
            
            self.nav_start_time = time.time()
            self._last_nav_goal_time = time.time()

            # [FIX #TIMEOUT] Store starting distance for progress validation
            if self.robot_position is not None:
                self._nav_goal_start_distance = float(np.linalg.norm(
                    self.robot_position - np.array([float(target.position[0]), float(target.position[1])])))
                        
            self.get_logger().info(
                f"📤 Nav2 goal: ({target.position[0]:.2f}, {target.position[1]:.2f}) "
                f"yaw: {target.orientation:.2f}rad")
            
            send_goal_future = self.nav_client.send_goal_async(
                goal_msg, feedback_callback=self._nav_feedback_callback)
            send_goal_future.add_done_callback(self._nav_goal_response_callback)
            
            # Publish simple goal for visualization
            self._publish_simple_goal(target)

        except Exception as e:
            self.get_logger().error(f"Error sending Nav2 goal: {e}")
            # [FIX #NAV] Try next POI on error
            self._handle_nav_failure()            
            
    def _publish_simple_goal(self, target: MissionTarget):
        """Publish simple goal for RViz visualization"""
        try:
            goal_msg = PoseStamped()
            goal_msg.header.frame_id = 'map'
            goal_msg.header.stamp = self.get_clock().now().to_msg()
            goal_msg.pose.position.x = float(target.position[0])
            goal_msg.pose.position.y = float(target.position[1])
            
            # from tf_transformations import quaternion_from_euler
            quat = quaternion_from_euler(0, 0, target.orientation)
            goal_msg.pose.orientation.x = quat[0]
            goal_msg.pose.orientation.y = quat[1]
            goal_msg.pose.orientation.z = quat[2]
            goal_msg.pose.orientation.w = quat[3]
            
            self.nav_goal_pub.publish(goal_msg)
        except Exception as e:
            self.get_logger().error(f"Error publishing simple goal: {e}")
            
            
    def _nav_goal_response_callback(self, future):
        """Handle Nav2 goal acceptance"""
        try:
            goal_handle = future.result()

            if not goal_handle.accepted:
                self.get_logger().warn("❌ Nav2 goal REJECTED")
                # [FIX #NAV] Try next POI instead of RECOVERY
                self._handle_nav_failure()
                return
                
            self.get_logger().info("✅ Nav2 goal ACCEPTED")
            self.current_nav_goal_handle = goal_handle
            
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(self._nav_result_callback)

        except Exception as e:
            self.get_logger().error(f"Error in goal response: {e}")
            # [FIX #NAV] Try next POI on error
            self._handle_nav_failure()            
            
    def _nav_feedback_callback(self, feedback_msg):
        """Handle Nav2 feedback (currently unused)"""
        pass
            
            
    def _nav_result_callback(self, future):
        """Handle Nav2 navigation completion"""
        try:
            result = future.result()
            status = result.status
            self.current_nav_goal_handle = None
            
            if status == GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().info("✅ Navigation SUCCEEDED")
                
                # [FIX #8] If this was a recovery navigation, handle it
                if self._is_recovery_action_active:
                    # Check if this is part of an orientation scan sequence
                    if hasattr(self, '_scan_angles') and self._scan_angles:
                        if self._scan_index < len(self._scan_angles):
                            # More scan angles to go — send next rotation
                            self.get_logger().info(
                                f"Scan rotation complete, pausing 2s before next angle...")
                            # Brief pause to let cameras detect humans at this orientation
                            # Then send next scan goal
                            self._scan_pause_timer = self.create_timer(
                                2.0, self._scan_pause_complete)
                            return
                        else:
                            # Full scan complete
                            self.get_logger().info("Full orientation scan complete")
                            self._scan_angles = []
                            self._is_recovery_action_active = False
                            return
                    else:
                        # Non-scan recovery nav (relocation, safe parking)
                        self.get_logger().info("Recovery navigation completed successfully")
                        self._is_recovery_action_active = False
                        return

                # [FIX #NAV] Reset retry state on success
                self._reset_retry_state()

                if self.current_state == MissionState.POSITIONING_OPTIMAL:
                    if self.current_target and self.current_target.cluster_id >= 0:
                        self.transition_to_stability_wait()
                    else:
                        self.transition_to_state(MissionState.IDLE)
                        self.current_target = None
                        
            elif status == GoalStatus.STATUS_CANCELED:
                self.get_logger().info("Navigation CANCELED (intentional)")
                self._is_recovery_action_active = False

            elif status == GoalStatus.STATUS_ABORTED:
                self.get_logger().warn("❌ Navigation ABORTED by Nav2")
                self._is_recovery_action_active = False
                if not self._recovery_lock:
                    # [FIX #NAV] Try next POI instead of going straight to RECOVERY
                    self.get_logger().info("🔄 Nav2 aborted — attempting next POI...")
                    self._handle_nav_failure()                    

            else:
                self.get_logger().warn(f"Navigation status: {status}")
                self._is_recovery_action_active = False
                if not self._recovery_lock:
                    # [FIX #NAV] Try next POI for unknown failures too
                    self._handle_nav_failure()

        except Exception as e:
            self.get_logger().error(f"Error in nav result: {e}")
            self._is_recovery_action_active = False
            if not self._recovery_lock:
                # [FIX #NAV] Try next POI on error
                self._handle_nav_failure()            
            
    def _cancel_navigation(self):
        """Cancel current Nav2 navigation and clean up scan state"""
        try:
            if self.current_nav_goal_handle is not None:
                self.get_logger().info("🛑 Canceling Nav2 navigation...")
                self.current_nav_goal_handle.cancel_goal_async()
                self.current_nav_goal_handle = None
                self.nav_start_time = None
            
            # Clean up any active scan sequence
            self._scan_angles = []
            self._scan_index = 0
            if hasattr(self, '_scan_pause_timer') and self._scan_pause_timer is not None:
                self._scan_pause_timer.cancel()
                self._scan_pause_timer = None
        except Exception as e:
            self.get_logger().error(f"Error canceling navigation: {e}")
            
            
    # ========================================================================
    # RECOVERY EXECUTION
    # [FIX #1] All recovery uses Nav2 action client, NEVER direct /cmd_vel
    # [FIX #8] All recovery actions track _is_recovery_action_active
    # ========================================================================
    
    def _execute_orientation_scan(self):
        """
        Execute orientation scanning — sequential in-place rotations.
        
        PURPOSE: "We haven't seen humans for 45+ seconds. They might be
        behind us or to our side. Rotate through 6 angles to look around."
        
        STRATEGY:
            Rotate through 6 angles (60° apart = full 360° coverage)
            At each angle: Nav2 goal at CURRENT position with new yaw
            → Nav2's rotate_to_heading does the in-place rotation
            → 3 ZED cameras scan for humans at each orientation
            → human_clustering_node processes detections
            → If humans found: RecoveryManager sees data, returns to normal
            → If no humans after full circle: RecoveryManager escalates
        
        WHY SEQUENTIAL NAV2 GOALS:
            Each goal uses Nav2's velocity chain properly:
            Controller → VelocitySmoother → CollisionMonitor → motors
            No direct /cmd_vel. Robot rotates smoothly and safely.
            
        The 6 scan angles are queued. Each completes via _nav_result_callback,
        which triggers the next one. If RecoveryManager publishes Level:0
        (humans found) at any point, _cancel_navigation stops the sequence.
        """
        try:
            self.get_logger().info("Starting orientation scan (6 angles, 60° apart)")
            
            if self.robot_position is None or self.robot_orientation is None:
                self.get_logger().warn("No robot pose for orientation scan")
                self._is_recovery_action_active = False
                return
            
            # Build scan sequence: 6 angles, 60° apart from current heading
            # Order: +60°, +120°, +180°, +240°, +300°, +360° (back toself.enable_autonomous_navigation = self.d start)
            self._scan_angles = []
            for i in range(1, 7):
                angle = self.robot_orientation + (i * np.pi / 3.0)  # 60° steps
                # Normalize to [-π, π]
                angle = np.arctan2(np.sin(angle), np.cos(angle))
                self._scan_angles.append(angle)
            
            self._scan_index = 0
            self._scan_position = self.robot_position.copy()  # Stay at current position
            
            # Start first rotation
            self._send_next_scan_goal()
            
        except Exception as e:
            self.get_logger().error(f"Error starting orientation scan: {e}")
            self._is_recovery_action_active = False
    
    
    def _send_next_scan_goal(self):
        """Send the next in-place rotation goal in the scan sequence"""
        try:
            if self._scan_index >= len(self._scan_angles):
                # Full circle complete — no humans found at any angle
                self.get_logger().info(
                    "Orientation scan complete — full 360°, no humans detected")
                self._is_recovery_action_active = False
                return
            
            target_yaw = self._scan_angles[self._scan_index]
            self.get_logger().info(
                f"Scan step {self._scan_index + 1}/6: "
                f"rotating to {np.degrees(target_yaw):.0f}°")
            
            scan_target = MissionTarget(
                position=self._scan_position.copy(),  # Same position (in-place)
                orientation=target_yaw,
                priority_score=0.0,
                cluster_id=-99,  # Recovery marker
                stability_score=1.0
            )
            self._send_recovery_nav_goal(scan_target)
            self._scan_index += 1
            
        except Exception as e:
            self.get_logger().error(f"Error sending scan goal: {e}")
            self._is_recovery_action_active = False
    
    
    def _scan_pause_complete(self):
        """
        Called after 2-second pause at each scan angle.
        
        The 2s pause gives the cameras time to:
        1. Capture frames at the new orientation
        2. Run YOLO detection
        3. Publish to /human_clustering/humans
        4. If humans found → RecoveryManager sees data → returns to normal
           → _cancel_navigation cancels remaining scan goals
        5. If no humans → send next scan rotation
        """
        # Destroy the one-shot timer
        if hasattr(self, '_scan_pause_timer') and self._scan_pause_timer is not None:
            self._scan_pause_timer.cancel()
            self._scan_pause_timer = None
        
        # If recovery was cancelled during the pause (humans found), stop
        if not self._is_recovery_action_active:
            self.get_logger().info("Scan interrupted — recovery resolved during pause")
            return
        
        # Send next scan angle
        self._send_next_scan_goal()
    
    
    def _execute_relocation(self, x: float, y: float):
        """Execute relocation/safe parking via Nav2"""
        try:
            self.get_logger().info(f"Relocating to ({x:.2f}, {y:.2f}) via Nav2...")
            relocation_target = MissionTarget(
                position=np.array([x, y]),
                orientation=0.0,
                priority_score=0.0,
                cluster_id=-99,
                stability_score=1.0
            )
            self._send_recovery_nav_goal(relocation_target)
            
        except Exception as e:
            self.get_logger().error(f"Error in relocation: {e}")
            self._is_recovery_action_active = False
    
    
    def _execute_emergency_stop(self):
        """Execute emergency stop — cancel Nav2, do NOT write /cmd_vel"""
        try:
            self.get_logger().error("EMERGENCY STOP — canceling all navigation")
            self._cancel_navigation()
            # [FIX #1] We do NOT publish zero velocity to /cmd_vel.
            # Nav2's cancellation will stop the controller from publishing.
            # The velocity smoother's velocity_timeout (1.0s) will zero out
            # any remaining velocity automatically.
            # The collision monitor is the safety net for immediate stops.
            self.current_target = None
            self._is_recovery_action_active = False
            
        except Exception as e:
            self.get_logger().error(f"Error in emergency stop: {e}")
            self._is_recovery_action_active = False
    
    
    def _send_recovery_nav_goal(self, target: MissionTarget):
        """Send a Nav2 goal for recovery purposes (tracked separately)"""
        try:
            if not self.nav_client.wait_for_server(timeout_sec=1.0):
                self.get_logger().error("Nav2 not available for recovery goal")
                self._is_recovery_action_active = False
                return

            # if not self.nav_client.wait_for_server(timeout_sec=1.0):
            #     self.get_logger().error("Nav2 not available!")
            #     # [FIX #NAV] Try next POI instead of RECOVERY
            #     self._handle_nav_failure()
            #     return
                            
            goal_msg = NavigateToPose.Goal()
            goal_msg.pose.header.frame_id = 'map'
            goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
            goal_msg.pose.pose.position.x = float(target.position[0])
            goal_msg.pose.pose.position.y = float(target.position[1])
            goal_msg.pose.pose.position.z = 0.0
            
            # from tf_transformations import quaternion_from_euler
            quat = quaternion_from_euler(0, 0, target.orientation)
            goal_msg.pose.pose.orientation.x = quat[0]
            goal_msg.pose.pose.orientation.y = quat[1]
            goal_msg.pose.pose.orientation.z = quat[2]
            goal_msg.pose.pose.orientation.w = quat[3]
            
            self.nav_start_time = time.time()
            
            send_goal_future = self.nav_client.send_goal_async(
                goal_msg, feedback_callback=self._nav_feedback_callback)
            send_goal_future.add_done_callback(self._nav_goal_response_callback)
            
        except Exception as e:
            self.get_logger().error(f"Error sending recovery nav goal: {e}")
            self._is_recovery_action_active = False


    # ========================================================================
    # UTILITIES
    # ========================================================================
    
    def check_robot_stability(self) -> bool:
        """Check if robot is stable"""
        try:
            if len(self.position_history) < 3:
                return False
                
            current_time = time.time()
            recent_positions = [
                pos for t, pos in self.position_history
                if current_time - t <= 2.0
            ]
            
            if len(recent_positions) < 2:
                return False
                
            positions_array = np.array(recent_positions)
            movement_variance = np.var(np.linalg.norm(
                positions_array[1:] - positions_array[:-1], axis=1))
            
            return movement_variance < self.stability_threshold
            
        except Exception as e:
            self.get_logger().error(f"Error checking stability: {e}")
            return False
            
            
    def evaluate_replanning_need(self):
        """Evaluate if replanning is needed"""
        pass
            
            
    def publish_mission_status(self):
        """Publish current mission status"""
        try:
            status_info = f"State:{self.current_state.value}"
            if self.current_target:
                status_info += f" Target:({self.current_target.position[0]:.1f},{self.current_target.position[1]:.1f})"
                status_info += f" Score:{self.current_target.priority_score:.2f}"
            if self._recovery_lock:
                status_info += " RecoveryLock:true"
            # [FIX #NAV] Include retry info in status
            if self._retry_active:
                status_info += f" Retry:{self._current_retry_count}/{self.max_poi_retries}"
            if self._failed_positions:
                status_info += f" FailedPOIs:{len(self._failed_positions)}"
            if self._camera_active:
                status_info += " Camera:active"
            if self._teleop_paused:
                status_info += " Teleop:paused"

            msg = String()
            msg.data = status_info
            self.mission_status_pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f"Error publishing status: {e}")
            
            
    def publish_metrics(self):
        """Publish mission metrics"""
        try:
            if self.metrics.targets_evaluated > 0:
                self.metrics.mission_efficiency = \
                    self.metrics.photos_completed / self.metrics.targets_evaluated
                    
            metrics_info = (
                f"Evaluated:{self.metrics.targets_evaluated} "
                f"Photos:{self.metrics.photos_completed} "
                f"Transitions:{self.metrics.transitions_made} "
                f"Distance:{self.metrics.total_distance_traveled:.1f}m "
                f"Efficiency:{self.metrics.mission_efficiency:.2f} "
                f"NavRetries:{self._current_retry_count} "
                f"FailedPOIs:{len(self._failed_positions)}")            
                          
            msg = String()
            msg.data = metrics_info
            self.mission_metrics_pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f"Error publishing metrics: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = MissionController()
    
    from rclpy.executors import MultiThreadedExecutor
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    # Signal handler to stop camera and nav before ROS2 shutdown
    import signal
    def shutdown_handler(sig, frame):
        try:
            if node.current_nav_goal_handle is not None:
                node.current_nav_goal_handle.cancel_goal_async()
                node.get_logger().info("🛑 Nav2 goal cancelled on signal")
            stop_msg = String()
            stop_msg.data = "no"
            node.photo_command_pub.publish(stop_msg)
            node.get_logger().info("📷 Camera stop sent on signal")
            rclpy.spin_once(node, timeout_sec=0.5)
        except Exception:
            pass
        raise KeyboardInterrupt
    signal.signal(signal.SIGINT, shutdown_handler)
        
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()