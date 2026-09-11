#!/usr/bin/env python3

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
import py_trees
import py_trees.blackboard
try:
    import py_trees_ros
    import py_trees_ros.trees
    _PY_TREES_ROS_AVAILABLE = True
except ImportError:
    _PY_TREES_ROS_AVAILABLE = False

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
            self._recently_photographed = {}
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

            # Service
            self.set_autonomous_srv = self.create_service(
                SetBool, '/mission_controller/set_autonomous',
                self._handle_set_autonomous, callback_group=self.cb_group)

            # ── Blackboard setup ──────────────────────────────────────
            # Phase 1: introduce blackboard as shared state store.
            # Controller callbacks write here. BT nodes read from here.
            # Controller instance variables stay in place during migration.
            py_trees.blackboard.Blackboard.enable_activity_stream(100)
            self._bb = py_trees.blackboard.Client(name="MissionController")
            self._bb.register_key("robot_position",           access=py_trees.common.Access.WRITE)
            self._bb.register_key("robot_orientation",        access=py_trees.common.Access.WRITE)
            self._bb.register_key("available_targets",        access=py_trees.common.Access.WRITE)
            self._bb.register_key("current_target",           access=py_trees.common.Access.WRITE)
            self._bb.register_key("camera_active",            access=py_trees.common.Access.WRITE)
            self._bb.register_key("teleop_paused",            access=py_trees.common.Access.WRITE)
            self._bb.register_key("recovery_lock",            access=py_trees.common.Access.WRITE)
            self._bb.register_key("recovery_cooldown_until",  access=py_trees.common.Access.WRITE)
            self._bb.register_key("recently_photographed",    access=py_trees.common.Access.WRITE)
            self._bb.register_key("failed_positions",         access=py_trees.common.Access.WRITE)
            self._bb.register_key("last_successful_position", access=py_trees.common.Access.WRITE)

            # Initialise all keys with defaults
            self._bb.robot_position           = None
            self._bb.robot_orientation        = None
            self._bb.available_targets        = []
            self._bb.current_target           = None
            self._bb.camera_active            = False
            self._bb.teleop_paused            = False
            self._bb.recovery_lock            = False
            self._bb.recovery_cooldown_until  = 0.0
            self._bb.recently_photographed    = {}
            self._bb.failed_positions         = []
            self._bb.last_successful_position = None
            self.get_logger().info("Blackboard initialised ✅")
            # ─────────────────────────────────────────────────────────

            # # Build Mission BT — replaces inner state machine
            # self._mission_tree = build_mission_tree(self)
            # self.get_logger().info("Mission BT built ✅")
            
            # Build Mission BT — replaces inner state machine
            _raw_tree = build_mission_tree(self)
            if _PY_TREES_ROS_AVAILABLE:
                self._mission_tree = py_trees_ros.trees.BehaviourTree(
                    root=_raw_tree.root,
                    unicode_tree_debug=True
                )
                self.get_logger().info("Mission BT built ✅ (py_trees_ros — tree-watcher enabled)")
            else:
                self._mission_tree = _raw_tree
                self.get_logger().info("Mission BT built ✅ (plain py_trees — no tree-watcher)")            
            self.get_logger().info("MissionController configured (NO direct /cmd_vel — Nav2 owns velocity)")
            return TransitionCallbackReturn.SUCCESS

        except Exception as e:
            self.get_logger().error(f"Configuration failed: {e}")
            return TransitionCallbackReturn.FAILURE

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        """Start mission control loop."""
        self.get_logger().info("Activating MissionController...")
        try:
            if _PY_TREES_ROS_AVAILABLE:
                try:
                    self._mission_tree.setup(node=self, timeout=15.0)
                except Exception as e:
                    self.get_logger().warn(
                        f"py_trees_ros setup warning (non-fatal): {e} — continuing")            
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
            # ── Blackboard write ──
            self._bb.available_targets = self.available_targets.copy()
            
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
            # ── Blackboard write — full POI list for retry loop ──
            self._bb.failed_positions = self._failed_positions.copy()
            
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
        """Track robot position — writes to both controller vars and blackboard."""
        try:
            self.robot_position = np.array([
                msg.pose.pose.position.x,
                msg.pose.pose.position.y
            ])

            q = msg.pose.pose.orientation
            _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
            self.robot_orientation = yaw

            # ── Blackboard write ──
            self._bb.robot_position    = self.robot_position.copy()
            self._bb.robot_orientation = self.robot_orientation

            current_time = time.time()
            self.position_history.append((current_time, self.robot_position.copy()))

            cutoff_time = current_time - 10.0
            self.position_history = [
                h for h in self.position_history if h[0] > cutoff_time]

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
                # Single source of truth for session end
                # Covers both action path and RC path
                self._camera_active = False
                self._bb.camera_active = False
                self._take_photo_goal_handle = None
                self.metrics.photos_completed += 1
                self._reset_retry_state()
                self.get_logger().info("PIS session completed — camera_active cleared")
                if self.current_state == MissionState.PHOTO_SEQUENCE:
                    self.transition_to_next_target()
        except Exception as e:
            self.get_logger().error(f"Error processing photo status: {e}")

    def recovery_status_callback(self, msg: String):
        try:
            status = msg.data
            
            if "Level:8" in status:
                # Emergency stop — lock everything
                self.get_logger().error("EMERGENCY STOP from RecoveryManager")
                self._recovery_lock = True
                self._bb.recovery_lock = True
                self._cancel_navigation()
                self.transition_to_state(MissionState.RECOVERY)
                self.current_target = None
                
            elif "Level:7" in status:
                # Safe parking — lock and let recovery manager drive
                self.get_logger().warn("SAFE PARKING from RecoveryManager")
                self._recovery_lock = True
                self._bb.recovery_lock = True
                if self.current_state != MissionState.RECOVERY:
                    self._cancel_navigation()
                    self.transition_to_state(MissionState.RECOVERY)
                    
            elif "Level:0" in status:
                # Recovery complete — UNLOCK
                if self._recovery_lock:
                    self.get_logger().info("Recovery complete — releasing lock, resuming normal operation")
                    self._recovery_cooldown_until = time.time() + 10.0  # 10s cooldown
                    self._recovery_lock = False
                    self._bb.recovery_lock           = False
                    self._bb.recovery_cooldown_until = self._recovery_cooldown_until
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
                self._bb.recovery_lock = True
                if self.current_state != MissionState.RECOVERY:
                    self.transition_to_state(MissionState.RECOVERY)
                
        except Exception as e:
            self.get_logger().error(f"Error processing recovery status: {e}")
            
            
    def recovery_command_callback(self, msg: String):
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
        try:

            if msg.data and not self._teleop_paused:
                # PAUSE
                self.get_logger().warn("🎮 TELEOP OVERRIDE — pausing autonomous mission")
                self._teleop_paused = True
                self._bb.teleop_paused = True
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
                self._bb.teleop_paused = False
                self._reset_retry_state()
                self._failed_positions = []
                self.current_target = None
                self.available_targets = []
                self.transition_to_state(MissionState.IDLE)
                self.get_logger().warn("🎮 Autonomous mode active — waiting for fresh POIs")
                
        except Exception as e:
            self.get_logger().error(f"Error in teleop override: {e}")

    def photo_command_callback(self, msg: String):
        try:
            if msg.data == "yes":
                if self._take_photo_goal_handle is not None:
                    # Action already running — ignore duplicate yes
                    self.get_logger().debug(
                        "📷 Camera YES ignored — TakePhoto action in flight")
                    return
                # RC direct path — no action, manual session
                self._camera_active = True
                self._bb.camera_active = True
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
                self._bb.camera_active = False
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
        self._bb.failed_positions = self._failed_positions.copy()
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
        Select the next viable POI and update current_target.
        Returns True if a new POI was found, False if none available.

        NOTE: This method ONLY selects the target — it does NOT send a
        Nav2 goal or transition state. NavigateMultiPOI in mission_bt.py
        owns goal sending and manages its own attempt counter (_attempt).
        """
        next_poi = self._get_next_viable_poi()

        if next_poi is None:
            self.get_logger().warn("No viable POIs remaining")
            return False

        self.current_target = next_poi
        self.current_target.evaluation_start_time = time.time()
        self._retry_active = True  # Blocks optimal_positions_callback interruption

        self.get_logger().info(
            f"🔄 Next POI selected: "
            f"({next_poi.position[0]:.2f}, {next_poi.position[1]:.2f}) "
            f"score={next_poi.priority_score:.2f}")

        return True
    
    def _handle_nav_failure(self):
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
        self._bb.camera_active = True

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
                self.get_logger().warn("TakePhoto goal REJECTED by PIS — waiting for active session")
                self._camera_active = False
                self._bb.camera_active = False
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
            # self._last_successful_position = None
            # self._recently_photographed = {}
            # camera_active is NOT cleared here.
            # PIS owns the session — photo_status_callback clears it
            # when PIS publishes "completed". This prevents BT cycling
            # while PIS is still running its photo session.
            # self._bb.recently_photographed    = {}
            # self._bb.last_successful_position = None
            # self.metrics.photos_completed += 1
            # self._reset_retry_state()
            # Store last successful position for recovery L3
            if result.photos_taken > 0 and self.robot_position is not None:
                self._last_successful_position = self.robot_position.copy()
                self._bb.last_successful_position = self.robot_position.copy()
            # Store POI position cooldown regardless of photos_taken
            # Prevents rapid re-selection of same target
            if self.current_target is not None:
                pos_key = (round(self.current_target.position[0], 1),
                           round(self.current_target.position[1], 1))
                self._recently_photographed[pos_key] = time.time()
                self._bb.recently_photographed = self._recently_photographed.copy()
                self.get_logger().info(
                    f"POI {pos_key} cooldown started — 120s "
                    f"(photos_taken={result.photos_taken})")
            self.get_logger().info(
                f"TakePhoto result: photos={result.photos_taken}, "
                f"duration={result.actual_duration:.1f}s, "
                f"interrupted={result.interrupted}, reason={result.stop_reason}")
            # Do NOT call transition_to_next_target() here.
            # photo_status_callback owns session completion.
        except Exception as e:
            self.get_logger().error(f"Error in TakePhoto result: {e}")
            self._camera_active = False
            self._bb.camera_active = False
            self.transition_to_next_target()
        
        
    # def transition_to_next_target(self):
    #     """Transition to moving to next target"""
    #     self.transition_to_state(MissionState.MOVING_TO_NEXT)
    #     self.metrics.transitions_made += 1

    def transition_to_next_target(self):
        """Transition to next target — clear old target and return to IDLE"""
        self.metrics.transitions_made += 1
        self.current_target = None
        self._bb.current_target = None
        self._reset_retry_state()
        self.transition_to_state(MissionState.IDLE)
        self.get_logger().info("Ready for next target")        
        
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