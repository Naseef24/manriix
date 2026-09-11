#!/usr/bin/env python3

"""
MissionTelemetryRecorder - Real-time Mission Event Logger
==========================================================

Records ALL mission events to timestamped log files for debugging,
analysis, and post-mission review.

WHAT IT RECORDS:
  - Every Nav2 goal published (position, orientation, POI score)
  - Every goal reached (arrival time, travel duration)
  - Every goal cancelled/aborted (reason, retry count)
  - POI retry attempts (which POI, why previous failed)
  - Smart timeout events (progress checks, extensions, cancellations)
  - Photo sequence events (trigger, duration, completion)
  - Stability wait events (duration, timeout?)
  - Recovery events (level, command, duration)
  - Failed positions list (position, radius, expiry)
  - Robot position samples (periodic, for path reconstruction)
  - State transitions (from → to, timestamp)

OUTPUT FILES:
  ~/manriix_logs/mission_YYYYMMDD_HHMMSS.csv    — Machine-readable CSV
  ~/manriix_logs/mission_YYYYMMDD_HHMMSS.txt    — Human-readable timeline
  ~/manriix_logs/mission_latest.txt              — Symlink to latest (tail -f friendly)

USAGE:
  ros2 run manriix_perception mission_telemetry_recorder
  
  # Watch live:
  tail -f ~/manriix_logs/mission_latest.txt
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.callback_groups import ReentrantCallbackGroup

from std_msgs.msg import String, Bool
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from action_msgs.msg import GoalStatusArray

import os
import csv
import time
import json
from datetime import datetime
from pathlib import Path
from dataclasses import dataclass, field
from typing import Optional, List, Tuple
import numpy as np


@dataclass
class GoalRecord:
    """Track a single Nav2 goal from publish to completion"""
    goal_id: int
    timestamp: float
    position: Tuple[float, float]
    orientation: float
    priority_score: float
    is_retry: bool
    retry_number: int
    # Filled in later
    result: str = "pending"          # succeeded, aborted, cancelled, timeout
    result_timestamp: float = 0.0
    travel_duration: float = 0.0
    distance_at_start: float = 0.0
    distance_at_end: float = 0.0
    timeout_type: str = ""           # none, soft_progress, soft_stuck, hard_cap
    progress_extensions: int = 0


@dataclass
class PhotoRecord:
    """Track a photo sequence"""
    photo_id: int
    trigger_timestamp: float
    stability_wait_start: float = 0.0
    stability_wait_duration: float = 0.0
    stability_timeout: bool = False
    photo_start_timestamp: float = 0.0
    photo_end_timestamp: float = 0.0
    photo_duration: float = 0.0
    completed: bool = False


@dataclass 
class RecoveryRecord:
    """Track a recovery event"""
    recovery_id: int
    start_timestamp: float
    trigger_reason: str = ""         # nav_exhausted, sensor_timeout, cpu_overload
    level: int = 0
    command: str = ""
    end_timestamp: float = 0.0
    duration: float = 0.0
    result: str = "active"           # resolved, timeout, escalated


class MissionTelemetryRecorder(Node):
    """
    Subscribes to all mission-related topics and records events
    to real-time log files.
    """
    
    def __init__(self):
        super().__init__('mission_telemetry_recorder')
        
        self.cb_group = ReentrantCallbackGroup()
        
        # Parameters
        self.declare_parameter('log_directory', '~/manriix_logs')
        self.declare_parameter('position_sample_interval', 2.0)  # seconds
        self.declare_parameter('flush_interval', 1.0)  # seconds
        
        log_dir = self.get_parameter('log_directory').value
        self.log_dir = Path(os.path.expanduser(log_dir))
        self.position_sample_interval = self.get_parameter('position_sample_interval').value
        self.flush_interval = self.get_parameter('flush_interval').value
        
        # Create log directory
        self.log_dir.mkdir(parents=True, exist_ok=True)
        
        # Generate timestamped filenames
        timestamp_str = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.csv_path = self.log_dir / f"mission_{timestamp_str}.csv"
        self.txt_path = self.log_dir / f"mission_{timestamp_str}.txt"
        self.latest_link = self.log_dir / "mission_latest.txt"
        
        # Create symlink for tail -f
        if self.latest_link.is_symlink() or self.latest_link.exists():
            self.latest_link.unlink()
        self.latest_link.symlink_to(self.txt_path)
        
        # Open files
        self.txt_file = open(self.txt_path, 'w', buffering=1)  # Line buffered
        self.csv_file = open(self.csv_path, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        
        # CSV header
        self.csv_writer.writerow([
            'timestamp', 'elapsed_s', 'event_type', 'event_detail',
            'robot_x', 'robot_y', 'robot_yaw',
            'target_x', 'target_y', 'distance_to_target',
            'state', 'retry_count', 'failed_pois',
            'recovery_lock', 'recovery_level',
            'extra_data'
        ])
        
        # Session state
        self.mission_start_time = time.time()
        self.robot_position: Optional[np.ndarray] = None
        self.robot_yaw: float = 0.0
        self.last_position_sample_time = 0.0
        
        # Current mission state (parsed from /mission/status)
        self.current_state = "idle"
        self.current_target_pos = (0.0, 0.0)
        self.current_score = 0.0
        self.current_retry_count = 0
        self.current_failed_pois = 0
        self.recovery_lock = False
        
        # Tracking records
        self.goal_counter = 0
        self.photo_counter = 0
        self.recovery_counter = 0
        self.current_goal: Optional[GoalRecord] = None
        self.current_photo: Optional[PhotoRecord] = None
        self.current_recovery: Optional[RecoveryRecord] = None
        self.all_goals: List[GoalRecord] = []
        self.all_photos: List[PhotoRecord] = []
        self.all_recoveries: List[RecoveryRecord] = []
        
        # Previous state for transition detection
        self._prev_state = "idle"
        self._prev_recovery_level = 0
        
        # QoS
        qos_reliable = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, depth=10)
        qos_best_effort = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=5)
        
        # ================================================================
        # SUBSCRIBERS — listen to everything
        # ================================================================
        
        # Mission controller status (State, Target, Score, Retry, FailedPOIs)
        self.create_subscription(
            String, '/mission/status',
            self._mission_status_cb, qos_reliable, callback_group=self.cb_group)
        
        # Mission controller metrics
        self.create_subscription(
            String, '/mission/metrics',
            self._mission_metrics_cb, qos_best_effort, callback_group=self.cb_group)
        
        # Nav2 goal (simple goal for visualization — published by mission controller)
        self.create_subscription(
            PoseStamped, '/move_base_simple/goal',
            self._nav_goal_cb, qos_reliable, callback_group=self.cb_group)
        
        # Robot pose
        self.create_subscription(
            Odometry, '/odometry/filtered',
            self._robot_pose_cb, qos_best_effort, callback_group=self.cb_group)
        
        # Photo trigger
        self.create_subscription(
            Bool, '/photo/intelligence/trigger',
            self._photo_trigger_cb, qos_reliable, callback_group=self.cb_group)
        
        # Photo status
        self.create_subscription(
            String, '/photo/intelligence/status',
            self._photo_status_cb, qos_reliable, callback_group=self.cb_group)
        
        # Recovery status
        self.create_subscription(
            String, '/recovery/status',
            self._recovery_status_cb, qos_reliable, callback_group=self.cb_group)
        
        # Recovery command
        self.create_subscription(
            String, '/recovery/command',
            self._recovery_command_cb, qos_reliable, callback_group=self.cb_group)
        
        # Recovery ACK from mission controller
        self.create_subscription(
            String, '/mission/recovery_ack',
            self._recovery_ack_cb, qos_reliable, callback_group=self.cb_group)
        
        # POI manager all POIs (to track how many are available)
        self.create_subscription(
            String, '/poi_manager/all_pois',
            self._all_pois_cb, qos_best_effort, callback_group=self.cb_group)
        
        # Flush timer
        self.create_timer(self.flush_interval, self._flush_files)
        
        # Summary timer (every 30s write a status summary)
        self.create_timer(30.0, self._write_periodic_summary)
        
        # Write header
        self._write_header()
        
        self.get_logger().info(
            f"📝 Mission Telemetry Recorder started\n"
            f"   TXT: {self.txt_path}\n"
            f"   CSV: {self.csv_path}\n"
            f"   Live: tail -f {self.latest_link}")
    
    
    # ====================================================================
    # LOGGING HELPERS
    # ====================================================================
    
    def _elapsed(self) -> float:
        """Seconds since mission start"""
        return time.time() - self.mission_start_time
    
    def _elapsed_str(self) -> str:
        """Human-readable elapsed time"""
        elapsed = self._elapsed()
        minutes = int(elapsed // 60)
        seconds = elapsed % 60
        return f"{minutes:3d}m {seconds:05.2f}s"
    
    def _timestamp_str(self) -> str:
        """Current time as HH:MM:SS.mmm"""
        return datetime.now().strftime('%H:%M:%S.') + f"{datetime.now().microsecond // 1000:03d}"
    
    def _write_txt(self, event_type: str, message: str):
        """Write a line to the human-readable log"""
        line = f"[{self._timestamp_str()}] [{self._elapsed_str()}] [{event_type:18s}] {message}\n"
        self.txt_file.write(line)
    
    def _write_csv(self, event_type: str, event_detail: str, extra_data: str = ""):
        """Write a row to the CSV log"""
        robot_x = f"{self.robot_position[0]:.3f}" if self.robot_position is not None else ""
        robot_y = f"{self.robot_position[1]:.3f}" if self.robot_position is not None else ""
        
        dist = ""
        if self.robot_position is not None and self.current_target_pos != (0.0, 0.0):
            d = np.linalg.norm(
                self.robot_position - np.array(self.current_target_pos))
            dist = f"{d:.3f}"
        
        self.csv_writer.writerow([
            f"{time.time():.3f}",
            f"{self._elapsed():.2f}",
            event_type,
            event_detail,
            robot_x, robot_y,
            f"{self.robot_yaw:.3f}",
            f"{self.current_target_pos[0]:.3f}",
            f"{self.current_target_pos[1]:.3f}",
            dist,
            self.current_state,
            self.current_retry_count,
            self.current_failed_pois,
            self.recovery_lock,
            self._prev_recovery_level,
            extra_data
        ])
    
    def _log_event(self, event_type: str, message: str, extra_data: str = ""):
        """Write to both TXT and CSV"""
        self._write_txt(event_type, message)
        self._write_csv(event_type, message, extra_data)
    
    def _flush_files(self):
        """Flush file buffers to disk"""
        try:
            self.txt_file.flush()
            self.csv_file.flush()
        except Exception:
            pass
    
    def _write_header(self):
        """Write session header to TXT log"""
        now = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        header = (
            f"{'='*80}\n"
            f"  MANRIIX MISSION TELEMETRY LOG\n"
            f"  Started: {now}\n"
            f"  Node: mission_telemetry_recorder\n"
            f"{'='*80}\n"
            f"\n"
            f"  LEGEND:\n"
            f"    NAV_GOAL_SENT     — Nav2 goal published to /navigate_to_pose\n"
            f"    NAV_GOAL_REACHED  — Robot arrived at goal (STATUS_SUCCEEDED)\n"
            f"    NAV_GOAL_ABORTED  — Nav2 could not reach goal (STATUS_ABORTED)\n"
            f"    NAV_GOAL_CANCEL   — Goal cancelled (intentional or timeout)\n"
            f"    NAV_TIMEOUT_SOFT  — Per-goal timeout, progress check triggered\n"
            f"    NAV_TIMEOUT_HARD  — Absolute timeout (180s), forced cancel\n"
            f"    NAV_PROGRESS_OK   — Progress check passed, timer extended\n"
            f"    NAV_PROGRESS_FAIL — Progress check failed, trying next POI\n"
            f"    POI_RETRY         — Trying alternative POI after failure\n"
            f"    POI_EXHAUSTED     — All POIs tried, entering RECOVERY\n"
            f"    POI_FAILED_MARK   — Position marked as unreachable\n"
            f"    PHOTO_TRIGGER     — Photo sequence triggered\n"
            f"    PHOTO_COMPLETE    — Photo sequence finished\n"
            f"    PHOTO_TIMEOUT     — Photo sequence timed out (180s)\n"
            f"    STABILITY_START   — Waiting for robot to stabilize\n"
            f"    STABILITY_OK      — Robot stable, ready for photos\n"
            f"    STABILITY_TIMEOUT — Stability timeout (30s), proceeding anyway\n"
            f"    RECOVERY_ENTER    — Entered RECOVERY state\n"
            f"    RECOVERY_LEVEL    — Recovery level changed\n"
            f"    RECOVERY_CMD      — Recovery command received\n"
            f"    RECOVERY_ACK      — Recovery command acknowledged\n"
            f"    RECOVERY_EXIT     — Exited RECOVERY (Level:0)\n"
            f"    STATE_CHANGE      — Mission state transition\n"
            f"    POSITION_SAMPLE   — Periodic robot position record\n"
            f"    SUMMARY           — Periodic status summary\n"
            f"\n"
            f"{'='*80}\n\n"
        )
        self.txt_file.write(header)
    
    
    # ====================================================================
    # TOPIC CALLBACKS
    # ====================================================================
    
    def _mission_status_cb(self, msg: String):
        """Parse mission status and detect state transitions"""
        try:
            status = msg.data
            
            # Parse fields
            new_state = self.current_state
            if "State:" in status:
                new_state = status.split("State:")[1].split()[0]
            
            if "Target:" in status:
                target_str = status.split("Target:")[1].split()[0]
                target_str = target_str.strip("()")
                parts = target_str.split(",")
                self.current_target_pos = (float(parts[0]), float(parts[1]))
            
            if "Score:" in status:
                self.current_score = float(status.split("Score:")[1].split()[0])
            
            if "Retry:" in status:
                retry_str = status.split("Retry:")[1].split()[0]
                self.current_retry_count = int(retry_str.split("/")[0])
            else:
                self.current_retry_count = 0
            
            if "FailedPOIs:" in status:
                self.current_failed_pois = int(status.split("FailedPOIs:")[1].split()[0])
            else:
                self.current_failed_pois = 0
            
            self.recovery_lock = "RecoveryLock:true" in status
            
            # Detect state transitions
            if new_state != self._prev_state:
                self._handle_state_transition(self._prev_state, new_state)
                self._prev_state = new_state
                self.current_state = new_state
            
        except Exception as e:
            self.get_logger().debug(f"Error parsing mission status: {e}")
    
    
    def _handle_state_transition(self, old_state: str, new_state: str):
        """Handle and log state transitions with context"""
        
        self._log_event("STATE_CHANGE", f"{old_state} → {new_state}")
        
        # Detect specific transitions
        if new_state == "waiting_for_stability" and old_state == "positioning_optimal":
            # Goal reached, entering stability wait
            if self.current_goal and self.current_goal.result == "pending":
                self.current_goal.result = "succeeded"
                self.current_goal.result_timestamp = time.time()
                self.current_goal.travel_duration = (
                    self.current_goal.result_timestamp - self.current_goal.timestamp)
                if self.robot_position is not None:
                    self.current_goal.distance_at_end = np.linalg.norm(
                        self.robot_position - np.array(self.current_target_pos))
                
                self._log_event("NAV_GOAL_REACHED",
                    f"Goal #{self.current_goal.goal_id} reached in "
                    f"{self.current_goal.travel_duration:.1f}s "
                    f"(final distance: {self.current_goal.distance_at_end:.2f}m)")
                self.all_goals.append(self.current_goal)
            
            # Start stability tracking
            if self.current_photo is None:
                self.photo_counter += 1
                self.current_photo = PhotoRecord(
                    photo_id=self.photo_counter,
                    trigger_timestamp=0.0,
                    stability_wait_start=time.time()
                )
            self._log_event("STABILITY_START", "Waiting for robot to stabilize")
        
        elif new_state == "photo_sequence" and old_state == "waiting_for_stability":
            # Stability achieved
            if self.current_photo:
                self.current_photo.stability_wait_duration = (
                    time.time() - self.current_photo.stability_wait_start)
                self._log_event("STABILITY_OK",
                    f"Stable after {self.current_photo.stability_wait_duration:.1f}s")
        
        elif new_state == "recovery":
            if self.current_recovery is None or self.current_recovery.result != "active":
                self.recovery_counter += 1
                trigger = "nav_exhausted" if self.current_failed_pois > 0 else "unknown"
                self.current_recovery = RecoveryRecord(
                    recovery_id=self.recovery_counter,
                    start_timestamp=time.time(),
                    trigger_reason=trigger
                )
                self._log_event("RECOVERY_ENTER",
                    f"Recovery #{self.recovery_counter} — trigger: {trigger} "
                    f"(failed POIs: {self.current_failed_pois})")
        
        elif old_state == "recovery" and new_state == "idle":
            if self.current_recovery and self.current_recovery.result == "active":
                self.current_recovery.end_timestamp = time.time()
                self.current_recovery.duration = (
                    self.current_recovery.end_timestamp - self.current_recovery.start_timestamp)
                self.current_recovery.result = "resolved"
                self._log_event("RECOVERY_EXIT",
                    f"Recovery #{self.current_recovery.recovery_id} resolved "
                    f"after {self.current_recovery.duration:.1f}s")
                self.all_recoveries.append(self.current_recovery)
                self.current_recovery = None
    
    
    def _mission_metrics_cb(self, msg: String):
        """Record mission metrics updates"""
        # Metrics are logged periodically via summary, no need to log every update
        pass
    
    
    def _nav_goal_cb(self, msg: PoseStamped):
        """Record every Nav2 goal published"""
        try:
            x = msg.pose.position.x
            y = msg.pose.position.y
            
            from tf_transformations import euler_from_quaternion
            q = msg.pose.orientation
            _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
            
            self.goal_counter += 1
            is_retry = self.current_retry_count > 0
            
            # Calculate distance from robot to goal
            dist = 0.0
            if self.robot_position is not None:
                dist = np.linalg.norm(self.robot_position - np.array([x, y]))
            
            self.current_goal = GoalRecord(
                goal_id=self.goal_counter,
                timestamp=time.time(),
                position=(x, y),
                orientation=yaw,
                priority_score=self.current_score,
                is_retry=is_retry,
                retry_number=self.current_retry_count,
                distance_at_start=dist
            )
            
            retry_str = f" [RETRY #{self.current_retry_count}]" if is_retry else ""
            self._log_event("NAV_GOAL_SENT",
                f"Goal #{self.goal_counter} → ({x:.2f}, {y:.2f}) "
                f"yaw={np.degrees(yaw):.0f}° dist={dist:.1f}m "
                f"score={self.current_score:.2f}{retry_str}")
            
        except Exception as e:
            self.get_logger().debug(f"Error recording nav goal: {e}")
    
    
    def _robot_pose_cb(self, msg: Odometry):
        """Track robot position, sample periodically"""
        try:
            self.robot_position = np.array([
                msg.pose.pose.position.x,
                msg.pose.pose.position.y
            ])
            
            from tf_transformations import euler_from_quaternion
            q = msg.pose.pose.orientation
            _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
            self.robot_yaw = yaw
            
            # Periodic position sampling
            current_time = time.time()
            if current_time - self.last_position_sample_time >= self.position_sample_interval:
                self.last_position_sample_time = current_time
                
                dist_str = ""
                if self.current_target_pos != (0.0, 0.0):
                    dist = np.linalg.norm(
                        self.robot_position - np.array(self.current_target_pos))
                    dist_str = f" dist_to_target={dist:.2f}m"
                
                self._write_csv("POSITION_SAMPLE",
                    f"({self.robot_position[0]:.3f}, {self.robot_position[1]:.3f}){dist_str}")
                
        except Exception:
            pass
    
    
    def _photo_trigger_cb(self, msg: Bool):
        """Record photo sequence trigger"""
        if msg.data:
            if self.current_photo:
                self.current_photo.photo_start_timestamp = time.time()
            self._log_event("PHOTO_TRIGGER",
                f"Photo sequence triggered at target "
                f"({self.current_target_pos[0]:.2f}, {self.current_target_pos[1]:.2f})")
    
    
    def _photo_status_cb(self, msg: String):
        """Record photo sequence completion"""
        try:
            if msg.data == "completed":
                duration = 0.0
                stability_dur = 0.0
                if self.current_photo:
                    self.current_photo.photo_end_timestamp = time.time()
                    self.current_photo.completed = True
                    if self.current_photo.photo_start_timestamp > 0:
                        self.current_photo.photo_duration = (
                            self.current_photo.photo_end_timestamp - 
                            self.current_photo.photo_start_timestamp)
                        duration = self.current_photo.photo_duration
                    stability_dur = self.current_photo.stability_wait_duration
                    self.all_photos.append(self.current_photo)
                
                self._log_event("PHOTO_COMPLETE",
                    f"Photo #{self.photo_counter} complete — "
                    f"stability_wait={stability_dur:.1f}s "
                    f"photo_duration={duration:.1f}s "
                    f"total={stability_dur + duration:.1f}s")
                self.current_photo = None
                
        except Exception as e:
            self.get_logger().debug(f"Error recording photo status: {e}")
    
    
    def _recovery_status_cb(self, msg: String):
        """Record recovery level changes"""
        try:
            status = msg.data
            level = 0
            for i in range(9):
                if f"Level:{i}" in status:
                    level = i
                    break
            
            if level != self._prev_recovery_level:
                direction = "↑" if level > self._prev_recovery_level else "↓"
                self._log_event("RECOVERY_LEVEL",
                    f"Level {self._prev_recovery_level} {direction} Level {level} — {status}")
                
                if self.current_recovery and self.current_recovery.result == "active":
                    self.current_recovery.level = level
                
                self._prev_recovery_level = level
                
        except Exception as e:
            self.get_logger().debug(f"Error recording recovery status: {e}")
    
    
    def _recovery_command_cb(self, msg: String):
        """Record recovery commands"""
        self._log_event("RECOVERY_CMD", f"Command: {msg.data}")
        if self.current_recovery:
            self.current_recovery.command = msg.data
    
    
    def _recovery_ack_cb(self, msg: String):
        """Record recovery command acknowledgments"""
        self._log_event("RECOVERY_ACK", f"ACK: {msg.data}")
    
    
    def _all_pois_cb(self, msg: String):
        """Track available POI count"""
        try:
            pois = json.loads(msg.data)
            # Only log when count changes significantly or periodically
            # (logged in summary instead to avoid spam)
            self._available_poi_count = len(pois) if pois else 0
        except Exception:
            self._available_poi_count = 0
    
    
    # ====================================================================
    # PERIODIC SUMMARY
    # ====================================================================
    
    def _write_periodic_summary(self):
        """Write a status summary every 30 seconds"""
        try:
            elapsed = self._elapsed()
            
            total_goals = len(self.all_goals)
            succeeded = sum(1 for g in self.all_goals if g.result == "succeeded")
            aborted = sum(1 for g in self.all_goals if g.result in ["aborted", "timeout"])
            cancelled = sum(1 for g in self.all_goals if g.result == "cancelled")
            
            avg_travel = 0.0
            if succeeded > 0:
                avg_travel = sum(
                    g.travel_duration for g in self.all_goals 
                    if g.result == "succeeded"
                ) / succeeded
            
            total_photos = len(self.all_photos)
            avg_photo_dur = 0.0
            avg_stability = 0.0
            if total_photos > 0:
                avg_photo_dur = sum(p.photo_duration for p in self.all_photos) / total_photos
                avg_stability = sum(p.stability_wait_duration for p in self.all_photos) / total_photos
            
            total_recoveries = len(self.all_recoveries)
            avg_recovery_dur = 0.0
            if total_recoveries > 0:
                avg_recovery_dur = sum(r.duration for r in self.all_recoveries) / total_recoveries
            
            pending_goal = ""
            if self.current_goal and self.current_goal.result == "pending":
                goal_elapsed = time.time() - self.current_goal.timestamp
                pending_goal = f"\n    Current goal: #{self.current_goal.goal_id} running for {goal_elapsed:.0f}s"
            
            active_recovery = ""
            if self.current_recovery and self.current_recovery.result == "active":
                rec_elapsed = time.time() - self.current_recovery.start_timestamp
                active_recovery = (
                    f"\n    Active recovery: #{self.current_recovery.recovery_id} "
                    f"L{self.current_recovery.level} for {rec_elapsed:.0f}s")
            
            poi_count = getattr(self, '_available_poi_count', 0)
            
            summary = (
                f"\n{'─'*60}\n"
                f"  SUMMARY at {self._elapsed_str()} (elapsed: {elapsed:.0f}s)\n"
                f"{'─'*60}\n"
                f"  State: {self.current_state} | Recovery lock: {self.recovery_lock}\n"
                f"  Available POIs: {poi_count} | Failed positions: {self.current_failed_pois}\n"
                f"\n"
                f"  NAVIGATION:\n"
                f"    Goals sent:      {self.goal_counter} (total)\n"
                f"    Succeeded:       {succeeded}\n"
                f"    Aborted/timeout: {aborted}\n"
                f"    Cancelled:       {cancelled}\n"
                f"    Pending:         {self.goal_counter - total_goals}\n"
                f"    Avg travel time: {avg_travel:.1f}s (succeeded only)\n"
                f"    Retry attempts:  {self.current_retry_count} (current cycle){pending_goal}\n"
                f"\n"
                f"  PHOTOGRAPHY:\n"
                f"    Photos taken:       {total_photos}\n"
                f"    Avg stability wait: {avg_stability:.1f}s\n"
                f"    Avg photo duration: {avg_photo_dur:.1f}s\n"
                f"\n"
                f"  RECOVERY:\n"
                f"    Total events:       {total_recoveries + (1 if self.current_recovery else 0)}\n"
                f"    Avg duration:       {avg_recovery_dur:.1f}s\n"
                f"    Current level:      {self._prev_recovery_level}{active_recovery}\n"
                f"{'─'*60}\n\n"
            )
            
            self.txt_file.write(summary)
            self._write_csv("SUMMARY", 
                f"goals={self.goal_counter} ok={succeeded} fail={aborted} "
                f"photos={total_photos} recoveries={total_recoveries}")
            
        except Exception as e:
            self.get_logger().error(f"Error writing summary: {e}")
    
    
    # ====================================================================
    # CLEANUP
    # ====================================================================
    
    def destroy_node(self):
        """Write final summary and close files"""
        try:
            self._write_txt("SESSION_END", "Mission telemetry recording stopped")
            self._write_periodic_summary()
            
            # Write goal history
            self.txt_file.write(f"\n{'='*60}\n  COMPLETE GOAL HISTORY\n{'='*60}\n\n")
            for g in self.all_goals:
                retry_str = f" [RETRY #{g.retry_number}]" if g.is_retry else ""
                self.txt_file.write(
                    f"  Goal #{g.goal_id}: ({g.position[0]:.2f}, {g.position[1]:.2f}) "
                    f"→ {g.result} in {g.travel_duration:.1f}s "
                    f"(score={g.priority_score:.2f}){retry_str}\n")
            
            # Write photo history
            self.txt_file.write(f"\n{'='*60}\n  COMPLETE PHOTO HISTORY\n{'='*60}\n\n")
            for p in self.all_photos:
                self.txt_file.write(
                    f"  Photo #{p.photo_id}: stability={p.stability_wait_duration:.1f}s "
                    f"photo={p.photo_duration:.1f}s "
                    f"total={p.stability_wait_duration + p.photo_duration:.1f}s "
                    f"{'✅' if p.completed else '❌ timeout'}\n")
            
            # Write recovery history
            self.txt_file.write(f"\n{'='*60}\n  COMPLETE RECOVERY HISTORY\n{'='*60}\n\n")
            for r in self.all_recoveries:
                self.txt_file.write(
                    f"  Recovery #{r.recovery_id}: L{r.level} trigger={r.trigger_reason} "
                    f"cmd={r.command} duration={r.duration:.1f}s → {r.result}\n")
            
            self.txt_file.write(f"\n{'='*60}\n  END OF LOG\n{'='*60}\n")
            
            self.txt_file.close()
            self.csv_file.close()
            
            self.get_logger().info(
                f"📝 Telemetry saved: {self.txt_path} ({self.goal_counter} goals, "
                f"{self.photo_counter} photos, {self.recovery_counter} recoveries)")
            
        except Exception as e:
            self.get_logger().error(f"Error closing telemetry files: {e}")
        
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MissionTelemetryRecorder()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()