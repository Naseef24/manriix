#!/usr/bin/env python3

"""
MissionTelemetryRecorder - Real-time Mission Event Logger (Updated)
====================================================================

Records ALL mission events to timestamped log files matching the
topics recorded in ros2 bag sessions.

TOPICS MONITORED:
  /mission/status                    - State, Target, Score, Retry, FailedPOIs, Camera, Teleop
  /mission/metrics                   - Evaluated, Photos, Transitions, Distance, Efficiency
  /move_base_simple/goal             - Nav2 goals published by mission controller
  /navigate_to_pose/_action/status   - Nav2 action results (ground truth)
  /odometry/filtered                 - Robot position samples
  /human_clustering/humans           - Human detection count
  /human_clustering/clusters         - Cluster data (size, formation)
  /poi_manager/all_pois              - Available POI count
  /photo/intelligence/trigger        - Photo session triggered
  /photo/intelligence/status         - Photo evaluating/active/completed/idle
  /photo/command                     - Camera yes/no (from PhotoIntelligence or RC)
  /recovery/status                   - Recovery level changes
  /recovery/command                  - Recovery commands
  /mission/recovery_ack              - Recovery ACKs
  /recovery/override                 - E-stop manual override
  /teleop/override                   - Teleop pause/resume
  /e_stop                            - Emergency stop
  /cmd_vel                           - Velocity commands (rate tracking)

OUTPUT:
  ~/manriix_logs/mission_YYYYMMDD_HHMMSS.txt  - Human-readable timeline
  ~/manriix_logs/mission_YYYYMMDD_HHMMSS.csv  - Machine-readable
  ~/manriix_logs/mission_latest.txt            - Symlink (tail -f friendly)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.callback_groups import ReentrantCallbackGroup

from std_msgs.msg import String, Bool
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from action_msgs.msg import GoalStatusArray, GoalStatus

import os
import csv
import time
import json
from datetime import datetime
from pathlib import Path
from typing import Optional, List, Dict
import numpy as np

try:
    from manriix_perception.msg import HumansList, ClusterArray
    HAS_CUSTOM_MSGS = True
except ImportError:
    HAS_CUSTOM_MSGS = False


class MissionTelemetryRecorder(Node):

    def __init__(self):
        super().__init__('mission_telemetry_recorder')
        self.cb_group = ReentrantCallbackGroup()

        self.declare_parameter('log_directory', '~/manriix_logs')
        self.declare_parameter('position_sample_interval', 2.0)
        self.declare_parameter('human_sample_interval', 5.0)
        self.declare_parameter('flush_interval', 1.0)

        log_dir = self.get_parameter('log_directory').value
        self.log_dir = Path(os.path.expanduser(log_dir))
        self.position_sample_interval = self.get_parameter('position_sample_interval').value
        self.human_sample_interval = self.get_parameter('human_sample_interval').value
        self.flush_interval = self.get_parameter('flush_interval').value

        self.log_dir.mkdir(parents=True, exist_ok=True)
        timestamp_str = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.csv_path = self.log_dir / f"mission_{timestamp_str}.csv"
        self.txt_path = self.log_dir / f"mission_{timestamp_str}.txt"
        self.latest_link = self.log_dir / "mission_latest.txt"

        if self.latest_link.is_symlink() or self.latest_link.exists():
            self.latest_link.unlink()
        self.latest_link.symlink_to(self.txt_path)

        self.txt_file = open(self.txt_path, 'w', buffering=1)
        self.csv_file = open(self.csv_path, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([
            'timestamp', 'elapsed_s', 'event_type', 'event_detail',
            'robot_x', 'robot_y', 'robot_yaw',
            'target_x', 'target_y', 'distance_to_target',
            'state', 'retry_count', 'failed_pois',
            'recovery_level', 'human_count', 'cluster_count', 'extra_data'
        ])

        # Session state
        self.mission_start_time = time.time()
        self.robot_position: Optional[np.ndarray] = None
        self.robot_yaw = 0.0
        self.last_position_sample_time = 0.0

        self.current_state = "unknown"
        self.current_target_pos = (0.0, 0.0)
        self.current_score = 0.0
        self.current_retry_count = 0
        self.current_failed_pois = 0
        self.recovery_lock = False
        self.camera_active = False
        self.teleop_paused = False
        self._prev_state = "unknown"

        self.human_count = 0
        self.stationary_count = 0
        self.last_human_sample_time = 0.0
        self.last_human_data_time = 0.0

        self.cluster_count = 0
        self.cluster_details: List[Dict] = []

        self.recovery_level = 0
        self._prev_recovery_level = -1

        self.goal_counter = 0
        self.goals_sent: List[Dict] = []
        self._prev_nav2_statuses: Dict[str, int] = {}

        self.photo_counter = 0
        self.photo_start_time: Optional[float] = None
        self.photo_durations: List[float] = []
        self.teleop_events = 0
        self.cmd_vel_count = 0
        self.available_poi_count = 0
        self.recovery_event_count = 0

        # QoS
        qos_r = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, depth=10)
        qos_b = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=5)

        # Subscribers
        self.create_subscription(String, '/mission/status', self._mission_status_cb, qos_r, callback_group=self.cb_group)
        self.create_subscription(String, '/mission/metrics', self._mission_metrics_cb, qos_b, callback_group=self.cb_group)
        self.create_subscription(PoseStamped, '/move_base_simple/goal', self._nav_goal_cb, qos_r, callback_group=self.cb_group)
        self.create_subscription(GoalStatusArray, '/navigate_to_pose/_action/status', self._nav2_status_cb, qos_b, callback_group=self.cb_group)
        self.create_subscription(Odometry, '/odometry/filtered', self._robot_pose_cb, qos_b, callback_group=self.cb_group)
        self.create_subscription(Bool, '/photo/intelligence/trigger', self._photo_trigger_cb, qos_r, callback_group=self.cb_group)
        self.create_subscription(String, '/photo/intelligence/status', self._photo_status_cb, qos_r, callback_group=self.cb_group)
        self.create_subscription(String, '/photo/command', self._photo_command_cb, qos_r, callback_group=self.cb_group)
        self.create_subscription(Bool, '/teleop/override', self._teleop_cb, qos_r, callback_group=self.cb_group)
        self.create_subscription(Bool, '/e_stop', self._estop_cb, qos_r, callback_group=self.cb_group)
        self.create_subscription(Bool, '/recovery/override', self._recovery_override_cb, qos_r, callback_group=self.cb_group)
        self.create_subscription(String, '/recovery/status', self._recovery_status_cb, qos_r, callback_group=self.cb_group)
        self.create_subscription(String, '/recovery/command', self._recovery_cmd_cb, qos_r, callback_group=self.cb_group)
        self.create_subscription(String, '/mission/recovery_ack', self._recovery_ack_cb, qos_r, callback_group=self.cb_group)
        self.create_subscription(String, '/poi_manager/all_pois', self._pois_cb, qos_b, callback_group=self.cb_group)
        self.create_subscription(Twist, '/cmd_vel', self._cmd_vel_cb, qos_b, callback_group=self.cb_group)

        if HAS_CUSTOM_MSGS:
            self.create_subscription(HumansList, '/human_clustering/humans', self._humans_cb, qos_r, callback_group=self.cb_group)
            self.create_subscription(ClusterArray, '/human_clustering/clusters', self._clusters_cb, qos_r, callback_group=self.cb_group)

        self.create_timer(self.flush_interval, self._flush)
        self.create_timer(30.0, self._summary)

        self._write_header()
        self.get_logger().info(f"Telemetry started -> {self.txt_path}")

    # ── Helpers ──

    def _elapsed(self): return time.time() - self.mission_start_time
    def _elapsed_str(self):
        e = self._elapsed(); return f"{int(e//60):3d}m {e%60:05.2f}s"
    def _ts(self):
        n = datetime.now(); return n.strftime('%H:%M:%S.') + f"{n.microsecond//1000:03d}"

    def _txt(self, evt, msg):
        self.txt_file.write(f"[{self._ts()}] [{self._elapsed_str()}] [{evt:20s}] {msg}\n")

    def _csv(self, evt, detail, extra=""):
        rx = f"{self.robot_position[0]:.3f}" if self.robot_position is not None else ""
        ry = f"{self.robot_position[1]:.3f}" if self.robot_position is not None else ""
        d = ""
        if self.robot_position is not None and self.current_target_pos != (0.0, 0.0):
            d = f"{np.linalg.norm(self.robot_position - np.array(self.current_target_pos)):.3f}"
        self.csv_writer.writerow([
            f"{time.time():.3f}", f"{self._elapsed():.2f}", evt, detail,
            rx, ry, f"{self.robot_yaw:.3f}",
            f"{self.current_target_pos[0]:.3f}", f"{self.current_target_pos[1]:.3f}", d,
            self.current_state, self.current_retry_count, self.current_failed_pois,
            self.recovery_level, self.human_count, self.cluster_count, extra])

    def _log(self, evt, msg, extra=""):
        self._txt(evt, msg); self._csv(evt, msg, extra)

    def _flush(self):
        try: self.txt_file.flush(); self.csv_file.flush()
        except: pass

    def _write_header(self):
        now = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        self.txt_file.write(
            f"{'='*80}\n  MANRIIX MISSION TELEMETRY LOG\n  Started: {now}\n{'='*80}\n\n"
            f"  EVENTS: STATE_CHANGE NAV_GOAL_SENT NAV2_ACCEPTED NAV2_SUCCEEDED NAV2_ABORTED\n"
            f"          NAV2_CANCELED POI_RETRY POI_FAILED PHOTO_TRIGGER PHOTO_EVALUATING\n"
            f"          PHOTO_ACTIVE PHOTO_COMPLETE CAMERA_CMD TELEOP_PAUSE TELEOP_RESUME\n"
            f"          E_STOP E_STOP_OVERRIDE RECOVERY_LEVEL RECOVERY_CMD RECOVERY_ACK\n"
            f"          HUMAN_DATA HUMAN_DETECTED HUMAN_LOST_ALL CLUSTER_DATA POI_COUNT SUMMARY\n"
            f"\n{'='*80}\n\n")

    # ── Mission Status ──

    def _mission_status_cb(self, msg: String):
        try:
            s = msg.data
            ns = self.current_state
            if "State:" in s: ns = s.split("State:")[1].split()[0]
            if "Target:" in s:
                t = s.split("Target:")[1].split()[0].strip("()")
                p = t.split(","); self.current_target_pos = (float(p[0]), float(p[1]))
            if "Score:" in s: self.current_score = float(s.split("Score:")[1].split()[0])
            or_ = self.current_retry_count
            self.current_retry_count = int(s.split("Retry:")[1].split()[0].split("/")[0]) if "Retry:" in s else 0
            of = self.current_failed_pois
            self.current_failed_pois = int(s.split("FailedPOIs:")[1].split()[0]) if "FailedPOIs:" in s else 0
            self.recovery_lock = "RecoveryLock:true" in s
            self.camera_active = "Camera:active" in s
            self.teleop_paused = "Teleop:paused" in s

            if ns != self._prev_state:
                self._log("STATE_CHANGE",
                    f"{self._prev_state} -> {ns} (retry={self.current_retry_count} "
                    f"failed={self.current_failed_pois} cam={'on' if self.camera_active else 'off'} "
                    f"teleop={'paused' if self.teleop_paused else 'off'})")
                self._prev_state = ns; self.current_state = ns
            if self.current_retry_count > or_ and self.current_retry_count > 0:
                self._log("POI_RETRY",
                    f"Retry {self.current_retry_count} target=({self.current_target_pos[0]:.2f}, "
                    f"{self.current_target_pos[1]:.2f}) score={self.current_score:.2f}")
            if self.current_failed_pois > of:
                self._log("POI_FAILED", f"Failed POIs: {of} -> {self.current_failed_pois}")
        except Exception as e:
            self.get_logger().debug(f"Status parse error: {e}")

    def _mission_metrics_cb(self, msg: String):
        try: self._csv("METRICS", msg.data)
        except: pass

    # ── Navigation ──

    def _nav_goal_cb(self, msg: PoseStamped):
        try:
            x, y = msg.pose.position.x, msg.pose.position.y
            from tf_transformations import euler_from_quaternion
            q = msg.pose.orientation
            _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
            self.goal_counter += 1
            dist = float(np.linalg.norm(self.robot_position - np.array([x, y]))) if self.robot_position is not None else 0.0
            r = f" [RETRY #{self.current_retry_count}]" if self.current_retry_count > 0 else ""
            self.goals_sent.append({'id': self.goal_counter, 'time': time.time(), 'x': x, 'y': y,
                'yaw': yaw, 'distance': dist, 'score': self.current_score, 'retry': self.current_retry_count})
            self._log("NAV_GOAL_SENT",
                f"Goal #{self.goal_counter} -> ({x:.2f}, {y:.2f}) dist={dist:.1f}m score={self.current_score:.2f}{r}")
        except Exception as e:
            self.get_logger().debug(f"Nav goal error: {e}")

    def _nav2_status_cb(self, msg: GoalStatusArray):
        try:
            for gs in msg.status_list:
                uid = ''.join(f'{b:02x}' for b in gs.goal_info.goal_id.uuid[:4])
                sc = gs.status
                if sc == self._prev_nav2_statuses.get(uid, -1): continue
                self._prev_nav2_statuses[uid] = sc
                ds = ""
                if self.robot_position is not None and self.current_target_pos != (0.0, 0.0):
                    ds = f" dist={np.linalg.norm(self.robot_position - np.array(self.current_target_pos)):.2f}m"
                if sc == GoalStatus.STATUS_SUCCEEDED:
                    self._log("NAV2_SUCCEEDED", f"Goal {uid} SUCCEEDED{ds}")
                elif sc == GoalStatus.STATUS_ABORTED:
                    self._log("NAV2_ABORTED", f"Goal {uid} ABORTED{ds} (retry={self.current_retry_count} failed={self.current_failed_pois})")
                elif sc == GoalStatus.STATUS_CANCELED:
                    self._log("NAV2_CANCELED", f"Goal {uid} CANCELED{ds}")
                elif sc == 1:
                    self._log("NAV2_ACCEPTED", f"Goal {uid} ACCEPTED{ds}")
            if len(self._prev_nav2_statuses) > 20:
                for k in list(self._prev_nav2_statuses.keys())[:-20]: del self._prev_nav2_statuses[k]
        except Exception as e:
            self.get_logger().debug(f"Nav2 status error: {e}")

    # ── Robot Pose ──

    def _robot_pose_cb(self, msg: Odometry):
        try:
            self.robot_position = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y])
            from tf_transformations import euler_from_quaternion
            q = msg.pose.pose.orientation
            _, _, self.robot_yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
            t = time.time()
            if t - self.last_position_sample_time >= self.position_sample_interval:
                self.last_position_sample_time = t
                self._csv("POSITION_SAMPLE", f"({self.robot_position[0]:.3f}, {self.robot_position[1]:.3f})")
        except: pass

    # ── Photo ──

    def _photo_trigger_cb(self, msg: Bool):
        if msg.data:
            self._log("PHOTO_TRIGGER",
                f"Triggered at ({self.current_target_pos[0]:.2f}, {self.current_target_pos[1]:.2f}) score={self.current_score:.2f}")

    def _photo_status_cb(self, msg: String):
        try:
            if msg.data == "evaluating":
                self._log("PHOTO_EVALUATING", "Evaluating crowd for photo timing")
            elif msg.data == "active":
                self.photo_start_time = time.time()
                self._log("PHOTO_ACTIVE", "Photo session started")
            elif msg.data == "completed":
                dur = time.time() - self.photo_start_time if self.photo_start_time else 0.0
                self.photo_durations.append(dur)
                self.photo_counter += 1
                self.photo_start_time = None
                self._log("PHOTO_COMPLETE", f"Photo #{self.photo_counter} duration={dur:.1f}s")
            elif msg.data == "idle":
                self._csv("PHOTO_IDLE", "Photo system idle")
        except Exception as e:
            self.get_logger().debug(f"Photo status error: {e}")

    def _photo_command_cb(self, msg: String):
        if msg.data == "yes":
            self._log("CAMERA_CMD", "Camera START (yes)")
        elif msg.data == "no":
            self._log("CAMERA_CMD", "Camera STOP (no)")

    # ── Teleop / E-Stop ──

    def _teleop_cb(self, msg: Bool):
        self.teleop_events += 1
        if msg.data:
            self._log("TELEOP_PAUSE", "Manual override ACTIVATED")
        else:
            self._log("TELEOP_RESUME", "Autonomous operation RESUMED")

    def _estop_cb(self, msg: Bool):
        self._log("E_STOP", "ACTIVATED" if msg.data else "CLEARED")

    def _recovery_override_cb(self, msg: Bool):
        if msg.data:
            self._log("E_STOP_OVERRIDE", "Manual override clearing e-stop")

    # ── Recovery ──

    def _recovery_status_cb(self, msg: String):
        try:
            s = msg.data; nl = 0
            for i in range(9):
                if f"Level:{i}" in s: nl = i; break
            if nl != self._prev_recovery_level:
                d = "ESCALATE" if nl > self._prev_recovery_level else ("DE-ESCALATE" if self._prev_recovery_level >= 0 else "INIT")
                self._log("RECOVERY_LEVEL", f"Level {self._prev_recovery_level} -> {nl} ({d}) [{s}]")
                if nl > 0 and self._prev_recovery_level <= 0: self.recovery_event_count += 1
                self._prev_recovery_level = nl; self.recovery_level = nl
        except Exception as e:
            self.get_logger().debug(f"Recovery status error: {e}")

    def _recovery_cmd_cb(self, msg: String):
        self._log("RECOVERY_CMD", f"Command: {msg.data}")

    def _recovery_ack_cb(self, msg: String):
        self._log("RECOVERY_ACK", f"ACK: {msg.data}")

    # ── Human / Cluster ──

    def _humans_cb(self, msg):
        try:
            t = time.time(); self.last_human_data_time = t
            oc = self.human_count; self.human_count = msg.human_count; self.stationary_count = msg.stationary_count
            if msg.human_count != oc or t - self.last_human_sample_time >= self.human_sample_interval:
                self.last_human_sample_time = t
                evt = "HUMAN_DATA"
                if msg.human_count == 0 and oc > 0: evt = "HUMAN_LOST_ALL"
                elif msg.human_count > 0 and oc == 0: evt = "HUMAN_DETECTED"
                self._log(evt, f"humans={msg.human_count} stationary={msg.stationary_count}")
        except Exception as e:
            self.get_logger().debug(f"Humans error: {e}")

    def _clusters_cb(self, msg):
        try:
            self.cluster_count = len(msg.clusters)
            self.cluster_details = [{'id': c.id, 'size': c.size, 'formation': c.formation} for c in msg.clusters]
        except: pass

    # ── POI / cmd_vel ──

    def _pois_cb(self, msg: String):
        try:
            pois = json.loads(msg.data); nc = len(pois) if pois else 0
            if nc != self.available_poi_count:
                o = self.available_poi_count; self.available_poi_count = nc
                self._log("POI_COUNT", f"POIs: {o} -> {nc}")
        except: pass

    def _cmd_vel_cb(self, msg: Twist):
        self.cmd_vel_count += 1

    # ── Summary ──

    def _summary(self):
        try:
            e = self._elapsed()
            hf = ""
            if self.last_human_data_time > 0:
                a = time.time() - self.last_human_data_time
                hf = f" (stale {a:.0f}s)" if a > 10 else f" (fresh)"
            n2ok = sum(1 for s in self._prev_nav2_statuses.values() if s == GoalStatus.STATUS_SUCCEEDED)
            n2ab = sum(1 for s in self._prev_nav2_statuses.values() if s == GoalStatus.STATUS_ABORTED)
            n2cn = sum(1 for s in self._prev_nav2_statuses.values() if s == GoalStatus.STATUS_CANCELED)
            ap = sum(self.photo_durations)/len(self.photo_durations) if self.photo_durations else 0.0

            self.txt_file.write(
                f"\n{'─'*70}\n"
                f"  SUMMARY at {self._elapsed_str()} ({e:.0f}s)\n"
                f"{'─'*70}\n"
                f"  State: {self.current_state} | Camera: {'ON' if self.camera_active else 'off'}"
                f" | Teleop: {'PAUSED' if self.teleop_paused else 'off'}\n\n"
                f"  NAV:    goals={self.goal_counter} ok={n2ok} abort={n2ab} cancel={n2cn}"
                f" retry={self.current_retry_count} failed={self.current_failed_pois} pois={self.available_poi_count}\n"
                f"  PHOTO:  taken={self.photo_counter} avg_dur={ap:.1f}s\n"
                f"  HUMAN:  count={self.human_count}{hf} clusters={self.cluster_count}\n"
                f"  RECOV:  level={self.recovery_level} events={self.recovery_event_count}\n"
                f"  SYS:    cmd_vel={self.cmd_vel_count} teleop_events={self.teleop_events}\n"
                f"{'─'*70}\n\n")
            self._csv("SUMMARY", f"goals={self.goal_counter} ok={n2ok} abort={n2ab} photos={self.photo_counter} humans={self.human_count}")
        except Exception as e:
            self.get_logger().error(f"Summary error: {e}")

    # ── Cleanup ──

    def destroy_node(self):
        try:
            self._log("SESSION_END", "Recording stopped")
            self._summary()
            self.txt_file.write(f"\n{'='*70}\n  GOAL HISTORY ({self.goal_counter})\n{'='*70}\n\n")
            for g in self.goals_sent:
                r = f" [RETRY #{g['retry']}]" if g['retry'] > 0 else ""
                self.txt_file.write(f"  #{g['id']}: ({g['x']:.2f}, {g['y']:.2f}) dist={g['distance']:.1f}m score={g['score']:.2f}{r}\n")
            self.txt_file.write(f"\n{'='*70}\n  PHOTO HISTORY ({self.photo_counter})\n{'='*70}\n\n")
            for i, d in enumerate(self.photo_durations, 1):
                self.txt_file.write(f"  #{i}: {d:.1f}s\n")
            self.txt_file.write(f"\n{'='*70}\n  END OF LOG\n{'='*70}\n")
            self.txt_file.close(); self.csv_file.close()
            self.get_logger().info(f"Saved: {self.txt_path} ({self.goal_counter} goals, {self.photo_counter} photos)")
        except Exception as e:
            self.get_logger().error(f"Cleanup error: {e}")
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