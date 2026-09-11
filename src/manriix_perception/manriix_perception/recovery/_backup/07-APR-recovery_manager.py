#!/usr/bin/env python3
"""
RecoveryManager - FIXED Version
================================
FIXES: [#8] Coordination protocol, [#1] No direct /cmd_vel

COORDINATION PROTOCOL:
    RecoveryManager                    MissionController
    ───────────────                    ─────────────────
    1. Detect issue
    2. Publish "Level:N" status  ────→ Sets _recovery_lock = True
    3. Send command              ────→ Receives, ACKs, executes via Nav2
       _command_state = SENT     ←──── ACK on /mission/recovery_ack
       _command_state = EXECUTING
    4. Wait for level duration
    5. If resolved → "Level:0"   ────→ _recovery_lock = False, resume
       If not → escalate (only if command not still executing)

KEY RULES:
    - ONE command per level (no flooding)
    - Never escalate while command is executing
    - Emergency stop bypasses protocol
    - Never publish to /cmd_vel (only /cmd_vel_recovery for emergency)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
import numpy as np
from typing import Dict, Optional
import time
import psutil
from dataclasses import dataclass, field
from enum import Enum

from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist
from manriix_perception.msg import HumansList


class RecoveryLevel(Enum):
    NORMAL = 0
    WAIT_AND_MONITOR = 1
    SENSOR_RESET = 2
    ORIENTATION_SCANNING = 3
    POSITION_RELOCATION = 4
    EXPLORATION_MODE = 5
    SYSTEM_DIAGNOSTIC = 6
    SAFE_PARKING = 7
    EMERGENCY_STOP = 8


class CommandState(Enum):
    IDLE = "idle"
    SENT = "sent"
    EXECUTING = "executing"
    TIMED_OUT = "timed_out"


@dataclass
class SystemHealth:
    cpu_usage: float = 0.0
    memory_usage: float = 0.0
    disk_usage: float = 0.0


class RecoveryManager(Node):

    def __init__(self):
        super().__init__('recovery_manager')

        # Parameters
        self.declare_parameter('no_data_timeout', 45.0)
        self.declare_parameter('stale_data_timeout', 15.0)
        self.declare_parameter('max_recovery_attempts', 3)
        self.declare_parameter('safe_parking_position', [0.0, 0.0])
        self.declare_parameter('command_ack_timeout', 10.0)
        self.declare_parameter('system_health_check_interval', 5.0)
        self.declare_parameter('enable_autonomous_navigation', False)

        self.no_data_timeout = self.get_parameter('no_data_timeout').value
        self.stale_data_timeout = self.get_parameter('stale_data_timeout').value
        self.max_attempts = self.get_parameter('max_recovery_attempts').value
        self.safe_parking_pos = np.array(self.get_parameter('safe_parking_position').value)
        self.command_ack_timeout = self.get_parameter('command_ack_timeout').value
        self.health_check_interval = self.get_parameter('system_health_check_interval').value
        self.enable_autonomous_navigation = self.get_parameter('enable_autonomous_navigation').value

        # Recovery state
        self.current_level = RecoveryLevel.NORMAL
        self.recovery_attempts = 0
        self.level_start_time = time.time()
        self.last_data_received = time.time()

        # [FIX #8] Command coordination
        self._command_state = CommandState.IDLE
        self._last_command = None
        self._command_sent_time = 0.0
        self._level_command_sent = False

        # [FIX #NAV] Mission state tracking for nav failure detection
        self._mission_state = "unknown"
        self._mission_failed_pois = 0

        # [FIX #ESTOP] Manual override for emergency stop
        self._estop_override = False

        # System health
        self.system_health = SystemHealth()

        # Level durations (seconds to wait before escalating)
        self.level_durations = {
            RecoveryLevel.WAIT_AND_MONITOR: 15.0,
            RecoveryLevel.SENSOR_RESET: 30.0,
            RecoveryLevel.ORIENTATION_SCANNING: 45.0,
            RecoveryLevel.POSITION_RELOCATION: 60.0,
            RecoveryLevel.EXPLORATION_MODE: 120.0,
            RecoveryLevel.SYSTEM_DIAGNOSTIC: 30.0,
            RecoveryLevel.SAFE_PARKING: 60.0,
            RecoveryLevel.EMERGENCY_STOP: float('inf'),
        }

        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, depth=5)

        # Subscribers
        self.humans_sub = self.create_subscription(
            HumansList, '/human_clustering/humans',
            self.humans_callback, qos)

        self.recovery_ack_sub = self.create_subscription(
            String, '/mission/recovery_ack',
            self.recovery_ack_callback, qos)

        self.mission_status_sub = self.create_subscription(
            String, '/mission/status',
            self.mission_status_callback, qos)

        # [FIX #ESTOP] Manual override to clear emergency stop
        self.override_sub = self.create_subscription(
            Bool, '/recovery/override',
            self.override_callback, qos)        

        # Publishers
        self.command_pub = self.create_publisher(String, '/recovery/command', qos)
        self.status_pub = self.create_publisher(String, '/recovery/status', qos)
        self.emergency_stop_pub = self.create_publisher(Bool, '/e_stop', qos)
        self.emergency_vel_pub = self.create_publisher(Twist, '/cmd_vel_recovery', qos)

        # Timers
        self.check_timer = self.create_timer(1.0, self.check_for_issues)
        self.health_timer = self.create_timer(self.health_check_interval, self.system_health_check)

        self.get_logger().info("RecoveryManager initialized with coordination protocol")

    # ====================================================================
    # CALLBACKS
    # ====================================================================

    # def humans_callback(self, msg: HumansList):
    #     """Track data reception — triggers recovery success check"""
    #     if msg.human_count > 0:
    #         self.last_data_received = time.time()
    #         if self.current_level != RecoveryLevel.NORMAL:
    #             self._check_recovery_success()

    def humans_callback(self, msg: HumansList):
        """Track data reception — triggers recovery success check"""
        if len(msg.humans) > 0:
            self.last_data_received = time.time()
            if self.current_level != RecoveryLevel.NORMAL:
                self._check_recovery_success()

    def recovery_ack_callback(self, msg: String):
        """[FIX #8] Process ACK from mission controller"""
        try:
            parts = msg.data.split(":")
            if len(parts) < 3 or parts[0] != "ACK":
                return

            acked_command = parts[1]
            ack_status = parts[2]

            self.get_logger().info(f"ACK: {acked_command} → {ack_status}")

            if acked_command == self._last_command:
                if ack_status in ["executing", "queued", "cancelling_nav_first"]:
                    self._command_state = CommandState.EXECUTING
                elif ack_status == "ignored_nav_disabled":
                    self._command_state = CommandState.IDLE
        except Exception as e:
            self.get_logger().error(f"Error processing ACK: {e}")

    # def mission_status_callback(self, msg: String):
    #     """Detect when mission controller finishes our command"""
    #     try:
    #         if ("State:idle" in msg.data and
    #             self._command_state == CommandState.EXECUTING):
    #             self.get_logger().info("Command execution completed")
    #             self._command_state = CommandState.IDLE
    #     except Exception as e:
    #         self.get_logger().error(f"Error in mission status: {e}")


    def mission_status_callback(self, msg: String):
        """Track mission controller state for nav failure detection"""
        try:
            # Track command execution completion
            if ("State:idle" in msg.data and
                self._command_state == CommandState.EXECUTING):
                self.get_logger().info("Command execution completed")
                self._command_state = CommandState.IDLE
            
            # [FIX #NAV] Track mission state and failed POIs
            self._mission_state = "unknown"
            if "State:" in msg.data:
                self._mission_state = msg.data.split("State:")[1].split()[0]
            
            self._mission_failed_pois = 0
            if "FailedPOIs:" in msg.data:
                try:
                    self._mission_failed_pois = int(
                        msg.data.split("FailedPOIs:")[1].split()[0])
                except ValueError:
                    pass
                    
        except Exception as e:
            self.get_logger().error(f"Error in mission status: {e}")

    def override_callback(self, msg: Bool):
        """
        [FIX #ESTOP] Manual override to clear emergency stop.
        
        Usage: ros2 topic pub --once /recovery/override std_msgs/Bool "data: true"
        
        Only works when at Level 8. Clears e-stop and returns to NORMAL.
        Operator can then use teleop or let autonomous system resume.
        """
        try:
            if msg.data and self.current_level == RecoveryLevel.EMERGENCY_STOP:
                self.get_logger().warn(
                    "🔓 MANUAL OVERRIDE — clearing emergency stop")
                self._estop_override = True
                
                # Clear e-stop
                emergency_msg = Bool()
                emergency_msg.data = False
                self.emergency_stop_pub.publish(emergency_msg)
                
                # Return to normal
                self.current_level = RecoveryLevel.NORMAL
                self.recovery_attempts = 0
                self._command_state = CommandState.IDLE
                self._level_command_sent = False
                self._last_command = None
                self._estop_override = False
                self._publish_status()  # Level:0 → mission controller unlocks
                
                self.get_logger().warn(
                    "✅ Emergency stop cleared — robot can be controlled")
            elif msg.data and self.current_level != RecoveryLevel.EMERGENCY_STOP:
                self.get_logger().info(
                    f"Override ignored — not in emergency stop "
                    f"(current level: {self.current_level.value})")
        except Exception as e:
            self.get_logger().error(f"Error in override: {e}")            

    # ====================================================================
    # MAIN LOOP
    # ====================================================================

    def check_for_issues(self):
        """1 Hz loop: detect issues and manage recovery"""
        try:
            current_time = time.time()

            # Check command ACK timeout
            if (self._command_state == CommandState.SENT and
                current_time - self._command_sent_time > self.command_ack_timeout):
                self.get_logger().warn(f"Command ACK timeout for '{self._last_command}'")
                self._command_state = CommandState.TIMED_OUT

            # if self.current_level == RecoveryLevel.NORMAL:
            #     # Normal state — watch for problems
            #     time_since_data = current_time - self.last_data_received

            #     if time_since_data > self.no_data_timeout:
            #         self.get_logger().warn(f"No data for {time_since_data:.0f}s")
            #         self._escalate_to(RecoveryLevel.WAIT_AND_MONITOR)

            #     elif (self.system_health.cpu_usage > 98 or
            #           self.system_health.memory_usage > 95):
            #         self.get_logger().warn("System overload")
            #         self._escalate_to(RecoveryLevel.SYSTEM_DIAGNOSTIC)

            if self.current_level == RecoveryLevel.NORMAL:
                # Normal state — watch for problems
                time_since_data = current_time - self.last_data_received

                if time_since_data > self.no_data_timeout:
                    self.get_logger().warn(f"No data for {time_since_data:.0f}s")
                    self._escalate_to(RecoveryLevel.WAIT_AND_MONITOR)

                # [FIX #NAV] Trigger 3: Mission controller exhausted all POIs
                elif (self._mission_state == "recovery" and 
                      self._mission_failed_pois > 0):
                    self.get_logger().warn(
                        f"Navigation failure detected — mission in recovery "
                        f"with {self._mission_failed_pois} failed POIs")
                    self._escalate_to(RecoveryLevel.WAIT_AND_MONITOR)

                elif (self.system_health.cpu_usage > 98 or
                      self.system_health.memory_usage > 95):
                    self.get_logger().warn("System overload")
                    self._escalate_to(RecoveryLevel.SYSTEM_DIAGNOSTIC)
            else:
                # In recovery — manage current level
                self._manage_recovery_level(current_time)

        except Exception as e:
            self.get_logger().error(f"Error in check loop: {e}")

    def _manage_recovery_level(self, current_time: float):
        """
        [FIX #8] Recovery level management with coordination.

        1. Send command for this level (once)
        2. Wait for level duration
        3. Check if issue resolved
        4. If not, escalate (but ONLY if command is not still executing)
        """
        # [FIX #ESTOP] Keep publishing e-stop every second while at Level 8
        if self.current_level == RecoveryLevel.EMERGENCY_STOP:
            stop_msg = Twist()
            self.emergency_vel_pub.publish(stop_msg)
            emergency_msg = Bool()
            emergency_msg.data = True
            self.emergency_stop_pub.publish(emergency_msg)
            return

        level_elapsed = current_time - self.level_start_time
        level_duration = self.level_durations.get(self.current_level, 30.0)

        # Step 1: Send command (once per level)
        if not self._level_command_sent:
            self._send_level_command()
            self._level_command_sent = True
            return

        # Step 2: Wait for duration
        if level_elapsed < level_duration:
            return

        # Step 3: Check if resolved
        if self._check_recovery_success():
            return

        # Step 4: Duration expired, not resolved
        # [FIX #8] NEVER escalate while command is executing
        if self._command_state == CommandState.EXECUTING:
            self.get_logger().info(
                f"Level {self.current_level.value} expired but "
                f"command still executing — waiting")
            return

        # Escalate or retry
        self.recovery_attempts += 1
        if self.recovery_attempts >= self.max_attempts:
            next_level_val = min(
                self.current_level.value + 1,
                RecoveryLevel.EMERGENCY_STOP.value)
            next_level = RecoveryLevel(next_level_val)
            self.get_logger().warn(
                f"Escalating: Level {self.current_level.value} → {next_level.value}")
            self.recovery_attempts = 0
            self._escalate_to(next_level)
        else:
            self.get_logger().info(
                f"Retrying Level {self.current_level.value} "
                f"({self.recovery_attempts + 1}/{self.max_attempts})")
            self._level_command_sent = False
            self.level_start_time = time.time()

    # def _check_recovery_success(self) -> bool:
    #     """Check if recovery condition is resolved"""
    #     time_since_data = time.time() - self.last_data_received
    #     if time_since_data < 5.0:
    #         self.get_logger().info("Data received — recovery successful!")
    #         self._return_to_normal()
    #         return True
    #     return False

    def _check_recovery_success(self) -> bool:
        """Check if recovery condition is resolved"""
        # [FIX #ESTOP] Never auto-recover from emergency stop
        if self.current_level == RecoveryLevel.EMERGENCY_STOP:
            return False
        
        time_since_data = time.time() - self.last_data_received
        if time_since_data < 5.0:
            self.get_logger().info("Data received — recovery successful!")
            self._return_to_normal()
            return True
        return False

    # ====================================================================
    # LEVEL TRANSITIONS
    # ====================================================================

    def _escalate_to(self, level: RecoveryLevel):
        """Enter a recovery level"""
        self.current_level = level
        self.level_start_time = time.time()
        self._level_command_sent = False
        self._command_state = CommandState.IDLE
        self.get_logger().warn(f"⚠️ Recovery Level {level.value}: {level.name}")
        self._publish_status()

    def _return_to_normal(self):
        """Return to normal operation — releases mission controller lock"""
        if self.current_level == RecoveryLevel.NORMAL:
            return
        self.get_logger().info(f"✅ Recovery complete (was Level {self.current_level.value})")
        self.current_level = RecoveryLevel.NORMAL
        self.recovery_attempts = 0
        self._command_state = CommandState.IDLE
        self._level_command_sent = False
        self._last_command = None
        self._publish_status()  # "Level:0" → mission controller unlocks

    # ====================================================================
    # COMMAND DISPATCH
    # ====================================================================

    def _send_level_command(self):
        """Send the appropriate command for current level"""

        if self.current_level == RecoveryLevel.WAIT_AND_MONITOR:
            self._command_state = CommandState.IDLE  # No command — just wait
            self.get_logger().info("Level 1: Waiting and monitoring...")

        elif self.current_level == RecoveryLevel.SENSOR_RESET:
            self._send_command("reset_sensors")

        elif self.current_level == RecoveryLevel.ORIENTATION_SCANNING:
            if self.enable_autonomous_navigation:
                self._send_command("SPIN_RECOVERY")
            else:
                self._command_state = CommandState.IDLE

        elif self.current_level == RecoveryLevel.POSITION_RELOCATION:
            if self.enable_autonomous_navigation:
                fx = self.safe_parking_pos[0] + 2.0
                fy = self.safe_parking_pos[1] + 2.0
                self._send_command(f"RELOCATE:{fx:.2f},{fy:.2f}")
            else:
                self._command_state = CommandState.IDLE

        elif self.current_level == RecoveryLevel.EXPLORATION_MODE:
            self._send_command("start_exploration")

        elif self.current_level == RecoveryLevel.SYSTEM_DIAGNOSTIC:
            self._run_diagnostics()
            self._command_state = CommandState.IDLE

        elif self.current_level == RecoveryLevel.SAFE_PARKING:
            if self.enable_autonomous_navigation:
                sx, sy = self.safe_parking_pos[0], self.safe_parking_pos[1]
                self._send_command(f"SAFE_PARKING:{sx:.2f},{sy:.2f}")
            else:
                self._command_state = CommandState.IDLE

        elif self.current_level == RecoveryLevel.EMERGENCY_STOP:
            self._execute_emergency_stop()

    def _send_command(self, command: str):
        """Publish command with lifecycle tracking"""
        self._last_command = command
        self._command_state = CommandState.SENT
        self._command_sent_time = time.time()
        msg = String()
        msg.data = command
        self.command_pub.publish(msg)
        self.get_logger().info(f"📤 Command: {command} (awaiting ACK)")

    def _publish_status(self):
        """Publish recovery status for mission controller"""
        msg = String()
        msg.data = (
            f"Level:{self.current_level.value} "
            f"Name:{self.current_level.name} "
            f"Attempts:{self.recovery_attempts} "
            f"CommandState:{self._command_state.value}")
        self.status_pub.publish(msg)

    # ====================================================================
    # EMERGENCY STOP — Only direct velocity override
    # ====================================================================

    def _execute_emergency_stop(self):
        """Emergency stop — bypasses coordination, uses /cmd_vel_recovery"""
        self.get_logger().error("🚨 EMERGENCY STOP — Level 8")
        stop_msg = Twist()
        self.emergency_vel_pub.publish(stop_msg)
        emergency_msg = Bool()
        emergency_msg.data = True
        self.emergency_stop_pub.publish(emergency_msg)
        self._send_command("EMERGENCY_STOP")
        self._command_state = CommandState.IDLE

    # ====================================================================
    # SYSTEM HEALTH
    # ====================================================================

    def system_health_check(self):
        """Periodic system health check"""
        try:
            self.system_health.cpu_usage = psutil.cpu_percent(interval=None)
            self.system_health.memory_usage = psutil.virtual_memory().percent
            self.system_health.disk_usage = psutil.disk_usage('/').percent
        except Exception as e:
            self.get_logger().error(f"Health check error: {e}")

    def _run_diagnostics(self):
        """Level 6 diagnostics"""
        self.system_health_check()
        issues = []
        if self.system_health.cpu_usage > 95:
            issues.append(f"CPU: {self.system_health.cpu_usage:.1f}%")
        if self.system_health.memory_usage > 90:
            issues.append(f"Memory: {self.system_health.memory_usage:.1f}%")
        if issues:
            self.get_logger().warn(f"Diagnostics: {', '.join(issues)}")
        else:
            self.get_logger().info("Diagnostics: all OK")


def main(args=None):
    rclpy.init(args=args)
    node = RecoveryManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()