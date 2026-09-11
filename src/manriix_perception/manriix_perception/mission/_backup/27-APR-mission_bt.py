#!/usr/bin/env python3
"""
MissionBT — Layer 2 inner loop as a Behaviour Tree.

Replaces the if/elif state machine in mission_control_loop().
The Python layer (mission_controller.py) still owns:
  - POI selection and retry logic
  - Teleop override
  - Recovery lock
  - Failed position memory
  - Nav2 action client

This BT owns:
  - Evaluate → Navigate → Stabilise → Photo sequence
  - Navigation failure → TryNextPOI → RecoverySubtree
  - Recovery: Wait → Spin → ReturnToLast → Idle
"""
import time
import numpy as np
import py_trees
from py_trees.behaviour import Behaviour
from py_trees.common import Status
from py_trees.composites import Sequence, Selector


# ─────────────────────────────────────────────────────────────────────────────
# Normal flow nodes
# ─────────────────────────────────────────────────────────────────────────────

class EvaluateTarget(Behaviour):
    """
    Condition — gate at the start of every tick.
    Returns SUCCESS when ready to pursue a target.
    Returns FAILURE when idle (paused, locked, no targets).
    """

    def __init__(self, controller):
        super().__init__(name="EvaluateTarget")
        self.ctrl = controller

    def update(self) -> Status:
        c = self.ctrl

        if c._teleop_paused:
            return Status.FAILURE
        if c._recovery_lock:
            return Status.FAILURE
        if time.time() < c._recovery_cooldown_until:
            return Status.FAILURE
        if c._camera_active:
            return Status.FAILURE
        if not c.enable_autonomous_navigation:
            return Status.FAILURE
        if not c.available_targets and not c.current_target:
            return Status.FAILURE

        # Check if current_target (set by optimal_positions_callback) is in cooldown
        if c.current_target is not None:
            now = time.time()
            # c._recently_photographed = {
            #     k: v for k, v in c._recently_photographed.items()
            if not hasattr(c, '_recently_photographed'):
                c._recently_photographed = {}
            c._recently_photographed = {
                k: v for k, v in c._recently_photographed.items()    
                if now - v < 120.0
            }
            pos_key = (round(c.current_target.position[0], 1),
                       round(c.current_target.position[1], 1))
            if pos_key in c._recently_photographed:
                remaining = 120.0 - (now - c._recently_photographed[pos_key])
                c.get_logger().info(
                    f"BT: Target {pos_key} on cooldown ({remaining:.0f}s) — clearing")
                c.current_target = None
                return Status.FAILURE

        if c.current_target is None:
            best = max(c.available_targets, key=lambda t: t.priority_score)

            if c.robot_position is not None:
                dist = np.linalg.norm(best.position - c.robot_position)
                if dist < 0.8:
                    c.get_logger().info(
                        f"BT: POI {dist:.2f}m away — already in position")
                    c.current_target = best
                    c.metrics.targets_evaluated += 1
                    return Status.SUCCESS

            # Check recently photographed cooldown — 120s
            now = time.time()
            # c._recently_photographed = {
            #     k: v for k, v in c._recently_photographed.items()
            if not hasattr(c, '_recently_photographed'):
                c._recently_photographed = {}
            c._recently_photographed = {
                k: v for k, v in c._recently_photographed.items()            
                if now - v < 120.0
            }
            pos_key = (round(best.position[0], 1), round(best.position[1], 1))
            if pos_key in c._recently_photographed:
                remaining = 120.0 - (now - c._recently_photographed[pos_key])
                c.get_logger().debug(
                    f"BT: Target {pos_key} on cooldown ({remaining:.0f}s) — waiting")
                return Status.FAILURE

            c.current_target = best
            c.metrics.targets_evaluated += 1
            c.get_logger().info(
                f"BT: Target selected ({best.position[0]:.2f}, "
                f"{best.position[1]:.2f}) score={best.priority_score:.2f}")

        return Status.SUCCESS


class NavigateToPOI(Behaviour):
    """
    Action — navigate to current_target via Nav2.
    RUNNING while navigating, SUCCESS on arrival, FAILURE on timeout/abort.
    """

    def __init__(self, controller):
        super().__init__(name="NavigateToPOI")
        self.ctrl = controller
        self._goal_sent = False

    def initialise(self):
        self._goal_sent = False

    def update(self) -> Status:
        c = self.ctrl

        if c.current_target is None or c.robot_position is None:
            return Status.FAILURE

        distance = np.linalg.norm(
            c.robot_position - c.current_target.position)

        if distance <= c.arrival_threshold:
            c.get_logger().info(
                f"BT: Arrived at POI ({distance:.2f}m <= {c.arrival_threshold}m)")
            if c.current_nav_goal_handle is not None:
                c._cancel_navigation()
            c._reset_retry_state()
            return Status.SUCCESS

        if not self._goal_sent:
            c._send_nav_goal(c.current_target)
            self._goal_sent = True
            c.nav_start_time = time.time()
            return Status.RUNNING

        if c.nav_start_time:
            nav_elapsed = time.time() - c.nav_start_time

            if nav_elapsed > c.absolute_nav_timeout:
                c.get_logger().warn(
                    f"BT: Absolute nav timeout ({c.absolute_nav_timeout:.0f}s)")
                c._cancel_navigation()
                return Status.FAILURE

            if nav_elapsed > c.per_goal_timeout:
                if c._is_making_progress():
                    c.nav_start_time = time.time()
                    return Status.RUNNING
                else:
                    c.get_logger().warn(
                        f"BT: Nav timeout, no progress ({distance:.1f}m remaining)")
                    c._cancel_navigation()
                    return Status.FAILURE

        return Status.RUNNING

    def terminate(self, new_status: Status):
        if new_status == Status.FAILURE:
            if self.ctrl.current_nav_goal_handle is not None:
                self.ctrl._cancel_navigation()


class WaitForStability(Behaviour):
    """
    Action — wait until robot is stable at the POI.
    SUCCESS when stable, SUCCESS on 30s timeout, FAILURE if no position.
    """

    def __init__(self, controller):
        super().__init__(name="WaitForStability")
        self.ctrl = controller
        self._start_time = None

    def initialise(self):
        self._start_time = time.time()
        self.ctrl.get_logger().info("BT: Waiting for robot stability...")

    def update(self) -> Status:
        c = self.ctrl

        if c.robot_position is None:
            return Status.FAILURE

        elapsed = time.time() - self._start_time
        is_stable = c.check_robot_stability()

        if is_stable and elapsed >= c.stability_wait_time:
            c.get_logger().info("BT: Robot stable — proceeding to photo")
            return Status.SUCCESS

        if elapsed > 30.0:
            c.get_logger().warn("BT: Stability timeout — proceeding anyway")
            return Status.SUCCESS

        if not is_stable:
            self._start_time = time.time()

        return Status.RUNNING


class TakePhotoAction(Behaviour):
    """
    Action — trigger photo session via TakePhoto action server.
    RUNNING while session in progress, SUCCESS when result received.
    """

    def __init__(self, controller):
        super().__init__(name="TakePhoto")
        self.ctrl = controller
        self._triggered = False

    def initialise(self):
        self._triggered = False

    def update(self) -> Status:
        c = self.ctrl

        if not self._triggered:
            c.get_logger().info("BT: Triggering TakePhoto action...")
            c.transition_to_photo_sequence()
            self._triggered = True
            return Status.RUNNING

        if not c._camera_active and self._triggered:
            c.get_logger().info("BT: Photo session complete")
            c.metrics.transitions_made += 1
            return Status.SUCCESS

        return Status.RUNNING

    def terminate(self, new_status: Status):
        if new_status == Status.FAILURE:
            if (hasattr(self.ctrl, '_take_photo_goal_handle') and
                    self.ctrl._take_photo_goal_handle is not None):
                self.ctrl._take_photo_goal_handle.cancel_goal_async()
                self.ctrl._take_photo_goal_handle = None
            self.ctrl._camera_active = False


class TryNextPOI(Behaviour):
    """
    Action — mark current position failed, try next viable POI.
    SUCCESS if next POI found, FAILURE if all POIs exhausted.
    """

    def __init__(self, controller):
        super().__init__(name="TryNextPOI")
        self.ctrl = controller

    def update(self) -> Status:
        c = self.ctrl

        if c.current_target is not None:
            c._mark_position_failed(c.current_target.position)

        if c._try_next_poi():
            return Status.SUCCESS

        c.get_logger().warn("BT: All POI alternatives exhausted — entering recovery")
        return Status.FAILURE


# ─────────────────────────────────────────────────────────────────────────────
# Recovery nodes
# ─────────────────────────────────────────────────────────────────────────────

class WaitForHumans(Behaviour):
    """
    Recovery L1 — wait 60s in place.
    SUCCESS if new targets appear. FAILURE after 60s.
    """

    WAIT_DURATION = 60.0

    def __init__(self, controller):
        super().__init__(name="WaitForHumans")
        self.ctrl = controller
        self._start_time = None

    def initialise(self):
        self._start_time = time.time()
        self.ctrl.get_logger().warn(
            "BT Recovery L1: Waiting 60s for humans to appear...")

    def update(self) -> Status:
        c = self.ctrl

        if c._teleop_paused or c._recovery_lock:
            return Status.SUCCESS

        if c.available_targets:
            c.get_logger().info("BT Recovery L1: Humans detected — resuming")
            return Status.SUCCESS

        elapsed = time.time() - self._start_time
        if elapsed >= self.WAIT_DURATION:
            c.get_logger().warn("BT Recovery L1: No humans after 60s — escalating")
            return Status.FAILURE

        if int(elapsed) % 10 == 0 and int(elapsed) > 0:
            remaining = self.WAIT_DURATION - elapsed
            c.get_logger().info(
                f"BT Recovery L1: Waiting... {remaining:.0f}s remaining")

        return Status.RUNNING


class SpinAndScan(Behaviour):
    """
    Recovery L2 — spin 360° using Nav2 Spin action.
    ZED cameras scan all directions. SUCCESS if humans found.
    FAILURE if no humans after 30s or spin server unavailable.
    """

    def __init__(self, controller):
        super().__init__(name="SpinAndScan")
        self.ctrl = controller
        self._spin_sent = False
        self._start_time = None

    def initialise(self):
        self._spin_sent = False
        self._start_time = time.time()
        self.ctrl.get_logger().warn(
            "BT Recovery L2: Spinning 360 degrees to scan for humans...")

    def update(self) -> Status:
        c = self.ctrl

        if c._teleop_paused or c._recovery_lock:
            return Status.SUCCESS

        if c.available_targets:
            c.get_logger().info(
                "BT Recovery L2: Humans detected during scan — resuming")
            return Status.SUCCESS

        if not self._spin_sent:
            try:
                from nav2_msgs.action import Spin
                from rclpy.action import ActionClient as RclpyActionClient
                if not hasattr(c, '_spin_client'):
                    c._spin_client = RclpyActionClient(c, Spin, 'spin')

                if not c._spin_client.wait_for_server(timeout_sec=2.0):
                    c.get_logger().warn(
                        "BT Recovery L2: Spin server not available — skipping")
                    return Status.FAILURE

                goal = Spin.Goal()
                goal.target_yaw = 6.28
                c._spin_client.send_goal_async(goal)
                self._spin_sent = True
                c.get_logger().info("BT Recovery L2: Spin goal sent (360 deg)")

            except Exception as e:
                c.get_logger().warn(f"BT Recovery L2: Spin error: {e} — skipping")
                return Status.FAILURE

        elapsed = time.time() - self._start_time
        if elapsed > 30.0:
            if c.available_targets:
                return Status.SUCCESS
            c.get_logger().warn(
                "BT Recovery L2: Spin complete, no humans — escalating")
            return Status.FAILURE

        return Status.RUNNING

    def terminate(self, new_status: Status):
        if hasattr(self.ctrl, '_spin_client'):
            try:
                self.ctrl._spin_client.cancel_all_goals_async()
            except Exception:
                pass


class ReturnToLastPosition(Behaviour):
    """
    Recovery L3 — navigate to last successful photo position.
    Skips immediately if no last position stored.
    SUCCESS if humans found. FAILURE on timeout or Nav2 abort.
    """

    def __init__(self, controller):
        super().__init__(name="ReturnToLastPosition")
        self.ctrl = controller
        self._nav_sent = False
        self._start_time = None

    def initialise(self):
        self._nav_sent = False
        self._start_time = time.time()
        self.ctrl.get_logger().warn(
            "BT Recovery L3: Returning to last successful position...")

    def update(self) -> Status:
        c = self.ctrl

        if c._teleop_paused or c._recovery_lock:
            return Status.SUCCESS

        if c.available_targets:
            c.get_logger().info(
                "BT Recovery L3: Humans detected — resuming normal flow")
            return Status.SUCCESS

        if not hasattr(c, '_last_successful_position') or \
                c._last_successful_position is None:
            c.get_logger().warn(
                "BT Recovery L3: No last position stored — skipping")
            return Status.FAILURE

        if not self._nav_sent:
            try:
                from manriix_perception.mission.mission_controller import MissionTarget
                last_pos = c._last_successful_position
                recovery_target = MissionTarget(
                    position=last_pos,
                    orientation=0.0,
                    priority_score=0.0,
                    cluster_id=-99,
                    stability_score=1.0
                )
                c._send_nav_goal(recovery_target)
                self._nav_sent = True
                c.get_logger().info(
                    f"BT Recovery L3: Navigating to "
                    f"({last_pos[0]:.2f}, {last_pos[1]:.2f})")
            except Exception as e:
                c.get_logger().warn(f"BT Recovery L3: Nav error: {e}")
                return Status.FAILURE

        # Nav2 aborted — goal handle gone
        if self._nav_sent and c.current_nav_goal_handle is None:
            if c.available_targets:
                return Status.SUCCESS
            c.get_logger().warn(
                "BT Recovery L3: Nav2 aborted, no humans — escalating")
            return Status.FAILURE

        elapsed = time.time() - self._start_time
        if elapsed > 90.0:
            c.get_logger().warn("BT Recovery L3: Timeout — escalating")
            if c.current_nav_goal_handle is not None:
                c._cancel_navigation()
            return Status.FAILURE

        return Status.RUNNING

    def terminate(self, new_status: Status):
        if self.ctrl.current_nav_goal_handle is not None:
            self.ctrl._cancel_navigation()


class RecoveryIdle(Behaviour):
    """
    Recovery L4 — all options exhausted. Wait indefinitely.
    SUCCESS as soon as humans reappear.
    RUNNING otherwise — operator must intervene or wait.
    """

    def __init__(self, controller):
        super().__init__(name="RecoveryIdle")
        self.ctrl = controller

    def initialise(self):
        self.ctrl.get_logger().warn(
            "BT Recovery L4: All automated recovery exhausted — "
            "waiting for humans or operator intervention")

    def update(self) -> Status:
        c = self.ctrl

        if c._teleop_paused or c._recovery_lock:
            return Status.SUCCESS

        if c.available_targets:
            c.get_logger().info(
                "BT Recovery L4: Humans detected — resuming mission")
            return Status.SUCCESS

        return Status.RUNNING


class RecoverySubtree(Behaviour):
    """
    Orchestrates the 4-level recovery Selector.

    Recovery Selector (OR — first SUCCESS wins):
      L1 WaitForHumans       60s wait in place
      L2 SpinAndScan         360 deg Nav2 spin
      L3 ReturnToLastPos     navigate to last photo position
      L4 RecoveryIdle        wait for operator / new detections

    Edge cases handled:
      - Cancels active Nav2 goal on entry
      - Teleop/recovery-lock exits immediately
      - Nav2 abort detected in L3
      - Spin server unavailable skips L2
      - No last position skips L3
    """

    def __init__(self, controller):
        super().__init__(name="RecoverySubtree")
        self.ctrl = controller
        self._recovery_selector = None

    def initialise(self):
        c = self.ctrl

        # Cancel any active navigation immediately
        if c.current_nav_goal_handle is not None:
            c._cancel_navigation()
            c.get_logger().info("BT Recovery: Cancelled active Nav2 goal")

        # Set recovery state for status publishing
        for state in type(c.current_state):
            if state.name == 'RECOVERY':
                c.current_state = state
                break

        c.current_target = None
        c._reset_retry_state()
        c.get_logger().warn(
            "BT: Entering recovery — L1:Wait → L2:Spin → L3:Return → L4:Idle")

        # Build recovery selector fresh each entry
        self._recovery_selector = Selector(
            name="RecoverySelector", memory=False)
        self._recovery_selector.add_children([
            WaitForHumans(c),
            SpinAndScan(c),
            ReturnToLastPosition(c),
            RecoveryIdle(c),
        ])

    def update(self) -> Status:
        c = self.ctrl

        # Teleop or recovery lock — exit recovery cleanly
        if c._teleop_paused or c._recovery_lock:
            c.get_logger().info("BT Recovery: Interrupted by teleop/lock — exiting")
            return Status.SUCCESS

        # Humans appeared — exit recovery immediately
        if c.available_targets:
            c.get_logger().info(
                "BT Recovery: Humans detected — exiting recovery")
            return Status.SUCCESS

        self._recovery_selector.tick_once()
        sel_status = self._recovery_selector.status

        if sel_status == py_trees.common.Status.SUCCESS:
            c.get_logger().info("BT Recovery: Recovery level succeeded")
            return Status.SUCCESS

        if sel_status == py_trees.common.Status.FAILURE:
            # All 4 levels failed — should not happen since L4 is RUNNING
            # but handle gracefully
            c.get_logger().warn(
                "BT Recovery: All levels failed — resetting for next cycle")
            return Status.SUCCESS

        return Status.RUNNING

    def terminate(self, new_status: Status):
        # Clean up inner tree
        if self._recovery_selector is not None:
            self._recovery_selector.stop(py_trees.common.Status.INVALID)
        # Cancel any nav started by recovery
        if self.ctrl.current_nav_goal_handle is not None:
            self.ctrl._cancel_navigation()


# ─────────────────────────────────────────────────────────────────────────────
# Tree factory
# ─────────────────────────────────────────────────────────────────────────────

def build_mission_tree(controller) -> py_trees.trees.BehaviourTree:
    """
    Build and return the Mission BT.

    Tree structure:
        Repeat(forever)
          Sequence (AND)
            EvaluateTarget     condition — gate
            Selector (OR)      navigate with retry
              NavigateToPOI
              Sequence (AND)
                TryNextPOI
                RecoverySubtree
            WaitForStability
            TakePhoto
    """
    evaluate  = EvaluateTarget(controller)
    navigate  = NavigateToPOI(controller)
    stabilise = WaitForStability(controller)
    photo     = TakePhotoAction(controller)
    try_next  = TryNextPOI(controller)
    recovery  = RecoverySubtree(controller)

    nav_retry = Sequence(name="NavRetry", memory=False)
    nav_retry.add_children([try_next, recovery])

    nav_fallback = Selector(name="NavFallback", memory=False)
    nav_fallback.add_children([navigate, nav_retry])

    mission_sequence = Sequence(name="MissionSequence", memory=False)
    mission_sequence.add_children([evaluate, nav_fallback, stabilise, photo])

    root = py_trees.decorators.Repeat(
        child=mission_sequence,
        name="MissionRoot",
        num_success=-1
    )

    tree = py_trees.trees.BehaviourTree(root=root)
    return tree
