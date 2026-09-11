#!/usr/bin/env python3
import time
import numpy as np
import py_trees
from py_trees.behaviour import Behaviour
from py_trees.common import Status
from py_trees.composites import Sequence, Selector

class EvaluateTarget(Behaviour):
    def __init__(self, controller):
        super().__init__(name="EvaluateTarget")
        self.ctrl = controller
        self._no_target_since = None          # patience timer — was missing, caused AttributeError

        # Blackboard client — READ shared state, WRITE current_target + recently_photographed
        self._bb = py_trees.blackboard.Client(name="EvaluateTarget")
        self._bb.register_key("teleop_paused",           access=py_trees.common.Access.READ)
        self._bb.register_key("recovery_lock",           access=py_trees.common.Access.READ)
        self._bb.register_key("recovery_cooldown_until", access=py_trees.common.Access.READ)
        self._bb.register_key("camera_active",           access=py_trees.common.Access.READ)
        self._bb.register_key("available_targets",       access=py_trees.common.Access.READ)
        self._bb.register_key("robot_position",          access=py_trees.common.Access.READ)
        self._bb.register_key("current_target",          access=py_trees.common.Access.WRITE)
        self._bb.register_key("recently_photographed",   access=py_trees.common.Access.WRITE)    

    def update(self) -> Status:
        c = self.ctrl
        bb = self._bb

        # ── Read guards from blackboard ────────────────────────────
        if bb.teleop_paused:
            return Status.FAILURE
        if bb.recovery_lock:
            return Status.FAILURE
        if time.time() < bb.recovery_cooldown_until:
            return Status.FAILURE
        if bb.camera_active:
            return Status.FAILURE
        if not c.enable_autonomous_navigation:
            return Status.FAILURE

        available = bb.available_targets
        current   = c.current_target     # still read from ctrl — written below

        # ── No targets — patience timer ────────────────────────────
        if not available and current is None:
            if self._no_target_since is None:
                self._no_target_since = time.time()
                c.get_logger().info("BT: No targets — starting 120s patience timer")

            elapsed = time.time() - self._no_target_since
            if elapsed >= 120.0:
                c.get_logger().warn("BT: No targets for 120s — escalating to recovery")
                self._no_target_since = None
                return Status.FAILURE

            return Status.RUNNING

        # Targets exist or current_target set — reset patience timer
        self._no_target_since = None

        # ── Current target cooldown check ──────────────────────────
        if current is not None:
            now = time.time()
            recently = c._recently_photographed if hasattr(c, '_recently_photographed') else {}
            recently = {k: v for k, v in recently.items() if now - v < 120.0}
            c._recently_photographed = recently
            bb.recently_photographed = recently        # sync blackboard

            cur_pos = np.array([current.position[0], current.position[1]])
            for (rx, ry), shot_time in recently.items():
                if now - shot_time < 120.0:
                    dist = np.linalg.norm(cur_pos - np.array([rx, ry]))
                    if dist < 1.5:
                        remaining = 120.0 - (now - shot_time)
                        c.get_logger().info(
                            f"BT: Current target within cooldown zone "
                            f"({remaining:.0f}s) — clearing")
                        c.current_target = None
                        bb.current_target = None
                        return Status.FAILURE

        # ── Select best target if none set ─────────────────────────
        if current is None:
            best = max(available, key=lambda t: t.priority_score)

            robot_pos = bb.robot_position
            if robot_pos is not None:
                dist = np.linalg.norm(best.position - robot_pos)
                if dist < 0.8:
                    c.get_logger().info(f"BT: POI {dist:.2f}m away — already in position")
                    c.current_target = best
                    bb.current_target = best          # sync blackboard
                    c.metrics.targets_evaluated += 1
                    return Status.SUCCESS

            # Cooldown check on best candidate
            now = time.time()
            recently = c._recently_photographed if hasattr(c, '_recently_photographed') else {}
            recently = {k: v for k, v in recently.items() if now - v < 120.0}
            c._recently_photographed = recently
            bb.recently_photographed = recently       # sync blackboard

            # Distance-based cooldown — handles POI position drift
            best_pos = np.array([best.position[0], best.position[1]])
            on_cooldown = False
            for (rx, ry), shot_time in recently.items():
                if now - shot_time < 120.0:
                    dist = np.linalg.norm(best_pos - np.array([rx, ry]))
                    if dist < 1.5:  # 1.5m radius cooldown zone
                        remaining = 120.0 - (now - shot_time)
                        c.get_logger().debug(
                            f"BT: Target {best_pos} within 1.5m of cooldown "
                            f"zone ({rx},{ry}) — {remaining:.0f}s remaining")
                        on_cooldown = True
                        break
            if on_cooldown:
                return Status.FAILURE

            c.current_target = best
            bb.current_target = best                  # sync blackboard
            c.metrics.targets_evaluated += 1
            c.get_logger().info(
                f"BT: Target selected ({best.position[0]:.2f}, "
                f"{best.position[1]:.2f}) score={best.priority_score:.2f}")

        return Status.SUCCESS

class WaitForStability(Behaviour):
    """
    Action — wait until robot is stable at the POI.
    SUCCESS when stable, SUCCESS on 30s timeout, FAILURE if no position.
    """
    def __init__(self, controller):
        super().__init__(name="WaitForStability")
        self.ctrl = controller
        self._start_time = None

        # Blackboard client — READ robot_position only
        self._bb = py_trees.blackboard.Client(name="WaitForStability")
        self._bb.register_key("robot_position", access=py_trees.common.Access.READ)

    def initialise(self):
        self._start_time = time.time()
        self.ctrl.get_logger().info("BT: Waiting for robot stability...")

    def update(self) -> Status:
        c = self.ctrl

        if self._bb.robot_position is None:
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

        # Blackboard client — READ camera_active
        self._bb = py_trees.blackboard.Client(name="TakePhoto")
        self._bb.register_key("camera_active", access=py_trees.common.Access.READ)

    def initialise(self):
        self._triggered = False

    def update(self) -> Status:
        c = self.ctrl

        if not self._triggered:
            c.get_logger().info("BT: Triggering TakePhoto action...")
            c.transition_to_photo_sequence()
            self._triggered = True
            return Status.RUNNING

        # ── Read camera_active from blackboard ─────────────────────
        # Written by: photo_command_callback (RC path)
        #             _take_photo_result_callback (action path)
        if not self._bb.camera_active and self._triggered:
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
            self._bb.camera_active = False            # sync blackboard on forced exit

class NavigateMultiPOI(Behaviour):
    """
    Action — navigate through prioritized POIs until one is reached.
    Internally loops through available_targets (by priority) before
    returning FAILURE. Recovery is only triggered once all are exhausted
    or max_poi_retries is reached.

    RUNNING  — navigating to current attempt
    SUCCESS  — arrived at a POI
    FAILURE  — all POIs tried (or max_poi_retries hit), none reachable
    """

    def __init__(self, controller, max_poi_retries: int = 5):
        super().__init__(name="NavigateMultiPOI")
        self.ctrl = controller
        self.max_poi_retries = max_poi_retries
        self._attempt = 0
        self._goal_sent = False
        self._nav_start_time = None

        # Blackboard client — READ position + current_target
        self._bb = py_trees.blackboard.Client(name="NavigateMultiPOI")
        self._bb.register_key("robot_position",  access=py_trees.common.Access.READ)
        self._bb.register_key("current_target",  access=py_trees.common.Access.READ)

    def initialise(self):
        self._attempt = 0
        self._goal_sent = False
        self._nav_start_time = None

    def update(self) -> Status:
        c = self.ctrl
        bb = self._bb

        # ── Read from blackboard ───────────────────────────────────
        current_target = bb.current_target
        robot_position = bb.robot_position

        if current_target is None or robot_position is None:
            return Status.FAILURE

        # ── Arrival check ──────────────────────────────────────────
        distance = np.linalg.norm(robot_position - current_target.position)
        if distance <= c.arrival_threshold:
            c.get_logger().info(
                f"BT MultiPOI: Arrived at attempt #{self._attempt + 1} "
                f"({distance:.2f}m) — SUCCESS")
            if c.current_nav_goal_handle is not None:
                c._cancel_navigation()
            c._reset_retry_state()
            return Status.SUCCESS

        # ── Send goal for current target ────────────────────────────
        if not self._goal_sent:
            c._send_nav_goal(current_target)
            self._goal_sent = True
            self._nav_start_time = time.time()
            c.get_logger().info(
                f"BT MultiPOI: Attempt #{self._attempt + 1}/{self.max_poi_retries} "
                f"→ POI ({current_target.position[0]:.2f}, "
                f"{current_target.position[1]:.2f})")
            return Status.RUNNING

        # ── Timeout / progress check ───────────────────────────────
        nav_elapsed = time.time() - self._nav_start_time
        nav_failed = False

        if nav_elapsed > c.absolute_nav_timeout:
            c.get_logger().warn(
                f"BT MultiPOI: Attempt #{self._attempt + 1} absolute timeout")
            nav_failed = True
        elif nav_elapsed > c.per_goal_timeout:
            if not c._is_making_progress():
                c.get_logger().warn(
                    f"BT MultiPOI: Attempt #{self._attempt + 1} no progress "
                    f"({distance:.1f}m remaining)")
                nav_failed = True
            else:
                self._nav_start_time = time.time()

        # ── On failure — try next POI ──────────────────────────────
        if nav_failed:
            c._cancel_navigation()
            c._mark_position_failed(current_target.position)
            self._attempt += 1

            if self._attempt >= self.max_poi_retries:
                c.get_logger().warn(
                    f"BT MultiPOI: Max retries ({self.max_poi_retries}) reached "
                    f"— handing to recovery")
                return Status.FAILURE

            if not c._try_next_poi():
                c.get_logger().warn(
                    "BT MultiPOI: No more POIs available — handing to recovery")
                return Status.FAILURE

            self._goal_sent = False
            self._nav_start_time = None
            # Read updated target from ctrl (just set by _try_next_poi)
            next_target = c.current_target
            c.get_logger().info(
                f"BT MultiPOI: Switching to next POI "
                f"({next_target.position[0]:.2f}, "
                f"{next_target.position[1]:.2f})")
            return Status.RUNNING

        return Status.RUNNING

    def terminate(self, new_status: Status):
        if new_status == Status.FAILURE:
            if self.ctrl.current_nav_goal_handle is not None:
                self.ctrl._cancel_navigation()
            # Reset retry state so next BT cycle starts fresh.
            # Without this, _retry_active stays True after recovery exits
            # and blocks optimal_positions_callback from delivering new POIs.
            self.ctrl._reset_retry_state()

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

        self._bb = py_trees.blackboard.Client(name="WaitForHumans")
        self._bb.register_key("teleop_paused",     access=py_trees.common.Access.READ)
        self._bb.register_key("recovery_lock",     access=py_trees.common.Access.READ)
        self._bb.register_key("available_targets", access=py_trees.common.Access.READ)

    def initialise(self):
        self._start_time = time.time()
        self.ctrl.get_logger().warn(
            "BT Recovery L1: Waiting 60s for humans to appear...")

    def update(self) -> Status:
        c = self.ctrl

        if self._bb.teleop_paused or self._bb.recovery_lock:
            return Status.SUCCESS

        if self._bb.available_targets:
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

        self._bb = py_trees.blackboard.Client(name="SpinAndScan")
        self._bb.register_key("teleop_paused",     access=py_trees.common.Access.READ)
        self._bb.register_key("recovery_lock",     access=py_trees.common.Access.READ)
        self._bb.register_key("available_targets", access=py_trees.common.Access.READ)

    def initialise(self):
        self._spin_sent = False
        self._start_time = time.time()
        self.ctrl.get_logger().warn(
            "BT Recovery L2: Spinning 360 degrees to scan for humans...")

    def update(self) -> Status:
        c = self.ctrl

        if self._bb.teleop_paused or self._bb.recovery_lock:
            return Status.SUCCESS

        if self._bb.available_targets:
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
            if self._bb.available_targets:
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

        self._bb = py_trees.blackboard.Client(name="ReturnToLastPosition")
        self._bb.register_key("teleop_paused",           access=py_trees.common.Access.READ)
        self._bb.register_key("recovery_lock",           access=py_trees.common.Access.READ)
        self._bb.register_key("available_targets",       access=py_trees.common.Access.READ)
        self._bb.register_key("last_successful_position",access=py_trees.common.Access.READ)

    def initialise(self):
        self._nav_sent = False
        self._start_time = time.time()
        self.ctrl.get_logger().warn(
            "BT Recovery L3: Returning to last successful position...")

    def update(self) -> Status:
        c = self.ctrl

        if self._bb.teleop_paused or self._bb.recovery_lock:
            return Status.SUCCESS

        if self._bb.available_targets:
            c.get_logger().info(
                "BT Recovery L3: Humans detected — resuming normal flow")
            return Status.SUCCESS

        if self._bb.last_successful_position is None:
            c.get_logger().warn(
                "BT Recovery L3: No last position stored — skipping")
            return Status.FAILURE

        if not self._nav_sent:
            try:
                from manriix_perception.mission.mission_controller import MissionTarget
                last_pos = self._bb.last_successful_position
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
            if self._bb.available_targets:
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

        self._bb = py_trees.blackboard.Client(name="RecoveryIdle")
        self._bb.register_key("teleop_paused",     access=py_trees.common.Access.READ)
        self._bb.register_key("recovery_lock",     access=py_trees.common.Access.READ)
        self._bb.register_key("available_targets", access=py_trees.common.Access.READ)

    def initialise(self):
        self.ctrl.get_logger().warn(
            "BT Recovery L4: All automated recovery exhausted — "
            "waiting for humans or operator intervention")

    def update(self) -> Status:
        c = self.ctrl

        if self._bb.teleop_paused or self._bb.recovery_lock:
            return Status.SUCCESS

        if self._bb.available_targets:
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

        self._bb = py_trees.blackboard.Client(name="RecoverySubtree")
        self._bb.register_key("teleop_paused",     access=py_trees.common.Access.READ)
        self._bb.register_key("recovery_lock",     access=py_trees.common.Access.READ)
        self._bb.register_key("available_targets", access=py_trees.common.Access.READ)

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

        if self._bb.teleop_paused or self._bb.recovery_lock:
            c.get_logger().info("BT Recovery: Interrupted by teleop/lock — exiting")
            return Status.SUCCESS

        if self._bb.available_targets:
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

def build_mission_tree(controller) -> py_trees.trees.BehaviourTree:
    """
    Build and return the Mission BT.

    Tree structure:
        Repeat(forever)
          Sequence (AND)
            EvaluateTarget       condition — gate
                                 RUNNING (60s patience) when no targets,
                                 then FAILURE → recovery
            Selector (OR)        navigate with POI retry, then recovery
              NavigateMultiPOI   tries up to max_poi_retries POIs internally
              RecoverySubtree    fires only when all POIs exhausted
            WaitForStability
            TakePhoto
    """
    evaluate  = EvaluateTarget(controller)
    navigate  = NavigateMultiPOI(controller, max_poi_retries=5)
    stabilise = WaitForStability(controller)
    photo     = TakePhotoAction(controller)
    recovery  = RecoverySubtree(controller)

    nav_or_recover = Selector(name="NavOrRecover", memory=False)
    nav_or_recover.add_children([navigate])

    mission_sequence = Sequence(name="MissionSequence", memory=False)
    mission_sequence.add_children([evaluate, nav_or_recover, stabilise, photo])

    mission_or_recover = Selector(name="MissionOrRecover", memory=False)
    mission_or_recover.add_children([mission_sequence, recovery])

    root = py_trees.decorators.Repeat(
        child=mission_or_recover,
        name="MissionRoot",
        num_success=-1
    )

    tree = py_trees.trees.BehaviourTree(root=root)
    return tree