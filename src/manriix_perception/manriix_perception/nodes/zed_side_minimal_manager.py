#!/usr/bin/env python3

import json
import os
import signal
import subprocess
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import Image
from std_msgs.msg import String


class Manager(Node):
    """
    Stage-2 MANRIIX side-camera manager.

    Architecture:
        FRONT:
            - Owned by the normal production ZED driver
            - Full navigation workload
            - cuVSLAM + nvblox

        LEFT:
            - Minimal capture only
            - Depth OFF
            - Tracking OFF
            - ZED OD OFF

        RIGHT:
            - Minimal capture only
            - Depth OFF
            - Tracking OFF
            - ZED OD OFF

    Startup sequence:
        1. Wait for front monitoring stream + cuVSLAM odometry.
        2. Require front to remain healthy for 5 seconds.
        3. Start LEFT.
        4. Require LEFT stream to remain healthy for 3 seconds.
        5. Start RIGHT.

    Shutdown:
        - Every side worker is started in its own process group.
        - Ctrl+C shuts down the complete process group:
            SIGINT -> wait
            SIGTERM -> wait
            SIGKILL -> final fallback
        - This prevents stale ros2-run wrapper / Python ZED processes from
          keeping a ZED X camera locked after shutdown.
    """

    def __init__(self):
        super().__init__("zed_side_minimal_manager")

        # ------------------------------------------------------------
        # Parameters
        # ------------------------------------------------------------
        self.declare_parameter("front_stable_sec", 5.0)
        self.declare_parameter("left_stable_sec", 3.0)
        self.declare_parameter("freshness_timeout_sec", 2.0)

        self.front_stable_sec = float(
            self.get_parameter("front_stable_sec").value
        )
        self.left_stable_sec = float(
            self.get_parameter("left_stable_sec").value
        )
        self.freshness_timeout_sec = float(
            self.get_parameter("freshness_timeout_sec").value
        )

        # ------------------------------------------------------------
        # Health timestamps
        # ------------------------------------------------------------
        self.last_stream = {
            "front": None,
            "left": None,
            "right": None,
        }

        self.last_odom = None

        self.front_healthy_since = None
        self.left_healthy_since = None

        # ------------------------------------------------------------
        # Side worker process handles
        # ------------------------------------------------------------
        self.processes = {
            "left": None,
            "right": None,
        }

        # Only attempt each camera once during Stage 2.
        # This intentionally avoids repeated SDK open attempts if a camera
        # fails, because repeated failures can destabilize zed_x_daemon.
        self.attempted = {
            "left": False,
            "right": False,
        }

        # ------------------------------------------------------------
        # ROS QoS
        # ------------------------------------------------------------
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # ------------------------------------------------------------
        # Monitor image subscriptions
        # ------------------------------------------------------------
        for camera in ("front", "left", "right"):
            self.create_subscription(
                Image,
                f"/monitor/zedx_{camera}/image_raw",
                lambda msg, cam=camera: self._image_callback(cam),
                sensor_qos,
            )

        # ------------------------------------------------------------
        # cuVSLAM odometry heartbeat
        # ------------------------------------------------------------
        self.create_subscription(
            Odometry,
            "/visual_slam/tracking/odometry",
            self._odom_callback,
            sensor_qos,
        )

        # ------------------------------------------------------------
        # Status publisher
        # ------------------------------------------------------------
        self.status_pub = self.create_publisher(
            String,
            "/manriix/zed_stage2_status",
            10,
        )

        # Health/startup state machine
        self.create_timer(0.5, self._tick)

        self.get_logger().info(
            "Stage2 manager active: "
            "wait FRONT healthy -> LEFT minimal -> RIGHT minimal"
        )

    # ----------------------------------------------------------------
    # ROS callbacks
    # ----------------------------------------------------------------
    def _image_callback(self, camera):
        self.last_stream[camera] = time.monotonic()

    def _odom_callback(self, msg):
        self.last_odom = time.monotonic()

    # ----------------------------------------------------------------
    # Health utilities
    # ----------------------------------------------------------------
    def _is_fresh(self, timestamp):
        if timestamp is None:
            return False

        return (
            time.monotonic() - timestamp
            <= self.freshness_timeout_sec
        )

    # ----------------------------------------------------------------
    # Build side-camera command
    # ----------------------------------------------------------------
    def _camera_command(self, camera):
        serials = {
            "left": 40487925,
            "right": 46311875,
        }

        serial = serials[camera]

        return [
            "ros2",
            "run",
            "manriix_perception",
            "zed_minimal_capture_node.py",

            "--ros-args",

            "-r",
            f"__node:=zedx_{camera}_minimal_capture",

            "-p",
            f"camera_name:=zedx_{camera}",

            "-p",
            f"serial_number:={serial}",

            "-p",
            "fps:=15",

            "-p",
            "resolution:=SVGA",

            "-p",
            "output_rate_hz:=5.0",

            "-p",
            "output_width:=640",
        ]

    # ----------------------------------------------------------------
    # Spawn one side-camera worker
    # ----------------------------------------------------------------
    def _spawn_camera(self, camera):
        if self.attempted[camera]:
            return

        self.attempted[camera] = True

        env = os.environ.copy()

        # ZED X / EGL should use the Jetson's local display rather than
        # an SSH-forwarded display.
        env["DISPLAY"] = ":0"
        env["XAUTHORITY"] = "/run/user/1000/gdm/Xauthority"

        self.get_logger().warning(
            f"Starting {camera.upper()} minimal capture"
        )

        try:
            process = subprocess.Popen(
                self._camera_command(camera),

                env=env,

                # Allow ZED SDK logs to appear in the same terminal.
                stdout=None,
                stderr=None,

                # CRITICAL:
                # Creates a separate process group/session so that shutdown
                # can terminate both:
                #   ros2 run wrapper
                #   actual Python ZED node
                start_new_session=True,
            )

            self.processes[camera] = process

            self.get_logger().info(
                f"{camera.upper()} minimal capture started "
                f"with launcher PID={process.pid}"
            )

        except Exception as exc:
            self.get_logger().error(
                f"Failed to start {camera.upper()} minimal capture: {exc}"
            )

    # ----------------------------------------------------------------
    # Gracefully terminate an entire side-camera process group
    # ----------------------------------------------------------------
    def _stop_camera(self, camera):
        process = self.processes.get(camera)

        if process is None:
            return

        # Already exited
        if process.poll() is not None:
            self.get_logger().info(
                f"{camera.upper()} process already exited "
                f"with return code {process.returncode}"
            )
            self.processes[camera] = None
            return

        try:
            pgid = os.getpgid(process.pid)
        except ProcessLookupError:
            self.processes[camera] = None
            return

        self.get_logger().info(
            f"Stopping {camera.upper()} minimal capture "
            f"(PID={process.pid}, PGID={pgid})"
        )

        # ------------------------------------------------------------
        # Stage 1: SIGINT
        # Gives rclpy + ZED SDK a chance to close cleanly.
        # ------------------------------------------------------------
        try:
            os.killpg(pgid, signal.SIGINT)

            process.wait(timeout=5.0)

            self.get_logger().info(
                f"{camera.upper()} stopped cleanly with SIGINT"
            )

            self.processes[camera] = None
            return

        except subprocess.TimeoutExpired:
            self.get_logger().warning(
                f"{camera.upper()} did not exit after SIGINT"
            )

        except ProcessLookupError:
            self.processes[camera] = None
            return

        # ------------------------------------------------------------
        # Stage 2: SIGTERM
        # ------------------------------------------------------------
        try:
            os.killpg(pgid, signal.SIGTERM)

            process.wait(timeout=3.0)

            self.get_logger().warning(
                f"{camera.upper()} required SIGTERM to stop"
            )

            self.processes[camera] = None
            return

        except subprocess.TimeoutExpired:
            self.get_logger().error(
                f"{camera.upper()} did not exit after SIGTERM"
            )

        except ProcessLookupError:
            self.processes[camera] = None
            return

        # ------------------------------------------------------------
        # Stage 3: SIGKILL
        # Last resort only.
        # ------------------------------------------------------------
        try:
            os.killpg(pgid, signal.SIGKILL)

            process.wait(timeout=2.0)

            self.get_logger().error(
                f"{camera.upper()} required SIGKILL"
            )

        except Exception as exc:
            self.get_logger().error(
                f"Final shutdown error for {camera.upper()}: {exc}"
            )

        self.processes[camera] = None

    # ----------------------------------------------------------------
    # Main startup/health state machine
    # ----------------------------------------------------------------
    def _tick(self):
        now = time.monotonic()

        # ============================================================
        # FRONT HEALTH
        # ============================================================
        front_stream_fresh = self._is_fresh(
            self.last_stream["front"]
        )

        cuvslam_fresh = self._is_fresh(
            self.last_odom
        )

        front_ready = (
            front_stream_fresh
            and cuvslam_fresh
        )

        if front_ready:
            if self.front_healthy_since is None:
                self.front_healthy_since = now
        else:
            self.front_healthy_since = None

        # ============================================================
        # START LEFT
        # ============================================================
        if (
            not self.attempted["left"]
            and self.front_healthy_since is not None
            and (
                now - self.front_healthy_since
                >= self.front_stable_sec
            )
        ):
            self._spawn_camera("left")

        # ============================================================
        # LEFT HEALTH
        # ============================================================
        left_stream_fresh = self._is_fresh(
            self.last_stream["left"]
        )

        left_process = self.processes["left"]

        left_process_alive = (
            left_process is not None
            and left_process.poll() is None
        )

        if (
            left_process_alive
            and left_stream_fresh
        ):
            if self.left_healthy_since is None:
                self.left_healthy_since = now
        else:
            self.left_healthy_since = None

        # ============================================================
        # START RIGHT
        # ============================================================
        if (
            not self.attempted["right"]
            and left_process_alive
            and self.left_healthy_since is not None
            and (
                now - self.left_healthy_since
                >= self.left_stable_sec
            )
        ):
            self._spawn_camera("right")

        # ============================================================
        # STATUS
        # ============================================================
        status = {
            "front": {
                "stream_fresh": front_stream_fresh,
                "cuvslam_fresh": cuvslam_fresh,
                "ready": front_ready,
            }
        }

        for camera in ("left", "right"):
            process = self.processes[camera]

            process_alive = (
                process is not None
                and process.poll() is None
            )

            return_code = (
                process.poll()
                if process is not None
                else None
            )

            status[camera] = {
                "attempted": self.attempted[camera],
                "stream_fresh": self._is_fresh(
                    self.last_stream[camera]
                ),
                "process_alive": process_alive,
                "pid": (
                    process.pid
                    if process is not None
                    else None
                ),
                "return_code": return_code,
            }

        message = String()
        message.data = json.dumps(status)

        self.status_pub.publish(message)

    # ----------------------------------------------------------------
    # Node shutdown
    # ----------------------------------------------------------------
    def destroy_node(self):
        self.get_logger().info(
            "Stage2 manager shutting down side cameras..."
        )

        # Stop RIGHT first because it was opened last.
        self._stop_camera("right")

        # Then LEFT.
        self._stop_camera("left")

        self.get_logger().info(
            "Stage2 side-camera shutdown complete"
        )

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = Manager()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        node.get_logger().info(
            "Ctrl+C received"
        )

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()