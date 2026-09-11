#!/usr/bin/env python3
"""
MANRIIX health-gated side ZED supervisor.

Critical rule:
  FRONT is launched externally by the main perceptor launch and is NEVER
  restarted here.

Sequence:
  1) Wait until front monitoring stream is fresh AND cuVSLAM odometry is fresh.
  2) Require that condition continuously for front_stable_sec.
  3) Spawn LEFT detection worker.
  4) Wait until LEFT monitoring stream is fresh for side_stable_sec.
  5) Spawn RIGHT detection worker.
  6) Monitor LEFT/RIGHT and restart only a failed side worker.

This avoids:
  - starting all three ZED SDK instances simultaneously;
  - ROS launch TimerAction / OnProcessStart issues;
  - coupling side-camera failures to front localization.
"""

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


class HealthGatedZedSupervisor(Node):
    def __init__(self):
        super().__init__("zed_side_worker_supervisor")

        self.declare_parameter("freshness_timeout_sec", 2.0)
        self.declare_parameter("front_stable_sec", 5.0)
        self.declare_parameter("side_stable_sec", 3.0)
        self.declare_parameter("restart_cooldown_sec", 15.0)
        self.declare_parameter("enable_side_autorestart", True)

        self.freshness_timeout = float(
            self.get_parameter("freshness_timeout_sec").value
        )
        self.front_stable_sec = float(
            self.get_parameter("front_stable_sec").value
        )
        self.side_stable_sec = float(
            self.get_parameter("side_stable_sec").value
        )
        self.restart_cooldown = float(
            self.get_parameter("restart_cooldown_sec").value
        )
        self.enable_side_autorestart = bool(
            self.get_parameter("enable_side_autorestart").value
        )

        self.last_stream = {"front": None, "left": None, "right": None}
        self.last_odom = None

        self.front_good_since = None
        self.left_good_since = None

        self.children = {"left": None, "right": None}
        self.last_restart = {"left": 0.0, "right": 0.0}
        self.last_reported_exit = {
            "left": None,
            "right": None,
        }

        self.left_started = False
        self.right_started = False

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        for camera in ("front", "left", "right"):
            self.create_subscription(
                Image,
                f"/monitor/zedx_{camera}/image_raw",
                lambda msg, cam=camera: self._stream_cb(cam, msg),
                sensor_qos,
            )

        self.create_subscription(
            Odometry,
            "/visual_slam/tracking/odometry",
            self._odom_cb,
            sensor_qos,
        )

        self.status_pub = self.create_publisher(
            String,
            "/manriix/zed_supervisor_status",
            10,
        )

        self.create_timer(0.5, self._tick)

        self.get_logger().info(
            "Health-gated ZED supervisor started: "
            "front -> left -> right; front is never auto-restarted."
        )

    def _stream_cb(self, camera, msg):
        self.last_stream[camera] = time.monotonic()

    def _odom_cb(self, msg):
        self.last_odom = time.monotonic()

    def _fresh(self, stamp):
        return (
            stamp is not None
            and (time.monotonic() - stamp) <= self.freshness_timeout
        )

    def _side_command(self, camera):
        node_name = f"zedx_{camera}_detection_driver"
        enabled = f"zedx_{camera}"

        return [
            "ros2", "run", "manriix_perception",
            "direct_zed_detection_node_modified.py",
            "--ros-args",
            "-r", f"__node:={node_name}",
            "-p", "mode:=full",
            "-p", f"enabled_cameras:={enabled}",
            "-p", "camera_front_serial:=41911351",
            "-p", "camera_left_serial:=40487925",
            "-p", "camera_right_serial:=46311875",
            "-p", "resolution:=SVGA",
            "-p", "fps:=15",
            "-p", "front_depth_mode:=NEURAL_LIGHT",
            "-p", "left_depth_mode:=NONE",
            "-p", "right_depth_mode:=NONE",
            "-p", "publish_rate:=15.0",
            "-p", "publish_pointcloud:=False",
            "-p", "publish_compressed_monitoring:=False",
            "-p", "publish_detection_images:=False",
            "-p", "publish_monitoring_thumbnails:=True",
            "-p", "monitoring_publish_divisor:=5",
            "-p", "monitoring_width:=640",
            "-p", "imu_rate_hz:=100.0",
        ]

    def _spawn_side(self, camera, reason):
        existing = self.children.get(camera)
        if existing is not None and existing.poll() is None:
            return

        self.get_logger().warning(
            f"{camera.upper()}: spawning detection worker ({reason})"
        )

        try:
            self.children[camera] = subprocess.Popen(
                self._side_command(camera),
                # IMPORTANT: inherit stdout/stderr so ZED SDK / Argus /
                # Python failures are visible in the parent launch terminal.
                stdout=None,
                stderr=None,
                start_new_session=False,
            )
            self.last_restart[camera] = time.monotonic()

            self.get_logger().info(
                f"{camera.upper()}: worker spawned "
                f"(pid={self.children[camera].pid})"
            )
        except Exception as exc:
            self.get_logger().error(
                f"{camera.upper()}: spawn failed: {exc}"
            )

    def _stop_side(self, camera):
        proc = self.children.get(camera)
        if proc is None or proc.poll() is not None:
            self.children[camera] = None
            return

        self.get_logger().warning(
            f"{camera.upper()}: terminating stale worker before restart"
        )
        try:
            proc.send_signal(signal.SIGINT)
            proc.wait(timeout=5)
        except Exception:
            try:
                proc.terminate()
                proc.wait(timeout=3)
            except Exception:
                try:
                    proc.kill()
                except Exception:
                    pass

        self.children[camera] = None

    def _report_child_exit(self, camera):
        proc = self.children.get(camera)
        if proc is None:
            return

        code = proc.poll()
        if code is None:
            return

        if self.last_reported_exit[camera] != code:
            self.get_logger().error(
                f"{camera.upper()}: worker EXITED "
                f"(pid={proc.pid}, return_code={code})"
            )
            self.last_reported_exit[camera] = code

    def _restart_side_if_needed(self, camera):
        if not self.enable_side_autorestart:
            return

        now = time.monotonic()
        if now - self.last_restart[camera] < self.restart_cooldown:
            return

        proc = self.children.get(camera)
        process_dead = proc is None or proc.poll() is not None
        stream_dead = not self._fresh(self.last_stream[camera])

        if process_dead or stream_dead:
            self._stop_side(camera)
            self._spawn_side(
                camera,
                "process dead" if process_dead else "camera stream stale",
            )

    def _tick(self):
        now = time.monotonic()

        self._report_child_exit("left")
        self._report_child_exit("right")

        front_stream_ok = self._fresh(self.last_stream["front"])
        odom_ok = self._fresh(self.last_odom)
        front_ready = front_stream_ok and odom_ok

        if front_ready:
            if self.front_good_since is None:
                self.front_good_since = now
        else:
            self.front_good_since = None

        # Start LEFT only after front localization has been continuously healthy.
        if (
            not self.left_started
            and self.front_good_since is not None
            and (now - self.front_good_since) >= self.front_stable_sec
        ):
            self._spawn_side("left", "front localization stable")
            self.left_started = True

        left_stream_ok = self._fresh(self.last_stream["left"])

        if self.left_started and left_stream_ok:
            if self.left_good_since is None:
                self.left_good_since = now
        else:
            self.left_good_since = None

        # Start RIGHT only after LEFT itself is continuously healthy.
        if (
            self.left_started
            and not self.right_started
            and self.left_good_since is not None
            and (now - self.left_good_since) >= self.side_stable_sec
        ):
            self._spawn_side("right", "left camera stable")
            self.right_started = True

        if self.left_started:
            self._restart_side_if_needed("left")
        if self.right_started:
            self._restart_side_if_needed("right")

        status = {
            "front": {
                "stream_fresh": front_stream_ok,
                "cuvslam_odom_fresh": odom_ok,
                "ready": front_ready,
            },
            "left": {
                "started": self.left_started,
                "stream_fresh": left_stream_ok,
                "process_alive": (
                    self.children["left"] is not None
                    and self.children["left"].poll() is None
                ),
                "pid": (
                    self.children["left"].pid
                    if self.children["left"] is not None
                    else None
                ),
                "return_code": (
                    self.children["left"].poll()
                    if self.children["left"] is not None
                    else None
                ),
            },
            "right": {
                "started": self.right_started,
                "stream_fresh": self._fresh(self.last_stream["right"]),
                "process_alive": (
                    self.children["right"] is not None
                    and self.children["right"].poll() is None
                ),
                "pid": (
                    self.children["right"].pid
                    if self.children["right"] is not None
                    else None
                ),
                "return_code": (
                    self.children["right"].poll()
                    if self.children["right"] is not None
                    else None
                ),
            },
        }

        msg = String()
        msg.data = json.dumps(status)
        self.status_pub.publish(msg)

    def destroy_node(self):
        for camera in ("left", "right"):
            self._stop_side(camera)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = HealthGatedZedSupervisor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()