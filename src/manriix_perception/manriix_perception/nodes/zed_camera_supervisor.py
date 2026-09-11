#!/usr/bin/env python3
import json
import subprocess
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
    HistoryPolicy,
)


class ZedCameraSupervisor(Node):
    def __init__(self):
        super().__init__("zed_camera_supervisor")

        self.declare_parameter("freshness_timeout_sec", 2.0)
        self.declare_parameter("startup_grace_sec", 25.0)
        self.declare_parameter("restart_cooldown_sec", 20.0)
        # self.declare_parameter("enable_side_autorestart", True)
        self.declare_parameter("enable_side_autorestart", False)

        self.freshness_timeout = float(
            self.get_parameter("freshness_timeout_sec").value
        )
        self.startup_grace = float(
            self.get_parameter("startup_grace_sec").value
        )
        self.restart_cooldown = float(
            self.get_parameter("restart_cooldown_sec").value
        )
        self.enable_side_autorestart = bool(
            self.get_parameter("enable_side_autorestart").value
        )

        self.started = time.monotonic()
        self.last_detection_seen = {
            "front": None,
            "left": None,
            "right": None,
        }
        self.last_stream_seen = {
            "front": None,
            "left": None,
            "right": None,
        }
        self.last_restart = {"left": 0.0, "right": 0.0}

        self.create_subscription(
            String,
            "/web/camera_detections",
            self._detection_cb,
            20,
        )

        self.monitor_qos = QoSProfile(
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
                self.monitor_qos,
            )

        self.status_pub = self.create_publisher(
            String,
            "/manriix/zed_camera_health",
            10,
        )

        self.create_timer(1.0, self._tick)

        self.get_logger().info(
            "ZED supervisor active: front protected, side autorestart=%s"
            % self.enable_side_autorestart
        )

    def _detection_cb(self, msg):
        try:
            data = json.loads(msg.data)
        except Exception:
            return
        now = time.monotonic()
        for camera in ("front", "left", "right"):
            if camera in data:
                self.last_detection_seen[camera] = now

    def _stream_cb(self, camera, msg):
        self.last_stream_seen[camera] = time.monotonic()

    def _node_alive(self, camera):
        node_name = f"/zedx_{camera}_detection_driver"
        try:
            result = subprocess.run(
                ["ros2", "node", "list"],
                stdout=subprocess.PIPE,
                stderr=subprocess.DEVNULL,
                text=True,
                timeout=3,
                check=False,
            )
            return node_name in set(result.stdout.splitlines())
        except Exception:
            return False

    def _detection_fresh(self, camera):
        ts = self.last_detection_seen[camera]
        return (
            ts is not None
            and (time.monotonic() - ts) <= self.freshness_timeout
        )

    def _stream_fresh(self, camera):
        ts = self.last_stream_seen[camera]
        return (
            ts is not None
            and (time.monotonic() - ts) <= self.freshness_timeout
        )

    def _restart_side(self, camera):
        now = time.monotonic()
        if now - self.last_restart[camera] < self.restart_cooldown:
            return

        self.last_restart[camera] = now
        node_name = f"zedx_{camera}_detection_driver"
        enabled_camera = f"zedx_{camera}"

        cmd = [
            "ros2", "run", "manriix_perception",
            "direct_zed_detection_node_modified.py",
            "--ros-args",
            "-r", f"__node:={node_name}",
            "-p", "mode:=full",
            "-p", f"enabled_cameras:={enabled_camera}",
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

        self.get_logger().warning(
            f"{camera}: restarting side detection worker only"
        )

        try:
            subprocess.Popen(
                cmd,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                start_new_session=True,
            )
        except Exception as exc:
            self.get_logger().error(
                f"{camera}: failed to spawn restart: {exc}"
            )

    def _tick(self):
        now = time.monotonic()
        in_grace = (now - self.started) < self.startup_grace

        health = {}
        for camera in ("front", "left", "right"):
            alive = self._node_alive(camera)
            stream_fresh = self._stream_fresh(camera)
            detection_fresh = self._detection_fresh(camera)

            # Production camera health is based on actual camera-stream
            # freshness, NOT whether an object happened to be detected.
            camera_healthy = alive and stream_fresh

            health[camera] = {
                "node_alive": alive,
                "stream_fresh": stream_fresh,
                "detection_fresh": detection_fresh,
                "healthy": camera_healthy,
            }

        if not in_grace:
            if not health["front"]["healthy"]:
                self.get_logger().error(
                    "FRONT ZED CAMERA STREAM unhealthy. Auto-restart disabled "
                    "because front owns cuVSLAM/nvblox."
                )
            elif not health["front"]["detection_fresh"]:
                self.get_logger().warning(
                    "FRONT camera stream is healthy but detection output is stale."
                )

            if self.enable_side_autorestart:
                for camera in ("left", "right"):
                    if not health[camera]["healthy"]:
                        self._restart_side(camera)
                    elif not health[camera]["detection_fresh"]:
                        self.get_logger().warning(
                            f"{camera}: camera stream healthy but detection "
                            "output is stale; not restarting camera."
                        )

        msg = String()
        msg.data = json.dumps({
            "front": health["front"],
            "left": health["left"],
            "right": health["right"],
            "startup_grace": in_grace,
        })
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ZedCameraSupervisor()
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