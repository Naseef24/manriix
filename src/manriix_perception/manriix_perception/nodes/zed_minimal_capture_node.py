#!/usr/bin/env python3
"""
Minimal ZED X side-camera capture node for MANRIIX.

Purpose:
- Open exactly ONE ZED X by serial number.
- DEPTH OFF.
- NO positional tracking.
- NO object detection.
- NO IMU.
- NO point cloud.
- Retrieve only the LEFT image.
- Keep only a low-rate, resized ROS monitoring stream for shared YOLO.

This node intentionally avoids the large MANRIIX ZED processing driver so we
can prove reliable multi-camera capture independently of the heavy perception
pipeline.
"""

import time

import cv2
import numpy as np
import pyzed.sl as sl
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
    HistoryPolicy,
)
from sensor_msgs.msg import Image


class MinimalZedCapture(Node):
    def __init__(self):
        super().__init__("zed_minimal_capture")

        self.declare_parameter("camera_name", "zedx_left")
        self.declare_parameter("serial_number", 40487925)
        self.declare_parameter("fps", 15)
        self.declare_parameter("resolution", "SVGA")
        self.declare_parameter("output_rate_hz", 5.0)
        self.declare_parameter("output_width", 640)

        self.camera_name = str(self.get_parameter("camera_name").value)
        self.serial = int(self.get_parameter("serial_number").value)
        self.fps = int(self.get_parameter("fps").value)
        self.resolution_name = str(self.get_parameter("resolution").value).upper()
        self.output_rate = float(self.get_parameter("output_rate_hz").value)
        self.output_width = int(self.get_parameter("output_width").value)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.publisher = self.create_publisher(
            Image,
            f"/monitor/{self.camera_name}/image_raw",
            qos,
        )

        self.bridge = CvBridge()
        self.zed = sl.Camera()
        self.image = sl.Mat()

        init = sl.InitParameters()
        init.set_from_serial_number(self.serial)
        init.camera_fps = self.fps
        init.depth_mode = sl.DEPTH_MODE.NONE

        resolution_map = {
            "SVGA": sl.RESOLUTION.SVGA,
            "HD720": sl.RESOLUTION.HD720,
            "HD1080": sl.RESOLUTION.HD1080,
        }
        init.camera_resolution = resolution_map.get(
            self.resolution_name,
            sl.RESOLUTION.SVGA,
        )

        self.get_logger().info(
            f"Opening minimal ZED capture: name={self.camera_name}, "
            f"serial={self.serial}, {self.resolution_name}@{self.fps}, "
            "depth=NONE, tracking=OFF, OD=OFF"
        )

        status = self.zed.open(init)
        if status != sl.ERROR_CODE.SUCCESS:
            raise RuntimeError(
                f"ZED open failed for {self.camera_name} "
                f"serial={self.serial}: {status}"
            )

        info = self.zed.get_camera_information()
        self.get_logger().info(
            f"OPEN OK: {self.camera_name}, "
            f"serial={info.serial_number}"
        )

        self.runtime = sl.RuntimeParameters()
        self.last_publish = 0.0
        self.grab_count = 0
        self.grab_errors = 0
        self.stat_time = time.monotonic()

        # Run grab loop from ROS timer. A short timer lets grab() regulate
        # against the camera FPS without creating an image queue.
        self.create_timer(0.001, self._tick)
        self.create_timer(10.0, self._status)

    def _tick(self):
        status = self.zed.grab(self.runtime)
        if status != sl.ERROR_CODE.SUCCESS:
            self.grab_errors += 1
            return

        self.grab_count += 1

        now = time.monotonic()
        if now - self.last_publish < 1.0 / max(1.0, self.output_rate):
            return

        self.zed.retrieve_image(self.image, sl.VIEW.LEFT)
        frame = np.asarray(self.image.get_data())

        # ZED image is typically BGRA.
        if frame.ndim != 3:
            return
        if frame.shape[2] == 4:
            frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
        elif frame.shape[2] != 3:
            return

        if self.output_width > 0 and frame.shape[1] != self.output_width:
            scale = self.output_width / float(frame.shape[1])
            height = max(1, int(round(frame.shape[0] * scale)))
            frame = cv2.resize(
                frame,
                (self.output_width, height),
                interpolation=cv2.INTER_AREA,
            )

        msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = f"{self.camera_name}_left_camera_frame_optical"
        self.publisher.publish(msg)

        self.last_publish = now

    def _status(self):
        elapsed = max(0.001, time.monotonic() - self.stat_time)
        rate = self.grab_count / elapsed
        self.get_logger().info(
            f"[MINIMAL CAPTURE] {self.camera_name}: "
            f"{rate:.1f} grab/s, errors={self.grab_errors}"
        )
        self.grab_count = 0
        self.grab_errors = 0
        self.stat_time = time.monotonic()

    def destroy_node(self):
        try:
            self.zed.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = MinimalZedCapture()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        if node is not None:
            node.get_logger().error(str(exc))
        else:
            print(f"[zed_minimal_capture] ERROR: {exc}")
        raise
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()