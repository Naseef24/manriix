#!/usr/bin/env python3
"""
Manriix Remote Camera Compressor

Runs OUTSIDE the critical ZED/cuvSLAM process.

Input (low-rate raw thumbnails):
  /monitor/zedx_front/image_raw
  /monitor/zedx_left/image_raw
  /monitor/zedx_right/image_raw

Output (Foxglove-compatible JPEG):
  /zedx_front/zed_node/left/image_rect_color/compressed
  /zedx_left/zed_node/left/image_rect_color/compressed
  /zedx_right/zed_node/left/image_rect_color/compressed

Safety/performance design:
  * separate OS process
  * launched with nice +15
  * OpenCV restricted to one thread
  * BEST_EFFORT + KEEP_LAST(1)
  * latest-frame-only storage
  * JPEG quality 55 by default
  * no depth compression
  * monitoring failure cannot block navigation
"""

from __future__ import annotations

from typing import Dict, Optional

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import CompressedImage, Image


class RemoteCameraCompressor(Node):
    def __init__(self) -> None:
        super().__init__("remote_camera_compressor")

        # Do not let JPEG compression consume all Jetson CPU workers.
        try:
            cv2.setNumThreads(1)
        except Exception:
            pass

        self.declare_parameter("jpeg_quality", 55)
        self.declare_parameter("output_rate_hz", 3.0)

        self.jpeg_quality = max(
            30,
            min(80, int(self.get_parameter("jpeg_quality").value)),
        )
        self.output_rate_hz = max(
            0.5,
            min(5.0, float(self.get_parameter("output_rate_hz").value)),
        )

        self.bridge = CvBridge()

        self.qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.latest: Dict[str, Optional[Image]] = {
            "front": None,
            "left": None,
            "right": None,
        }

        self.output_publishers = {}
        self.input_subscriptions = []

        for name in ("front", "left", "right"):
            input_topic = f"/monitor/zedx_{name}/image_raw"
            output_topic = (
                f"/zedx_{name}/zed_node/left/"
                "image_rect_color/compressed"
            )

            self.output_publishers[name] = self.create_publisher(
                CompressedImage,
                output_topic,
                self.qos,
            )

            self.input_subscriptions.append(
                self.create_subscription(
                    Image,
                    input_topic,
                    lambda msg, camera=name: self._input_callback(
                        camera,
                        msg,
                    ),
                    self.qos,
                )
            )

        self.timer = self.create_timer(
            1.0 / self.output_rate_hz,
            self._publish_latest,
        )

        self.get_logger().info(
            "Remote compressor ready: "
            f"{self.output_rate_hz:.1f} Hz, "
            f"JPEG quality={self.jpeg_quality}, "
            "BEST_EFFORT depth=1, OpenCV threads=1"
        )

    def _input_callback(self, camera: str, msg: Image) -> None:
        # Latest-frame-only: overwrite without queueing old frames.
        self.latest[camera] = msg

    def _publish_latest(self) -> None:
        for camera in ("front", "left", "right"):
            msg = self.latest[camera]
            if msg is None:
                continue

            # Consume it once. If compression is slow, newer callback data
            # simply overwrites the slot; stale frames never accumulate.
            self.latest[camera] = None

            try:
                image = self.bridge.imgmsg_to_cv2(
                    msg,
                    desired_encoding="bgr8",
                )

                ok, encoded = cv2.imencode(
                    ".jpg",
                    image,
                    [
                        cv2.IMWRITE_JPEG_QUALITY,
                        self.jpeg_quality,
                    ],
                )
                if not ok:
                    continue

                out = CompressedImage()
                out.header = msg.header
                out.format = "jpeg"
                out.data = encoded.tobytes()

                self.output_publishers[camera].publish(out)

            except Exception as exc:
                self.get_logger().warning(
                    f"{camera}: JPEG encode failed: {exc}"
                )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RemoteCameraCompressor()

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