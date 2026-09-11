#!/usr/bin/env python3
"""
MANRIIX multi-ZED capture manager
=================================

ONE process owns all three ZED X cameras for the entire robot session.

Camera roles
------------
FRONT  S/N 41911351
  - SVGA @ 15 FPS
  - NEURAL_LIGHT depth
  - left/right rectified grayscale for cuVSLAM
  - left rectified color
  - registered depth for nvblox
  - IMU
  - low-rate monitor image

LEFT   S/N 40487925
  - SVGA @ 15 FPS
  - depth NONE
  - tracking OFF
  - built-in ZED object detection OFF
  - low-rate monitor image only

RIGHT  S/N 46311875
  - SVGA @ 15 FPS
  - depth NONE
  - tracking OFF
  - built-in ZED object detection OFF
  - low-rate monitor image only

Important reliability behaviour
-------------------------------
1. All 3 sl.Camera instances are opened sequentially BEFORE capture threads start.
2. If any camera cannot open, already-open cameras are closed and the node exits.
3. No runtime camera reopen/retry loop.
4. On shutdown, capture threads stop first, then cameras close RIGHT -> LEFT -> FRONT.
5. Side cameras never enable depth, tracking, IMU processing, point clouds or ZED OD.

Front ROS topics intentionally match the existing MANRIIX/cuVSLAM naming:
  /zedx_front/zed_node/left/gray/rect/image
  /zedx_front/zed_node/right/gray/rect/image
  /zedx_front/zed_node/left/gray/rect/camera_info
  /zedx_front/zed_node/right/gray/rect/camera_info
  /zedx_front/zed_node/left/color/rect/image
  /zedx_front/zed_node/left/color/rect/camera_info
  /zedx_front/zed_node/depth/depth_registered
  /zedx_front/zed_node/depth/camera_info
  /zedx_front/zed_node/imu/data

Monitoring:
  /monitor/zedx_front/image_raw
  /monitor/zedx_left/image_raw
  /monitor/zedx_right/image_raw
"""

from __future__ import annotations

import math
import threading
import time
from typing import Dict, Optional

import cv2
import numpy as np
import pyzed.sl as sl
import rclpy
from builtin_interfaces.msg import Time as TimeMsg
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import CameraInfo, Image, Imu


FRONT_SERIAL = 41911351
LEFT_SERIAL = 40487925
RIGHT_SERIAL = 46311875


class MultiZedCaptureNode(Node):
    def __init__(self):
        super().__init__("multi_zed_capture_node")

        # ------------------------------------------------------------
        # Parameters
        # ------------------------------------------------------------
        self.declare_parameter("fps", 15)
        self.declare_parameter("resolution", "SVGA")
        self.declare_parameter("monitor_rate_hz", 5.0)
        self.declare_parameter("monitor_width", 640)
        self.declare_parameter("front_depth_mode", "NEURAL_LIGHT")
        self.declare_parameter("imu_publish_rate_hz", 100.0)

        self.fps = int(self.get_parameter("fps").value)
        self.resolution_name = str(
            self.get_parameter("resolution").value
        ).upper()
        self.monitor_rate_hz = float(
            self.get_parameter("monitor_rate_hz").value
        )
        self.monitor_width = int(
            self.get_parameter("monitor_width").value
        )
        self.front_depth_mode_name = str(
            self.get_parameter("front_depth_mode").value
        ).upper()
        self.imu_publish_rate_hz = float(
            self.get_parameter("imu_publish_rate_hz").value
        )

        self.bridge = CvBridge()
        self.stop_event = threading.Event()
        self.threads = []

        self.sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # ------------------------------------------------------------
        # Publishers - front navigation
        # ------------------------------------------------------------
        self.front_left_gray_pub = self.create_publisher(
            Image,
            "/zedx_front/zed_node/left/gray/rect/image",
            self.sensor_qos,
        )
        self.front_right_gray_pub = self.create_publisher(
            Image,
            "/zedx_front/zed_node/right/gray/rect/image",
            self.sensor_qos,
        )
        self.front_left_gray_info_pub = self.create_publisher(
            CameraInfo,
            "/zedx_front/zed_node/left/gray/rect/camera_info",
            self.sensor_qos,
        )
        self.front_right_gray_info_pub = self.create_publisher(
            CameraInfo,
            "/zedx_front/zed_node/right/gray/rect/camera_info",
            self.sensor_qos,
        )

        self.front_color_pub = self.create_publisher(
            Image,
            "/zedx_front/zed_node/left/color/rect/image",
            self.sensor_qos,
        )
        self.front_color_info_pub = self.create_publisher(
            CameraInfo,
            "/zedx_front/zed_node/left/color/rect/camera_info",
            self.sensor_qos,
        )

        self.front_depth_pub = self.create_publisher(
            Image,
            "/zedx_front/zed_node/depth/depth_registered",
            self.sensor_qos,
        )
        self.front_depth_info_pub = self.create_publisher(
            CameraInfo,
            "/zedx_front/zed_node/depth/camera_info",
            self.sensor_qos,
        )

        self.front_imu_pub = self.create_publisher(
            Imu,
            "/zedx_front/zed_node/imu/data",
            self.sensor_qos,
        )

        # ------------------------------------------------------------
        # Monitor publishers
        # ------------------------------------------------------------
        self.monitor_pubs = {
            "front": self.create_publisher(
                Image, "/monitor/zedx_front/image_raw", self.sensor_qos
            ),
            "left": self.create_publisher(
                Image, "/monitor/zedx_left/image_raw", self.sensor_qos
            ),
            "right": self.create_publisher(
                Image, "/monitor/zedx_right/image_raw", self.sensor_qos
            ),
        }

        self.side_color_info_pubs = {
            "left": self.create_publisher(
                CameraInfo,
                "/zedx_left/zed_node/left/color/rect/camera_info",
                self.sensor_qos,
            ),
            "right": self.create_publisher(
                CameraInfo,
                "/zedx_right/zed_node/left/color/rect/camera_info",
                self.sensor_qos,
            ),
        }
        # ------------------------------------------------------------
        # Camera objects
        # ------------------------------------------------------------
        self.cameras: Dict[str, sl.Camera] = {
            "front": sl.Camera(),
            "left": sl.Camera(),
            "right": sl.Camera(),
        }

        self.runtime: Dict[str, sl.RuntimeParameters] = {
            "front": sl.RuntimeParameters(),
            "left": sl.RuntimeParameters(),
            "right": sl.RuntimeParameters(),
        }

        self.camera_info_msgs: Dict[str, Dict[str, CameraInfo]] = {}

        # Statistics
        self.grabs = {"front": 0, "left": 0, "right": 0}
        self.errors = {"front": 0, "left": 0, "right": 0}
        self.stats_started = time.monotonic()

        # ------------------------------------------------------------
        # Open ALL cameras before starting ANY grab thread
        # ------------------------------------------------------------
        try:
            self._open_all_cameras()
            self._build_front_camera_info()
            self._build_side_camera_info("left")
            self._build_side_camera_info("right")
            self._start_workers()
        except Exception:
            self._close_all_cameras()
            raise

        self.create_timer(10.0, self._log_statistics)

        self.get_logger().info(
            "MULTI-ZED READY: front(depth=" + self.front_depth_mode_name + ") + "
            "left(minimal) + right(minimal), all cameras owned by one process"
        )

    # ================================================================
    # ZED configuration
    # ================================================================
    def _resolution(self):
        table = {
            "SVGA": sl.RESOLUTION.SVGA,
            "HD720": sl.RESOLUTION.HD720,
            "HD1080": sl.RESOLUTION.HD1080,
        }
        if self.resolution_name not in table:
            raise RuntimeError(
                f"Unsupported resolution: {self.resolution_name}"
            )
        return table[self.resolution_name]

    def _front_depth_mode(self):
        table = {
            "NONE": sl.DEPTH_MODE.NONE,
            "PERFORMANCE": sl.DEPTH_MODE.PERFORMANCE,
            "QUALITY": sl.DEPTH_MODE.QUALITY,
            "ULTRA": sl.DEPTH_MODE.ULTRA,
            "NEURAL": sl.DEPTH_MODE.NEURAL,
            "NEURAL_LIGHT": sl.DEPTH_MODE.NEURAL_LIGHT,
            "NEURAL_PLUS": sl.DEPTH_MODE.NEURAL_PLUS,
        }
        if self.front_depth_mode_name not in table:
            raise RuntimeError(
                f"Unsupported front depth mode: "
                f"{self.front_depth_mode_name}"
            )
        return table[self.front_depth_mode_name]

    def _make_init_params(self, serial: int, front: bool):
        init = sl.InitParameters()
        init.set_from_serial_number(serial)
        init.camera_resolution = self._resolution()
        init.camera_fps = self.fps

        if front:
            init.depth_mode = self._front_depth_mode()

            # Metric depth output for nvblox.
            init.coordinate_units = sl.UNIT.METER

            # ROS REP-103 style coordinate convention.
            init.coordinate_system = (
                sl.COORDINATE_SYSTEM.RIGHT_HANDED_Z_UP_X_FWD
            )
        else:
            init.depth_mode = sl.DEPTH_MODE.NONE

        return init

    def _open_camera(self, name: str, serial: int, front: bool):
        self.get_logger().info(
            f"[OPEN BEGIN] {name.upper()} "
            f"serial={serial}, {self.resolution_name}@{self.fps}, "
            f"depth={'%s' % self.front_depth_mode_name if front else 'NONE'}"
        )

        status = self.cameras[name].open(
            self._make_init_params(serial, front)
        )

        if status != sl.ERROR_CODE.SUCCESS:
            raise RuntimeError(
                f"{name.upper()} ZED open failed "
                f"(serial={serial}): {status}"
            )

        actual_serial = (
            self.cameras[name]
            .get_camera_information()
            .serial_number
        )

        self.get_logger().info(
            f"[OPEN OK] {name.upper()} serial={actual_serial}"
        )

    def _open_all_cameras(self):
        # Deliberately sequential, before any camera starts a grab loop.
        self._open_camera(
            "front",
            FRONT_SERIAL,
            front=True,
        )

        self._open_camera(
            "left",
            LEFT_SERIAL,
            front=False,
        )

        self._open_camera(
            "right",
            RIGHT_SERIAL,
            front=False,
        )

        self.get_logger().info(
            "[OPEN+VERIFY] ALL THREE ZED X CAMERAS OPEN"
        )

    # ================================================================
    # CameraInfo generation
    # ================================================================
    def _camera_info_from_calib(
        self,
        width: int,
        height: int,
        camera_params,
        frame_id: str,
        tx: float = 0.0,
    ):
        msg = CameraInfo()
        msg.width = int(width)
        msg.height = int(height)
        msg.distortion_model = "plumb_bob"

        disto = list(camera_params.disto)
        msg.d = [float(v) for v in disto[:5]]

        fx = float(camera_params.fx)
        fy = float(camera_params.fy)
        cx = float(camera_params.cx)
        cy = float(camera_params.cy)

        msg.k = [
            fx, 0.0, cx,
            0.0, fy, cy,
            0.0, 0.0, 1.0,
        ]

        msg.r = [
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0,
        ]

        msg.p = [
            fx, 0.0, cx, tx,
            0.0, fy, cy, 0.0,
            0.0, 0.0, 1.0, 0.0,
        ]

        msg.header.frame_id = frame_id
        return msg

    def _extract_stereo_baseline_m(self, calib):
        """
        Return the physical stereo baseline magnitude in metres.

        IMPORTANT:
        With RIGHT_HANDED_Z_UP_X_FWD, the left->right stereo translation is
        not guaranteed to lie on element [0]. Use the norm of the complete
        translation vector instead of abs(tx).
        """
        def translation_vector(transform):
            candidates = []

            try:
                candidates.append(transform.get_translation())
            except Exception:
                pass

            try:
                candidates.append(transform.getTranslation())
            except Exception:
                pass

            for tr in candidates:
                values_candidates = []

                try:
                    values_candidates.append(tr.get())
                except Exception:
                    pass

                values_candidates.append(tr)

                for values in values_candidates:
                    try:
                        x = float(values[0])
                        y = float(values[1])
                        z = float(values[2])
                    except Exception:
                        continue

                    baseline = math.sqrt(x*x + y*y + z*z)

                    if baseline > 1e-4:
                        return baseline

            return None

        # First use the rectified calibration.
        baseline = translation_vector(calib.stereo_transform)
        if baseline is not None:
            return baseline

        # Fallback to raw factory calibration if binding behaviour differs.
        try:
            info = self.cameras["front"].get_camera_information()
            raw_calib = (
                info.camera_configuration
                .calibration_parameters_raw
            )

            baseline = translation_vector(
                raw_calib.stereo_transform
            )

            if baseline is not None:
                return baseline
        except Exception:
            pass

        raise RuntimeError(
            "Unable to extract a valid non-zero ZED stereo baseline "
            "from SDK calibration."
        )

    def _build_front_camera_info(self):
        info = self.cameras["front"].get_camera_information()
        cfg = info.camera_configuration
        resolution = cfg.resolution
        calib = cfg.calibration_parameters

        width = int(resolution.width)
        height = int(resolution.height)

        left_frame = "zedx_front_left_camera_frame_optical"
        right_frame = "zedx_front_right_camera_frame_optical"

        # ZED SDK exposes the left->right stereo transform. We require a
        # non-zero baseline because cuVSLAM consumes true stereo CameraInfo.
        baseline = self._extract_stereo_baseline_m(calib)

        self.get_logger().info(
            f"Front stereo baseline from SDK: {baseline:.6f} m"
        )

        left = self._camera_info_from_calib(
            width,
            height,
            calib.left_cam,
            left_frame,
            tx=0.0,
        )

        right_fx = float(calib.right_cam.fx)
        right = self._camera_info_from_calib(
            width,
            height,
            calib.right_cam,
            right_frame,
            tx=(-right_fx * baseline),
        )

        color = self._camera_info_from_calib(
            width,
            height,
            calib.left_cam,
            left_frame,
            tx=0.0,
        )

        depth = self._camera_info_from_calib(
            width,
            height,
            calib.left_cam,
            left_frame,
            tx=0.0,
        )

        self.camera_info_msgs["front"] = {
            "left": left,
            "right": right,
            "color": color,
            "depth": depth,
        }

        self.get_logger().info(
            f"Front calibration ready: {width}x{height}, "
            f"baseline={baseline:.4f} m"
        )

    def _build_side_camera_info(self, camera_name: str):
        info = self.cameras[camera_name].get_camera_information()
        cfg = info.camera_configuration
        resolution = cfg.resolution
        calib = cfg.calibration_parameters

        native_width = int(resolution.width)
        native_height = int(resolution.height)

        # Monitor images are resized while preserving aspect ratio.
        if self.monitor_width > 0:
            width = self.monitor_width
            scale = width / float(native_width)
            height = max(1, int(round(native_height * scale)))
        else:
            width = native_width
            height = native_height
            scale = 1.0

        frame_id = f"zedx_{camera_name}_left_camera_frame_optical"

        msg = self._camera_info_from_calib(
            native_width,
            native_height,
            calib.left_cam,
            frame_id,
            tx=0.0,
        )

        # Scale calibration to the actual /monitor image resolution.
        msg.width = width
        msg.height = height

        msg.k[0] *= scale   # fx
        msg.k[2] *= scale   # cx
        msg.k[4] *= scale   # fy
        msg.k[5] *= scale   # cy

        msg.p[0] *= scale   # fx
        msg.p[2] *= scale   # cx
        msg.p[5] *= scale   # fy
        msg.p[6] *= scale   # cy

        self.camera_info_msgs[camera_name] = {
            "color": msg,
        }

        self.get_logger().info(
            f"{camera_name.upper()} monitor calibration ready: "
            f"{width}x{height} "
            f"(native={native_width}x{native_height}, scale={scale:.4f})"
        )

    # ================================================================
    # Timestamp helpers
    # ================================================================
    @staticmethod
    def _ns_to_ros_time(ns: int) -> TimeMsg:
        msg = TimeMsg()
        msg.sec = int(ns // 1_000_000_000)
        msg.nanosec = int(ns % 1_000_000_000)
        return msg

    def _image_timestamp(self, camera: sl.Camera) -> TimeMsg:
        try:
            ns = int(
                camera.get_timestamp(
                    sl.TIME_REFERENCE.IMAGE
                ).get_nanoseconds()
            )
            return self._ns_to_ros_time(ns)
        except Exception:
            return self.get_clock().now().to_msg()

    # ================================================================
    # Image conversion helpers
    # ================================================================
    def _mat_to_bgr(self, mat: sl.Mat):
        frame = np.asarray(mat.get_data())

        if frame.ndim != 3:
            raise RuntimeError("Unexpected ZED image dimensions")

        if frame.shape[2] == 4:
            return cv2.cvtColor(
                frame,
                cv2.COLOR_BGRA2BGR,
            )

        if frame.shape[2] == 3:
            return frame

        raise RuntimeError(
            f"Unexpected ZED channel count: {frame.shape[2]}"
        )

    def _publish_monitor(
        self,
        camera_name: str,
        frame_bgr,
        stamp,
    ):
        frame = frame_bgr

        if (
            self.monitor_width > 0
            and frame.shape[1] != self.monitor_width
        ):
            scale = (
                self.monitor_width
                / float(frame.shape[1])
            )

            height = max(
                1,
                int(round(frame.shape[0] * scale)),
            )

            frame = cv2.resize(
                frame,
                (self.monitor_width, height),
                interpolation=cv2.INTER_AREA,
            )

        msg = self.bridge.cv2_to_imgmsg(
            frame,
            encoding="bgr8",
        )
        msg.header.stamp = stamp
        msg.header.frame_id = (
            f"zedx_{camera_name}_left_camera_frame_optical"
        )

        self.monitor_pubs[camera_name].publish(msg)

    # ================================================================
    # Front worker
    # ================================================================
    def _front_worker(self):
        camera = self.cameras["front"]
        runtime = self.runtime["front"]

        left_mat = sl.Mat()
        right_mat = sl.Mat()

        depth_enabled = (
            self.front_depth_mode_name != "NONE"
        )

        depth_mat = (
            sl.Mat()
            if depth_enabled
            else None
        )

        monitor_period = (
            1.0 / max(1.0, self.monitor_rate_hz)
        )
        last_monitor = 0.0

        while not self.stop_event.is_set():
            status = camera.grab(runtime)

            if status != sl.ERROR_CODE.SUCCESS:
                self.errors["front"] += 1
                continue

            self.grabs["front"] += 1

            stamp = self._image_timestamp(camera)

            camera.retrieve_image(
                left_mat,
                sl.VIEW.LEFT,
            )
            camera.retrieve_image(
                right_mat,
                sl.VIEW.RIGHT,
            )
            if depth_enabled:
                camera.retrieve_measure(
                    depth_mat,
                    sl.MEASURE.DEPTH,
                )

            left_bgr = self._mat_to_bgr(left_mat)
            right_bgr = self._mat_to_bgr(right_mat)

            left_gray = cv2.cvtColor(
                left_bgr,
                cv2.COLOR_BGR2GRAY,
            )
            right_gray = cv2.cvtColor(
                right_bgr,
                cv2.COLOR_BGR2GRAY,
            )

            # ---------------- cuVSLAM stereo ----------------
            left_msg = self.bridge.cv2_to_imgmsg(
                left_gray,
                encoding="mono8",
            )
            left_msg.header.stamp = stamp
            left_msg.header.frame_id = (
                "zedx_front_left_camera_frame_optical"
            )

            right_msg = self.bridge.cv2_to_imgmsg(
                right_gray,
                encoding="mono8",
            )
            right_msg.header.stamp = stamp
            right_msg.header.frame_id = (
                "zedx_front_right_camera_frame_optical"
            )

            self.front_left_gray_pub.publish(left_msg)
            self.front_right_gray_pub.publish(right_msg)

            left_info = self.camera_info_msgs["front"]["left"]
            left_info.header.stamp = stamp
            self.front_left_gray_info_pub.publish(left_info)

            right_info = self.camera_info_msgs["front"]["right"]
            right_info.header.stamp = stamp
            self.front_right_gray_info_pub.publish(right_info)

            # ---------------- nvblox color ----------------
            # nvblox expects an RGB image. Keep monitor/YOLO images as BGR,
            # but convert the front navigation color stream to RGB8.
            left_rgb = cv2.cvtColor(
                left_bgr,
                cv2.COLOR_BGR2RGB,
            )

            color_msg = self.bridge.cv2_to_imgmsg(
                left_rgb,
                encoding="rgb8",
            )
            color_msg.header.stamp = stamp
            color_msg.header.frame_id = (
                "zedx_front_left_camera_frame_optical"
            )
            self.front_color_pub.publish(color_msg)

            color_info = self.camera_info_msgs["front"]["color"]
            color_info.header.stamp = stamp
            self.front_color_info_pub.publish(color_info)

            # ---------------- nvblox depth ----------------
            # DEPTH_MODE.NONE deliberately produces no depth map. Do not
            # request or publish one in that mode.
            if depth_enabled:
                depth = np.asarray(depth_mat.get_data())

                if (
                    depth.ndim != 2
                    or depth.shape[0] == 0
                    or depth.shape[1] == 0
                ):
                    self.errors["front"] += 1
                    self.get_logger().warning(
                        "Front depth retrieval returned an empty matrix"
                    )
                else:
                    # ZED DEPTH in UNIT.METER -> 32FC1 metres.
                    if depth.dtype != np.float32:
                        depth = depth.astype(
                            np.float32,
                            copy=False,
                        )

                    depth_msg = self.bridge.cv2_to_imgmsg(
                        depth,
                        encoding="32FC1",
                    )
                    depth_msg.header.stamp = stamp
                    depth_msg.header.frame_id = (
                        "zedx_front_left_camera_frame_optical"
                    )
                    self.front_depth_pub.publish(depth_msg)

                    depth_info = self.camera_info_msgs["front"]["depth"]
                    depth_info.header.stamp = stamp
                    self.front_depth_info_pub.publish(depth_info)

            # ---------------- monitor ----------------
            now = time.monotonic()

            if now - last_monitor >= monitor_period:
                self._publish_monitor(
                    "front",
                    left_bgr,
                    stamp,
                )
                last_monitor = now

    # ================================================================
    # Side workers
    # ================================================================
    def _side_worker(self, camera_name: str):
        camera = self.cameras[camera_name]
        runtime = self.runtime[camera_name]
        image_mat = sl.Mat()

        monitor_period = (
            1.0 / max(1.0, self.monitor_rate_hz)
        )
        last_monitor = 0.0

        while not self.stop_event.is_set():
            status = camera.grab(runtime)

            if status != sl.ERROR_CODE.SUCCESS:
                self.errors[camera_name] += 1
                continue

            self.grabs[camera_name] += 1

            now = time.monotonic()

            # Still grab at 15 FPS, but retrieve/copy the image only
            # when a monitor/detector frame is due.
            if now - last_monitor < monitor_period:
                continue

            camera.retrieve_image(
                image_mat,
                sl.VIEW.LEFT,
            )

            frame = self._mat_to_bgr(
                image_mat
            )

            stamp = self._image_timestamp(
                camera
            )

            self._publish_monitor(
                camera_name,
                frame,
                stamp,
            )
            info_msg = self.camera_info_msgs[camera_name]["color"]
            info_msg.header.stamp = stamp
            self.side_color_info_pubs[camera_name].publish(info_msg)
            last_monitor = now

    # ================================================================
    # IMU worker
    # ================================================================
    def _imu_worker(self):
        camera = self.cameras["front"]
        sensors = sl.SensorsData()

        period = (
            1.0 / max(
                1.0,
                self.imu_publish_rate_hz,
            )
        )

        last_timestamp_ns = -1

        while not self.stop_event.is_set():
            started = time.monotonic()

            status = camera.get_sensors_data(
                sensors,
                sl.TIME_REFERENCE.CURRENT,
            )

            if status == sl.ERROR_CODE.SUCCESS:
                try:
                    imu_data = sensors.get_imu_data()
                    ts_ns = int(
                        imu_data.timestamp
                        .get_nanoseconds()
                    )

                    # Do not publish the same hardware sample repeatedly.
                    if ts_ns != last_timestamp_ns:
                        msg = Imu()
                        msg.header.stamp = self._ns_to_ros_time(
                            ts_ns
                        )
                        msg.header.frame_id = (
                            "zedx_front_imu_link"
                        )

                        # angular = (
                        #     imu_data
                        #     .get_angular_velocity()
                        # )
                        # linear = (
                        #     imu_data
                        #     .get_linear_acceleration()
                        # )

                        # msg.angular_velocity.x = float(
                        #     angular[0]
                        # )
                        # msg.angular_velocity.y = float(
                        #     angular[1]
                        # )
                        # msg.angular_velocity.z = float(
                        #     angular[2]
                        # )

                        # msg.linear_acceleration.x = float(
                        #     linear[0]
                        # )
                        # msg.linear_acceleration.y = float(
                        #     linear[1]
                        # )
                        # msg.linear_acceleration.z = float(
                        #     linear[2]
                        # )
                        
                        angular = (
                            imu_data
                            .get_angular_velocity()
                        )

                        linear = (
                            imu_data
                            .get_linear_acceleration()
                        )

                        # ZED SDK gyro output is in degrees/second.
                        # ROS sensor_msgs/Imu requires radians/second.
                        deg_to_rad = math.pi / 180.0

                        msg.angular_velocity.x = float(
                            angular[0]
                        ) * deg_to_rad

                        msg.angular_velocity.y = float(
                            angular[1]
                        ) * deg_to_rad

                        msg.angular_velocity.z = float(
                            angular[2]
                        ) * deg_to_rad

                        # Accelerometer is already in m/s^2, which matches ROS.
                        msg.linear_acceleration.x = float(
                            linear[0]
                        )
                        msg.linear_acceleration.y = float(
                            linear[1]
                        )
                        msg.linear_acceleration.z = float(
                            linear[2]
                        )

                        # Orientation is useful to downstream consumers,
                        # but cuVSLAM primarily needs gyro/accel.
                        try:
                            quat = (
                                imu_data
                                .get_pose()
                                .get_orientation()
                                .get()
                            )

                            msg.orientation.x = float(
                                quat[0]
                            )
                            msg.orientation.y = float(
                                quat[1]
                            )
                            msg.orientation.z = float(
                                quat[2]
                            )
                            msg.orientation.w = float(
                                quat[3]
                            )

                            msg.orientation_covariance = [
                                0.01, 0.0, 0.0,
                                0.0, 0.01, 0.0,
                                0.0, 0.0, 0.01,
                            ]
                        except Exception:
                            msg.orientation_covariance[0] = -1.0

                        msg.angular_velocity_covariance = [
                            0.001, 0.0, 0.0,
                            0.0, 0.001, 0.0,
                            0.0, 0.0, 0.001,
                        ]

                        msg.linear_acceleration_covariance = [
                            0.01, 0.0, 0.0,
                            0.0, 0.01, 0.0,
                            0.0, 0.0, 0.01,
                        ]

                        self.front_imu_pub.publish(msg)

                        last_timestamp_ns = ts_ns

                except Exception as exc:
                    self.get_logger().warning(
                        f"IMU publish error: {exc}"
                    )

            elapsed = time.monotonic() - started
            sleep_time = period - elapsed

            if sleep_time > 0:
                time.sleep(sleep_time)

    # ================================================================
    # Worker lifecycle
    # ================================================================
    def _start_workers(self):
        worker_specs = [
            (
                "front",
                self._front_worker,
                (),
            ),
            (
                "left",
                self._side_worker,
                ("left",),
            ),
            (
                "right",
                self._side_worker,
                ("right",),
            ),
            (
                "imu",
                self._imu_worker,
                (),
            ),
        ]

        for name, target, args in worker_specs:
            thread = threading.Thread(
                target=target,
                args=args,
                name=f"multi_zed_{name}",
                daemon=True,
            )
            thread.start()
            self.threads.append(thread)

    # ================================================================
    # Statistics
    # ================================================================
    def _log_statistics(self):
        elapsed = max(
            0.001,
            time.monotonic()
            - self.stats_started,
        )

        pieces = []

        for name in (
            "front",
            "left",
            "right",
        ):
            rate = (
                self.grabs[name]
                / elapsed
            )

            pieces.append(
                f"{name}:{rate:.1f}fps/"
                f"{self.errors[name]}err"
            )

        self.get_logger().info(
            "[MULTI-ZED] "
            + " | ".join(pieces)
        )

        self.grabs = {
            "front": 0,
            "left": 0,
            "right": 0,
        }

        self.errors = {
            "front": 0,
            "left": 0,
            "right": 0,
        }

        self.stats_started = time.monotonic()

    # ================================================================
    # Shutdown
    # ================================================================
    def _close_all_cameras(self):
        # Reverse order of opening.
        for name in (
            "right",
            "left",
            "front",
        ):
            camera = self.cameras.get(name)

            if camera is None:
                continue

            try:
                self.get_logger().info(
                    f"Closing {name.upper()} ZED"
                )
                camera.close()
            except Exception as exc:
                self.get_logger().warning(
                    f"{name.upper()} close warning: {exc}"
                )

    def destroy_node(self):
        self.get_logger().info(
            "Stopping multi-ZED workers..."
        )

        self.stop_event.set()

        for thread in self.threads:
            thread.join(timeout=3.0)

        self._close_all_cameras()

        self.get_logger().info(
            "All ZED cameras closed"
        )

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node: Optional[MultiZedCaptureNode] = None

    try:
        node = MultiZedCaptureNode()
        rclpy.spin(node)

    except KeyboardInterrupt:
        if node is not None:
            node.get_logger().info(
                "Ctrl+C received"
            )

    except Exception as exc:
        if node is not None:
            node.get_logger().error(
                f"Multi-ZED fatal error: {exc}"
            )
        else:
            print(
                f"[multi_zed_capture_node] "
                f"FATAL: {exc}"
            )
        raise

    finally:
        if node is not None:
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()