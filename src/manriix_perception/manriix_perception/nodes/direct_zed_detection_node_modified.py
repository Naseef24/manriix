#!/usr/bin/env python3
"""
Direct ZED SDK ROS 2 Driver - Multi-Camera cuVSLAM + nvblox + Detection

Replacement for the ZED ROS 2 wrapper in the Manriix navigation pipeline.

This node opens ZED X cameras directly through the ZED SDK and publishes the
ROS topics expected by:

  * Isaac ROS Visual SLAM (cuVSLAM)
  * Isaac ROS nvblox
  * Nav2/STVL point-cloud consumers
  * Existing Manriix detection and web-visualisation consumers

Navigation topics
-----------------
For each enabled camera, the node publishes:

  cuVSLAM:
    /zedx_<name>/zed_node/left/gray/rect/image
    /zedx_<name>/zed_node/left/gray/rect/camera_info
    /zedx_<name>/zed_node/right/gray/rect/image
    /zedx_<name>/zed_node/right/gray/rect/camera_info

  nvblox:
    /zedx_<name>/zed_node/depth/depth_registered
    /zedx_<name>/zed_node/depth/camera_info
    /zedx_<name>/zed_node/left/color/rect/image
    /zedx_<name>/zed_node/left/color/rect/camera_info

  Other navigation consumers:
    /zedx_<name>/zed_node/point_cloud/cloud_registered

Front-camera IMU:
    /zedx_front/zed_node/imu/data

Optional monitoring topics:
    /zedx_<name>/zed_node/left/image_rect_color/compressed
    /zedx_<name>/zed_node/depth/depth_registered/compressedDepth

Modes
-----
  nav:
    Raw stereo/depth/colour topics, IMU and point clouds. Detection disabled.

  full:
    All nav-mode outputs plus ZED SDK object detection and annotated images.

Test/diagnostic build
---------------------
This version adds a fast cuVSLAM test path intended to diagnose low stereo FPS.

When ``fast_vslam_test:=True`` (default in this test file):
  * left/right stereo and depth remain available;
  * CPU XYZRGBA point-cloud extraction/publication is disabled;
  * the publish timer does not deep-copy an unchanged frame repeatedly.

This keeps nvblox depth available while removing the heaviest CPU point-cloud
work from the camera capture path. Set ``fast_vslam_test:=False`` to restore
normal point-cloud behaviour (subject to ``publish_pointcloud``).

Production Manriix configuration
--------------------------------
  * Front ZED X: cuVSLAM stereo + IMU + nvblox depth/color + detection.
  * Left/right ZED X: detection only.
  * Camera source and ROS publisher cadence: 15 Hz.
  * CPU PointCloud2 publication: disabled.
  * Remote monitoring: 640 px raw thumbnails at ~3 Hz; JPEG encoded in a separate low-priority process.
  * Annotated detection images: disabled.

These settings protect the localization fast path and were validated at
approximately 15 Hz for front stereo, depth and cuVSLAM odometry.

Important assumptions
---------------------
  * robot_state_publisher publishes the static camera and IMU transforms.
  * The frame names below exactly match vslam.launch.py and the robot URDF.
  * Front depth is normally enabled for nvblox.
  * Left/right depth may be disabled when they are cuVSLAM-only cameras.
  * For reliable three-camera cuVSLAM, cameras should be hardware-synchronised.
"""

from __future__ import annotations

import copy
import json
import math
import os
import threading
import time
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Tuple

import cv2
import numpy as np
import pyzed.sl as sl
import rclpy
import yaml
from builtin_interfaces.msg import Time as TimeMsg
from cv_bridge import CvBridge
from geometry_msgs.msg import Point, Vector3
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import CameraInfo, CompressedImage, Image, Imu, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header, String

try:
    from manriix_perception.msg import (
        HumanTrackingData,
        HumansList,
        ObjectTrackingData,
        ObjectsList,
    )

    PERCEPTION_MSGS_AVAILABLE = True
except ImportError:
    PERCEPTION_MSGS_AVAILABLE = False


COCO_CLASS_NAMES = {
    0: "person", 1: "bicycle", 2: "car", 3: "motorcycle", 4: "airplane",
    5: "bus", 6: "train", 7: "truck", 8: "boat", 9: "traffic light",
    10: "fire hydrant", 11: "stop sign", 12: "parking meter", 13: "bench",
    14: "bird", 15: "cat", 16: "dog", 17: "horse", 18: "sheep", 19: "cow",
    20: "elephant", 21: "bear", 22: "zebra", 23: "giraffe", 24: "backpack",
    25: "umbrella", 26: "handbag", 27: "tie", 28: "suitcase", 29: "frisbee",
    30: "skis", 31: "snowboard", 32: "sports ball", 33: "kite",
    34: "baseball bat", 35: "baseball glove", 36: "skateboard",
    37: "surfboard", 38: "tennis racket", 39: "bottle", 40: "wine glass",
    41: "cup", 42: "fork", 43: "knife", 44: "spoon", 45: "bowl",
    46: "banana", 47: "apple", 48: "sandwich", 49: "orange",
    50: "broccoli", 51: "carrot", 52: "hot dog", 53: "pizza", 54: "donut",
    55: "cake", 56: "chair", 57: "couch", 58: "potted plant", 59: "bed",
    60: "dining table", 61: "toilet", 62: "tv", 63: "laptop", 64: "mouse",
    65: "remote", 66: "keyboard", 67: "cell phone", 68: "microwave",
    69: "oven", 70: "toaster", 71: "sink", 72: "refrigerator", 73: "book",
    74: "clock", 75: "vase", 76: "scissors", 77: "teddy bear",
    78: "hair drier", 79: "toothbrush",
}


CAMERA_TRANSFORMS = {
    "front": {"position": [0.145, 0.0, 0.815], "yaw": 0.0},
    "left": {"position": [0.0, 0.175, 0.815], "yaw": 2.094},
    "right": {"position": [0.0, -0.175, 0.815], "yaw": -2.094},
}


@dataclass
class PersonDetection:
    tracking_id: int
    raw_position: List[float]
    position: Point
    confidence: float
    distance: float
    bounding_box_2d: List[List[float]]
    velocity: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    camera_name: str = ""
    timestamp: float = 0.0


@dataclass
class ObjectDetection:
    tracking_id: int
    class_id: int
    class_name: str
    raw_position: List[float]
    position: Point
    confidence: float
    distance: float
    bounding_box_2d: List[List[float]]
    velocity: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    camera_name: str = ""
    timestamp: float = 0.0


@dataclass
class StereoCalibration:
    width: int
    height: int
    left_fx: float
    left_fy: float
    left_cx: float
    left_cy: float
    right_fx: float
    right_fy: float
    right_cx: float
    right_cy: float
    baseline_m: float


@dataclass
class CameraFrame:
    sequence: int
    sdk_timestamp_ns: int
    left_color_bgr: np.ndarray
    left_gray: np.ndarray
    right_gray: np.ndarray
    depth_m: Optional[np.ndarray]
    point_cloud_xyz: Optional[np.ndarray]
    persons: List[PersonDetection]
    objects: List[ObjectDetection]


def ros_time_from_nanoseconds(total_ns: int) -> TimeMsg:
    msg = TimeMsg()
    msg.sec = int(total_ns // 1_000_000_000)
    msg.nanosec = int(total_ns % 1_000_000_000)
    return msg


class CameraHandler:
    """Owns one ZED camera and its capture thread."""

    def __init__(
        self,
        camera_name: str,
        serial_number: int,
        logger,
        depth_enabled: bool,
    ) -> None:
        self.camera_name = camera_name
        self.serial_number = serial_number
        self.logger = logger
        self.depth_enabled = depth_enabled

        self.zed: Optional[sl.Camera] = None
        self.running = False
        self.capture_thread: Optional[threading.Thread] = None
        self.imu_thread: Optional[threading.Thread] = None
        self.imu_thread_running = False

        self.frame_lock = threading.Lock()
        self.current_frame: Optional[CameraFrame] = None
        self.frame_sequence = 0
        self.calibration: Optional[StereoCalibration] = None

        self.frame_count = 0
        self.fps_samples: List[float] = []
        self.grab_errors = 0

        self.imu_publisher = None
        self.imu_clock = None
        self.imu_ros_offset_ns: Optional[int] = None

    def setup(
        self,
        resolution: sl.RESOLUTION,
        depth_mode: sl.DEPTH_MODE,
        fps: int,
        enable_tracking: bool,
        depth_min_m: float,
        depth_max_m: float,
    ) -> bool:
        self.zed = sl.Camera()

        init_params = sl.InitParameters()
        init_params.camera_resolution = resolution
        init_params.camera_fps = fps
        init_params.depth_mode = depth_mode
        init_params.coordinate_units = sl.UNIT.METER
        init_params.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Z_UP_X_FWD
        init_params.depth_minimum_distance = depth_min_m
        init_params.depth_maximum_distance = depth_max_m
        init_params.set_from_serial_number(self.serial_number)

        for attempt in range(1, 4):
            error = self.zed.open(init_params)
            if error == sl.ERROR_CODE.SUCCESS:
                break

            self.logger.warning(
                f"{self.camera_name}: open attempt {attempt}/3 failed: {error}"
            )
            time.sleep(2.0)
        else:
            self.logger.error(
                f"{self.camera_name}: failed to open serial {self.serial_number}"
            )
            return False

        if enable_tracking:
            tracking = sl.PositionalTrackingParameters()
            tracking.enable_imu_fusion = True
            tracking_error = self.zed.enable_positional_tracking(tracking)
            if tracking_error != sl.ERROR_CODE.SUCCESS:
                self.logger.warning(
                    f"{self.camera_name}: positional tracking enable failed: "
                    f"{tracking_error}"
                )

        self.calibration = self._read_stereo_calibration()
        if self.calibration is None:
            self.logger.error(
                f"{self.camera_name}: failed to obtain stereo calibration"
            )
            self.zed.close()
            return False

        for _ in range(10):
            self.zed.grab()
            time.sleep(0.05)

        self.logger.info(
            f"{self.camera_name}: ready, serial={self.serial_number}, "
            f"depth={'ON' if self.depth_enabled else 'OFF'}, "
            f"resolution={self.calibration.width}x{self.calibration.height}"
        )
        return True

    def _read_stereo_calibration(self) -> Optional[StereoCalibration]:
        if self.zed is None:
            return None

        try:
            info = self.zed.get_camera_information()
            config = info.camera_configuration
            calibration = config.calibration_parameters
            left = calibration.left_cam
            right = calibration.right_cam

            translation = calibration.stereo_transform.get_translation().get()
            baseline_m = float(
                math.sqrt(
                    float(translation[0]) ** 2
                    + float(translation[1]) ** 2
                    + float(translation[2]) ** 2
                )
            )

            # Some SDK versions return translation using configured units;
            # configured units are metres. Guard against millimetre-like values.
            if baseline_m > 1.0:
                baseline_m /= 1000.0

            return StereoCalibration(
                width=int(config.resolution.width),
                height=int(config.resolution.height),
                left_fx=float(left.fx),
                left_fy=float(left.fy),
                left_cx=float(left.cx),
                left_cy=float(left.cy),
                right_fx=float(right.fx),
                right_fy=float(right.fy),
                right_cx=float(right.cx),
                right_cy=float(right.cy),
                baseline_m=baseline_m,
            )
        except Exception as exc:
            self.logger.error(
                f"{self.camera_name}: calibration read failed: {exc}"
            )
            return None

    def enable_detection(
        self,
        model_type: sl.OBJECT_DETECTION_MODEL,
        custom_onnx_path: Optional[str],
    ) -> bool:
        if self.zed is None:
            return False

        params = sl.ObjectDetectionParameters()
        params.enable_tracking = True
        params.enable_segmentation = False
        params.detection_model = model_type

        if custom_onnx_path:
            expanded = os.path.expanduser(custom_onnx_path)
            if os.path.exists(expanded):
                params.custom_onnx_file = expanded
            else:
                self.logger.warning(
                    f"{self.camera_name}: model does not exist: {expanded}"
                )

        error = self.zed.enable_object_detection(params)
        if error != sl.ERROR_CODE.SUCCESS:
            self.logger.error(
                f"{self.camera_name}: object detection enable failed: {error}"
            )
            return False

        return True

    def start(
        self,
        confidence_threshold: int,
        enable_detection: bool,
        pointcloud_downsample: int,
        capture_pointcloud: bool,
    ) -> None:
        self.running = True
        self.capture_thread = threading.Thread(
            target=self._capture_loop,
            args=(
                confidence_threshold,
                enable_detection,
                max(1, pointcloud_downsample),
                capture_pointcloud,
            ),
            daemon=True,
        )
        self.capture_thread.start()

    def _capture_loop(
        self,
        confidence_threshold: int,
        enable_detection: bool,
        pointcloud_downsample: int,
        capture_pointcloud: bool,
    ) -> None:
        assert self.zed is not None

        left_mat = sl.Mat()
        right_mat = sl.Mat()
        depth_mat = sl.Mat()
        point_cloud_mat = sl.Mat()
        objects = sl.Objects()

        runtime = sl.RuntimeParameters()
        object_runtime = sl.ObjectDetectionRuntimeParameters()
        object_runtime.detection_confidence_threshold = confidence_threshold

        while self.running:
            started = time.monotonic()
            status = self.zed.grab(runtime)

            if status != sl.ERROR_CODE.SUCCESS:
                self.grab_errors += 1
                if self.grab_errors <= 5 or self.grab_errors % 50 == 0:
                    self.logger.warning(
                        f"{self.camera_name}: grab failed: {status}, "
                        f"count={self.grab_errors}"
                    )
                time.sleep(0.002)
                continue

            self.grab_errors = 0

            self.zed.retrieve_image(left_mat, sl.VIEW.LEFT)
            self.zed.retrieve_image(right_mat, sl.VIEW.RIGHT)

            left_rgba = np.asarray(left_mat.get_data())
            right_rgba = np.asarray(right_mat.get_data())

            left_color_bgr = cv2.cvtColor(left_rgba, cv2.COLOR_RGBA2BGR)
            left_gray = cv2.cvtColor(left_rgba, cv2.COLOR_RGBA2GRAY)
            right_gray = cv2.cvtColor(right_rgba, cv2.COLOR_RGBA2GRAY)

            depth_m: Optional[np.ndarray] = None
            point_cloud_xyz: Optional[np.ndarray] = None

            if self.depth_enabled:
                self.zed.retrieve_measure(depth_mat, sl.MEASURE.DEPTH)
                depth_m = np.asarray(depth_mat.get_data(), dtype=np.float32).copy()

                # Point-cloud extraction is deliberately optional.
                # cuVSLAM needs only stereo + CameraInfo + IMU, and nvblox
                # consumes depth/color directly.  XYZRGBA -> CPU -> NumPy
                # conversion is expensive and must not block every localization
                # frame during the fast-VSLAM diagnostic test.
                if capture_pointcloud:
                    self.zed.retrieve_measure(
                        point_cloud_mat,
                        sl.MEASURE.XYZRGBA,
                        sl.MEM.CPU,
                    )
                    cloud = np.asarray(point_cloud_mat.get_data())
                    cloud = cloud[
                        ::pointcloud_downsample,
                        ::pointcloud_downsample,
                        :3,
                    ]
                    points = cloud.reshape(-1, 3)
                    valid = np.all(np.isfinite(points), axis=1)
                    point_cloud_xyz = points[valid].astype(
                        np.float32,
                        copy=True,
                    )

            persons: List[PersonDetection] = []
            objects_detected: List[ObjectDetection] = []

            if enable_detection:
                self.zed.retrieve_objects(objects, object_runtime)
                persons, objects_detected = self._parse_detections(objects)

            sdk_timestamp_ns = int(
                self.zed.get_timestamp(sl.TIME_REFERENCE.IMAGE).get_nanoseconds()
            )

            self.frame_sequence += 1
            frame = CameraFrame(
                sequence=self.frame_sequence,
                sdk_timestamp_ns=sdk_timestamp_ns,
                left_color_bgr=left_color_bgr.copy(),
                left_gray=left_gray.copy(),
                right_gray=right_gray.copy(),
                depth_m=depth_m,
                point_cloud_xyz=point_cloud_xyz,
                persons=persons,
                objects=objects_detected,
            )

            with self.frame_lock:
                self.current_frame = frame

            self.frame_count += 1
            elapsed = time.monotonic() - started
            if elapsed > 0.0:
                self.fps_samples.append(1.0 / elapsed)
                if len(self.fps_samples) > 30:
                    self.fps_samples.pop(0)

    def _parse_detections(
        self,
        objects: sl.Objects,
    ) -> Tuple[List[PersonDetection], List[ObjectDetection]]:
        persons: List[PersonDetection] = []
        all_objects: List[ObjectDetection] = []

        for obj in objects.object_list:
            if obj.tracking_state != sl.OBJECT_TRACKING_STATE.OK:
                continue

            position = [float(v) for v in obj.position[:3]]
            velocity = [0.0, 0.0, 0.0]
            try:
                velocity = [float(v) for v in obj.velocity[:3]]
            except Exception:
                pass

            bbox = []
            try:
                bbox = [
                    [float(corner[0]), float(corner[1])]
                    for corner in obj.bounding_box_2d
                ]
            except Exception:
                pass

            class_id = 0
            try:
                class_id = int(obj.raw_label)
            except Exception:
                pass

            distance = float(np.linalg.norm(position))
            common = {
                "tracking_id": int(obj.id),
                "raw_position": position,
                "position": Point(),
                "confidence": float(obj.confidence),
                "distance": distance,
                "bounding_box_2d": bbox,
                "velocity": velocity,
                "camera_name": self.camera_name,
                "timestamp": time.time(),
            }

            all_objects.append(
                ObjectDetection(
                    class_id=class_id,
                    class_name=COCO_CLASS_NAMES.get(class_id, "unknown"),
                    **common,
                )
            )

            if class_id == 0:
                persons.append(PersonDetection(**common))

        return persons, all_objects

    def get_frame(
        self,
        last_sequence: Optional[int] = None,
    ) -> Optional[CameraFrame]:
        """Return a copy only when a new frame is available.

        The old implementation copied all image/depth/point-cloud arrays every
        30 Hz publish-timer tick and only afterwards discovered that the camera
        sequence had not changed.  With a 15 Hz (or slower) camera this wasted
        CPU and memory bandwidth.  Check the sequence while holding the lock,
        before any large NumPy copies are made.
        """
        with self.frame_lock:
            if self.current_frame is None:
                return None
            if (
                last_sequence is not None
                and self.current_frame.sequence == last_sequence
            ):
                return None
            frame = self.current_frame
            return CameraFrame(
                sequence=frame.sequence,
                sdk_timestamp_ns=frame.sdk_timestamp_ns,
                left_color_bgr=frame.left_color_bgr.copy(),
                left_gray=frame.left_gray.copy(),
                right_gray=frame.right_gray.copy(),
                depth_m=(
                    frame.depth_m.copy()
                    if frame.depth_m is not None
                    else None
                ),
                point_cloud_xyz=(
                    frame.point_cloud_xyz.copy()
                    if frame.point_cloud_xyz is not None
                    else None
                ),
                persons=copy.deepcopy(frame.persons),
                objects=copy.deepcopy(frame.objects),
            )

    def get_fps(self) -> float:
        if not self.fps_samples:
            return 0.0
        return float(sum(self.fps_samples) / len(self.fps_samples))

    def start_imu_thread(
        self,
        rate_hz: float,
        publisher,
        clock,
        imu_frame: str,
    ) -> None:
        if self.zed is None or not self.zed.is_opened():
            self.logger.warning(
                f"{self.camera_name}: cannot start IMU; camera is not open"
            )
            return

        self.imu_publisher = publisher
        self.imu_clock = clock
        self.imu_frame = imu_frame
        self.imu_thread_running = True
        self.imu_thread = threading.Thread(
            target=self._imu_loop,
            args=(rate_hz,),
            daemon=True,
        )
        self.imu_thread.start()

    def _imu_loop(self, rate_hz: float) -> None:
        assert self.zed is not None
        assert self.imu_clock is not None

        period = 1.0 / max(rate_hz, 1.0)
        sensors = sl.SensorsData()
        previous_sdk_timestamp_ns = -1

        while self.running and self.imu_thread_running:
            try:
                status = self.zed.get_sensors_data(
                    sensors,
                    sl.TIME_REFERENCE.CURRENT,
                )
                if status != sl.ERROR_CODE.SUCCESS:
                    time.sleep(period)
                    continue

                imu = sensors.get_imu_data()
                sdk_timestamp_ns = int(imu.timestamp.get_nanoseconds())
                if sdk_timestamp_ns == previous_sdk_timestamp_ns:
                    time.sleep(period)
                    continue
                previous_sdk_timestamp_ns = sdk_timestamp_ns

                if self.imu_ros_offset_ns is None:
                    ros_now_ns = int(self.imu_clock.now().nanoseconds)
                    self.imu_ros_offset_ns = ros_now_ns - sdk_timestamp_ns

                ros_timestamp_ns = sdk_timestamp_ns + self.imu_ros_offset_ns

                orientation = imu.get_pose().get_orientation().get()
                angular_velocity = imu.get_angular_velocity()
                linear_acceleration = imu.get_linear_acceleration()

                # The SDK camera is configured as RIGHT_HANDED_Z_UP_X_FWD.
                # Publish the SDK values directly in the declared IMU frame.
                msg = Imu()
                msg.header.stamp = ros_time_from_nanoseconds(ros_timestamp_ns)
                msg.header.frame_id = self.imu_frame

                msg.orientation.x = float(orientation[0])
                msg.orientation.y = float(orientation[1])
                msg.orientation.z = float(orientation[2])
                msg.orientation.w = float(orientation[3])

                # ZED SDK angular velocity is reported in degrees/second.
                msg.angular_velocity.x = math.radians(float(angular_velocity[0]))
                msg.angular_velocity.y = math.radians(float(angular_velocity[1]))
                msg.angular_velocity.z = math.radians(float(angular_velocity[2]))

                msg.linear_acceleration.x = float(linear_acceleration[0])
                msg.linear_acceleration.y = float(linear_acceleration[1])
                msg.linear_acceleration.z = float(linear_acceleration[2])

                msg.orientation_covariance = [
                    0.01, 0.0, 0.0,
                    0.0, 0.01, 0.0,
                    0.0, 0.0, 0.01,
                ]
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
                self.imu_publisher.publish(msg)
            except Exception as exc:
                if self.imu_thread_running:
                    self.logger.warning(
                        f"{self.camera_name}: IMU loop error: {exc}"
                    )

            time.sleep(period)

    def cleanup(self) -> None:
        self.running = False
        self.imu_thread_running = False

        if self.capture_thread is not None:
            self.capture_thread.join(timeout=3.0)
        if self.imu_thread is not None:
            self.imu_thread.join(timeout=3.0)

        if self.zed is not None:
            try:
                self.zed.disable_object_detection()
            except Exception:
                pass
            try:
                self.zed.disable_positional_tracking()
            except Exception:
                pass
            try:
                self.zed.close()
            except Exception:
                pass


class DirectZedDetectionNodeModified(Node):
    def __init__(self) -> None:
        super().__init__("direct_zed_detection_node_modified")

        self._declare_parameters()

        # PRODUCTION VSLAM: make these settings authoritative inside the node.
        # This deliberately overrides any launch-time parameter values so
        # `ros2 param get` also reports False during the production localization run.
        self._enforce_vslam_performance_flags()

        self.config = self._load_config()

        self.mode = str(self.config["mode"])
        self.enable_detection = self.mode == "full"
        self.fast_vslam_test = bool(self.config["fast_vslam_test"])
        self.bridge = CvBridge()

        self.sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self.reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # Remote monitoring must never back-pressure navigation.
        # BEST_EFFORT + KEEP_LAST(1) means stale camera frames are dropped.
        self.monitoring_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.cameras: Dict[str, CameraHandler] = {}
        self.last_published_sequence: Dict[str, int] = {}
        self.camera_ros_offset_ns: Optional[int] = None

        self._setup_publishers()

        if not self._setup_cameras():
            raise RuntimeError("No ZED cameras were successfully initialized")

        self._start_cameras()

        publish_rate = float(self.config["publish_rate"])
        self.publish_timer = self.create_timer(
            1.0 / max(1.0, publish_rate),
            self._publish_callback,
        )
        self.status_timer = self.create_timer(10.0, self._status_callback)

        self.get_logger().info(
            f"Direct ZED driver ready: mode={self.mode}, "
            f"cameras={','.join(self.cameras.keys())}, "
            f"fast_vslam_test={self.fast_vslam_test}, "
            f"pointcloud_capture="
            f"{bool(self.config['publish_pointcloud']) and not self.fast_vslam_test}"
        )

    def _declare_parameters(self) -> None:
        self.declare_parameter("mode", "nav")
        self.declare_parameter("config_file", "")

        self.declare_parameter("enabled_cameras", "zedx_front,zedx_left,zedx_right")
        self.declare_parameter("camera_front_serial", 41911351)
        self.declare_parameter("camera_left_serial", 40487925)
        self.declare_parameter("camera_right_serial", 46311875)

        self.declare_parameter("resolution", "SVGA")
        self.declare_parameter("front_resolution", "")
        self.declare_parameter("fps", 15)

        self.declare_parameter("front_depth_mode", "NEURAL_LIGHT")
        self.declare_parameter("left_depth_mode", "NONE")
        self.declare_parameter("right_depth_mode", "NONE")
        self.declare_parameter("depth_minimum_distance", 0.3)
        self.declare_parameter("depth_maximum_distance", 10.0)

        self.declare_parameter("publish_rate", 15.0)

        # Diagnostic switch.  Keep stereo + depth fast, but remove CPU XYZ
        # point-cloud work from the capture path.  This is True by default in
        # this test build; set False after the FPS root cause is confirmed.
        self.declare_parameter("fast_vslam_test", True)

        self.declare_parameter("publish_pointcloud", False)
        self.declare_parameter("pointcloud_downsample", 4)
        self.declare_parameter("publish_compressed_monitoring", False)
        self.declare_parameter("publish_monitoring_thumbnails", True)
        self.declare_parameter("monitoring_publish_divisor", 5)
        self.declare_parameter("jpeg_quality", 60)
        self.declare_parameter("monitoring_width", 640)

        self.declare_parameter("imu_rate_hz", 100.0)

        self.declare_parameter("detection_model", "YOLO11")
        self.declare_parameter(
            "yolo_model_path",
            "~/models/yolo11/yolo11s.onnx",
        )
        self.declare_parameter("confidence_threshold", 50)
        self.declare_parameter("publish_detection_images", False)
        self.declare_parameter("draw_bounding_boxes", True)
        self.declare_parameter("detection_image_scale", 0.5)

    def _enforce_vslam_performance_flags(self) -> None:
        """Keep heavy optional publishers out of the critical VSLAM path.

        Detection itself remains enabled in FULL mode. Only these expensive
        outputs are disabled:
          * ROS PointCloud2 publication / CPU XYZ cloud extraction
          * compressed monitoring JPEG/PNG publication
          * annotated detection JPEG publication

        set_parameters() is used so the ROS parameter server reflects the
        effective values too; `ros2 param get` therefore reports False for these production-safe settings.
        """
        forced = {
            "publish_pointcloud": False,
            "publish_compressed_monitoring": False,
            "publish_detection_images": False,
            "publish_monitoring_thumbnails": True,
        }

        results = self.set_parameters(
            [
                Parameter(name, Parameter.Type.BOOL, value)
                for name, value in forced.items()
            ]
        )

        failed = [
            name
            for name, result in zip(forced.keys(), results)
            if not result.successful
        ]
        if failed:
            raise RuntimeError(
                "Failed to force test publication flags: "
                + ", ".join(failed)
            )

        self.get_logger().info(
            "PRODUCTION VSLAM: pointcloud and annotated detection images OFF; "
            "low-rate RAW thumbnails ON (~3 Hz); JPEG moved to separate process "
            "to protect the 15 FPS localization path"
        )

    def _load_config(self) -> Dict[str, Any]:
        camera_name_aliases = {
            "front": "front",
            "left": "left",
            "right": "right",
            "zedx_front": "front",
            "zedx_left": "left",
            "zedx_right": "right",
        }

        requested_cameras = [
            item.strip()
            for item in str(
                self.get_parameter("enabled_cameras").value
            ).split(",")
            if item.strip()
        ]

        unknown_cameras = [
            item
            for item in requested_cameras
            if item not in camera_name_aliases
        ]

        if unknown_cameras:
            self.get_logger().warning(
                "Ignoring unknown camera names: "
                + ", ".join(unknown_cameras)
            )

        # enabled = []
        # for item in requested_cameras:
        #     normalized = camera_name_aliases.get(item)
        #     if normalized is not None and normalized not in enabled:
        #         enabled.append(normalized)

        # self.get_logger().info(
        #     "Enabled camera mapping: "
        #     f"requested={requested_cameras}, internal={enabled}"
        # )

        enabled = []

        for item in requested_cameras:
            normalized = camera_name_aliases.get(item)

            if normalized is not None and normalized not in enabled:
                enabled.append(normalized)

        # Multi-process production architecture:
        # FULL enables object detection, but each process owns only the
        # cameras explicitly listed in enabled_cameras.
        requested_mode = str(self.get_parameter("mode").value).lower()

        if requested_mode == "full":
            self.get_logger().info(
                "FULL mode: detection enabled; honoring explicit camera "
                f"ownership: {','.join(enabled) or '(none)'}"
            )

        self.get_logger().info(
            "Enabled camera mapping: "
            f"requested={requested_cameras}, internal={enabled}"
        )

        config: Dict[str, Any] = {
            "mode": self.get_parameter("mode").value,
            "enabled_cameras": enabled,
            "cameras": {
                "front": {
                    "serial": self.get_parameter("camera_front_serial").value,
                    "depth_mode": self.get_parameter(
                        "front_depth_mode"
                    ).value,
                },
                "left": {
                    "serial": self.get_parameter("camera_left_serial").value,
                    "depth_mode": self.get_parameter(
                        "left_depth_mode"
                    ).value,
                },
                "right": {
                    "serial": self.get_parameter("camera_right_serial").value,
                    "depth_mode": self.get_parameter(
                        "right_depth_mode"
                    ).value,
                },
            },
            "resolution": self.get_parameter("resolution").value,
            "front_resolution": self.get_parameter(
                "front_resolution"
            ).value,
            "fps": self.get_parameter("fps").value,
            "depth_minimum_distance": self.get_parameter(
                "depth_minimum_distance"
            ).value,
            "depth_maximum_distance": self.get_parameter(
                "depth_maximum_distance"
            ).value,
            "publish_rate": self.get_parameter("publish_rate").value,
            "fast_vslam_test": self.get_parameter(
                "fast_vslam_test"
            ).value,
            "publish_pointcloud": self.get_parameter(
                "publish_pointcloud"
            ).value,
            "pointcloud_downsample": self.get_parameter(
                "pointcloud_downsample"
            ).value,
            "publish_compressed_monitoring": self.get_parameter(
                "publish_compressed_monitoring"
            ).value,
            "monitoring_publish_divisor": self.get_parameter(
                "monitoring_publish_divisor"
            ).value,
            "jpeg_quality": self.get_parameter("jpeg_quality").value,
            "monitoring_width": self.get_parameter("monitoring_width").value,
            "imu_rate_hz": self.get_parameter("imu_rate_hz").value,
            "detection_model": self.get_parameter(
                "detection_model"
            ).value,
            "yolo_model_path": self.get_parameter(
                "yolo_model_path"
            ).value,
            "confidence_threshold": self.get_parameter(
                "confidence_threshold"
            ).value,
            "publish_detection_images": self.get_parameter(
                "publish_detection_images"
            ).value,
            "draw_bounding_boxes": self.get_parameter(
                "draw_bounding_boxes"
            ).value,
            "detection_image_scale": self.get_parameter(
                "detection_image_scale"
            ).value,
        }

        config_file = str(self.get_parameter("config_file").value)
        if config_file:
            expanded = os.path.expanduser(config_file)
            if os.path.exists(expanded):
                try:
                    with open(expanded, "r", encoding="utf-8") as stream:
                        loaded = yaml.safe_load(stream)
                    if loaded and "direct_sdk" in loaded:
                        config.update(loaded["direct_sdk"])
                    self.get_logger().info(
                        f"Loaded direct SDK config: {expanded}"
                    )
                except Exception as exc:
                    self.get_logger().warning(
                        f"Failed to load config file {expanded}: {exc}"
                    )

        # PRODUCTION VSLAM safety: a future config file must not re-enable the
        # expensive publishers during production localization.
        config["publish_pointcloud"] = False
        config["publish_compressed_monitoring"] = False
        config["publish_detection_images"] = False
        config["publish_monitoring_thumbnails"] = True
        config["monitoring_publish_divisor"] = 5
        config["monitoring_width"] = 640

        return config

    def _setup_publishers(self) -> None:
        self.left_gray_publishers: Dict[str, Any] = {}
        self.right_gray_publishers: Dict[str, Any] = {}
        self.left_gray_info_publishers: Dict[str, Any] = {}
        self.right_gray_info_publishers: Dict[str, Any] = {}

        self.raw_color_publishers: Dict[str, Any] = {}
        self.raw_color_info_publishers: Dict[str, Any] = {}
        self.raw_depth_publishers: Dict[str, Any] = {}
        self.raw_depth_info_publishers: Dict[str, Any] = {}

        self.pointcloud_publishers: Dict[str, Any] = {}
        self.compressed_color_publishers: Dict[str, Any] = {}
        self.compressed_depth_publishers: Dict[str, Any] = {}
        self.monitoring_thumbnail_publishers: Dict[str, Any] = {}

        for name in ("front", "left", "right"):
            base = f"/zedx_{name}/zed_node"

            self.left_gray_publishers[name] = self.create_publisher(
                Image,
                f"{base}/left/gray/rect/image",
                self.sensor_qos,
            )
            self.left_gray_info_publishers[name] = self.create_publisher(
                CameraInfo,
                f"{base}/left/gray/rect/camera_info",
                self.sensor_qos,
            )
            self.right_gray_publishers[name] = self.create_publisher(
                Image,
                f"{base}/right/gray/rect/image",
                self.sensor_qos,
            )
            self.right_gray_info_publishers[name] = self.create_publisher(
                CameraInfo,
                f"{base}/right/gray/rect/camera_info",
                self.sensor_qos,
            )

            self.raw_color_publishers[name] = self.create_publisher(
                Image,
                f"{base}/left/color/rect/image",
                self.sensor_qos,
            )
            self.raw_color_info_publishers[name] = self.create_publisher(
                CameraInfo,
                f"{base}/left/color/rect/camera_info",
                self.sensor_qos,
            )
            self.raw_depth_publishers[name] = self.create_publisher(
                Image,
                f"{base}/depth/depth_registered",
                self.sensor_qos,
            )
            self.raw_depth_info_publishers[name] = self.create_publisher(
                CameraInfo,
                f"{base}/depth/camera_info",
                self.sensor_qos,
            )

            self.pointcloud_publishers[name] = self.create_publisher(
                PointCloud2,
                f"{base}/point_cloud/cloud_registered",
                self.reliable_qos,
            )
            self.monitoring_thumbnail_publishers[name] = self.create_publisher(
                Image,
                f"/monitor/zedx_{name}/image_raw",
                self.monitoring_qos,
            )

            self.compressed_color_publishers[name] = self.create_publisher(
                CompressedImage,
                f"{base}/left/image_rect_color/compressed",
                self.monitoring_qos,
            )
            self.compressed_depth_publishers[name] = self.create_publisher(
                CompressedImage,
                f"{base}/depth/depth_registered/compressedDepth",
                self.monitoring_qos,
            )

        self.imu_publisher = self.create_publisher(
            Imu,
            "/zedx_front/zed_node/imu/data",
            self.sensor_qos,
        )

        if self.enable_detection and PERCEPTION_MSGS_AVAILABLE:
            self.humans_publisher = self.create_publisher(
                HumansList,
                "/human_clustering/humans",
                self.reliable_qos,
            )
            self.objects_publisher = self.create_publisher(
                ObjectsList,
                "/object_tracking/objects",
                self.reliable_qos,
            )
            self.web_detections_publisher = self.create_publisher(
                String,
                "/web/camera_detections",
                self.reliable_qos,
            )
        else:
            self.humans_publisher = None
            self.objects_publisher = None
            self.web_detections_publisher = None

        self.detection_image_publishers: Dict[str, Any] = {}
        if self.enable_detection and bool(
            self.config["publish_detection_images"]
        ):
            for name in ("front", "left", "right"):
                self.detection_image_publishers[name] = self.create_publisher(
                    CompressedImage,
                    f"/viz/camera/zedx_{name}/"
                    "image_rect_color/compressed",
                    self.sensor_qos,
                )

    def _resolution_from_string(self, value: str) -> sl.RESOLUTION:
        mapping = {
            "HD2K": sl.RESOLUTION.HD2K,
            "HD1200": sl.RESOLUTION.HD1200,
            "HD1080": sl.RESOLUTION.HD1080,
            "HD720": sl.RESOLUTION.HD720,
            "SVGA": sl.RESOLUTION.SVGA,
            "VGA": sl.RESOLUTION.VGA,
        }
        return mapping.get(str(value).upper(), sl.RESOLUTION.SVGA)

    def _depth_mode_from_string(self, value: str) -> sl.DEPTH_MODE:
        mapping = {
            "NONE": sl.DEPTH_MODE.NONE,
            "PERFORMANCE": sl.DEPTH_MODE.PERFORMANCE,
            "QUALITY": sl.DEPTH_MODE.QUALITY,
            "ULTRA": sl.DEPTH_MODE.ULTRA,
            "NEURAL": sl.DEPTH_MODE.NEURAL,
            "NEURAL_LIGHT": sl.DEPTH_MODE.NEURAL_LIGHT,
            "NEURAL_PLUS": sl.DEPTH_MODE.NEURAL_PLUS,
        }
        return mapping.get(
            str(value).upper(),
            sl.DEPTH_MODE.NEURAL_LIGHT,
        )

    def _setup_cameras(self) -> bool:
        enabled = list(self.config["enabled_cameras"])
        configured = self.config["cameras"]

        for index, name in enumerate(("front", "left", "right")):
            if name not in enabled:
                continue

            # camera_config = configured[name]
            # depth_mode = self._depth_mode_from_string(
            #     camera_config["depth_mode"]
            # )
            # depth_enabled = depth_mode != sl.DEPTH_MODE.NONE

            camera_config = configured[name]

            depth_mode = self._depth_mode_from_string(
                camera_config["depth_mode"]
            )

            # In FULL mode, all cameras need depth for ZED object tracking/detection.
            # Left/right may normally use DEPTH_MODE.NONE when they are only used
            # as cuVSLAM stereo inputs, so force a lightweight depth mode here.
            if self.enable_detection and depth_mode == sl.DEPTH_MODE.NONE:
                depth_mode = sl.DEPTH_MODE.NEURAL_LIGHT

                self.get_logger().info(
                    f"{name}: FULL mode detection enabled; "
                    "forcing depth mode to NEURAL_LIGHT"
                )

            depth_enabled = depth_mode != sl.DEPTH_MODE.NONE

            resolution_name = self.config["resolution"]
            if name == "front" and self.config["front_resolution"]:
                resolution_name = self.config["front_resolution"]

            handler = CameraHandler(
                camera_name=name,
                serial_number=int(camera_config["serial"]),
                logger=self.get_logger(),
                depth_enabled=depth_enabled,
            )

            if handler.setup(
                resolution=self._resolution_from_string(resolution_name),
                depth_mode=depth_mode,
                fps=int(self.config["fps"]),
                enable_tracking=self.enable_detection,
                depth_min_m=float(
                    self.config["depth_minimum_distance"]
                ),
                depth_max_m=float(
                    self.config["depth_maximum_distance"]
                ),
            ):
                self.cameras[name] = handler
                self.last_published_sequence[name] = -1
            else:
                self.get_logger().error(
                    f"Camera initialization failed: {name}"
                )

            # Reproduce the established launch-relative pattern:
            # front at ~2 s, left at ~9 s, right at ~16 s.
            remaining = [
                item
                for item in ("front", "left", "right")
                if item in enabled
            ]
            if name != remaining[-1]:
                time.sleep(7.0)

        return bool(self.cameras)

    def _detection_model(
        self,
    ) -> Tuple[sl.OBJECT_DETECTION_MODEL, Optional[str]]:
        name = str(self.config["detection_model"]).upper()
        built_in = {
            "ZED_MULTI_CLASS_FAST":
                sl.OBJECT_DETECTION_MODEL.MULTI_CLASS_BOX_FAST,
            "ZED_MULTI_CLASS_MEDIUM":
                sl.OBJECT_DETECTION_MODEL.MULTI_CLASS_BOX_MEDIUM,
            "ZED_MULTI_CLASS_ACCURATE":
                sl.OBJECT_DETECTION_MODEL.MULTI_CLASS_BOX_ACCURATE,
            "ZED_PERSON_FAST":
                sl.OBJECT_DETECTION_MODEL.PERSON_HEAD_BOX_FAST,
            "ZED_PERSON_ACCURATE":
                sl.OBJECT_DETECTION_MODEL.PERSON_HEAD_BOX_ACCURATE,
        }

        if name in built_in:
            return built_in[name], None

        if "YOLO" in name:
            return (
                sl.OBJECT_DETECTION_MODEL.CUSTOM_YOLOLIKE_BOX_OBJECTS,
                str(self.config["yolo_model_path"]),
            )

        return (
            sl.OBJECT_DETECTION_MODEL.MULTI_CLASS_BOX_MEDIUM,
            None,
        )

    def _start_cameras(self) -> None:
        detection_model, detection_path = self._detection_model()

        for name, handler in self.cameras.items():
            if self.enable_detection:
                handler.enable_detection(
                    detection_model,
                    detection_path,
                )

            capture_pointcloud = (
                bool(self.config["publish_pointcloud"])
                and not self.fast_vslam_test
            )

            handler.start(
                confidence_threshold=int(
                    self.config["confidence_threshold"]
                ),
                enable_detection=self.enable_detection,
                pointcloud_downsample=int(
                    self.config["pointcloud_downsample"]
                ),
                capture_pointcloud=capture_pointcloud,
            )

            if name == "front":
                handler.start_imu_thread(
                    rate_hz=float(self.config["imu_rate_hz"]),
                    publisher=self.imu_publisher,
                    clock=self.get_clock(),
                    imu_frame="zedx_front_imu_link",
                )

    def _frame_stamp(
        self,
        camera_name: str,
        sdk_timestamp_ns: int,
    ) -> TimeMsg:
        # A single SDK-to-ROS offset is correct for front-only operation and
        # keeps every ZED stream in one ROS time domain if side cameras are
        # enabled later.
        del camera_name

        if self.camera_ros_offset_ns is None:
            self.camera_ros_offset_ns = (
                int(self.get_clock().now().nanoseconds)
                - sdk_timestamp_ns
            )

        ros_ns = sdk_timestamp_ns + self.camera_ros_offset_ns
        return ros_time_from_nanoseconds(ros_ns)

    def _camera_info_pair(
        self,
        camera_name: str,
        calibration: StereoCalibration,
        stamp: TimeMsg,
    ) -> Tuple[CameraInfo, CameraInfo]:
        left_frame = (
            f"zedx_{camera_name}_left_camera_frame_optical"
        )
        right_frame = (
            f"zedx_{camera_name}_right_camera_frame_optical"
        )

        left = CameraInfo()
        left.header.stamp = stamp
        left.header.frame_id = left_frame
        left.width = calibration.width
        left.height = calibration.height
        left.distortion_model = "plumb_bob"
        left.d = [0.0] * 5
        left.k = [
            calibration.left_fx, 0.0, calibration.left_cx,
            0.0, calibration.left_fy, calibration.left_cy,
            0.0, 0.0, 1.0,
        ]
        left.r = [
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0,
        ]
        left.p = [
            calibration.left_fx, 0.0, calibration.left_cx, 0.0,
            0.0, calibration.left_fy, calibration.left_cy, 0.0,
            0.0, 0.0, 1.0, 0.0,
        ]

        right = CameraInfo()
        right.header.stamp = stamp
        right.header.frame_id = right_frame
        right.width = calibration.width
        right.height = calibration.height
        right.distortion_model = "plumb_bob"
        right.d = [0.0] * 5
        right.k = [
            calibration.right_fx, 0.0, calibration.right_cx,
            0.0, calibration.right_fy, calibration.right_cy,
            0.0, 0.0, 1.0,
        ]
        right.r = [
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0,
        ]
        right.p = [
            calibration.right_fx,
            0.0,
            calibration.right_cx,
            -calibration.right_fx * calibration.baseline_m,
            0.0,
            calibration.right_fy,
            calibration.right_cy,
            0.0,
            0.0,
            0.0,
            1.0,
            0.0,
        ]

        return left, right

    def _publish_callback(self) -> None:
        if not hasattr(self, "_monitoring_counter"):
            self._monitoring_counter = 0
        self._monitoring_counter += 1

        all_persons: List[PersonDetection] = []
        all_objects: List[ObjectDetection] = []
        detections_by_camera: Dict[str, List[PersonDetection]] = {}

        for name, handler in self.cameras.items():
            frame = handler.get_frame(
                self.last_published_sequence[name]
            )
            if frame is None:
                continue

            self.last_published_sequence[name] = frame.sequence

            if handler.calibration is None:
                continue

            stamp = self._frame_stamp(
                name,
                frame.sdk_timestamp_ns,
            )

            # Navigation sensor path is FRONT-CAMERA ONLY (production architecture).
            # FULL mode may still open front + left + right for object detection,
            # but side cameras must not feed cuVSLAM or nvblox.
            if name == "front":
                self._publish_vslam_inputs(
                    name,
                    frame,
                    handler.calibration,
                    stamp,
                )
                self._publish_nvblox_inputs(
                    name,
                    frame,
                    handler.calibration,
                    stamp,
                )

                if (
                    bool(self.config["publish_pointcloud"])
                    and not self.fast_vslam_test
                ):
                    self._publish_pointcloud(name, frame, stamp)

            divisor = max(
                1,
                int(self.config["monitoring_publish_divisor"]),
            )
            if (
                bool(self.config["publish_monitoring_thumbnails"])
                and self._monitoring_counter % divisor == 0
            ):
                self._publish_monitoring_thumbnail(name, frame, stamp)

            if self.enable_detection:
                for detection in frame.persons:
                    detection.position = self._transform_to_base(
                        detection.raw_position,
                        name,
                    )
                for detection in frame.objects:
                    detection.position = self._transform_to_base(
                        detection.raw_position,
                        name,
                    )

                detections_by_camera[name] = frame.persons
                all_persons.extend(frame.persons)
                all_objects.extend(frame.objects)

                if name in self.detection_image_publishers:
                    self._publish_detection_image(
                        name,
                        frame,
                        stamp,
                    )

        if self.enable_detection:
            self._publish_detection_messages(
                all_persons,
                all_objects,
                detections_by_camera,
            )

    def _publish_vslam_inputs(
        self,
        name: str,
        frame: CameraFrame,
        calibration: StereoCalibration,
        stamp: TimeMsg,
    ) -> None:
        left_frame = f"zedx_{name}_left_camera_frame_optical"
        right_frame = f"zedx_{name}_right_camera_frame_optical"

        left_image = self.bridge.cv2_to_imgmsg(
            frame.left_gray,
            encoding="mono8",
        )
        left_image.header.stamp = stamp
        left_image.header.frame_id = left_frame

        right_image = self.bridge.cv2_to_imgmsg(
            frame.right_gray,
            encoding="mono8",
        )
        right_image.header.stamp = stamp
        right_image.header.frame_id = right_frame

        left_info, right_info = self._camera_info_pair(
            name,
            calibration,
            stamp,
        )

        self.left_gray_publishers[name].publish(left_image)
        self.left_gray_info_publishers[name].publish(left_info)
        self.right_gray_publishers[name].publish(right_image)
        self.right_gray_info_publishers[name].publish(right_info)

    def _publish_nvblox_inputs(
        self,
        name: str,
        frame: CameraFrame,
        calibration: StereoCalibration,
        stamp: TimeMsg,
    ) -> None:
        optical_frame = f"zedx_{name}_left_camera_frame_optical"

        color_rgb = cv2.cvtColor(
            frame.left_color_bgr,
            cv2.COLOR_BGR2RGB,
        )

        color = self.bridge.cv2_to_imgmsg(
            color_rgb,
            encoding="rgb8",
        )
        color.header.stamp = stamp
        color.header.frame_id = optical_frame

        left_info, _ = self._camera_info_pair(
            name,
            calibration,
            stamp,
        )
        color_info = copy.deepcopy(left_info)
        color_info.header.frame_id = optical_frame

        self.raw_color_publishers[name].publish(color)
        self.raw_color_info_publishers[name].publish(color_info)

        if frame.depth_m is None:
            return

        depth = np.asarray(frame.depth_m, dtype=np.float32).copy()
        depth[~np.isfinite(depth)] = 0.0
        depth[depth < 0.0] = 0.0

        depth_msg = self.bridge.cv2_to_imgmsg(
            depth,
            encoding="32FC1",
        )
        depth_msg.header.stamp = stamp
        depth_msg.header.frame_id = optical_frame

        depth_info = copy.deepcopy(left_info)
        depth_info.header.frame_id = optical_frame

        self.raw_depth_publishers[name].publish(depth_msg)
        self.raw_depth_info_publishers[name].publish(depth_info)

    def _publish_pointcloud(
        self,
        name: str,
        frame: CameraFrame,
        stamp: TimeMsg,
    ) -> None:
        if frame.point_cloud_xyz is None:
            return
        if len(frame.point_cloud_xyz) == 0:
            return

        header = Header()
        header.stamp = stamp
        header.frame_id = f"zedx_{name}_left_camera_frame"
        message = point_cloud2.create_cloud_xyz32(
            header,
            frame.point_cloud_xyz.tolist(),
        )
        self.pointcloud_publishers[name].publish(message)

    def _publish_monitoring_thumbnail(
        self,
        name: str,
        frame: CameraFrame,
        stamp: TimeMsg,
    ) -> None:
        """Publish a tiny raw operator-view thumbnail.

        IMPORTANT:
        This function does NO JPEG/PNG compression. JPEG encoding is performed
        by a separate low-priority process (remote_camera_compressor_node.py).

        At the default 15 Hz source and divisor=5 this publishes ~3 Hz.
        BEST_EFFORT + KEEP_LAST(1) ensures stale thumbnails are discarded.
        """
        try:
            image = frame.left_color_bgr
            target_width = max(
                160,
                int(self.config.get("monitoring_width", 640)),
            )

            height, width = image.shape[:2]
            if width > target_width:
                scale = target_width / float(width)
                target_height = max(1, int(round(height * scale)))
                image = cv2.resize(
                    image,
                    (target_width, target_height),
                    interpolation=cv2.INTER_AREA,
                )

            msg = self.bridge.cv2_to_imgmsg(
                image,
                encoding="bgr8",
            )
            msg.header.stamp = stamp
            msg.header.frame_id = (
                f"zedx_{name}_left_camera_frame_optical"
            )

            self.monitoring_thumbnail_publishers[name].publish(msg)

        except Exception as exc:
            self.get_logger().warning(
                f"{name}: monitoring thumbnail publication failed: {exc}"
            )

    def _transform_to_base(
        self,
        raw_position: List[float],
        camera_name: str,
    ) -> Point:
        # SDK coordinate system is X forward, Y left, Z up.
        transform = CAMERA_TRANSFORMS[camera_name]
        yaw = transform["yaw"]
        offset = transform["position"]

        x = float(raw_position[0])
        y = float(raw_position[1])
        z = float(raw_position[2])

        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)

        return Point(
            x=x * cos_yaw - y * sin_yaw + offset[0],
            y=x * sin_yaw + y * cos_yaw + offset[1],
            z=z + offset[2],
        )

    def _transform_velocity(
        self,
        velocity: List[float],
        camera_name: str,
    ) -> List[float]:
        yaw = CAMERA_TRANSFORMS[camera_name]["yaw"]
        x = float(velocity[0])
        y = float(velocity[1])
        z = float(velocity[2])
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)

        return [
            x * cos_yaw - y * sin_yaw,
            x * sin_yaw + y * cos_yaw,
            z,
        ]

    def _publish_detection_messages(
        self,
        persons: List[PersonDetection],
        objects: List[ObjectDetection],
        by_camera: Dict[str, List[PersonDetection]],
    ) -> None:
        if (
            PERCEPTION_MSGS_AVAILABLE
            and self.humans_publisher is not None
            and persons
        ):
            msg = HumansList()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "base_footprint"

            stationary_count = 0
            for detection in persons:
                item = HumanTrackingData()
                item.header.stamp = msg.header.stamp
                item.header.frame_id = "base_footprint"
                item.position = detection.position
                item.tracking_id = detection.tracking_id
                item.distance = detection.distance
                item.confidence = detection.confidence
                item.camera_name = detection.camera_name

                velocity = self._transform_velocity(
                    detection.velocity,
                    detection.camera_name,
                )
                item.velocity = Vector3(
                    x=velocity[0],
                    y=velocity[1],
                    z=velocity[2],
                )
                item.velocity_magnitude = float(
                    np.linalg.norm(velocity)
                )
                if item.velocity_magnitude < 0.5:
                    stationary_count += 1
                msg.humans.append(item)

            msg.human_count = len(msg.humans)
            msg.stationary_count = stationary_count
            self.humans_publisher.publish(msg)

        if (
            PERCEPTION_MSGS_AVAILABLE
            and self.objects_publisher is not None
            and objects
        ):
            msg = ObjectsList()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "base_footprint"

            for detection in objects:
                item = ObjectTrackingData()
                item.tracking_id = detection.tracking_id
                item.class_id = detection.class_id
                item.class_name = detection.class_name
                item.position = detection.position
                item.confidence = detection.confidence
                item.distance = detection.distance
                item.camera_name = detection.camera_name

                velocity = self._transform_velocity(
                    detection.velocity,
                    detection.camera_name,
                )
                item.velocity = Vector3(
                    x=velocity[0],
                    y=velocity[1],
                    z=velocity[2],
                )
                item.velocity_magnitude = float(
                    np.linalg.norm(velocity)
                )
                msg.objects.append(item)

            msg.object_count = len(msg.objects)
            self.objects_publisher.publish(msg)

        if self.web_detections_publisher is not None:
            output: Dict[str, Any] = {}
            for camera_name, detections in by_camera.items():
                output[camera_name] = {
                    "count": len(detections),
                    "detections": [
                        {
                            "label": "PERSON",
                            "label_id": item.tracking_id,
                            "confidence": item.confidence,
                            "bbox": item.bounding_box_2d,
                            "position": {
                                "x": item.position.x,
                                "y": item.position.y,
                                "z": item.position.z,
                            },
                            "distance": item.distance,
                        }
                        for item in detections
                    ],
                }

            message = String()
            message.data = json.dumps(output)
            self.web_detections_publisher.publish(message)

    def _publish_detection_image(
        self,
        name: str,
        frame: CameraFrame,
        stamp: TimeMsg,
    ) -> None:
        image = frame.left_color_bgr.copy()

        if bool(self.config["draw_bounding_boxes"]):
            for detection in frame.persons:
                if len(detection.bounding_box_2d) < 4:
                    continue
                x1 = int(detection.bounding_box_2d[0][0])
                y1 = int(detection.bounding_box_2d[0][1])
                x2 = int(detection.bounding_box_2d[2][0])
                y2 = int(detection.bounding_box_2d[2][1])
                cv2.rectangle(image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(
                    image,
                    f"ID:{detection.tracking_id} "
                    f"{detection.distance:.1f}m",
                    (x1, max(20, y1 - 7)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.55,
                    (0, 255, 0),
                    2,
                )

        scale = float(self.config["detection_image_scale"])
        if scale != 1.0:
            height, width = image.shape[:2]
            image = cv2.resize(
                image,
                (
                    max(1, int(width * scale)),
                    max(1, int(height * scale)),
                ),
            )

        ok, encoded = cv2.imencode(
            ".jpg",
            image,
            [
                cv2.IMWRITE_JPEG_QUALITY,
                int(self.config["jpeg_quality"]),
            ],
        )
        if not ok:
            return

        message = CompressedImage()
        message.header.stamp = stamp
        message.header.frame_id = (
            f"zedx_{name}_left_camera_frame_optical"
        )
        message.format = "jpeg"
        message.data = encoded.tobytes()
        self.detection_image_publishers[name].publish(message)

    def _status_callback(self) -> None:
        camera_status = []
        for name, handler in self.cameras.items():
            camera_status.append(
                f"{name}:{handler.get_fps():.1f}fps/"
                f"{handler.grab_errors}err"
            )

        test_tag = " VSLAM-PROD" if self.fast_vslam_test else ""
        self.get_logger().info(
            f"[{self.mode.upper()}{test_tag}] "
            + " | ".join(camera_status)
        )

    def destroy_node(self) -> bool:
        self.get_logger().info("Stopping direct ZED SDK driver")
        for handler in self.cameras.values():
            handler.cleanup()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node: Optional[DirectZedDetectionNodeModified] = None

    try:
        node = DirectZedDetectionNodeModified()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        if node is not None:
            node.get_logger().fatal(f"Fatal direct ZED error: {exc}")
        else:
            print(f"Fatal direct ZED error: {exc}")
        raise
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()