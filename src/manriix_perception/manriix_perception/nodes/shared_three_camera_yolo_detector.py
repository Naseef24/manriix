#!/usr/bin/env python3
"""
Shared 2D detector for MANRIIX three-ZED architecture.

Inputs (already validated lightweight monitor topics):
  /monitor/zedx_front/image_raw
  /monitor/zedx_left/image_raw
  /monitor/zedx_right/image_raw

One YOLO model is shared across all cameras.
Latest-frame-only buffers prevent inference backlog.

Output:
  /web/camera_detections
  std_msgs/String JSON

2D only:
- class
- confidence
- bounding box

No side-camera depth is needed.
"""

import json
import threading
import time
import cv2
import math
import numpy as np

import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
import tf2_geometry_msgs

from tf2_ros import Buffer, TransformListener
from std_msgs.msg import String
from manriix_perception.msg import (
    HumansList,
    HumanTrackingData,
    ObjectsList,
    ObjectTrackingData,
)

class SharedThreeCameraYolo(Node):
    def __init__(self):
        super().__init__("shared_three_camera_yolo_detector")

        self.declare_parameter(
            "model_path",
            "/home/hype/models/yolo11/yolo11s.pt",
        )
        self.declare_parameter("confidence", 0.25)
        self.declare_parameter("imgsz", 640)
        self.declare_parameter("tracker", "bytetrack.yaml")
        self.declare_parameter("max_total_inference_hz", 9.0)
        self.declare_parameter("device", "0")

        self.model_path = str(self.get_parameter("model_path").value)
        self.confidence = float(self.get_parameter("confidence").value)
        self.tracker = str(self.get_parameter("tracker").value)
        self.imgsz = int(self.get_parameter("imgsz").value)
        self.max_hz = float(
            self.get_parameter("max_total_inference_hz").value
        )
        self.device = str(self.get_parameter("device").value)

        from ultralytics import YOLO

        self.get_logger().info(
            f"Loading shared YOLO: {self.model_path}, device={self.device}"
        )
        self.model = YOLO(self.model_path)

        self.bridge = CvBridge()

        # TF2 transform: camera optical frame -> robot base frame
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(
            self.tf_buffer,
            self,
        )

        self.lock = threading.Lock()

        self.cameras = ("front", "left", "right")

        self.latest = {
            c: None
            for c in self.cameras
        }

        # Depth and calibration buffers for 2D -> 3D fusion
        self.latest_depth = {
            c: None
            for c in self.cameras
        }

        self.camera_info = {
            c: None
            for c in self.cameras
        }

        # Store actual incoming monitor image dimensions per camera
        self.image_shapes = {}

        self.last_processed = {
            c: -1
            for c in self.cameras
        }

        self.next_index = 0

        self.input_count = {
            c: 0
            for c in self.cameras
        }

        self.infer_count = {
            c: 0
            for c in self.cameras
        }

        self.stats_started = time.monotonic()

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # for camera in self.cameras:
        #     self.create_subscription(
        #         Image,
        #         f"/zedx_{camera}/zed_node/left/color/rect/image",
        #         lambda msg, c=camera: self._image_cb(c, msg),
        #         qos,
        #     )

        for camera in self.cameras:
            self.create_subscription(
                Image,
                f"/monitor/zedx_{camera}/image_raw",
                lambda msg, c=camera: self._image_cb(c, msg),
                qos,
            )

        # Depth + calibration inputs for all three ZED-X cameras
        for camera in self.cameras:
            self.create_subscription(
                Image,
                f"/zedx_{camera}/zed_node/depth/depth_registered",
                lambda msg, c=camera: self._depth_cb(c, msg),
                qos,
            )

            self.create_subscription(
                CameraInfo,
                f"/zedx_{camera}/zed_node/left/color/rect/camera_info",
                lambda msg, c=camera: self._camera_info_cb(c, msg),
                qos,
            )

        self.pub = self.create_publisher(
            String,
            "/web/camera_detections",
            10,
        )

        self.humans_pub = self.create_publisher(
            HumansList,
            "/human_clustering/humans",
            10,
        )

        self.objects_pub = self.create_publisher(
            ObjectsList,
            "/object_tracking/objects",
            10,
        )

        self.detection_image_pubs = {}

        for camera in self.cameras:
            self.detection_image_pubs[camera] = self.create_publisher(
                Image,
                f"/viz/camera/zedx_{camera}/detection",
                10,
            )

        self.create_timer(
            1.0 / max(1.0, self.max_hz),
            self._infer_tick,
        )

        self.create_timer(
            10.0,
            self._stats,
        )

        self.get_logger().info(
            "Shared detector ready: latest-frame-only, "
            f"total inference cap={self.max_hz:.1f} Hz"
        )

        self.get_logger().info(
            "Detection topics active: "
            "/web/camera_detections | "
            "/human_clustering/humans | "
            "/object_tracking/objects"
        )

    def _image_cb(self, camera, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding="bgr8",
            )
        except Exception as exc:
            self.get_logger().warning(
                f"{camera}: cv_bridge failed: {exc}"
            )
            return

        stamp_ns = (
            int(msg.header.stamp.sec) * 1_000_000_000
            + int(msg.header.stamp.nanosec)
        )

        with self.lock:
            self.latest[camera] = (
                stamp_ns,
                frame,
            )

            # Store the actual monitor-image geometry used by YOLO.
            # This is required because the front depth stream is 960x600
            # while the monitor image is 640x400.
            self.image_shapes[camera] = frame.shape[:2]

        self.input_count[camera] += 1


    def _depth_cb(self, camera, msg):
        try:
            depth = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding="32FC1",
            )
            with self.lock:
                stamp_ns = (
                int(msg.header.stamp.sec) * 1_000_000_000
                + int(msg.header.stamp.nanosec)
            )

            with self.lock:
                self.latest_depth[camera] = (
                    stamp_ns,
                    depth,
                )
        except Exception as exc:
            self.get_logger().warning(
                f"{camera}: depth conversion failed: {exc}"
            )

    def _camera_info_cb(self, camera, msg):
        with self.lock:
            self.camera_info[camera] = msg

    def _estimate_3d_position(self, camera, bbox):
        """
        Estimate target geometry from a YOLO bounding box.

        FRONT:
            Uses the registered metric depth image. YOLO runs on the resized
            monitor image, so bbox coordinates are scaled to the depth image
            before depth lookup and native CameraInfo projection.

        LEFT / RIGHT:
            Side cameras intentionally run without ZED depth. Use CameraInfo
            to convert the bbox centre into a calibrated optical viewing ray.
            A nominal 5 m range represents direction only; it is not a metric
            distance estimate.

        Returns:
            (x, y, z, distance) in the camera optical frame, or None when the
            target geometry cannot be determined safely.
        """

        # Bounding-box centre in the detector/monitor image.
        u_monitor = 0.5 * (bbox[0][0] + bbox[2][0])
        v_monitor = 0.5 * (bbox[0][1] + bbox[2][1])

        with self.lock:
            depth_entry = self.latest_depth.get(camera)
            info = self.camera_info.get(camera)
            monitor_shape = self.image_shapes.get(camera)

        if info is None:
            self.get_logger().warning(
                f"No CameraInfo available for {camera}"
            )
            return None

        fx = float(info.k[0])
        fy = float(info.k[4])
        cx = float(info.k[2])
        cy = float(info.k[5])

        if fx <= 0.0 or fy <= 0.0:
            self.get_logger().warning(
                f"Invalid CameraInfo intrinsics for {camera}"
            )
            return None

        # LEFT / RIGHT: no metric depth by design; use calibrated viewing ray.
        if depth_entry is None:
            ray_x = (u_monitor - cx) / fx
            ray_y = (v_monitor - cy) / fy
            ray_z = 1.0

            ray_norm = math.sqrt(
                ray_x * ray_x
                + ray_y * ray_y
                + ray_z * ray_z
            )

            if ray_norm < 1e-6:
                return None

            ray_x /= ray_norm
            ray_y /= ray_norm
            ray_z /= ray_norm

            nominal_range = 5.0

            return (
                ray_x * nominal_range,
                ray_y * nominal_range,
                ray_z * nominal_range,
                nominal_range,
            )

        # FRONT: map monitor-image bbox centre into native depth geometry.
        depth = depth_entry[1]

        if depth is None or depth.ndim != 2:
            return None

        depth_height, depth_width = depth.shape[:2]

        if monitor_shape is None:
            self.get_logger().warning(
                f"No monitor image geometry available for {camera}"
            )
            return None

        monitor_height, monitor_width = monitor_shape

        if monitor_width <= 0 or monitor_height <= 0:
            return None

        scale_x = depth_width / float(monitor_width)
        scale_y = depth_height / float(monitor_height)

        u_depth = int(round(u_monitor * scale_x))
        v_depth = int(round(v_monitor * scale_y))

        u_depth = max(0, min(depth_width - 1, u_depth))
        v_depth = max(0, min(depth_height - 1, v_depth))

        # Use a small patch around the bbox centre and take the median
        # of valid depth samples for better robustness than a single pixel.
        radius = 4

        x1 = max(0, u_depth - radius)
        x2 = min(depth_width, u_depth + radius + 1)
        y1 = max(0, v_depth - radius)
        y2 = min(depth_height, v_depth + radius + 1)

        patch = depth[y1:y2, x1:x2]

        valid = patch[
            np.isfinite(patch)
            & (patch > 0.05)
            & (patch < 50.0)
        ]

        if valid.size == 0:
            # Never return (0,0,0): after TF that becomes the camera origin.
            return None

        z = float(np.median(valid))

        x = (u_depth - cx) * z / fx
        y = (v_depth - cy) * z / fy

        distance = math.sqrt(
            x * x
            + y * y
            + z * z
        )

        return (
            x,
            y,
            z,
            distance,
        )


    def _transform_to_base(
        self,
        camera,
        x,
        y,
        z,
    ):
        """Transform ZED optical-frame XYZ into base_footprint."""

        point = PointStamped()

        point.header.frame_id = (
            f"zedx_{camera}_left_camera_frame_optical"
        )

        point.header.stamp = (
            self.get_clock().now().to_msg()
        )

        point.point.x = x
        point.point.y = y
        point.point.z = z

        try:
            transformed = self.tf_buffer.transform(
                point,
                "base_footprint",
                timeout=rclpy.duration.Duration(
                    seconds=0.2
                ),
            )

            return (
                float(transformed.point.x),
                float(transformed.point.y),
                float(transformed.point.z),
            )

        except Exception as exc:
            # self.get_logger().warning(
            #     f"TF transform failed {camera}->base_footprint: {exc}"
            # )
            self.get_logger().warning(
                f"TF transform failed "
                f"zedx_{camera}_left_camera_frame_optical "
                f"-> base_footprint: {exc}"
            )
            return x, y, z


    def _select(self):
        for offset in range(len(self.cameras)):
            index = (
                self.next_index + offset
            ) % len(self.cameras)

            camera = self.cameras[index]

            with self.lock:
                item = self.latest[camera]

            if item is None:
                continue

            stamp_ns, frame = item

            if stamp_ns == self.last_processed[camera]:
                continue

            self.next_index = (
                index + 1
            ) % len(self.cameras)

            return camera, stamp_ns, frame

        return None

    def _infer_tick(self):
        selected = self._select()

        if selected is None:
            return

        camera, stamp_ns, frame = selected

        self.last_processed[camera] = stamp_ns

        start = time.monotonic()

        try:
            result = self.model.track(
                source=frame,
                conf=self.confidence,
                imgsz=self.imgsz,
                device=self.device,
                tracker=self.tracker,
                persist=True,
                verbose=False,
            )[0]
        except Exception as exc:
            self.get_logger().error(
                f"{camera}: inference failed: {exc}"
            )
            return

        inference_ms = (
            time.monotonic() - start
        ) * 1000.0

        detections = []

        boxes = result.boxes

        if boxes is not None:
            xyxy = boxes.xyxy.detach().cpu().numpy()
            confs = boxes.conf.detach().cpu().numpy()
            classes = (
                boxes.cls.detach()
                .cpu()
                .numpy()
                .astype(int)
            )

            track_ids = None

            if getattr(boxes, "id", None) is not None:
                track_ids = (
                    boxes.id
                    .detach()
                    .cpu()
                    .numpy()
                    .astype(int)
                )

            for det_index, (bbox, conf, class_id) in enumerate(
                zip(
                    xyxy,
                    confs,
                    classes,
                )
            ):
                x1, y1, x2, y2 = [
                    float(v)
                    for v in bbox
                ]

                detections.append({
                    "label": str(
                        result.names.get(
                            int(class_id),
                            int(class_id),
                        )
                    ),
                    "label_id": int(class_id),
                    "tracking_id": (
                        int(track_ids[det_index])
                        if track_ids is not None
                        else int(det_index)
                    ),
                    "confidence": float(conf * 100.0),
                    "bbox": [
                        [x1, y1],
                        [x2, y1],
                        [x2, y2],
                        [x1, y2],
                    ],
                })

        payload = {
            camera: {
                "count": len(detections),
                "detections": detections,
                "stamp_ns": int(stamp_ns),
                "inference_ms": round(
                    inference_ms,
                    2,
                ),
                "source": "shared_yolo11n_2d",
            }
        }

        msg = String()
        msg.data = json.dumps(payload)
        self.pub.publish(msg)

        self._publish_tracking_messages(
            camera,
            stamp_ns,
            detections,
        )

        self._publish_detection_image(
            camera,
            frame,
            detections,
            stamp_ns,
        )

        self.infer_count[camera] += 1

        # msg = String()
        # msg.data = json.dumps(payload)
        # self.pub.publish(msg)

        # self._publish_tracking_messages(
        #     camera,
        #     frame,
        #     stamp_ns,
        #     detections,
        # )
        
        # self.infer_count[camera] += 1

    def _publish_detection_image(
        self,
        camera,
        frame,
        detections,
        stamp_ns,
    ):
        annotated = frame.copy()

        for det in detections:

            bbox = det["bbox"]

            x1 = int(bbox[0][0])
            y1 = int(bbox[0][1])
            x2 = int(bbox[2][0])
            y2 = int(bbox[2][1])

            label = det["label"]

            confidence = det["confidence"]

            text = (
                f"{label} "
                f"{confidence:.1f}%"
            )

            cv2.rectangle(
                annotated,
                (x1, y1),
                (x2, y2),
                (0,255,0),
                2,
            )

            cv2.putText(
                annotated,
                text,
                (x1, max(20,y1-5)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0,255,0),
                2,
            )


        msg = self.bridge.cv2_to_imgmsg(
            annotated,
            encoding="bgr8",
        )

        msg.header.stamp.sec = int(
            stamp_ns // 1_000_000_000
        )

        msg.header.stamp.nanosec = int(
            stamp_ns % 1_000_000_000
        )

        msg.header.frame_id = (
            f"zedx_{camera}_camera"
        )

        self.detection_image_pubs[camera].publish(
            msg
        )

    def _stats(self):
        elapsed = max(
            0.001,
            time.monotonic() - self.stats_started,
        )

        parts = []

        for camera in self.cameras:
            parts.append(
                f"{camera}:"
                f"in={self.input_count[camera]/elapsed:.1f}Hz/"
                f"infer={self.infer_count[camera]/elapsed:.1f}Hz"
            )

        self.get_logger().info(
            "[SHARED-YOLO] "
            + " | ".join(parts)
        )

        for camera in self.cameras:
            self.input_count[camera] = 0
            self.infer_count[camera] = 0

        self.stats_started = time.monotonic()

    def _publish_tracking_messages(
        self,
        camera,
        stamp_ns,
        detections,
    ):

        humans_msg = HumansList()
        objects_msg = ObjectsList()


        humans_msg.header.stamp.sec = int(
            stamp_ns // 1_000_000_000
        )

        humans_msg.header.stamp.nanosec = int(
            stamp_ns % 1_000_000_000
        )

        humans_msg.header.frame_id = (
            "base_footprint"
        )


        objects_msg.header.stamp.sec = int(
            stamp_ns // 1_000_000_000
        )

        objects_msg.header.stamp.nanosec = int(
            stamp_ns % 1_000_000_000
        )

        objects_msg.header.frame_id = (
            "base_footprint"
        )


        human_count = 0
        object_count = 0


        for idx, det in enumerate(detections):

            label = det["label"]

            confidence = (
                det["confidence"]
            )

            class_id = (
                det["label_id"]
            )

            # x3d, y3d, z3d, distance = self._estimate_3d_position(
            #     camera,
            #     det["bbox"],
            # )
            position_result = self._estimate_3d_position(
                camera,
                det["bbox"],
            )

            if position_result is None:
                self.get_logger().warning(
                    f"Skipping {camera} detection: "
                    "unable to determine target geometry"
                )
                continue

            x3d, y3d, z3d, distance = position_result

            # Convert camera optical coordinates to robot coordinates
            x3d, y3d, z3d = self._transform_to_base(
                camera,
                x3d,
                y3d,
                z3d,
            )

            distance = (
                x3d * x3d
                + y3d * y3d
                + z3d * z3d
            ) ** 0.5


            # if label == "person":

            #     human = HumanTrackingData()

            #     human.header = (
            #         humans_msg.header
            #     )

            #     human.tracking_id = int(det.get("tracking_id", idx))

            #     human.confidence = (
            #         confidence
            #     )

            #     human.distance = distance

            #     human.camera_name = camera

            #     human.position.x = x3d
            #     human.position.y = y3d
            #     human.position.z = z3d


            #     humans_msg.humans.append(
            #         human
            #     )

            #     human_count += 1


            # else:

            #     obj = ObjectTrackingData()

            #     obj.tracking_id = int(det.get("tracking_id", idx))

            #     obj.class_id = (
            #         class_id
            #     )

            #     obj.class_name = (
            #         label
            #     )

            #     obj.confidence = (
            #         confidence
            #     )

            #     obj.distance = distance

            #     obj.camera_name = camera

            #     obj.position.x = x3d
            #     obj.position.y = y3d
            #     obj.position.z = z3d


            #     objects_msg.objects.append(
            #         obj
            #     )

            # object_count += 1

            if label == "person":

                human = HumanTrackingData()

                human.header = humans_msg.header
                human.tracking_id = int(
                    det.get("tracking_id", -1)
                )
                human.confidence = confidence
                human.distance = distance
                human.camera_name = camera

                human.position.x = x3d
                human.position.y = y3d
                human.position.z = z3d

                humans_msg.humans.append(
                    human
                )

                human_count += 1


            # IMPORTANT:
            # Do not use else here.
            # Every detection including PERSON must enter ObjectsList.

            obj = ObjectTrackingData()

            obj.tracking_id = int(
                det.get("tracking_id", -1)
            )

            obj.class_id = class_id
            obj.class_name = label
            obj.confidence = confidence
            obj.distance = distance
            obj.camera_name = camera

            obj.position.x = x3d
            obj.position.y = y3d
            obj.position.z = z3d

            objects_msg.objects.append(obj)

            object_count += 1

        humans_msg.human_count = (
            human_count
        )

        humans_msg.stationary_count = (
            human_count
        )


        objects_msg.object_count = (
            object_count
        )


        self.humans_pub.publish(
            humans_msg
        )

        self.objects_pub.publish(
            objects_msg
        )

def main(args=None):
    rclpy.init(args=args)
    node = SharedThreeCameraYolo()

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