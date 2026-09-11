#!/usr/bin/env python3
"""
manriix_oak_obstacles — OAK-D on-chip edge-based obstacle detection.

Architecture:
  OAK chip (Myriad X): color capture, stereo depth, grayscale conversion,
                       Sobel edge detection — all in hardware
  Host (Jetson):       RANSAC floor (cached), mask arithmetic,
                       connected components, ROS2 publishing

Topics published:
  /oak_obstacles/detections                       vision_msgs/Detection3DArray
  /oak_obstacles/image_overlay/compressed         sensor_msgs/CompressedImage
  /oak_obstacles/obstacle_mask                    sensor_msgs/Image (optional)
  /oak_obstacles/floor_plane                      geometry_msgs/PolygonStamped

Frame: oak_d_pro_w_rgb_camera_optical_frame (camera optical convention)
"""

import time
from collections import deque

import cv2
import numpy as np
import depthai as dai

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy)

from sensor_msgs.msg import CompressedImage, Image, PointCloud2, PointField
from vision_msgs.msg import (
    Detection3D, Detection3DArray, BoundingBox3D,
    ObjectHypothesisWithPose)
from geometry_msgs.msg import (
    PolygonStamped, Point32, Pose, Vector3)
from std_msgs.msg import Header
from cv_bridge import CvBridge


def make_qos(depth: int = 5,
             reliable: bool = True,
             durable: bool = False) -> QoSProfile:
    return QoSProfile(
        depth=depth,
        history=QoSHistoryPolicy.KEEP_LAST,
        reliability=(QoSReliabilityPolicy.RELIABLE if reliable
                     else QoSReliabilityPolicy.BEST_EFFORT),
        durability=(QoSDurabilityPolicy.TRANSIENT_LOCAL if durable
                    else QoSDurabilityPolicy.VOLATILE),
    )


class OakObstacleNode(Node):
    def __init__(self):
        super().__init__('oak_obstacle_node')

        # ── Parameters ──
        self.declare_parameter('overlay_topic',
                               '/oak_obstacles/image_overlay/compressed')
        self.declare_parameter('detections_topic',
                               '/oak_obstacles/detections')
        self.declare_parameter('mask_topic',
                               '/oak_obstacles/obstacle_mask')
        self.declare_parameter('plane_topic',
                               '/oak_obstacles/floor_plane')

        self.declare_parameter('camera_frame',
                               'oak_d_pro_w_rgb_camera_optical_frame')

        self.declare_parameter('image_width', 640)
        self.declare_parameter('image_height', 400)
        self.declare_parameter('camera_fps', 15)

        self.declare_parameter('floor_thresh_m', 0.03)
        self.declare_parameter('ransac_interval_frames', 5)
        self.declare_parameter('ransac_iterations', 60)

        self.declare_parameter('edge_threshold', 60)
        self.declare_parameter('min_obstacle_area', 300)
        # Range filtering (mm)
        self.declare_parameter('min_obstacle_range_m', 0.3)
        self.declare_parameter('max_obstacle_range_m', 4.0)

        # Size sanity (meters)
        self.declare_parameter('max_obstacle_size_m', 1.5)
        self.declare_parameter('min_obstacle_size_m', 0.03)        
        self.declare_parameter('speckle_min_area', 80)

        self.declare_parameter('temporal_window', 7)
        self.declare_parameter('temporal_k', 4)

        self.declare_parameter('dilate_kernel', 3)
        self.declare_parameter('close_kernel', 7)

        self.declare_parameter('publish_overlay', True)
        self.declare_parameter('publish_mask', False)
        self.declare_parameter('publish_detections', True)
        self.declare_parameter('publish_plane', True)
        self.declare_parameter('publish_pointcloud', True)
        # Point cloud subsampling — every Nth obstacle pixel
        self.declare_parameter('cloud_subsample', 3)

        self.declare_parameter('log_profile', False)

        # Bind into instance attrs
        gp = self.get_parameter
        self.camera_frame = gp('camera_frame').value
        self.W = gp('image_width').value
        self.H = gp('image_height').value
        self.fps = gp('camera_fps').value

        self.floor_thresh = gp('floor_thresh_m').value
        self.ransac_interval = gp('ransac_interval_frames').value
        self.ransac_iter = gp('ransac_iterations').value

        self.edge_threshold = gp('edge_threshold').value
        self.min_area = gp('min_obstacle_area').value
        self.min_range_mm = int(gp('min_obstacle_range_m').value * 1000)
        self.max_range_mm = int(gp('max_obstacle_range_m').value * 1000)
        self.max_size_m = gp('max_obstacle_size_m').value
        self.min_size_m = gp('min_obstacle_size_m').value
        self.speckle_area = gp('speckle_min_area').value

        self.tw = gp('temporal_window').value
        self.tk = gp('temporal_k').value

        self.publish_overlay = gp('publish_overlay').value
        self.publish_mask = gp('publish_mask').value
        self.publish_detections = gp('publish_detections').value
        self.publish_plane = gp('publish_plane').value
        self.log_profile = gp('log_profile').value

        # ── Cached structures ──
        self.dilate_kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE,
            (gp('dilate_kernel').value, gp('dilate_kernel').value))
        self.close_kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE,
            (gp('close_kernel').value, gp('close_kernel').value))

        self.u_grid = None
        self.v_grid = None
        self.last_plane = None
        self.floor_K = None
        self.floor_d = None
        self.history = deque(maxlen=self.tw)
        self.ransac_counter = 0
        self.frame_count = 0
        self.t_start = time.time()

        self.bridge = CvBridge()

        # ── Publishers ──
        if self.publish_overlay:
            self.pub_overlay = self.create_publisher(
                CompressedImage,
                gp('overlay_topic').value,
                make_qos(reliable=False))
        if self.publish_mask:
            self.pub_mask = self.create_publisher(
                Image,
                gp('mask_topic').value,
                make_qos(reliable=False))
        if self.publish_detections:
            self.pub_det = self.create_publisher(
                Detection3DArray,
                gp('detections_topic').value,
                make_qos(reliable=True))
        if self.publish_plane:
            self.pub_plane = self.create_publisher(
                PolygonStamped,
                gp('plane_topic').value,
                make_qos(reliable=True))

        self.publish_pointcloud = gp('publish_pointcloud').value
        self.cloud_subsample = gp('cloud_subsample').value
        if self.publish_pointcloud:
            self.pub_marking = self.create_publisher(
                PointCloud2,
                '/oak_obstacles/marking_points',
                make_qos(reliable=False))
            self.pub_clearing = self.create_publisher(
                PointCloud2,
                '/oak_obstacles/clearing_points',
                make_qos(reliable=False))
            
        # ── Build OAK pipeline (v3.x API: pipeline owns the device) ──
        self.get_logger().info('Building OAK pipeline...')
        self.pipeline = dai.Pipeline()
        self._build_pipeline_nodes(self.pipeline)
        self.get_logger().info('OAK pipeline created.')

        # Output queues are created BEFORE start()
        self.rgb_queue = self.rgb_color_out.createOutputQueue(
            maxSize=4, blocking=False)
        self.depth_queue = self.stereo.depth.createOutputQueue(
            maxSize=4, blocking=False)
        self.edge_queue = self.edge.outputImage.createOutputQueue(
            maxSize=4, blocking=False)

        # ── Start pipeline (this binds and opens the device) ──
        self.pipeline.start()

        # ── Now we can ask the pipeline for its device ──
        self.device = self.pipeline.getDefaultDevice()
        self.get_logger().info(
            f'Connected to: {self.device.getDeviceName()} '
            f'USB: {self.device.getUsbSpeed().name}')

        # ── Read intrinsics ──
        calib = self.device.readCalibration()
        K = np.array(calib.getCameraIntrinsics(
            dai.CameraBoardSocket.CAM_A,
            dai.Size2f(self.W, self.H)))
        self.fx = float(K[0][0])
        self.fy = float(K[1][1])
        self.cx = float(K[0][2])
        self.cy = float(K[1][2])
        self.get_logger().info(
            f'Intrinsics fx={self.fx:.1f} fy={self.fy:.1f} '
            f'cx={self.cx:.1f} cy={self.cy:.1f}')

        # Pre-build pixel grids
        u, v = np.meshgrid(np.arange(self.W), np.arange(self.H))
        self.u_grid = u.astype(np.float32)
        self.v_grid = v.astype(np.float32)

        # ── Main processing timer ──
        # 30 Hz tick — camera is at 15 FPS, so we poll fast enough
        self.timer = self.create_timer(1.0 / 30.0, self.tick)
        self.get_logger().info('OAK obstacle node ready.')

    # ────────────────────────────────────────────────────────────
    def _build_pipeline_nodes(self, pipeline):
        """Build all OAK pipeline nodes on the given pipeline object."""

        cam_rgb = pipeline.create(dai.node.Camera).build(
            boardSocket=dai.CameraBoardSocket.CAM_A)

        self.rgb_color_out = cam_rgb.requestOutput(
            size=(self.W, self.H),
            type=dai.ImgFrame.Type.NV12, fps=self.fps)
        rgb_for_edge = cam_rgb.requestOutput(
            size=(self.W, self.H),
            type=dai.ImgFrame.Type.NV12, fps=self.fps)

        manip = pipeline.create(dai.node.ImageManip)
        manip.initialConfig.setOutputSize(self.W, self.H)
        manip.initialConfig.setFrameType(dai.ImgFrame.Type.GRAY8)
        manip.setMaxOutputFrameSize(self.W * self.H)
        rgb_for_edge.link(manip.inputImage)

        self.edge = pipeline.create(dai.node.EdgeDetector)
        self.edge.setMaxOutputFrameSize(self.W * self.H)
        manip.out.link(self.edge.inputImage)

        mono_left = pipeline.create(dai.node.Camera).build(
            boardSocket=dai.CameraBoardSocket.CAM_B)
        mono_right = pipeline.create(dai.node.Camera).build(
            boardSocket=dai.CameraBoardSocket.CAM_C)
        ml = mono_left.requestOutput(
            size=(self.W, self.H),
            type=dai.ImgFrame.Type.NV12, fps=self.fps)
        mr = mono_right.requestOutput(
            size=(self.W, self.H),
            type=dai.ImgFrame.Type.NV12, fps=self.fps)

        self.stereo = pipeline.create(dai.node.StereoDepth)
        self.stereo.setDefaultProfilePreset(
            dai.node.StereoDepth.PresetMode.HIGH_DETAIL)
        self.stereo.setDepthAlign(dai.CameraBoardSocket.CAM_A)
        self.stereo.setOutputSize(self.W, self.H)
        self.stereo.setLeftRightCheck(True)
        self.stereo.setSubpixel(True)
        ml.link(self.stereo.left)
        mr.link(self.stereo.right)

    # ────────────────────────────────────────────────────────────
    # Algorithm helpers (same as Tier 6, tightened)
    # ────────────────────────────────────────────────────────────
    def ransac_plane(self, points):
        if points.shape[0] < 100:
            return None
        best_inliers = 0
        best_plane = None
        rng = np.random.default_rng()
        for _ in range(self.ransac_iter):
            idx = rng.choice(points.shape[0], 3, replace=False)
            p1, p2, p3 = points[idx]
            v1, v2 = p2 - p1, p3 - p1
            normal = np.cross(v1, v2)
            norm_len = np.linalg.norm(normal)
            if norm_len < 1e-6:
                continue
            normal = normal / norm_len
            a, b, c = normal
            d = -np.dot(normal, p1)
            if b > 0:
                a, b, c, d = -a, -b, -c, -d
            if abs(b) < 0.6:
                continue
            dists = np.abs(points @ np.array([a, b, c]) + d)
            n_in = np.sum(dists < self.floor_thresh)
            if n_in > best_inliers:
                best_inliers = n_in
                best_plane = (a, b, c, d)
        return best_plane

    def update_floor_lookup(self, plane):
        a, b, c, d = plane
        K = (a * (self.u_grid - self.cx) / self.fx
             + b * (self.v_grid - self.cy) / self.fy
             + c).astype(np.float32)
        return K, float(d)

    def compute_floor_mask(self, depth_mm):
        depth_m = depth_mm.astype(np.float32) / 1000.0
        signed = -depth_m * self.floor_K - self.floor_d
        valid = depth_mm > 0
        return valid & (np.abs(signed) <= self.floor_thresh)

    def in_floor_band(self, floor_mask):
        v_idx = np.arange(self.H).reshape(-1, 1)
        floor_v = np.where(floor_mask, v_idx, -1)
        max_v = floor_v.max(axis=0)
        return v_idx < max_v[np.newaxis, :]

    def vectorized_speckle_filter(self, mask_u8):
        n_lbl, lbl_im, stats, _ = cv2.connectedComponentsWithStats(
            mask_u8, connectivity=8)
        if n_lbl <= 1:
            return np.zeros_like(mask_u8, dtype=bool)
        areas = stats[:, cv2.CC_STAT_AREA]
        keep = areas >= self.speckle_area
        keep[0] = False
        return keep[lbl_im]

    def temporal_stable(self, current):
        self.history.append(current)
        if len(self.history) < self.tk:
            return current.copy()
        stack = np.stack(list(self.history), axis=0).astype(np.uint8)
        return np.sum(stack, axis=0) >= self.tk

    def find_objects(self, stable_mask, depth_mm):
        mask_u8 = stable_mask.astype(np.uint8) * 255
        n_lbl, lbl_im, stats, centroids = \
            cv2.connectedComponentsWithStats(mask_u8, connectivity=8)
        if n_lbl <= 1:
            return []

        objects = []
        for i in range(1, n_lbl):
            area = stats[i, cv2.CC_STAT_AREA]
            if area < self.min_area:
                continue
            x = stats[i, cv2.CC_STAT_LEFT]
            y = stats[i, cv2.CC_STAT_TOP]
            w = stats[i, cv2.CC_STAT_WIDTH]
            h = stats[i, cv2.CC_STAT_HEIGHT]
            cxp, cyp = centroids[i]

            roi_lbl = lbl_im[y:y+h, x:x+w]
            roi_d = depth_mm[y:y+h, x:x+w]
            blob_d = roi_d[(roi_lbl == i) & (roi_d > 0)]

            if blob_d.size == 0:
                continue

            min_z = float(blob_d.min()) / 1000.0
            median_z = float(np.median(blob_d)) / 1000.0
            cx_m = (cxp - self.cx) * median_z / self.fx
            cy_m = (cyp - self.cy) * median_z / self.fy

            # Size estimate from bounding box pixels at median depth
            size_x = w * median_z / self.fx
            size_y = h * median_z / self.fy
            d5, d95 = np.percentile(blob_d / 1000.0, [5, 95])
            size_z = max(0.05, d95 - d5)

            # ── Size sanity filter ──
            # Discard absurdly large blobs (likely wall masses, not objects)
            max_dim = max(size_x, size_y, size_z)
            min_dim = min(size_x, size_y, size_z)
            if max_dim > self.max_size_m:
                continue
            if max_dim < self.min_size_m:
                continue

            objects.append({
                'bbox': (x, y, w, h),
                'centroid_px': (int(cxp), int(cyp)),
                'centroid_xyz': (cx_m, cy_m, median_z),
                'size_xyz': (size_x, size_y, size_z),
                'area': int(area),
                'min_z': min_z,
            })
        objects.sort(key=lambda o: o['min_z'])
        return objects

    def build_overlay(self, rgb, floor_mask, obstacle_mask, objects):
        overlay = rgb.copy()
        overlay[floor_mask] = (
            0.6 * overlay[floor_mask] + 0.4 * np.array([0, 180, 0])
        ).astype(np.uint8)
        overlay[obstacle_mask] = (
            0.2 * overlay[obstacle_mask] + 0.8 * np.array([0, 0, 255])
        ).astype(np.uint8)

        for i, o in enumerate(objects):
            x, y, w, h = o['bbox']
            cv2.rectangle(overlay, (x, y), (x + w, y + h),
                          (0, 255, 255), 2)
            label = f"#{i+1} {o['min_z']:.2f}m"
            (tw, th), _ = cv2.getTextSize(
                label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
            cv2.rectangle(overlay, (x, y - th - 6),
                          (x + tw + 6, y), (0, 0, 0), -1)
            cv2.putText(overlay, label, (x + 3, y - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                        (0, 255, 255), 1)

        cv2.putText(overlay, f"Obstacles: {len(objects)}",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                    (0, 255, 255), 2)
        return overlay

    # ────────────────────────────────────────────────────────────
    # Main tick
    # ────────────────────────────────────────────────────────────
    def tick(self):
        # Pull latest frames (non-blocking)
        in_rgb = self.rgb_queue.tryGet()
        in_depth = self.depth_queue.tryGet()
        in_edge = self.edge_queue.tryGet()

        if in_rgb is None or in_depth is None or in_edge is None:
            return

        rgb = in_rgb.getCvFrame()
        depth_mm = in_depth.getFrame()
        self._latest_depth = depth_mm  # cache for publishers
        edges = in_edge.getCvFrame() > self.edge_threshold

        profile = {}
        t_total = time.time()
        self.frame_count += 1
        self.ransac_counter += 1

        # ── RANSAC every N frames ──
        t0 = time.time()
        if (self.last_plane is None
                or self.ransac_counter >= self.ransac_interval):
            self.ransac_counter = 0
            z = depth_mm.astype(np.float32) / 1000.0
            valid = z > 0
            if np.count_nonzero(valid) > 500:
                sv = self.v_grid[valid][::8]
                su = self.u_grid[valid][::8]
                sz = z[valid][::8]
                sx = (su - self.cx) * sz / self.fx
                sy = (sv - self.cy) * sz / self.fy
                points = np.stack([sx, sy, sz], axis=1)
                plane = self.ransac_plane(points)
                if plane is not None:
                    self.last_plane = plane
                    self.floor_K, self.floor_d = self.update_floor_lookup(
                        plane)
        profile['ransac'] = (time.time() - t0) * 1000

        if self.floor_K is None:
            return

        # ── Floor mask ──
        t0 = time.time()
        floor_mask = self.compute_floor_mask(depth_mm)
        band = self.in_floor_band(floor_mask)
        profile['floor'] = (time.time() - t0) * 1000

        # ── Edges that are not floor and within band ──
        # ALSO restrict to within reasonable range (max_obstacle_range_m)
        t0 = time.time()
        in_range = (depth_mm > self.min_range_mm) & \
                   (depth_mm < self.max_range_mm)
        obstacle_edges = edges & ~floor_mask & band & in_range
        profile['mask'] = (time.time() - t0) * 1000

        # ── Morphology + speckle removal ──
        t0 = time.time()
        oe_u8 = obstacle_edges.astype(np.uint8) * 255
        oe_u8 = cv2.dilate(oe_u8, self.dilate_kernel)
        oe_u8 = cv2.morphologyEx(oe_u8, cv2.MORPH_CLOSE, self.close_kernel)
        cleaned = self.vectorized_speckle_filter(oe_u8)
        profile['morph'] = (time.time() - t0) * 1000

        # ── Temporal ──
        t0 = time.time()
        stable = self.temporal_stable(cleaned)
        profile['temporal'] = (time.time() - t0) * 1000

        # ── Find objects ──
        t0 = time.time()
        objects = self.find_objects(stable, depth_mm)
        profile['find_obj'] = (time.time() - t0) * 1000

        # ── Publish ──
        t0 = time.time()
        stamp = self.get_clock().now().to_msg()
        self.publish_results(rgb, floor_mask, stable, objects, stamp)
        profile['publish'] = (time.time() - t0) * 1000

        total_ms = (time.time() - t_total) * 1000.0

        if self.log_profile and self.frame_count % 30 == 0:
            parts = "  ".join(f"{k}={v:.1f}" for k, v in profile.items())
            self.get_logger().info(
                f"total={total_ms:5.1f}ms  {parts}  "
                f"objs={len(objects)}")

    # ────────────────────────────────────────────────────────────
    # Publishing helpers
    # ────────────────────────────────────────────────────────────
    def publish_results(self, rgb, floor_mask, stable_mask,
                        objects, stamp):
        header = Header()
        header.stamp = stamp
        header.frame_id = self.camera_frame

        # Overlay image
        if self.publish_overlay:
            overlay = self.build_overlay(
                rgb, floor_mask, stable_mask, objects)
            # JPEG compress
            ok, buf = cv2.imencode('.jpg', overlay,
                                   [cv2.IMWRITE_JPEG_QUALITY, 70])
            if ok:
                msg = CompressedImage()
                msg.header = header
                msg.format = 'jpeg'
                msg.data = buf.tobytes()
                self.pub_overlay.publish(msg)

        # Binary mask (optional, heavy)
        if self.publish_mask:
            mask_u8 = stable_mask.astype(np.uint8) * 255
            msg = self.bridge.cv2_to_imgmsg(mask_u8, encoding='mono8')
            msg.header = header
            self.pub_mask.publish(msg)

        # Detections
        if self.publish_detections:
            det_array = Detection3DArray()
            det_array.header = header
            for i, o in enumerate(objects):
                det = Detection3D()
                det.header = header

                bbox = BoundingBox3D()
                bbox.center = Pose()
                cx_m, cy_m, cz_m = o['centroid_xyz']
                bbox.center.position.x = float(cx_m)
                bbox.center.position.y = float(cy_m)
                bbox.center.position.z = float(cz_m)
                bbox.center.orientation.w = 1.0
                sx, sy, sz = o['size_xyz']
                bbox.size = Vector3()
                bbox.size.x = float(sx)
                bbox.size.y = float(sy)
                bbox.size.z = float(sz)
                det.bbox = bbox

                hyp = ObjectHypothesisWithPose()
                hyp.hypothesis.class_id = 'obstacle'
                hyp.hypothesis.score = 1.0
                hyp.pose.pose = bbox.center
                det.results.append(hyp)

                det_array.detections.append(det)
            self.pub_det.publish(det_array)

        # Floor plane — published as 4 corners of a 5m × 5m patch
        if self.publish_plane and self.last_plane is not None:
            self.publish_floor_polygon(stamp)

        # Dense obstacle point cloud — primary feed to Nav2 costmap
        if self.publish_pointcloud:
            self.publish_obstacle_pointcloud(
                self._latest_depth, stable_mask, stamp)

    def publish_floor_polygon(self, stamp):
        """Publish the floor as a polygon (4 corners) for visualization."""
        a, b, c, d = self.last_plane
        # Build 4 corners of a 4m × 4m square on the plane,
        # centered ~1.5m ahead of the camera in XZ.
        # Plane equation: a*x + b*y + c*z + d = 0
        # We solve for y at sample (x, z) corners.

        if abs(b) < 1e-6:
            return  # degenerate

        corners_xz = [
            (-2.0, 0.3), (2.0, 0.3), (2.0, 4.0), (-2.0, 4.0)
        ]

        msg = PolygonStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = self.camera_frame
        for x, z in corners_xz:
            y = -(a * x + c * z + d) / b
            p = Point32()
            p.x = float(x)
            p.y = float(y)
            p.z = float(z)
            msg.polygon.points.append(p)
        self.pub_plane.publish(msg)

    def publish_obstacle_pointcloud(self, depth_mm, stable_mask, stamp):
        """
        Publish TWO clouds:
        1. /oak_obstacles/marking_points   — obstacle pixels only
        2. /oak_obstacles/clearing_points  — all valid depth (for raytracing)

        Nav2 ObstacleLayer uses marking_points for marking obstacles,
        clearing_points for raytracing through free space.
        """
        ss = self.cloud_subsample
        depth_ss = depth_mm[::ss, ::ss]
        u_ss = self.u_grid[::ss, ::ss]
        v_ss = self.v_grid[::ss, ::ss]
        mask_ss = stable_mask[::ss, ::ss]

        valid = depth_ss > 0
        if not np.any(valid):
            empty = np.zeros((0, 3), dtype=np.float32)
            self.pub_marking.publish(self._build_pc2(empty, stamp))
            self.pub_clearing.publish(self._build_pc2(empty, stamp))
            return

        # Project ALL valid pixels to 3D
        z_m = depth_ss[valid].astype(np.float32) / 1000.0
        u_v = u_ss[valid]
        v_v = v_ss[valid]
        x_m = (u_v - self.cx) * z_m / self.fx
        y_m = (v_v - self.cy) * z_m / self.fy
        all_pts = np.stack([x_m, y_m, z_m], axis=1).astype(np.float32)

        # Marking cloud: obstacle pixels only
        is_obs = mask_ss[valid]
        marking_pts = all_pts[is_obs]

        # Clearing cloud: everything (floor + obstacles)
        # The costmap will raytrace from sensor origin to each point.
        # Cells along the ray that don't have a marking point get cleared.
        clearing_pts = all_pts

        self.pub_marking.publish(self._build_pc2(marking_pts, stamp))
        self.pub_clearing.publish(self._build_pc2(clearing_pts, stamp))

    def _build_pc2(self, points, stamp):
        """Build a PointCloud2 from (N, 3) numpy array."""
        msg = PointCloud2()
        msg.header.stamp = stamp
        msg.header.frame_id = self.camera_frame
        msg.height = 1
        msg.width = points.shape[0]
        msg.is_bigendian = False
        msg.is_dense = True
        msg.point_step = 12
        msg.row_step = msg.point_step * msg.width
        msg.fields = [
            PointField(name='x', offset=0,
                       datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4,
                       datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8,
                       datatype=PointField.FLOAT32, count=1),
        ]
        msg.data = points.tobytes()
        return msg

    def destroy_node(self):
        try:
            self.pipeline.stop()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = OakObstacleNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()