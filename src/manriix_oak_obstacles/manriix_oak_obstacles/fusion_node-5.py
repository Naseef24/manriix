#!/usr/bin/env python3
"""
fusion_node.py  (v2 -- pixel-projection association, paper's actual method)

Implements Section III-F (Fusion of 2D and 3D Detection and Tracking) of:
  Eppenberger et al., "Leveraging Stereo-Camera Data for Real-Time
  Dynamic Obstacle Detection and Tracking", IROS 2020.

CHANGE FROM v1:
  v1 associated a person box to a cluster by 3D x-y distance, using the
  on-device spatial-detection position from DepthAI as the person's 3D
  location. That position proved unreliable (verified via tape-measured
  calibration testing -- see people_tracker_node.py docstring).

  v2 implements what the paper ACTUALLY specifies for this step:
  "linking the detection to the cluster having the highest amount of
  points within the bounding box" (Sec III-E, on 2D-3D association).
  Concretely: each 3D cluster's points (world/base_footprint frame) are
  transformed into the camera's optical frame and projected into pixel
  space using the camera intrinsics that match the DETECTOR'S input
  resolution (the RGB preview stream, which is what the NN actually
  sees -- NOT the stereo depth camera's native intrinsics, since the
  preview is a separately-scaled/squashed image). Whichever cluster has
  the most projected points landing inside a given 2D box is associated
  with it.

  This also fixes the person's 3D POSITION as a side effect: once a
  cluster is confirmed as 'person', its own centroid (already verified
  geometrically accurate from the point-cloud pipeline) becomes the
  person's position everywhere downstream (motion estimation, occupancy
  grid) -- no DepthAI-derived number is used anywhere in this chain
  anymore.

Association frequency f_a and the gamma-threshold person-labeling logic
(Sec III-F) are otherwise unchanged from v1.
"""

import numpy as np
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.time import Time

import tf2_ros
from tf2_ros import TransformException

from sensor_msgs.msg import CameraInfo
from sensor_msgs_py import point_cloud2 as pc2
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

from manriix_oak_obstacles_msgs.msg import (
    ClusterTrackArray, BoundingBoxTrackArray)


def quat_to_rot_matrix(x, y, z, w):
    n = x * x + y * y + z * z + w * w
    if n < 1e-12:
        return np.eye(3)
    s = 2.0 / n
    xx, yy, zz = x * x * s, y * y * s, z * z * s
    xy, xz, yz = x * y * s, x * z * s, y * z * s
    wx, wy, wz = w * x * s, w * y * s, w * z * s
    return np.array([
        [1.0 - (yy + zz), xy - wz, xz + wy],
        [xy + wz, 1.0 - (xx + zz), yz - wx],
        [xz - wy, yz + wx, 1.0 - (xx + yy)],
    ])


class FusionNode(Node):

    def __init__(self):
        super().__init__('fusion_node')

        self.declare_parameter('tracks_topic', '/manriix/cluster_tracks_classified')
        self.declare_parameter('person_boxes_topic', '/manriix/person_boxes')
        self.declare_parameter('output_tracks_topic', '/manriix/cluster_tracks_fused')
        self.declare_parameter('output_markers_topic', '/manriix/fusion_markers')

        # Camera intrinsics matching the DETECTOR'S input (the RGB
        # preview stream the NN actually runs on), NOT the stereo depth
        # camera's native intrinsics -- these can differ significantly
        # when the preview is a squashed/rescaled image.
        self.declare_parameter('preview_camera_info_topic', '/oak/rgb/preview/camera_info')

        self.declare_parameter('gamma', 1.5)          # [1/s] association-frequency threshold (paper Sec IV-A)
        self.declare_parameter('freq_window', 2.0)     # [s] window over which f_a is measured
        self.declare_parameter('person_label_hold', 1.5)  # [s] keep 'person' label after last association
        self.declare_parameter('min_points_in_box', 5)     # min projected points to count as a valid association
        self.declare_parameter('max_proj_points_per_cluster', 400)  # perf cap
        # For a track being labeled 'person', report the NEAR-SURFACE
        # position (average of the closest N% of the cluster's own
        # points) instead of the cluster's mean centroid. A mean
        # centroid blends a person's whole body depth (which legitimately
        # spans a wide range for a real 3D body -- more so when
        # kneeling/reaching) and can also be pulled outward by any
        # residual background bleed at the cluster's edges. For obstacle
        # avoidance, the nearest point of a person is what actually
        # matters for safety margins, and it is far less sensitive to
        # pose or residual clustering noise than an average.
        self.declare_parameter('person_position_percentile', 10.0)

        self.gamma = float(self.get_parameter('gamma').value)
        self.freq_window = float(self.get_parameter('freq_window').value)
        self.person_label_hold = float(self.get_parameter('person_label_hold').value)
        self.min_points_in_box = int(self.get_parameter('min_points_in_box').value)
        self.max_proj_points = int(self.get_parameter('max_proj_points_per_cluster').value)
        self.person_position_percentile = float(
            self.get_parameter('person_position_percentile').value)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.K = None
        self.create_subscription(
            CameraInfo, self.get_parameter('preview_camera_info_topic').value,
            self._camera_info_cb, 5)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.create_subscription(
            ClusterTrackArray,
            self.get_parameter('tracks_topic').value,
            self._tracks_cb, qos)
        self.create_subscription(
            BoundingBoxTrackArray,
            self.get_parameter('person_boxes_topic').value,
            self._boxes_cb, 10)

        self.pub_tracks = self.create_publisher(
            ClusterTrackArray,
            self.get_parameter('output_tracks_topic').value, qos)
        self.pub_markers = self.create_publisher(
            MarkerArray,
            self.get_parameter('output_markers_topic').value, 10)

        # State
        self.latest_boxes_msg = None  # BoundingBoxTrackArray, cached

        self.assoc_stamps = {}          # cluster track_id -> deque[stamp]
        self.person_until = {}          # cluster track_id -> stamp until which 'person' holds
        self.box_last_cluster = {}      # person box_id -> cluster track_id (consistency check)

        self._prev_marker_count = 0

        self.get_logger().info(
            f'gamma={self.gamma}/s window={self.freq_window}s '
            f'min_points_in_box={self.min_points_in_box} hold={self.person_label_hold}s | '
            f'waiting for camera intrinsics on '
            f'{self.get_parameter("preview_camera_info_topic").value}')

    # ------------------------------------------------------------------
    def _camera_info_cb(self, msg: CameraInfo):
        if self.K is None:
            self.K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
            self.get_logger().info(
                f'Preview camera intrinsics received ({msg.width}x{msg.height}):\n{self.K}')

    # ------------------------------------------------------------------
    def _boxes_cb(self, msg: BoundingBoxTrackArray):
        self.latest_boxes_msg = msg

    # ------------------------------------------------------------------
    def _get_world_to_camera(self, camera_frame, world_frame, stamp_time):
        try:
            tf = self.tf_buffer.lookup_transform(
                camera_frame, world_frame, rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().warn(
                f'fusion: TF {world_frame}->{camera_frame} unavailable: {ex}',
                throttle_duration_sec=2.0)
            return None
        q = tf.transform.rotation
        t = tf.transform.translation
        R = quat_to_rot_matrix(q.x, q.y, q.z, q.w)
        return R, np.array([t.x, t.y, t.z])

    # ------------------------------------------------------------------
    def _tracks_cb(self, msg: ClusterTrackArray):
        now = Time.from_msg(msg.header.stamp).nanoseconds * 1e-9

        boxes_msg = self.latest_boxes_msg
        boxes_fresh = (
            boxes_msg is not None and
            (now - Time.from_msg(boxes_msg.header.stamp).nanoseconds * 1e-9) < 0.5
        )

        if self.K is not None and boxes_fresh and boxes_msg.boxes:
            self._associate_by_projection(msg, boxes_msg, now)
        elif self.K is None:
            self.get_logger().warn(
                'fusion: no camera intrinsics yet -- skipping 2D-3D association '
                'this frame', throttle_duration_sec=5.0)

        # ---- compute f_a per cluster track, latch 'person' label ----
        for tid, stamps in list(self.assoc_stamps.items()):
            while stamps and (now - stamps[0]) > self.freq_window:
                stamps.popleft()
            if not stamps:
                del self.assoc_stamps[tid]
                continue
            f_a = len(stamps) / self.freq_window
            if f_a >= self.gamma:
                self.person_until[tid] = now + self.person_label_hold

        live = {trk.track_id for trk in msg.tracks}
        for d in (self.assoc_stamps, self.person_until):
            for tid in [t for t in d if t not in live]:
                del d[tid]

        self._publish(msg, now)

    # ------------------------------------------------------------------
    def _associate_by_projection(self, tracks_msg, boxes_msg, now):
        """Paper's method: project each cluster's points into the
        camera image; associate to the box containing the most points."""
        camera_frame = boxes_msg.header.frame_id
        world_frame = tracks_msg.header.frame_id

        tf_result = self._get_world_to_camera(camera_frame, world_frame, now)
        if tf_result is None:
            return
        R, t = tf_result

        # box_id -> list of (cluster_track_id, count)
        box_counts = {b.box_id: [] for b in boxes_msg.boxes}

        for trk in tracks_msg.tracks:
            pts = pc2.read_points_numpy(
                trk.points, field_names=('x', 'y', 'z'), skip_nans=True)
            if pts.shape[0] == 0:
                continue
            if pts.shape[0] > self.max_proj_points:
                sel = np.random.choice(pts.shape[0], self.max_proj_points, replace=False)
                pts = pts[sel]

            pts_cam = pts @ R.T + t
            depth = pts_cam[:, 2]
            valid = depth > 1e-3
            if not np.any(valid):
                continue
            proj = (self.K @ pts_cam[valid].T).T
            pix = proj[:, :2] / proj[:, 2:3]

            for b in boxes_msg.boxes:
                inside = (
                    (pix[:, 0] >= b.xmin) & (pix[:, 0] <= b.xmax) &
                    (pix[:, 1] >= b.ymin) & (pix[:, 1] <= b.ymax)
                )
                count = int(inside.sum())
                if count >= self.min_points_in_box:
                    box_counts[b.box_id].append((trk.track_id, count))

        # For each box, pick the cluster with the most points inside it.
        for b in boxes_msg.boxes:
            candidates = box_counts.get(b.box_id, [])
            if not candidates:
                continue
            best_tid, _ = max(candidates, key=lambda c: c[1])

            prev_tid = self.box_last_cluster.get(b.box_id)
            if prev_tid is not None and prev_tid != best_tid:
                self.assoc_stamps.pop(prev_tid, None)
            self.box_last_cluster[b.box_id] = best_tid

            self.assoc_stamps.setdefault(best_tid, deque()).append(now)

    # ------------------------------------------------------------------
    def _near_surface_position(self, trk):
        """Average of the closest `person_position_percentile`% of a
        cluster's own points -- a stable, pose-robust proxy for 'the
        nearest surface of this person', instead of a whole-body mean
        centroid that a real 3D body (or any residual clustering noise)
        can pull outward."""
        pts = pc2.read_points_numpy(
            trk.points, field_names=('x', 'y', 'z'), skip_nans=True)
        if pts.shape[0] == 0:
            return None
        xs = pts[:, 0]
        thresh = np.percentile(xs, self.person_position_percentile)
        near = pts[xs <= thresh]
        if near.shape[0] == 0:
            # fallback: at least the single nearest point
            near = pts[np.argsort(xs)[:1]]
        return near.mean(axis=0)

    # ------------------------------------------------------------------
    def _publish(self, msg: ClusterTrackArray, now):
        out = ClusterTrackArray()
        out.header = msg.header

        marker_array = MarkerArray()
        marker_id = 0

        for trk in msg.tracks:
            if self.person_until.get(trk.track_id, 0.0) > now:
                trk.classification = 'person'
                near_pos = self._near_surface_position(trk)
                if near_pos is not None:
                    trk.centroid.x = float(near_pos[0])
                    trk.centroid.y = float(near_pos[1])
                    trk.centroid.z = float(near_pos[2])
            out.tracks.append(trk)

            for shape_marker in self._make_shape_markers(msg.header, trk):
                shape_marker.id = marker_id
                marker_array.markers.append(shape_marker)
                marker_id += 1
            marker_array.markers.append(
                self._make_label(msg.header, trk, marker_id))
            marker_id += 1

        for extra in range(marker_id, self._prev_marker_count):
            m = Marker()
            m.header = out.header
            m.id = extra
            m.action = Marker.DELETE
            marker_array.markers.append(m)
        self._prev_marker_count = marker_id

        self.pub_tracks.publish(out)
        self.pub_markers.publish(marker_array)

    # ------------------------------------------------------------------
    # Marker colors, matching the paper's Fig. 1 rendering convention:
    #   static    -> red VOXELS (small cubes, STVL-style -- NOT one big
    #                filled bounding box)
    #   dynamic   -> a distinct voxel color (orange) so it's visually
    #                separable from static at a glance
    #   uncertain -> dim gray/yellow voxels, low emphasis
    #   person    -> sparse YELLOW points (the actual point cloud) plus
    #                a thin BLUE WIREFRAME outline (unfilled) -- matches
    #                Fig. 1 exactly: "the yellow point cloud... the blue
    #                cuboid indicates the visual people detector
    #                recognized this cluster as a person"
    VOXEL_RGB = {
        'static':    (0.85, 0.1, 0.1),
        'dynamic':   (1.0, 0.55, 0.0),
        'uncertain': (0.6, 0.6, 0.35),
        'unknown':   (0.5, 0.5, 0.5),
    }
    VOXEL_SIZE = 0.05  # matches typical occupancy grid cell resolution

    def _voxelize(self, pts):
        """Downsample to ONE point per occupied voxel cell (true
        voxelization), instead of one CUBE_LIST entry per raw point.
        A dense cluster can have 1000+ raw points but only ~50-150
        occupied 5cm cells -- this keeps the paper's Fig.1 'voxel'
        look while cutting the published message size drastically,
        which was the actual cause of fusion_node's publish rate
        dropping to ~1Hz (a 1700+ point CUBE_LIST every ~50ms is a lot
        of geometry_msgs/Point structs to serialize)."""
        if pts.shape[0] == 0:
            return pts
        keys = np.floor(pts / self.VOXEL_SIZE).astype(np.int64)
        _, unique_idx = np.unique(keys, axis=0, return_index=True)
        # snap to voxel CENTER for a clean grid look, not the raw point
        centers = (keys[unique_idx] + 0.5) * self.VOXEL_SIZE
        return centers

    def _bounds(self, trk):
        pts = pc2.read_points_numpy(
            trk.points, field_names=('x', 'y', 'z'), skip_nans=True)
        if pts.shape[0] == 0:
            c = np.array([trk.centroid.x, trk.centroid.y, trk.centroid.z])
            return c, c
        return pts.min(axis=0), pts.max(axis=0)

    @staticmethod
    def _wireframe_edges(mins, maxs):
        x0, y0, z0 = mins
        x1, y1, z1 = maxs
        corners = [
            (x0, y0, z0), (x1, y0, z0), (x1, y1, z0), (x0, y1, z0),
            (x0, y0, z1), (x1, y0, z1), (x1, y1, z1), (x0, y1, z1),
        ]
        edges = [(0, 1), (1, 2), (2, 3), (3, 0),
                 (4, 5), (5, 6), (6, 7), (7, 4),
                 (0, 4), (1, 5), (2, 6), (3, 7)]
        pts = []
        for a, b in edges:
            pts.append(Point(x=float(corners[a][0]), y=float(corners[a][1]), z=float(corners[a][2])))
            pts.append(Point(x=float(corners[b][0]), y=float(corners[b][1]), z=float(corners[b][2])))
        return pts

    def _make_shape_markers(self, header, trk):
        """Returns a list of Marker (1 for static/dynamic/uncertain --
        a voxel CUBE_LIST; 2 for person -- points + wireframe box).
        Caller assigns .id and appends to the array."""
        markers = []

        if trk.classification == 'person':
            pts = pc2.read_points_numpy(
                trk.points, field_names=('x', 'y', 'z'), skip_nans=True)
            # Sparse subsample: matches the paper's Fig. 1 visual (a
            # scattered point cloud, not a dense one) and keeps message
            # size small regardless of how many raw points the cluster has.
            max_person_points = 200
            if pts.shape[0] > max_person_points:
                sel = np.random.choice(pts.shape[0], max_person_points, replace=False)
                pts = pts[sel]

            pm = Marker()
            pm.header = header
            pm.ns = 'fused_person_points'
            pm.type = Marker.POINTS
            pm.action = Marker.ADD
            pm.pose.orientation.w = 1.0
            pm.scale.x = 0.03
            pm.scale.y = 0.03
            pm.color.r, pm.color.g, pm.color.b, pm.color.a = 1.0, 0.9, 0.1, 0.95
            pm.points = [Point(x=float(p[0]), y=float(p[1]), z=float(p[2])) for p in pts]
            pm.lifetime = rclpy.duration.Duration(seconds=0.3).to_msg()
            markers.append(pm)

            mins, maxs = self._bounds(trk)
            wf = Marker()
            wf.header = header
            wf.ns = 'fused_person_box'
            wf.type = Marker.LINE_LIST
            wf.action = Marker.ADD
            wf.pose.orientation.w = 1.0
            wf.scale.x = 0.02  # line width
            wf.color.r, wf.color.g, wf.color.b, wf.color.a = 0.2, 0.4, 1.0, 1.0
            wf.points = self._wireframe_edges(mins, maxs)
            wf.lifetime = rclpy.duration.Duration(seconds=0.3).to_msg()
            markers.append(wf)

        else:
            pts = pc2.read_points_numpy(
                trk.points, field_names=('x', 'y', 'z'), skip_nans=True)
            pts = self._voxelize(pts)
            r, g, b = self.VOXEL_RGB.get(trk.classification, self.VOXEL_RGB['unknown'])

            vm = Marker()
            vm.header = header
            vm.ns = 'fused_voxels'
            vm.type = Marker.CUBE_LIST
            vm.action = Marker.ADD
            vm.pose.orientation.w = 1.0
            vm.scale.x = vm.scale.y = vm.scale.z = self.VOXEL_SIZE
            vm.color.r, vm.color.g, vm.color.b = r, g, b
            vm.color.a = 0.85 if trk.classification == 'static' else 0.7
            vm.points = [Point(x=float(p[0]), y=float(p[1]), z=float(p[2])) for p in pts]
            vm.lifetime = rclpy.duration.Duration(seconds=0.3).to_msg()
            markers.append(vm)

        return markers

    def _make_label(self, header, trk, marker_id) -> Marker:
        mins, maxs = self._bounds(trk)
        m = Marker()
        m.header = header
        m.ns = 'fused_labels'
        m.id = marker_id
        m.type = Marker.TEXT_VIEW_FACING
        m.action = Marker.ADD
        m.pose.position.x = float(trk.centroid.x)
        m.pose.position.y = float(trk.centroid.y)
        m.pose.position.z = float(maxs[2]) + 0.15
        m.pose.orientation.w = 1.0
        m.scale.z = 0.18
        m.color.r = m.color.g = m.color.b = m.color.a = 1.0
        m.text = f'{trk.track_id}:{trk.classification}'
        m.lifetime = rclpy.duration.Duration(seconds=0.3).to_msg()
        return m


def main(args=None):
    rclpy.init(args=args)
    node = FusionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()