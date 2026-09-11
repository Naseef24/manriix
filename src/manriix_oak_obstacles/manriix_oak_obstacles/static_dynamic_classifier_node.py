#!/usr/bin/env python3
"""
static_dynamic_classifier_node.py

Implements Section III-D (Classification as Dynamic or Static) of:
  Eppenberger et al., "Leveraging Stereo-Camera Data for Real-Time
  Dynamic Obstacle Detection and Tracking", IROS 2020.

III-D.1 Voting of an individual point:
  For each point k of a current *filtered* cluster (h^s), measure the
  global nearest-neighbor distance d^k to a *dense, non-filtered* point
  cloud from delta seconds ago (h^d_{t-delta}). Convert to a velocity
  d^k/delta. Vote "dynamic" if that velocity >= l_NN, else "static".

III-D.2 Excluding points from voting (Fig. 4):
  A point can only vote if we could have observed it in both frames.
  Two cases are excluded:
    (a) it falls in a part of the current FOV that did not overlap the
        previous FOV (never observed before -> no info)
    (b) it was occluded in the previous frame by a DIFFERENT object's
        track (can't tell "moved" from "was hidden, now revealed").
        Self-occlusion (occluded by the SAME track, e.g. the object
        itself rotating) does NOT exclude the point.
  Both cases require reprojecting into the *reference* camera's image
  plane at t-delta (its pose obtained via a tf2 lookup at that past
  stamp), approximating that frame's depth map, exactly as described
  for Figure 4 in the paper.

Cluster-level decision:
  dynamic if (dynamic_vote_count >= l_abs_dyn) OR
            (dynamic_vote_count / total_votes >= l_rel_dyn)
  else static. If the decision is inconsistent over a short time
  horizon tau, the cluster is marked "uncertain" instead.
"""

import numpy as np
from scipy.spatial import cKDTree
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.time import Time

import tf2_ros
from tf2_ros import TransformException

import message_filters
from sensor_msgs.msg import PointCloud2, CameraInfo
from sensor_msgs_py import point_cloud2 as pc2
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

from manriix_oak_obstacles_msgs.msg import ClusterTrackArray


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


class BufferedFrame:
    __slots__ = ('stamp', 'dense_pts', 'labeled_pts', 'labeled_ids',
                 '_dense_tree', '_labeled_tree')

    def __init__(self, stamp, dense_pts, labeled_pts, labeled_ids):
        self.stamp = stamp
        self.dense_pts = dense_pts          # (N,3) world/target_frame
        self.labeled_pts = labeled_pts      # (M,3) world/target_frame
        self.labeled_ids = labeled_ids      # (M,) track_id per labeled point
        self._dense_tree = None
        self._labeled_tree = None

    # KD-trees are built lazily ONCE per buffered frame and reused for
    # every callback that selects this frame as its delta-reference --
    # the same frame is typically reused for ~delta seconds of callbacks,
    # so this avoids rebuilding identical trees ~4-10x.
    @property
    def dense_tree(self):
        if self._dense_tree is None and self.dense_pts.shape[0] > 0:
            self._dense_tree = cKDTree(self.dense_pts)
        return self._dense_tree

    @property
    def labeled_tree(self):
        if self._labeled_tree is None and self.labeled_pts.shape[0] > 0:
            self._labeled_tree = cKDTree(self.labeled_pts)
        return self._labeled_tree


class StaticDynamicClassifierNode(Node):

    def __init__(self):
        super().__init__('static_dynamic_classifier_node')

        self.declare_parameter('dense_topic', '/manriix/points_dense')
        self.declare_parameter('tracks_topic', '/manriix/cluster_tracks')
        self.declare_parameter('camera_info_topic', '/oak/right/camera_info')
        self.declare_parameter('camera_frame', 'oak_right_camera_optical_frame')
        self.declare_parameter('map_frame', 'base_footprint')

        self.declare_parameter('output_tracks_topic', '/manriix/cluster_tracks_classified')
        self.declare_parameter('output_markers_topic', '/manriix/classification_markers')

        self.declare_parameter('delta', 0.4)          # [s] frame separation for voting, Sec IV-A
        self.declare_parameter('tau', 0.4)             # [s] consistency horizon
        self.declare_parameter('l_NN', 0.45)           # [m/s] velocity threshold
        self.declare_parameter('l_abs_dyn', 100)       # absolute vote-count threshold
        self.declare_parameter('l_rel_dyn', 0.8)       # relative vote-fraction threshold

        self.declare_parameter('occlusion_pixel_radius', 4.0)   # [px]
        self.declare_parameter('occlusion_depth_margin', 0.05)  # [m]
        self.declare_parameter('buffer_horizon', 1.5)  # [s] how much history to retain
        self.declare_parameter('max_proj_points', 6000)  # dense-cloud subsample for occlusion depth map
        # Diagnostics / bisection aids:
        self.declare_parameter('debug_votes', False)     # log per-cluster vote breakdown
        self.declare_parameter('enable_occlusion_exclusion', True)  # III-D.2 on/off

        self.dense_topic = self.get_parameter('dense_topic').value
        self.tracks_topic = self.get_parameter('tracks_topic').value
        self.camera_info_topic = self.get_parameter('camera_info_topic').value
        self.camera_frame = self.get_parameter('camera_frame').value
        self.map_frame = self.get_parameter('map_frame').value

        self.delta = float(self.get_parameter('delta').value)
        self.tau = float(self.get_parameter('tau').value)
        self.l_NN = float(self.get_parameter('l_NN').value)
        self.l_abs_dyn = int(self.get_parameter('l_abs_dyn').value)
        self.l_rel_dyn = float(self.get_parameter('l_rel_dyn').value)

        self.occlusion_pixel_radius = float(self.get_parameter('occlusion_pixel_radius').value)
        self.occlusion_depth_margin = float(self.get_parameter('occlusion_depth_margin').value)
        self.buffer_horizon = float(self.get_parameter('buffer_horizon').value)
        self.max_proj_points = int(self.get_parameter('max_proj_points').value)
        self.debug_votes = bool(self.get_parameter('debug_votes').value)
        self.enable_occlusion_exclusion = bool(
            self.get_parameter('enable_occlusion_exclusion').value)

        self.get_logger().info(
            f'delta={self.delta} tau={self.tau} l_NN={self.l_NN} '
            f'l_abs_dyn={self.l_abs_dyn} l_rel_dyn={self.l_rel_dyn}'
        )

        # ---- TF ----
        self.tf_buffer = tf2_ros.Buffer(cache_time=rclpy.duration.Duration(seconds=5.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ---- Camera intrinsics ----
        self.K = None
        self.create_subscription(
            CameraInfo, self.camera_info_topic, self._camera_info_cb, 5)

        # ---- History buffer (Sec III-D reference frame lookback) ----
        self.frame_buffer = deque()

        # ---- Per-track classification history, for tau-consistency ----
        # track_id -> deque[(stamp, label)]
        self.track_label_history = {}

        # ---- I/O ----
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            durability=DurabilityPolicy.VOLATILE,
        )

        dense_sub = message_filters.Subscriber(self, PointCloud2, self.dense_topic, qos_profile=qos)
        tracks_sub = message_filters.Subscriber(self, ClusterTrackArray, self.tracks_topic, qos_profile=qos)
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [dense_sub, tracks_sub], queue_size=30, slop=0.03)
        self.sync.registerCallback(self._synced_callback)

        self.pub_tracks = self.create_publisher(
            ClusterTrackArray, self.get_parameter('output_tracks_topic').value, qos)
        self.pub_markers = self.create_publisher(
            MarkerArray, self.get_parameter('output_markers_topic').value, 10)  # reliable: RViz default

        self._prev_marker_count = 0

    # ------------------------------------------------------------------
    def _camera_info_cb(self, msg: CameraInfo):
        if self.K is None:
            self.K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
            self.get_logger().info(f'Camera intrinsics received:\n{self.K}')

    # ------------------------------------------------------------------
    def _get_cam_pose(self, stamp_msg_time):
        """tf2 lookup of camera pose in map_frame at a given past time.
        Returns (cam_pos (3,), R_cw (3,3) camera<-world rotation) or None.
        """
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame, self.camera_frame, stamp_msg_time)
        except TransformException as ex:
            self.get_logger().warn(
                f'TF lookup failed for occlusion reasoning: {ex}',
                throttle_duration_sec=2.0)
            return None

        q = tf.transform.rotation
        t = tf.transform.translation
        R_wc = quat_to_rot_matrix(q.x, q.y, q.z, q.w)  # world <- camera
        cam_pos = np.array([t.x, t.y, t.z])
        R_cw = R_wc.T  # camera <- world
        return cam_pos, R_cw

    # ------------------------------------------------------------------
    def _synced_callback(self, dense_msg: PointCloud2, tracks_msg: ClusterTrackArray):
        if self.K is None:
            self.get_logger().warn(
                f'No camera intrinsics yet from {self.camera_info_topic} -- '
                'classification stalled', throttle_duration_sec=5.0)
            return  # wait for intrinsics

        now = Time.from_msg(dense_msg.header.stamp).nanoseconds * 1e-9
        stamp_msg_time = dense_msg.header.stamp

        dense_pts = pc2.read_points_numpy(
            dense_msg, field_names=('x', 'y', 'z'), skip_nans=True).astype(np.float64)

        # Build labeled (clustered) points for THIS frame, for buffering
        # (needed later as the "h^s_{t-delta}" reference in occlusion
        # self/other-track disambiguation).
        labeled_pts_list = []
        labeled_ids_list = []
        cluster_points = {}  # track_id -> (N,3) for this frame, used below
        for trk in tracks_msg.tracks:
            pts = pc2.read_points_numpy(
                trk.points, field_names=('x', 'y', 'z'), skip_nans=True).astype(np.float64)
            if pts.shape[0] == 0:
                continue
            cluster_points[trk.track_id] = pts
            labeled_pts_list.append(pts)
            labeled_ids_list.append(np.full(pts.shape[0], trk.track_id, dtype=np.int64))

        labeled_pts = (np.concatenate(labeled_pts_list, axis=0)
                       if labeled_pts_list else np.zeros((0, 3)))
        labeled_ids = (np.concatenate(labeled_ids_list, axis=0)
                        if labeled_ids_list else np.zeros((0,), dtype=np.int64))

        # ---- find reference frame at t - delta ----
        ref_frame = self._find_reference_frame(now - self.delta)

        classifications = {}  # track_id -> (label, dyn_ratio)

        if ref_frame is None or ref_frame.dense_pts.shape[0] == 0:
            # Not enough history yet -- mark everything uncertain rather
            # than guessing.
            for tid in cluster_points:
                classifications[tid] = ('uncertain', 0.0)
        else:
            classifications = self._classify_all_clusters(
                cluster_points, ref_frame, stamp_msg_time)

        # ---- apply tau-consistency check + publish ----
        self._publish(dense_msg.header, tracks_msg, classifications, now)

        # ---- push this frame into the buffer, prune old ones ----
        self.frame_buffer.append(
            BufferedFrame(now, dense_pts, labeled_pts, labeled_ids))
        while self.frame_buffer and (now - self.frame_buffer[0].stamp) > self.buffer_horizon:
            self.frame_buffer.popleft()

    # ------------------------------------------------------------------
    def _find_reference_frame(self, target_stamp):
        """Closest buffered frame to target_stamp, within a tolerance."""
        if not self.frame_buffer:
            return None
        best = min(self.frame_buffer, key=lambda f: abs(f.stamp - target_stamp))
        if abs(best.stamp - target_stamp) > 0.5 * self.delta + 0.15:
            return None
        return best

    # ------------------------------------------------------------------
    def _classify_all_clusters(self, cluster_points, ref_frame: BufferedFrame, stamp_msg_time):
        cam_pose = self._get_cam_pose(stamp_msg_time)

        ref_dense_tree = ref_frame.dense_tree  # cached, built once per frame

        ref_pixel_tree = None
        ref_depth = None
        ref_labeled_tree = None
        ref_dense_valid_pts = None
        R_cw = cam_pos = None

        if cam_pose is not None and ref_frame.dense_pts.shape[0] > 0:
            cam_pos, R_cw = cam_pose
            # Paper (Sec III-D.2): approximate the previous frame's depth
            # map "by projecting randomly-sampled points of the
            # non-filtered, dense point cloud" -- subsample here, which
            # is both faithful to the paper and a large speedup.
            dense = ref_frame.dense_pts
            if dense.shape[0] > self.max_proj_points:
                sel = np.random.choice(dense.shape[0], self.max_proj_points, replace=False)
                dense = dense[sel]
            pts_cam = (dense - cam_pos) @ R_cw.T
            depth = pts_cam[:, 2]
            valid = depth > 1e-3
            if np.any(valid):
                pix = (self.K @ pts_cam[valid].T).T
                pix = pix[:, :2] / pix[:, 2:3]
                ref_pixel_tree = cKDTree(pix)
                ref_depth = depth[valid]
                ref_dense_valid_pts = dense[valid]
            if ref_frame.labeled_pts.shape[0] > 0:
                ref_labeled_tree = ref_frame.labeled_tree  # cached

        results = {}
        for track_id, pts in cluster_points.items():
            dyn_votes, static_votes = self._vote_for_cluster(
                pts, track_id, ref_dense_tree,
                ref_pixel_tree, ref_depth,
                (ref_dense_valid_pts if ref_pixel_tree is not None else None),
                ref_labeled_tree, ref_frame,
                cam_pos, R_cw,
            )
            total = dyn_votes + static_votes
            if total == 0:
                results[track_id] = ('uncertain', 0.0)
                continue
            ratio = dyn_votes / total
            is_dynamic = (dyn_votes >= self.l_abs_dyn) or (ratio >= self.l_rel_dyn)
            results[track_id] = ('dynamic' if is_dynamic else 'static', ratio)

        return results

    # ------------------------------------------------------------------
    def _vote_for_cluster(self, pts, track_id, ref_dense_tree,
                           ref_pixel_tree, ref_depth, ref_dense_valid_pts,
                           ref_labeled_tree, ref_frame, cam_pos, R_cw):
        n = pts.shape[0]
        excluded = np.zeros(n, dtype=bool)

        # ---- III-D.2 occlusion / FOV exclusion (only possible if we
        # have a valid camera pose + projected reference depth map) ----
        if (self.enable_occlusion_exclusion
                and ref_pixel_tree is not None and cam_pos is not None):
            pts_cam = (pts - cam_pos) @ R_cw.T
            q_depth = pts_cam[:, 2]
            in_front = q_depth > 1e-3

            pix = np.zeros((n, 2))
            valid_idx = np.where(in_front)[0]
            if valid_idx.size > 0:
                proj = (self.K @ pts_cam[valid_idx].T).T
                pix[valid_idx] = proj[:, :2] / proj[:, 2:3]

            # points behind the reference camera couldn't have been seen
            excluded[~in_front] = True

            if valid_idx.size > 0:
                dists, nn_idx = ref_pixel_tree.query(pix[valid_idx])
                # Case (a): no reference point nearby in image space ->
                # outside the previous FOV, never observed before.
                not_seen_before = dists > self.occlusion_pixel_radius
                excluded[valid_idx[not_seen_before]] = True

                seen_mask = ~not_seen_before
                seen_local_idx = valid_idx[seen_mask]
                matched_ref_depth = ref_depth[nn_idx[seen_mask]]
                q_depth_seen = q_depth[seen_local_idx]

                # Case (b): matched reference point is substantially
                # CLOSER to the camera than our query point -> our point
                # was occluded at t-delta by whatever that closer point
                # belonged to.
                was_occluded = matched_ref_depth < (q_depth_seen - self.occlusion_depth_margin)

                occluded_local_idx = seen_local_idx[was_occluded]
                if occluded_local_idx.size > 0 and ref_labeled_tree is not None and ref_frame.labeled_pts.shape[0] > 0:
                    occ_ref_positions = ref_dense_valid_pts[nn_idx[seen_mask][was_occluded]]
                    _, occ_label_idx = ref_labeled_tree.query(occ_ref_positions)
                    occluder_track_ids = ref_frame.labeled_ids[occ_label_idx]
                    # Exclude only if occluded by a DIFFERENT track;
                    # self-occlusion (same track) is NOT excluded.
                    different_track = occluder_track_ids != track_id
                    excluded[occluded_local_idx[different_track]] = True
        # else: no camera pose / no reference projection available yet
        # (e.g. TF not warmed up) -> fall back to voting on all points,
        # matching the paper's baseline voting-only behavior.

        voting_pts = pts[~excluded]
        if self.debug_votes:
            self.get_logger().info(
                f'trk {track_id}: pts={n} excluded={int(excluded.sum())} '
                f'voting={voting_pts.shape[0]}', throttle_duration_sec=0.5)
        if voting_pts.shape[0] == 0:
            return 0, 0

        d, _ = ref_dense_tree.query(voting_pts)
        velocities = d / self.delta
        dyn_mask = velocities >= self.l_NN

        if self.debug_votes:
            self.get_logger().info(
                f'trk {track_id}: dyn={int(dyn_mask.sum())} '
                f'stat={int((~dyn_mask).sum())} '
                f'ratio={dyn_mask.mean():.2f} '
                f'med_v={np.median(velocities):.2f}m/s',
                throttle_duration_sec=0.5)

        return int(dyn_mask.sum()), int((~dyn_mask).sum())

    # ------------------------------------------------------------------
    def _apply_consistency(self, track_id, label, now):
        """Sec III-D: if classification is inconsistent over tau, mark
        the cluster 'uncertain' instead."""
        hist = self.track_label_history.setdefault(track_id, deque())
        hist.append((now, label))
        while hist and (now - hist[0][0]) > self.tau:
            hist.popleft()

        labels_in_window = {lbl for (_, lbl) in hist if lbl != 'uncertain'}
        if len(labels_in_window) > 1:
            return 'uncertain'
        return label

    # ------------------------------------------------------------------
    def _publish(self, header, tracks_msg: ClusterTrackArray, classifications, now):
        out_msg = ClusterTrackArray()
        out_msg.header = header

        marker_array = MarkerArray()
        marker_id = 0

        for trk in tracks_msg.tracks:
            label, ratio = classifications.get(trk.track_id, ('uncertain', 0.0))
            label = self._apply_consistency(trk.track_id, label, now)

            trk.classification = label
            trk.dynamic_vote_ratio = float(ratio)
            out_msg.tracks.append(trk)

            marker_array.markers.append(
                self._make_marker(header, trk, label, marker_id))
            marker_id += 1

        for extra_id in range(marker_id, self._prev_marker_count):
            m = Marker()
            m.header = header
            m.id = extra_id
            m.action = Marker.DELETE
            marker_array.markers.append(m)
        self._prev_marker_count = marker_id

        self.pub_tracks.publish(out_msg)
        self.pub_markers.publish(marker_array)

    # ------------------------------------------------------------------
    def _make_marker(self, header, trk, label, marker_id) -> Marker:
        # Paper style (Fig. 2 "Static/Dynamic Classification" thumbnail):
        # sparse individual points colored by label, not a solid box.
        pts = pc2.read_points_numpy(
            trk.points, field_names=('x', 'y', 'z'), skip_nans=True).astype(np.float64)

        m = Marker()
        m.header = header
        m.ns = 'classification'
        m.id = marker_id
        m.type = Marker.POINTS
        m.action = Marker.ADD
        m.pose.orientation.w = 1.0
        m.scale.x = 0.03
        m.scale.y = 0.03

        if label == 'dynamic':
            m.color.r, m.color.g, m.color.b = 1.0, 0.15, 0.15
        elif label == 'static':
            m.color.r, m.color.g, m.color.b = 0.15, 1.0, 0.15
        else:
            m.color.r, m.color.g, m.color.b = 1.0, 0.85, 0.1
        m.color.a = 0.9
        m.points = [Point(x=float(p[0]), y=float(p[1]), z=float(p[2])) for p in pts]
        m.lifetime = rclpy.duration.Duration(seconds=0.3).to_msg()
        return m


def main(args=None):
    rclpy.init(args=args)
    node = StaticDynamicClassifierNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()