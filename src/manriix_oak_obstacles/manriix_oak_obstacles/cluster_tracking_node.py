#!/usr/bin/env python3
"""
cluster_tracking_node.py

Implements Section III-C (Clustering and 3D Tracking) of:
  Eppenberger et al., "Leveraging Stereo-Camera Data for Real-Time
  Dynamic Obstacle Detection and Tracking", IROS 2020.

1) Clustering (Sec III-C.1):
   DBSCAN on the filtered cloud h^s (/manriix/points_filtered) -> a set
   of clusters. (Bounding-box refinement using the 2D people detector is
   added in Step 5 / Section III-F; skipped here.)

2) 3D tracking (Sec III-C.2):
   At each frame t, compute centroids c_t of all current clusters C_t.
   Associate them to their closest previous centroid c*_{t-dt}. We
   implement this as a Global Nearest Neighbor (GNN) assignment via the
   Hungarian algorithm (the paper's abstract explicitly describes their
   method as combining "global nearest neighbor searches" with the
   people detector) with a distance gate, rather than naive greedy
   matching. Centroids that cannot be related to any previous centroid
   become new tracks ("newly appeared objects"); unmatched previous
   tracks are kept alive for a short timeout to survive brief gaps
   before being marked lost ("lost objects").

Publishes:
  - /manriix/cluster_tracks   (manriix_oak_obstacles_msgs/ClusterTrackArray)
        Full structured per-cluster data for downstream steps
        (classification, fusion, motion estimation).
  - /manriix/cluster_markers  (visualization_msgs/MarkerArray)
        Bounding boxes + track-ID text, for RViz.
"""

import numpy as np
from scipy.optimize import linear_sum_assignment
from sklearn.cluster import DBSCAN

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.time import Time
from rclpy.duration import Duration

from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Vector3

from manriix_oak_obstacles_msgs.msg import ClusterTrack, ClusterTrackArray


class Track:
    """Internal (non-ROS) bookkeeping for a single tracked cluster."""

    _next_id = 0

    def __init__(self, centroid: np.ndarray, points: np.ndarray, stamp):
        self.track_id = Track._next_id
        Track._next_id += 1

        self.centroid = centroid
        self.points = points
        self.created_stamp = stamp
        self.last_seen_stamp = stamp
        self.missed_frames = 0
        self.hit_count = 1  # consecutive matched frames; gates confirmation

        # Filled in by later pipeline steps; kept here so this node is
        # forward-compatible without needing rewrites downstream.
        self.velocity = np.zeros(2)
        self.velocity_cov = np.array([1.0, 1.0])  # xx, yy
        self.classification = 'unknown'
        self.dynamic_vote_ratio = 0.0

    def age(self, now_stamp) -> float:
        return max(0.0, (now_stamp - self.created_stamp))


class ClusterTrackingNode(Node):

    def __init__(self):
        super().__init__('cluster_tracking_node')

        self.declare_parameter('input_topic', '/manriix/points_filtered')
        self.declare_parameter('output_tracks_topic', '/manriix/cluster_tracks')
        self.declare_parameter('output_markers_topic', '/manriix/cluster_markers')

        self.declare_parameter('dbscan_eps', 0.3)          # [m]
        self.declare_parameter('dbscan_min_samples', 15)   # points
        self.declare_parameter('min_cluster_points', 15)   # discard tiny clusters

        self.declare_parameter('track_max_assoc_dist', 0.6)  # [m] GNN gate
        self.declare_parameter('track_timeout', 1.0)          # [s] before dropping a lost track
        self.declare_parameter('min_hits_to_confirm', 3)      # consecutive matched frames before publishing/drawing

        self.input_topic = self.get_parameter('input_topic').value
        self.output_tracks_topic = self.get_parameter('output_tracks_topic').value
        self.output_markers_topic = self.get_parameter('output_markers_topic').value

        self.dbscan_eps = float(self.get_parameter('dbscan_eps').value)
        self.dbscan_min_samples = int(self.get_parameter('dbscan_min_samples').value)
        self.min_cluster_points = int(self.get_parameter('min_cluster_points').value)

        self.track_max_assoc_dist = float(self.get_parameter('track_max_assoc_dist').value)
        self.track_timeout = float(self.get_parameter('track_timeout').value)
        self.min_hits_to_confirm = int(self.get_parameter('min_hits_to_confirm').value)

        self.get_logger().info(
            f'DBSCAN eps={self.dbscan_eps} min_samples={self.dbscan_min_samples} | '
            f'assoc_gate={self.track_max_assoc_dist}m timeout={self.track_timeout}s'
        )

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.sub = self.create_subscription(
            PointCloud2, self.input_topic, self.cloud_callback, qos)
        self.pub_tracks = self.create_publisher(
            ClusterTrackArray, self.output_tracks_topic, qos)
        self.pub_markers = self.create_publisher(
            MarkerArray, self.output_markers_topic, 10)  # reliable: RViz default

        self.tracks = {}  # track_id -> Track
        self._prev_marker_count = 0

    # ------------------------------------------------------------------
    def cloud_callback(self, msg: PointCloud2):
        now = Time.from_msg(msg.header.stamp).nanoseconds * 1e-9

        pts = pc2.read_points_numpy(
            msg, field_names=('x', 'y', 'z'), skip_nans=True).astype(np.float64)

        clusters = self._cluster(pts)
        self._associate_and_update(clusters, now)
        self._prune_lost_tracks(now)

        self._publish(msg.header, now)

    # ------------------------------------------------------------------
    def _cluster(self, pts: np.ndarray):
        """DBSCAN -> list of (centroid, points) tuples."""
        if pts.shape[0] < self.min_cluster_points:
            return []

        labels = DBSCAN(
            eps=self.dbscan_eps,
            min_samples=self.dbscan_min_samples,
        ).fit_predict(pts)

        clusters = []
        for label in set(labels):
            if label == -1:  # DBSCAN noise label -- discarded, as intended
                continue
            cluster_pts = pts[labels == label]
            if cluster_pts.shape[0] < self.min_cluster_points:
                continue
            centroid = cluster_pts.mean(axis=0)
            clusters.append((centroid, cluster_pts))

        return clusters

    # ------------------------------------------------------------------
    def _associate_and_update(self, clusters, now):
        """Global Nearest Neighbor association (Hungarian) + track update."""
        track_ids = list(self.tracks.keys())
        n_tracks = len(track_ids)
        n_clusters = len(clusters)

        matched_track_ids = set()
        matched_cluster_idxs = set()

        if n_tracks > 0 and n_clusters > 0:
            cost = np.full((n_tracks, n_clusters), fill_value=1e6)
            for i, tid in enumerate(track_ids):
                t_centroid = self.tracks[tid].centroid
                for j, (c_centroid, _) in enumerate(clusters):
                    d = np.linalg.norm(t_centroid - c_centroid)
                    if d <= self.track_max_assoc_dist:
                        cost[i, j] = d

            row_idx, col_idx = linear_sum_assignment(cost)
            for i, j in zip(row_idx, col_idx):
                if cost[i, j] >= 1e6:
                    continue  # assignment exists only because matrix is square; reject bad ones
                tid = track_ids[i]
                centroid, points = clusters[j]
                trk = self.tracks[tid]
                trk.centroid = centroid
                trk.points = points
                trk.last_seen_stamp = now
                trk.missed_frames = 0
                trk.hit_count += 1
                matched_track_ids.add(tid)
                matched_cluster_idxs.add(j)

        # Unmatched previous tracks: "lost objects" this frame
        for tid in track_ids:
            if tid not in matched_track_ids:
                self.tracks[tid].missed_frames += 1
                self.tracks[tid].hit_count = 0  # any miss resets confirmation streak

        # Unmatched current clusters: "newly appeared objects"
        for j, (centroid, points) in enumerate(clusters):
            if j not in matched_cluster_idxs:
                trk = Track(centroid, points, now)
                self.tracks[trk.track_id] = trk

    # ------------------------------------------------------------------
    def _prune_lost_tracks(self, now):
        to_delete = [
            tid for tid, trk in self.tracks.items()
            if (now - trk.last_seen_stamp) > self.track_timeout
        ]
        for tid in to_delete:
            del self.tracks[tid]

    # ------------------------------------------------------------------
    def _publish(self, header, now):
        arr_msg = ClusterTrackArray()
        arr_msg.header = header

        marker_array = MarkerArray()
        marker_id = 0

        for trk in self.tracks.values():
            # Only publish CONFIRMED, currently-matched tracks downstream.
            # This is the fix for transient noise clusters (e.g. far-range
            # ground noise piercing the height crop for one frame) flashing
            # on as boxes: real objects persist across many consecutive
            # frames, single-frame noise usually doesn't.
            if trk.hit_count < self.min_hits_to_confirm or trk.missed_frames > 0:
                continue

            ct = ClusterTrack()
            ct.track_id = trk.track_id
            ct.centroid = Point(x=float(trk.centroid[0]),
                                 y=float(trk.centroid[1]),
                                 z=float(trk.centroid[2]))
            ct.velocity = Vector3(x=float(trk.velocity[0]),
                                   y=float(trk.velocity[1]),
                                   z=0.0)
            ct.velocity_cov_xx = float(trk.velocity_cov[0])
            ct.velocity_cov_yy = float(trk.velocity_cov[1])
            ct.num_points = int(trk.points.shape[0])
            ct.age = float(trk.age(now))
            ct.missed_frames = int(trk.missed_frames)
            ct.classification = trk.classification
            ct.dynamic_vote_ratio = float(trk.dynamic_vote_ratio)
            ct.points = pc2.create_cloud_xyz32(header, trk.points)

            arr_msg.tracks.append(ct)

            marker_array.markers.append(
                self._make_box_marker(header, trk, marker_id))
            marker_id += 1
            marker_array.markers.append(
                self._make_text_marker(header, trk, marker_id))
            marker_id += 1

        # Clear stale markers from previous frame that we didn't overwrite
        for extra_id in range(marker_id, self._prev_marker_count):
            m = Marker()
            m.header = header
            m.id = extra_id
            m.action = Marker.DELETE
            marker_array.markers.append(m)
        self._prev_marker_count = marker_id

        self.pub_tracks.publish(arr_msg)
        self.pub_markers.publish(marker_array)

    # ------------------------------------------------------------------
    def _make_box_marker(self, header, trk: Track, marker_id: int) -> Marker:
        # Paper style (Fig. 2 "Clustering and 3D Tracking" thumbnail):
        # sparse individual points, NOT a solid bounding box.
        m = Marker()
        m.header = header
        m.ns = 'cluster_points'
        m.id = marker_id
        m.type = Marker.POINTS
        m.action = Marker.ADD
        m.pose.orientation.w = 1.0
        m.scale.x = 0.03  # point width
        m.scale.y = 0.03  # point height
        m.color.r = 0.25
        m.color.g = 0.85
        m.color.b = 0.25
        m.color.a = 0.9
        m.points = [Point(x=float(p[0]), y=float(p[1]), z=float(p[2]))
                    for p in trk.points]
        m.lifetime = Duration(seconds=0.3).to_msg()
        return m

    def _make_text_marker(self, header, trk: Track, marker_id: int) -> Marker:
        pts = trk.points
        top = pts[:, 2].max()

        m = Marker()
        m.header = header
        m.ns = 'cluster_ids'
        m.id = marker_id
        m.type = Marker.TEXT_VIEW_FACING
        m.action = Marker.ADD
        m.pose.position.x = float(trk.centroid[0])
        m.pose.position.y = float(trk.centroid[1])
        m.pose.position.z = float(top) + 0.15
        m.pose.orientation.w = 1.0
        m.scale.z = 0.2
        m.color.r = 1.0
        m.color.g = 1.0
        m.color.b = 1.0
        m.color.a = 1.0
        m.text = f'ID:{trk.track_id}'
        m.lifetime = Duration(seconds=0.3).to_msg()
        return m


def main(args=None):
    rclpy.init(args=args)
    node = ClusterTrackingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()