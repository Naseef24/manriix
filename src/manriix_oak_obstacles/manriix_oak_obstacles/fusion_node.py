#!/usr/bin/env python3
"""
fusion_node.py

Implements Section III-F (Fusion of 2D and 3D Detection and Tracking) of:
  Eppenberger et al., "Leveraging Stereo-Camera Data for Real-Time
  Dynamic Obstacle Detection and Tracking", IROS 2020.

Inputs:
  - /manriix/cluster_tracks_classified  (ClusterTrackArray)  3D tracks,
    already labeled static/dynamic/uncertain by the voting module
  - /manriix/person_boxes               (BoundingBoxTrackArray) tracked
    person detections with 3D positions in the same world frame

Logic (paper):
  For every cluster track T^i, compute the frequency f_a^i of associated
  person detections. If f_a^i exceeds the confidence threshold gamma
  (paper: 1.5 s^-1), classify all clusters added to this cluster track
  as representing a PEDESTRIAN -- i.e. a dynamic object -- regardless of
  their initial (motion-based) classification. This is what makes a
  STANDING person correctly classified as dynamic.

  Consistency check: if the boxes of one person-box track are not
  consistently associated to clusters of the same cluster track, reset
  f_a (prevents a person box hopping between a human and nearby
  furniture from poisoning the furniture's track).

Association is nearest-centroid in the x-y plane (both sources are in
the same world frame; the OAK's ROI-averaged z is unreliable, and the
paper's downstream occupancy grid is 2D anyway).

Publishes:
  - /manriix/cluster_tracks_fused   (ClusterTrackArray) final labels,
        with person tracks relabeled classification='person'
  - /manriix/fusion_markers         (MarkerArray) RViz: person=blue,
        dynamic=red, static=green, uncertain=yellow
"""

import numpy as np
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.time import Time

import struct
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2 as pc2
from visualization_msgs.msg import Marker, MarkerArray

from manriix_oak_obstacles_msgs.msg import (
    ClusterTrackArray, BoundingBoxTrackArray)


class FusionNode(Node):

    def __init__(self):
        super().__init__('fusion_node')

        self.declare_parameter('tracks_topic', '/manriix/cluster_tracks_classified')
        self.declare_parameter('person_boxes_topic', '/manriix/person_boxes')
        self.declare_parameter('output_tracks_topic', '/manriix/cluster_tracks_fused')
        self.declare_parameter('output_markers_topic', '/manriix/fusion_markers')

        self.declare_parameter('gamma', 1.5)          # [1/s] association-frequency threshold (paper Sec IV-A)
        self.declare_parameter('freq_window', 2.0)     # [s] window over which f_a is measured
        self.declare_parameter('assoc_gate_xy', 0.7)   # [m] person<->cluster x-y association gate
        self.declare_parameter('person_label_hold', 1.5)  # [s] keep 'person' label after last association

        self.gamma = float(self.get_parameter('gamma').value)
        self.freq_window = float(self.get_parameter('freq_window').value)
        self.assoc_gate_xy = float(self.get_parameter('assoc_gate_xy').value)
        self.person_label_hold = float(self.get_parameter('person_label_hold').value)

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
        # Fig.1-style labeled cloud: every cluster's POINTS colored by
        # final classification (green=static, red=dynamic, blue=person,
        # yellow=uncertain). Pure visualization; reliable QoS for RViz.
        self.pub_labeled_cloud = self.create_publisher(
            PointCloud2, '/manriix/points_labeled', 10)

        # State
        self.latest_boxes = []          # [(box_id, np.array([x,y]))]
        self.latest_boxes_stamp = 0.0

        self.assoc_stamps = {}          # cluster track_id -> deque[stamp]
        self.person_until = {}          # cluster track_id -> stamp until which 'person' holds
        self.box_last_cluster = {}      # person box_id -> cluster track_id (consistency check)

        self._prev_marker_count = 0

        self.get_logger().info(
            f'gamma={self.gamma}/s window={self.freq_window}s '
            f'gate={self.assoc_gate_xy}m hold={self.person_label_hold}s')

    # ------------------------------------------------------------------
    def _boxes_cb(self, msg: BoundingBoxTrackArray):
        self.latest_boxes = [
            (b.box_id, np.array([b.position.x, b.position.y]))
            for b in msg.boxes
        ]
        self.latest_boxes_stamp = Time.from_msg(msg.header.stamp).nanoseconds * 1e-9

    # ------------------------------------------------------------------
    def _tracks_cb(self, msg: ClusterTrackArray):
        now = Time.from_msg(msg.header.stamp).nanoseconds * 1e-9

        # Use person boxes only if reasonably fresh (detector runs ~9Hz)
        boxes = self.latest_boxes if (now - self.latest_boxes_stamp) < 0.5 else []

        centroids = {
            trk.track_id: np.array([trk.centroid.x, trk.centroid.y])
            for trk in msg.tracks
        }

        # ---- associate each person box to nearest cluster (x-y gate) ----
        for box_id, box_xy in boxes:
            best_tid, best_d = None, self.assoc_gate_xy
            for tid, c_xy in centroids.items():
                d = np.linalg.norm(c_xy - box_xy)
                if d < best_d:
                    best_tid, best_d = tid, d
            if best_tid is None:
                continue

            # Consistency check (paper): if this person-box track was
            # previously associated to a DIFFERENT cluster track, reset
            # that association frequency instead of accumulating.
            prev_tid = self.box_last_cluster.get(box_id)
            if prev_tid is not None and prev_tid != best_tid:
                self.assoc_stamps.pop(prev_tid, None)
            self.box_last_cluster[box_id] = best_tid

            self.assoc_stamps.setdefault(best_tid, deque()).append(now)

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

        # drop bookkeeping for dead tracks
        live = set(centroids.keys())
        for d in (self.assoc_stamps, self.person_until):
            for tid in [t for t in d if t not in live]:
                del d[tid]

        # ---- publish fused labels ----
        out = ClusterTrackArray()
        out.header = msg.header

        marker_array = MarkerArray()
        marker_id = 0

        for trk in msg.tracks:
            if self.person_until.get(trk.track_id, 0.0) > now:
                trk.classification = 'person'
            out.tracks.append(trk)

            marker_array.markers.append(
                self._make_marker(msg.header, trk, marker_id))
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

        self._publish_labeled_cloud(out)
        self.pub_tracks.publish(out)
        self.pub_markers.publish(marker_array)

    # ------------------------------------------------------------------
    COLORS = {
        'person':    (0.2, 0.4, 1.0),
        'dynamic':   (1.0, 0.15, 0.15),
        'static':    (0.15, 1.0, 0.15),
        'uncertain': (1.0, 0.85, 0.1),
        'unknown':   (0.6, 0.6, 0.6),
    }

    def _bounds(self, trk):
        pts = pc2.read_points_numpy(
            trk.points, field_names=('x', 'y', 'z'), skip_nans=True)
        if pts.shape[0] == 0:
            c = np.array([trk.centroid.x, trk.centroid.y, trk.centroid.z])
            return c, c
        return pts.min(axis=0), pts.max(axis=0)

    def _make_marker(self, header, trk, marker_id) -> Marker:
        mins, maxs = self._bounds(trk)
        size = np.maximum(maxs - mins, 0.05)
        center = (mins + maxs) / 2.0

        m = Marker()
        m.header = header
        m.ns = 'fused'
        m.id = marker_id
        m.type = Marker.CUBE
        m.action = Marker.ADD
        m.pose.position.x, m.pose.position.y, m.pose.position.z = [float(v) for v in center]
        m.pose.orientation.w = 1.0
        m.scale.x, m.scale.y, m.scale.z = [float(s) for s in size]
        r, g, b = self.COLORS.get(trk.classification, self.COLORS['unknown'])
        m.color.r, m.color.g, m.color.b, m.color.a = r, g, b, 0.5
        m.lifetime = rclpy.duration.Duration(seconds=0.3).to_msg()
        return m

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


    # ------------------------------------------------------------------
    LABEL_RGB = {
        'person':    (40, 90, 255),
        'dynamic':   (255, 40, 40),
        'static':    (40, 255, 40),
        'uncertain': (255, 210, 30),
        'unknown':   (150, 150, 150),
    }

    def _publish_labeled_cloud(self, tracks_msg: ClusterTrackArray):
        """Fig.1-style cloud: cluster points colored by classification."""
        all_pts = []
        for trk in tracks_msg.tracks:
            pts = pc2.read_points_numpy(
                trk.points, field_names=('x', 'y', 'z'), skip_nans=True)
            if pts.shape[0] == 0:
                continue
            r, g, b = self.LABEL_RGB.get(
                trk.classification, self.LABEL_RGB['unknown'])
            rgb = struct.unpack('f', struct.pack(
                'I', (r << 16) | (g << 8) | b))[0]
            colored = np.column_stack(
                [pts, np.full(pts.shape[0], rgb, dtype=np.float32)])
            all_pts.append(colored)

        data = (np.concatenate(all_pts, axis=0).astype(np.float32)
                if all_pts else np.zeros((0, 4), dtype=np.float32))
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        cloud = pc2.create_cloud(tracks_msg.header, fields, data)
        self.pub_labeled_cloud.publish(cloud)


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


# #!/usr/bin/env python3
# """
# fusion_node.py

# Implements Section III-F (Fusion of 2D and 3D Detection and Tracking) of:
#   Eppenberger et al., "Leveraging Stereo-Camera Data for Real-Time
#   Dynamic Obstacle Detection and Tracking", IROS 2020.

# Inputs:
#   - /manriix/cluster_tracks_classified  (ClusterTrackArray)  3D tracks,
#     already labeled static/dynamic/uncertain by the voting module
#   - /manriix/person_boxes               (BoundingBoxTrackArray) tracked
#     person detections with 3D positions in the same world frame

# Logic (paper):
#   For every cluster track T^i, compute the frequency f_a^i of associated
#   person detections. If f_a^i exceeds the confidence threshold gamma
#   (paper: 1.5 s^-1), classify all clusters added to this cluster track
#   as representing a PEDESTRIAN -- i.e. a dynamic object -- regardless of
#   their initial (motion-based) classification. This is what makes a
#   STANDING person correctly classified as dynamic.

#   Consistency check: if the boxes of one person-box track are not
#   consistently associated to clusters of the same cluster track, reset
#   f_a (prevents a person box hopping between a human and nearby
#   furniture from poisoning the furniture's track).

# Association is nearest-centroid in the x-y plane (both sources are in
# the same world frame; the OAK's ROI-averaged z is unreliable, and the
# paper's downstream occupancy grid is 2D anyway).

# Publishes:
#   - /manriix/cluster_tracks_fused   (ClusterTrackArray) final labels,
#         with person tracks relabeled classification='person'
#   - /manriix/fusion_markers         (MarkerArray) RViz: person=blue,
#         dynamic=red, static=green, uncertain=yellow
# """

# import numpy as np
# from collections import deque

# import rclpy
# from rclpy.node import Node
# from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
# from rclpy.time import Time

# from sensor_msgs_py import point_cloud2 as pc2
# from visualization_msgs.msg import Marker, MarkerArray

# from manriix_oak_obstacles_msgs.msg import (
#     ClusterTrackArray, BoundingBoxTrackArray)


# class FusionNode(Node):

#     def __init__(self):
#         super().__init__('fusion_node')

#         self.declare_parameter('tracks_topic', '/manriix/cluster_tracks_classified')
#         self.declare_parameter('person_boxes_topic', '/manriix/person_boxes')
#         self.declare_parameter('output_tracks_topic', '/manriix/cluster_tracks_fused')
#         self.declare_parameter('output_markers_topic', '/manriix/fusion_markers')

#         self.declare_parameter('gamma', 1.5)          # [1/s] association-frequency threshold (paper Sec IV-A)
#         self.declare_parameter('freq_window', 2.0)     # [s] window over which f_a is measured
#         self.declare_parameter('assoc_gate_xy', 0.7)   # [m] person<->cluster x-y association gate
#         self.declare_parameter('person_label_hold', 1.5)  # [s] keep 'person' label after last association

#         self.gamma = float(self.get_parameter('gamma').value)
#         self.freq_window = float(self.get_parameter('freq_window').value)
#         self.assoc_gate_xy = float(self.get_parameter('assoc_gate_xy').value)
#         self.person_label_hold = float(self.get_parameter('person_label_hold').value)

#         qos = QoSProfile(
#             reliability=ReliabilityPolicy.BEST_EFFORT,
#             history=HistoryPolicy.KEEP_LAST,
#             depth=5,
#             durability=DurabilityPolicy.VOLATILE,
#         )

#         self.create_subscription(
#             ClusterTrackArray,
#             self.get_parameter('tracks_topic').value,
#             self._tracks_cb, qos)
#         self.create_subscription(
#             BoundingBoxTrackArray,
#             self.get_parameter('person_boxes_topic').value,
#             self._boxes_cb, 10)

#         self.pub_tracks = self.create_publisher(
#             ClusterTrackArray,
#             self.get_parameter('output_tracks_topic').value, qos)
#         self.pub_markers = self.create_publisher(
#             MarkerArray,
#             self.get_parameter('output_markers_topic').value, 10)

#         # State
#         self.latest_boxes = []          # [(box_id, np.array([x,y]))]
#         self.latest_boxes_stamp = 0.0

#         self.assoc_stamps = {}          # cluster track_id -> deque[stamp]
#         self.person_until = {}          # cluster track_id -> stamp until which 'person' holds
#         self.box_last_cluster = {}      # person box_id -> cluster track_id (consistency check)

#         self._prev_marker_count = 0

#         self.get_logger().info(
#             f'gamma={self.gamma}/s window={self.freq_window}s '
#             f'gate={self.assoc_gate_xy}m hold={self.person_label_hold}s')

#     # ------------------------------------------------------------------
#     def _boxes_cb(self, msg: BoundingBoxTrackArray):
#         self.latest_boxes = [
#             (b.box_id, np.array([b.position.x, b.position.y]))
#             for b in msg.boxes
#         ]
#         self.latest_boxes_stamp = Time.from_msg(msg.header.stamp).nanoseconds * 1e-9

#     # ------------------------------------------------------------------
#     def _tracks_cb(self, msg: ClusterTrackArray):
#         now = Time.from_msg(msg.header.stamp).nanoseconds * 1e-9

#         # Use person boxes only if reasonably fresh (detector runs ~9Hz)
#         boxes = self.latest_boxes if (now - self.latest_boxes_stamp) < 0.5 else []

#         centroids = {
#             trk.track_id: np.array([trk.centroid.x, trk.centroid.y])
#             for trk in msg.tracks
#         }

#         # ---- associate each person box to nearest cluster (x-y gate) ----
#         for box_id, box_xy in boxes:
#             best_tid, best_d = None, self.assoc_gate_xy
#             for tid, c_xy in centroids.items():
#                 d = np.linalg.norm(c_xy - box_xy)
#                 if d < best_d:
#                     best_tid, best_d = tid, d
#             if best_tid is None:
#                 continue

#             # Consistency check (paper): if this person-box track was
#             # previously associated to a DIFFERENT cluster track, reset
#             # that association frequency instead of accumulating.
#             prev_tid = self.box_last_cluster.get(box_id)
#             if prev_tid is not None and prev_tid != best_tid:
#                 self.assoc_stamps.pop(prev_tid, None)
#             self.box_last_cluster[box_id] = best_tid

#             self.assoc_stamps.setdefault(best_tid, deque()).append(now)

#         # ---- compute f_a per cluster track, latch 'person' label ----
#         for tid, stamps in list(self.assoc_stamps.items()):
#             while stamps and (now - stamps[0]) > self.freq_window:
#                 stamps.popleft()
#             if not stamps:
#                 del self.assoc_stamps[tid]
#                 continue
#             f_a = len(stamps) / self.freq_window
#             if f_a >= self.gamma:
#                 self.person_until[tid] = now + self.person_label_hold

#         # drop bookkeeping for dead tracks
#         live = set(centroids.keys())
#         for d in (self.assoc_stamps, self.person_until):
#             for tid in [t for t in d if t not in live]:
#                 del d[tid]

#         # ---- publish fused labels ----
#         out = ClusterTrackArray()
#         out.header = msg.header

#         marker_array = MarkerArray()
#         marker_id = 0

#         for trk in msg.tracks:
#             if self.person_until.get(trk.track_id, 0.0) > now:
#                 trk.classification = 'person'
#             out.tracks.append(trk)

#             marker_array.markers.append(
#                 self._make_marker(msg.header, trk, marker_id))
#             marker_id += 1
#             marker_array.markers.append(
#                 self._make_label(msg.header, trk, marker_id))
#             marker_id += 1

#         for extra in range(marker_id, self._prev_marker_count):
#             m = Marker()
#             m.header = out.header
#             m.id = extra
#             m.action = Marker.DELETE
#             marker_array.markers.append(m)
#         self._prev_marker_count = marker_id

#         self.pub_tracks.publish(out)
#         self.pub_markers.publish(marker_array)

#     # ------------------------------------------------------------------
#     COLORS = {
#         'person':    (0.2, 0.4, 1.0),
#         'dynamic':   (1.0, 0.15, 0.15),
#         'static':    (0.15, 1.0, 0.15),
#         'uncertain': (1.0, 0.85, 0.1),
#         'unknown':   (0.6, 0.6, 0.6),
#     }

#     def _bounds(self, trk):
#         pts = pc2.read_points_numpy(
#             trk.points, field_names=('x', 'y', 'z'), skip_nans=True)
#         if pts.shape[0] == 0:
#             c = np.array([trk.centroid.x, trk.centroid.y, trk.centroid.z])
#             return c, c
#         return pts.min(axis=0), pts.max(axis=0)

#     def _make_marker(self, header, trk, marker_id) -> Marker:
#         mins, maxs = self._bounds(trk)
#         size = np.maximum(maxs - mins, 0.05)
#         center = (mins + maxs) / 2.0

#         m = Marker()
#         m.header = header
#         m.ns = 'fused'
#         m.id = marker_id
#         m.type = Marker.CUBE
#         m.action = Marker.ADD
#         m.pose.position.x, m.pose.position.y, m.pose.position.z = [float(v) for v in center]
#         m.pose.orientation.w = 1.0
#         m.scale.x, m.scale.y, m.scale.z = [float(s) for s in size]
#         r, g, b = self.COLORS.get(trk.classification, self.COLORS['unknown'])
#         m.color.r, m.color.g, m.color.b, m.color.a = r, g, b, 0.5
#         m.lifetime = rclpy.duration.Duration(seconds=0.3).to_msg()
#         return m

#     def _make_label(self, header, trk, marker_id) -> Marker:
#         mins, maxs = self._bounds(trk)
#         m = Marker()
#         m.header = header
#         m.ns = 'fused_labels'
#         m.id = marker_id
#         m.type = Marker.TEXT_VIEW_FACING
#         m.action = Marker.ADD
#         m.pose.position.x = float(trk.centroid.x)
#         m.pose.position.y = float(trk.centroid.y)
#         m.pose.position.z = float(maxs[2]) + 0.15
#         m.pose.orientation.w = 1.0
#         m.scale.z = 0.18
#         m.color.r = m.color.g = m.color.b = m.color.a = 1.0
#         m.text = f'{trk.track_id}:{trk.classification}'
#         m.lifetime = rclpy.duration.Duration(seconds=0.3).to_msg()
#         return m


# def main(args=None):
#     rclpy.init(args=args)
#     node = FusionNode()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()


# if __name__ == '__main__':
#     main()