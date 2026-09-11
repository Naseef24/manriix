#!/usr/bin/env python3
"""
motion_estimation_node.py

Implements Section III-G (Motion Estimation) of:
  Eppenberger et al., "Leveraging Stereo-Camera Data for Real-Time
  Dynamic Obstacle Detection and Tracking", IROS 2020.

For every DYNAMIC object (classification 'dynamic' or 'person') we run a
constant-velocity 2D Kalman filter, exactly as in the paper:

  state        x = [x, y, xdot, ydot]^T
  measurement  z = [c_x, c_y]^T   (cluster centroid in the world x-y plane)

  x[k+1] = A x[k] + N(0, Q)
  z[k]   = H x[k] + N(0, R)

        [1 0 Ts 0 ]
  A  =  [0 1 0  Ts]      H = first two rows of identity
        [0 0 1  0 ]
        [0 0 0  1 ]

Occlusion handling (paper): when a tracked object i is lost, we KEEP its
KF predicting. For every newly appearing object j we evaluate
p(j = i) = N(c_j | c_hat_i, C_i(x_i)) using the predicted position and
its covariance; if it exceeds a threshold, the new track is CONNECTED to
the lost one (it inherits the old identity and its filter). This is what
carries a person's identity through short occlusions (walking behind a
pillar) instead of assigning a fresh ID.

Publishes:
  - /manriix/objects            (ClusterTrackArray) final tracks with
        velocity + velocity covariance filled in
  - /manriix/motion_markers     (MarkerArray) velocity arrows + boxes
"""

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.time import Time

from geometry_msgs.msg import Point, Vector3
from sensor_msgs_py import point_cloud2 as pc2
from visualization_msgs.msg import Marker, MarkerArray

from manriix_oak_obstacles_msgs.msg import ClusterTrackArray


DYNAMIC_LABELS = {'dynamic', 'person'}


class KalmanCV2D:
    """Constant-velocity 2D KF with the paper's A/H structure."""

    def __init__(self, x0, y0, q_pos, q_vel, r_meas):
        self.x = np.array([x0, y0, 0.0, 0.0])
        self.P = np.diag([0.1, 0.1, 1.0, 1.0])
        self.q_pos = q_pos
        self.q_vel = q_vel
        self.R = np.eye(2) * r_meas
        self.H = np.array([[1., 0., 0., 0.],
                           [0., 1., 0., 0.]])

    def predict(self, ts):
        A = np.array([[1., 0., ts, 0.],
                      [0., 1., 0., ts],
                      [0., 0., 1., 0.],
                      [0., 0., 0., 1.]])
        Q = np.diag([self.q_pos, self.q_pos, self.q_vel, self.q_vel]) * max(ts, 1e-3)
        self.x = A @ self.x
        self.P = A @ self.P @ A.T + Q

    def update(self, zx, zy):
        z = np.array([zx, zy])
        y = z - self.H @ self.x
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        self.P = (np.eye(4) - K @ self.H) @ self.P

    def position(self):
        return self.x[:2]

    def velocity(self):
        return self.x[2:]

    def pos_cov(self):
        return self.P[:2, :2]


class TrackedObject:
    def __init__(self, track_id, kf, stamp):
        self.track_id = track_id       # pipeline (cluster) track id currently feeding this object
        self.object_id = track_id      # persistent identity (survives re-association)
        self.kf = kf
        self.last_update = stamp
        self.lost_since = None          # stamp when it went lost, None if live
        self.trail = []                 # [(x, y, z)] recent centroids (Fig.1 yellow track)


class MotionEstimationNode(Node):

    def __init__(self):
        super().__init__('motion_estimation_node')

        self.declare_parameter('tracks_topic', '/manriix/cluster_tracks_fused')
        self.declare_parameter('output_tracks_topic', '/manriix/objects')
        self.declare_parameter('output_markers_topic', '/manriix/motion_markers')

        self.declare_parameter('q_pos', 0.05)    # process noise, position
        self.declare_parameter('q_vel', 0.5)      # process noise, velocity
        self.declare_parameter('r_meas', 0.02)    # measurement noise (centroid jitter)

        self.declare_parameter('lost_keepalive', 2.0)      # [s] keep predicting a lost object's KF
        self.declare_parameter('reassoc_mahalanobis', 3.0)  # gate: sqrt of Mahalanobis^2 threshold
        self.declare_parameter('min_speed_arrow', 0.15)     # [m/s] draw arrow only above this
        self.declare_parameter('trail_length', 60)          # max trail points (Fig.1 yellow track)

        self.q_pos = float(self.get_parameter('q_pos').value)
        self.q_vel = float(self.get_parameter('q_vel').value)
        self.r_meas = float(self.get_parameter('r_meas').value)
        self.lost_keepalive = float(self.get_parameter('lost_keepalive').value)
        self.reassoc_mahalanobis = float(self.get_parameter('reassoc_mahalanobis').value)
        self.min_speed_arrow = float(self.get_parameter('min_speed_arrow').value)
        self.trail_length = int(self.get_parameter('trail_length').value)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.create_subscription(
            ClusterTrackArray,
            self.get_parameter('tracks_topic').value,
            self._cb, qos)
        self.pub_tracks = self.create_publisher(
            ClusterTrackArray,
            self.get_parameter('output_tracks_topic').value, qos)
        self.pub_markers = self.create_publisher(
            MarkerArray,
            self.get_parameter('output_markers_topic').value, 10)

        self.objects = {}        # pipeline track_id -> TrackedObject (live)
        self.lost_objects = []   # [TrackedObject] still predicting
        self.last_stamp = None
        self._prev_object_ids = set()

        self.get_logger().info(
            f'KF q_pos={self.q_pos} q_vel={self.q_vel} r={self.r_meas} | '
            f'lost_keepalive={self.lost_keepalive}s '
            f'reassoc_gate={self.reassoc_mahalanobis}')

    # ------------------------------------------------------------------
    def _cb(self, msg: ClusterTrackArray):
        now = Time.from_msg(msg.header.stamp).nanoseconds * 1e-9
        ts = 0.1 if self.last_stamp is None else max(1e-3, now - self.last_stamp)
        self.last_stamp = now

        # ---- predict every filter (live + lost) forward by ts ----
        for obj in self.objects.values():
            obj.kf.predict(ts)
        for obj in self.lost_objects:
            obj.kf.predict(ts)

        dynamic_now = {
            trk.track_id: trk for trk in msg.tracks
            if trk.classification in DYNAMIC_LABELS
        }

        # ---- update / spawn ----
        seen_ids = set()
        for tid, trk in dynamic_now.items():
            cx, cy = trk.centroid.x, trk.centroid.y
            seen_ids.add(tid)

            if tid in self.objects:
                obj = self.objects[tid]
                obj.kf.update(cx, cy)
                obj.last_update = now
                obj.trail.append((cx, cy, trk.centroid.z))
                if len(obj.trail) > self.trail_length:
                    obj.trail.pop(0)
                continue

            # New dynamic pipeline track: first try to RECONNECT it to a
            # lost object (paper: p(j=i) = N(c_j | c_hat_i, C_i))
            reconnected = self._try_reassociate(cx, cy, tid, now)
            if reconnected is None:
                kf = KalmanCV2D(cx, cy, self.q_pos, self.q_vel, self.r_meas)
                self.objects[tid] = TrackedObject(tid, kf, now)

        # ---- move disappeared objects to the lost list ----
        for tid in [t for t in self.objects if t not in seen_ids]:
            obj = self.objects.pop(tid)
            obj.lost_since = now
            self.lost_objects.append(obj)

        # ---- expire stale lost objects ----
        self.lost_objects = [
            o for o in self.lost_objects
            if (now - o.lost_since) <= self.lost_keepalive
        ]

        # ---- publish ----
        self._publish(msg, now)

    # ------------------------------------------------------------------
    def _try_reassociate(self, cx, cy, new_tid, now):
        """Gated max-likelihood match of a new detection against lost
        objects' predicted positions. Returns the reconnected object or
        None."""
        best, best_m2 = None, self.reassoc_mahalanobis ** 2
        z = np.array([cx, cy])
        for obj in self.lost_objects:
            mu = obj.kf.position()
            C = obj.kf.pos_cov()
            try:
                Cinv = np.linalg.inv(C)
            except np.linalg.LinAlgError:
                continue
            d = z - mu
            m2 = float(d @ Cinv @ d)
            if m2 < best_m2:
                best, best_m2 = obj, m2

        if best is None:
            return None

        # Reconnect: the new pipeline track inherits the lost object's
        # identity and filter (paper: "connect the cluster tracks of
        # object i and j").
        self.lost_objects.remove(best)
        best.kf.update(cx, cy)
        best.trail.append((cx, cy, 0.0))
        if len(best.trail) > self.trail_length:
            best.trail.pop(0)
        best.track_id = new_tid
        best.last_update = now
        best.lost_since = None
        self.objects[new_tid] = best
        self.get_logger().info(
            f'Re-associated new track {new_tid} to lost object '
            f'{best.object_id} (m2={best_m2:.2f})')
        return best

    # ------------------------------------------------------------------
    def _publish(self, msg: ClusterTrackArray, now):
        out = ClusterTrackArray()
        out.header = msg.header

        marker_array = MarkerArray()
        current_ids = set()

        for trk in msg.tracks:
            obj = self.objects.get(trk.track_id)
            if obj is not None and trk.classification in DYNAMIC_LABELS:
                vx, vy = obj.kf.velocity()
                C = obj.kf.pos_cov()
                trk.velocity = Vector3(x=float(vx), y=float(vy), z=0.0)
                trk.velocity_cov_xx = float(obj.kf.P[2, 2])
                trk.velocity_cov_yy = float(obj.kf.P[3, 3])
                # expose persistent identity downstream
                trk.track_id = obj.object_id
            out.tracks.append(trk)

            if obj is not None and trk.classification in DYNAMIC_LABELS:
                # Marker id = the object's own persistent object_id, NOT a
                # loop counter -- same fix as fusion_node's flicker bug:
                # a counter-based id shifts for every track whenever ANY
                # other track appears/disappears elsewhere in the scene.
                oid = obj.object_id
                current_ids.add(oid)

                speed = float(np.hypot(trk.velocity.x, trk.velocity.y))
                if speed >= self.min_speed_arrow:
                    marker_array.markers.append(
                        self._make_arrow(msg.header, trk, oid))
                if len(obj.trail) >= 2:
                    marker_array.markers.append(
                        self._make_trail(msg.header, obj, oid))
                # NEW: visible persistent-ID label -- this is what makes
                # Sec III-G occlusion re-association OBSERVABLE (previously
                # only visible via the "Re-associated..." log line). The
                # id shown here is the object_id that survives occlusion,
                # unlike the box_id / cluster track_id shown in earlier
                # pipeline stages (people_tracker_node, fusion_node),
                # which intentionally reset on any gap longer than their
                # own short timeouts.
                marker_array.markers.append(
                    self._make_id_label(msg.header, trk, oid))

        dropped = self._prev_object_ids - current_ids
        for oid in dropped:
            for ns in ('velocity', 'trajectory', 'object_id'):
                m = Marker()
                m.header = out.header
                m.ns = ns
                m.id = oid
                m.action = Marker.DELETE
                marker_array.markers.append(m)
        self._prev_object_ids = current_ids

        self.pub_tracks.publish(out)
        self.pub_markers.publish(marker_array)

    # ------------------------------------------------------------------
    def _make_arrow(self, header, trk, marker_id) -> Marker:
        m = Marker()
        m.header = header
        m.ns = 'velocity'
        m.id = marker_id
        m.type = Marker.ARROW
        m.action = Marker.ADD
        start = Point(x=trk.centroid.x, y=trk.centroid.y, z=trk.centroid.z)
        # arrow length = 1s of travel at current velocity
        end = Point(x=trk.centroid.x + trk.velocity.x,
                    y=trk.centroid.y + trk.velocity.y,
                    z=trk.centroid.z)
        m.points = [start, end]
        m.scale.x = 0.05   # shaft diameter
        m.scale.y = 0.12   # head diameter
        m.scale.z = 0.12   # head length
        m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.3, 0.0, 0.9
        m.lifetime = rclpy.duration.Duration(seconds=0.3).to_msg()
        return m


    def _make_trail(self, header, obj, marker_id) -> Marker:
        """Fig.1 yellow past-trajectory track."""
        m = Marker()
        m.header = header
        m.ns = 'trajectory'
        m.id = marker_id
        m.type = Marker.LINE_STRIP
        m.action = Marker.ADD
        m.pose.orientation.w = 1.0
        m.scale.x = 0.04
        m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.9, 0.1, 0.9
        m.points = [Point(x=float(x), y=float(y), z=0.05)
                    for (x, y, _z) in obj.trail]
        m.lifetime = rclpy.duration.Duration(seconds=0.4).to_msg()
        return m

    def _make_id_label(self, header, trk, marker_id) -> Marker:
        """Persistent object_id, visible so occlusion re-association
        (Sec III-G) can be confirmed visually, not just via log lines."""
        m = Marker()
        m.header = header
        m.ns = 'object_id'
        m.id = marker_id
        m.type = Marker.TEXT_VIEW_FACING
        m.action = Marker.ADD
        m.pose.position.x = float(trk.centroid.x)
        m.pose.position.y = float(trk.centroid.y)
        m.pose.position.z = float(trk.centroid.z) + 0.35
        m.pose.orientation.w = 1.0
        m.scale.z = 0.16
        m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.6, 0.0, 1.0
        m.text = f'obj:{marker_id}'
        m.lifetime = rclpy.duration.Duration(seconds=0.3).to_msg()
        return m


def main(args=None):
    rclpy.init(args=args)
    node = MotionEstimationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()