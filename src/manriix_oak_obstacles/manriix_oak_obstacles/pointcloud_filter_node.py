#!/usr/bin/env python3
"""
pointcloud_filter_node.py

Implements Section III-A (consumed, not re-implemented -- depthai-ros's
/oak/points already gives us the stereo-derived 3D point cloud) and
Section III-B (Point Cloud Filtering) of:

  Eppenberger et al., "Leveraging Stereo-Camera Data for Real-Time
  Dynamic Obstacle Detection and Tracking", IROS 2020.
  https://arxiv.org/abs/2007.10743

Paper's filtering sequence (Sec III-B), applied here:
  1. crop the point cloud at the depth limit l_d (trusted depth range)
  2. crop at heights l_g / l_h to remove ground plane / ceiling points
  3. voxel filter with leaf size l_l (down-sample, even 3D density)
  4. radius outlier filter: remove points with < l_n neighbors within l_r

Because steps 2-4 require a "height" that means something (z-up, robot
frame), we first transform the incoming optical-frame cloud into
`target_frame` (default: base_footprint) using tf2, BEFORE height
cropping. The depth crop (step 1) is applied in the *camera* frame
first, since it reflects a sensor-measurement trust limit, not a
world-space quantity.

Publishes two clouds, matching the paper's notation:
  - <output_dense_topic>:    h^d  (cropped + transformed, NOT voxel/
                                    outlier filtered -- kept dense on
                                    purpose, needed later for occlusion
                                    reasoning / voting in Section III-D)
  - <output_filtered_topic>: h^s  (fully filtered -- used for
                                    clustering in Section III-C)
"""

import numpy as np
import open3d as o3d

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2

import tf2_ros
from tf2_ros import TransformException


def quat_to_rot_matrix(x, y, z, w):
    """Quaternion -> 3x3 rotation matrix (no external dep needed)."""
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


class PointCloudFilterNode(Node):

    def __init__(self):
        super().__init__('pointcloud_filter_node')

        # ---- Parameters (names mirror the paper's symbols, Sec IV-A) ----
        self.declare_parameter('input_topic', '/oak/points')
        self.declare_parameter('output_dense_topic', '/manriix/points_dense')
        self.declare_parameter('output_filtered_topic', '/manriix/points_filtered')
        self.declare_parameter('target_frame', 'base_footprint')

        self.declare_parameter('l_d', 5.0)     # depth trust limit [m], camera frame
        self.declare_parameter('l_g', 0.15)    # ground crop height [m] AT RANGE ~0 (near-field base margin)
        self.declare_parameter('l_h', 1.8)     # ceiling crop height [m], target frame
        self.declare_parameter('l_l', 0.05)    # voxel leaf size [m]
        self.declare_parameter('l_n', 30)      # min neighbors for radius filter
        self.declare_parameter('l_r', 0.5)     # radius for outlier filter [m]

        # --- Range-adaptive ground margin ---
        # Stereo depth error grows ~quadratically with range:
        #   sigma_z(z) ~= z^2 * sigma_disparity / (focal_px * baseline_m)
        # A flat l_g works near the camera but gets pierced by this noise
        # at distance (real ground points get measured ABOVE l_g). We
        # inflate the ground threshold with range to compensate:
        #   l_g_eff(z) = l_g + ground_margin_k * sigma_z(z)
        self.declare_parameter('focal_px', 772.0)          # from /oak/rgb/camera_info K[0]
        self.declare_parameter('baseline_m', 0.075)         # OAK-D Pro W nominal baseline
        self.declare_parameter('disparity_sigma_px', 0.5)   # sub-pixel disparity noise, typical
        self.declare_parameter('ground_margin_k', 3.0)      # safety multiplier (~3-sigma)
        # Hard cap on points entering the filter chain. The full-res
        # OAK depth cloud can be ~900k points; Open3D's OpenMP kernels
        # will happily eat every core processing them. Random
        # subsampling to this budget bounds CPU regardless of camera
        # resolution, with negligible effect downstream (we voxel to
        # 5cm anyway). The paper likewise downsampled inputs (Sec IV-B).
        self.declare_parameter('max_input_points', 60000)

        self.input_topic = self.get_parameter('input_topic').value
        self.output_dense_topic = self.get_parameter('output_dense_topic').value
        self.output_filtered_topic = self.get_parameter('output_filtered_topic').value
        self.target_frame = self.get_parameter('target_frame').value

        self.l_d = float(self.get_parameter('l_d').value)
        self.l_g = float(self.get_parameter('l_g').value)
        self.l_h = float(self.get_parameter('l_h').value)
        self.l_l = float(self.get_parameter('l_l').value)
        self.l_n = int(self.get_parameter('l_n').value)
        self.l_r = float(self.get_parameter('l_r').value)

        self.focal_px = float(self.get_parameter('focal_px').value)
        self.baseline_m = float(self.get_parameter('baseline_m').value)
        self.disparity_sigma_px = float(self.get_parameter('disparity_sigma_px').value)
        self.ground_margin_k = float(self.get_parameter('ground_margin_k').value)
        self.max_input_points = int(self.get_parameter('max_input_points').value)

        self.get_logger().info(
            f'Params: l_d={self.l_d} l_g={self.l_g} l_h={self.l_h} '
            f'l_l={self.l_l} l_n={self.l_n} l_r={self.l_r} '
            f'target_frame={self.target_frame} | '
            f'ground_margin: focal={self.focal_px} baseline={self.baseline_m} '
            f'sigma_d={self.disparity_sigma_px} k={self.ground_margin_k}'
        )

        # ---- TF ----
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ---- I/O ----
        # /oak/points is published BEST_EFFORT / KEEP_LAST(5) -> match it.
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.sub = self.create_subscription(
            PointCloud2, self.input_topic, self.cloud_callback, qos)

        self.pub_dense = self.create_publisher(
            PointCloud2, self.output_dense_topic, qos)
        self.pub_filtered = self.create_publisher(
            PointCloud2, self.output_filtered_topic, qos)

        # Cache the rigid transform matrix; re-looked-up periodically in
        # case TF changes (cheap for static transforms, robust if you
        # later switch to a moving base_link -> camera chain).
        self._R = None
        self._t = None
        self._source_frame_cached = None

        self.get_logger().info(
            f'Subscribed to {self.input_topic}, publishing '
            f'{self.output_dense_topic} (h^d) and '
            f'{self.output_filtered_topic} (h^s)'
        )

    # ------------------------------------------------------------------
    def _update_transform(self, source_frame: str) -> bool:
        try:
            tf = self.tf_buffer.lookup_transform(
                self.target_frame, source_frame, rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().warn(
                f'TF {source_frame} -> {self.target_frame} not available: {ex}',
                throttle_duration_sec=2.0,
            )
            return False

        q = tf.transform.rotation
        t = tf.transform.translation
        self._R = quat_to_rot_matrix(q.x, q.y, q.z, q.w)
        self._t = np.array([t.x, t.y, t.z])
        self._source_frame_cached = source_frame
        return True

    # ------------------------------------------------------------------
    def cloud_callback(self, msg: PointCloud2):
        source_frame = msg.header.frame_id

        if self._R is None or self._source_frame_cached != source_frame:
            if not self._update_transform(source_frame):
                return

        # --- Read points (camera optical frame: x-right, y-down, z-fwd) ---
        pts = pc2.read_points_numpy(
            msg, field_names=('x', 'y', 'z'), skip_nans=True)
        if pts.size == 0:
            return
        pts = pts.astype(np.float64)

        # Bound worst-case CPU: random-subsample huge input clouds.
        if pts.shape[0] > self.max_input_points:
            sel = np.random.choice(pts.shape[0], self.max_input_points, replace=False)
            pts = pts[sel]

        # --- Step 1: crop by depth limit l_d (camera-frame z = depth) ---
        z_cam = pts[:, 2]
        keep = (z_cam > 0.0) & (z_cam <= self.l_d)
        pts = pts[keep]
        z_cam = z_cam[keep]
        if pts.shape[0] == 0:
            return

        # --- Transform into target_frame (z-up) ---
        pts_world = pts @ self._R.T + self._t

        # --- Step 2: crop by height l_g (ground, RANGE-ADAPTIVE) / l_h (ceiling) ---
        # sigma_z(z) = z^2 * sigma_disparity / (focal * baseline)
        sigma_z = (z_cam ** 2) * self.disparity_sigma_px / (self.focal_px * self.baseline_m)
        l_g_eff = self.l_g + self.ground_margin_k * sigma_z

        z_world = pts_world[:, 2]
        keep = (z_world >= l_g_eff) & (z_world <= self.l_h)
        pts_dense = pts_world[keep]
        if pts_dense.shape[0] == 0:
            return

        # Publish h^d: dense, cropped, transformed -- NOT downsampled.
        header = msg.header
        header.frame_id = self.target_frame
        cloud_dense_msg = pc2.create_cloud_xyz32(header, pts_dense)
        self.pub_dense.publish(cloud_dense_msg)

        # --- Step 3 & 4: voxel downsample + radius outlier removal ---
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(pts_dense)

        pcd = pcd.voxel_down_sample(voxel_size=self.l_l)

        if len(pcd.points) > 0 and self.l_n > 0:
            pcd, _ = pcd.remove_radius_outlier(
                nb_points=self.l_n, radius=self.l_r)

        pts_filtered = np.asarray(pcd.points)

        cloud_filtered_msg = pc2.create_cloud_xyz32(header, pts_filtered)
        self.pub_filtered.publish(cloud_filtered_msg)


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudFilterNode()
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
# pointcloud_filter_node.py

# Implements Section III-A (consumed, not re-implemented -- depthai-ros's
# /oak/points already gives us the stereo-derived 3D point cloud) and
# Section III-B (Point Cloud Filtering) of:

#   Eppenberger et al., "Leveraging Stereo-Camera Data for Real-Time
#   Dynamic Obstacle Detection and Tracking", IROS 2020.
#   https://arxiv.org/abs/2007.10743

# Paper's filtering sequence (Sec III-B), applied here:
#   1. crop the point cloud at the depth limit l_d (trusted depth range)
#   2. crop at heights l_g / l_h to remove ground plane / ceiling points
#   3. voxel filter with leaf size l_l (down-sample, even 3D density)
#   4. radius outlier filter: remove points with < l_n neighbors within l_r

# Because steps 2-4 require a "height" that means something (z-up, robot
# frame), we first transform the incoming optical-frame cloud into
# `target_frame` (default: base_footprint) using tf2, BEFORE height
# cropping. The depth crop (step 1) is applied in the *camera* frame
# first, since it reflects a sensor-measurement trust limit, not a
# world-space quantity.

# Publishes two clouds, matching the paper's notation:
#   - <output_dense_topic>:    h^d  (cropped + transformed, NOT voxel/
#                                     outlier filtered -- kept dense on
#                                     purpose, needed later for occlusion
#                                     reasoning / voting in Section III-D)
#   - <output_filtered_topic>: h^s  (fully filtered -- used for
#                                     clustering in Section III-C)
# """

# import numpy as np
# import open3d as o3d

# import rclpy
# from rclpy.node import Node
# from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

# from sensor_msgs.msg import PointCloud2
# from sensor_msgs_py import point_cloud2 as pc2

# import tf2_ros
# from tf2_ros import TransformException


# def quat_to_rot_matrix(x, y, z, w):
#     """Quaternion -> 3x3 rotation matrix (no external dep needed)."""
#     n = x * x + y * y + z * z + w * w
#     if n < 1e-12:
#         return np.eye(3)
#     s = 2.0 / n
#     xx, yy, zz = x * x * s, y * y * s, z * z * s
#     xy, xz, yz = x * y * s, x * z * s, y * z * s
#     wx, wy, wz = w * x * s, w * y * s, w * z * s
#     return np.array([
#         [1.0 - (yy + zz), xy - wz, xz + wy],
#         [xy + wz, 1.0 - (xx + zz), yz - wx],
#         [xz - wy, yz + wx, 1.0 - (xx + yy)],
#     ])


# class PointCloudFilterNode(Node):

#     def __init__(self):
#         super().__init__('pointcloud_filter_node')

#         # ---- Parameters (names mirror the paper's symbols, Sec IV-A) ----
#         self.declare_parameter('input_topic', '/oak/points')
#         self.declare_parameter('output_dense_topic', '/manriix/points_dense')
#         self.declare_parameter('output_filtered_topic', '/manriix/points_filtered')
#         self.declare_parameter('target_frame', 'base_footprint')

#         self.declare_parameter('l_d', 5.0)     # depth trust limit [m], camera frame
#         self.declare_parameter('l_g', 0.15)    # ground crop height [m] AT RANGE ~0 (near-field base margin)
#         self.declare_parameter('l_h', 1.8)     # ceiling crop height [m], target frame
#         self.declare_parameter('l_l', 0.05)    # voxel leaf size [m]
#         self.declare_parameter('l_n', 30)      # min neighbors for radius filter
#         self.declare_parameter('l_r', 0.5)     # radius for outlier filter [m]

#         # --- Range-adaptive ground margin ---
#         # Stereo depth error grows ~quadratically with range:
#         #   sigma_z(z) ~= z^2 * sigma_disparity / (focal_px * baseline_m)
#         # A flat l_g works near the camera but gets pierced by this noise
#         # at distance (real ground points get measured ABOVE l_g). We
#         # inflate the ground threshold with range to compensate:
#         #   l_g_eff(z) = l_g + ground_margin_k * sigma_z(z)
#         self.declare_parameter('focal_px', 772.0)          # from /oak/rgb/camera_info K[0]
#         self.declare_parameter('baseline_m', 0.075)         # OAK-D Pro W nominal baseline
#         self.declare_parameter('disparity_sigma_px', 0.5)   # sub-pixel disparity noise, typical
#         self.declare_parameter('ground_margin_k', 3.0)      # safety multiplier (~3-sigma)

#         self.input_topic = self.get_parameter('input_topic').value
#         self.output_dense_topic = self.get_parameter('output_dense_topic').value
#         self.output_filtered_topic = self.get_parameter('output_filtered_topic').value
#         self.target_frame = self.get_parameter('target_frame').value

#         self.l_d = float(self.get_parameter('l_d').value)
#         self.l_g = float(self.get_parameter('l_g').value)
#         self.l_h = float(self.get_parameter('l_h').value)
#         self.l_l = float(self.get_parameter('l_l').value)
#         self.l_n = int(self.get_parameter('l_n').value)
#         self.l_r = float(self.get_parameter('l_r').value)

#         self.focal_px = float(self.get_parameter('focal_px').value)
#         self.baseline_m = float(self.get_parameter('baseline_m').value)
#         self.disparity_sigma_px = float(self.get_parameter('disparity_sigma_px').value)
#         self.ground_margin_k = float(self.get_parameter('ground_margin_k').value)

#         self.get_logger().info(
#             f'Params: l_d={self.l_d} l_g={self.l_g} l_h={self.l_h} '
#             f'l_l={self.l_l} l_n={self.l_n} l_r={self.l_r} '
#             f'target_frame={self.target_frame} | '
#             f'ground_margin: focal={self.focal_px} baseline={self.baseline_m} '
#             f'sigma_d={self.disparity_sigma_px} k={self.ground_margin_k}'
#         )

#         # ---- TF ----
#         self.tf_buffer = tf2_ros.Buffer()
#         self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

#         # ---- I/O ----
#         # /oak/points is published BEST_EFFORT / KEEP_LAST(5) -> match it.
#         qos = QoSProfile(
#             reliability=ReliabilityPolicy.BEST_EFFORT,
#             history=HistoryPolicy.KEEP_LAST,
#             depth=5,
#             durability=DurabilityPolicy.VOLATILE,
#         )

#         self.sub = self.create_subscription(
#             PointCloud2, self.input_topic, self.cloud_callback, qos)

#         self.pub_dense = self.create_publisher(
#             PointCloud2, self.output_dense_topic, qos)
#         self.pub_filtered = self.create_publisher(
#             PointCloud2, self.output_filtered_topic, qos)

#         # Cache the rigid transform matrix; re-looked-up periodically in
#         # case TF changes (cheap for static transforms, robust if you
#         # later switch to a moving base_link -> camera chain).
#         self._R = None
#         self._t = None
#         self._source_frame_cached = None

#         self.get_logger().info(
#             f'Subscribed to {self.input_topic}, publishing '
#             f'{self.output_dense_topic} (h^d) and '
#             f'{self.output_filtered_topic} (h^s)'
#         )

#     # ------------------------------------------------------------------
#     def _update_transform(self, source_frame: str) -> bool:
#         try:
#             tf = self.tf_buffer.lookup_transform(
#                 self.target_frame, source_frame, rclpy.time.Time())
#         except TransformException as ex:
#             self.get_logger().warn(
#                 f'TF {source_frame} -> {self.target_frame} not available: {ex}',
#                 throttle_duration_sec=2.0,
#             )
#             return False

#         q = tf.transform.rotation
#         t = tf.transform.translation
#         self._R = quat_to_rot_matrix(q.x, q.y, q.z, q.w)
#         self._t = np.array([t.x, t.y, t.z])
#         self._source_frame_cached = source_frame
#         return True

#     # ------------------------------------------------------------------
#     def cloud_callback(self, msg: PointCloud2):
#         source_frame = msg.header.frame_id

#         if self._R is None or self._source_frame_cached != source_frame:
#             if not self._update_transform(source_frame):
#                 return

#         # --- Read points (camera optical frame: x-right, y-down, z-fwd) ---
#         pts = pc2.read_points_numpy(
#             msg, field_names=('x', 'y', 'z'), skip_nans=True)
#         if pts.size == 0:
#             return
#         pts = pts.astype(np.float64)

#         # --- Step 1: crop by depth limit l_d (camera-frame z = depth) ---
#         z_cam = pts[:, 2]
#         keep = (z_cam > 0.0) & (z_cam <= self.l_d)
#         pts = pts[keep]
#         z_cam = z_cam[keep]
#         if pts.shape[0] == 0:
#             return

#         # --- Transform into target_frame (z-up) ---
#         pts_world = pts @ self._R.T + self._t

#         # --- Step 2: crop by height l_g (ground, RANGE-ADAPTIVE) / l_h (ceiling) ---
#         # sigma_z(z) = z^2 * sigma_disparity / (focal * baseline)
#         sigma_z = (z_cam ** 2) * self.disparity_sigma_px / (self.focal_px * self.baseline_m)
#         l_g_eff = self.l_g + self.ground_margin_k * sigma_z

#         z_world = pts_world[:, 2]
#         keep = (z_world >= l_g_eff) & (z_world <= self.l_h)
#         pts_dense = pts_world[keep]
#         if pts_dense.shape[0] == 0:
#             return

#         # Publish h^d: dense, cropped, transformed -- NOT downsampled.
#         header = msg.header
#         header.frame_id = self.target_frame
#         cloud_dense_msg = pc2.create_cloud_xyz32(header, pts_dense)
#         self.pub_dense.publish(cloud_dense_msg)

#         # --- Step 3 & 4: voxel downsample + radius outlier removal ---
#         pcd = o3d.geometry.PointCloud()
#         pcd.points = o3d.utility.Vector3dVector(pts_dense)

#         pcd = pcd.voxel_down_sample(voxel_size=self.l_l)

#         if len(pcd.points) > 0 and self.l_n > 0:
#             pcd, _ = pcd.remove_radius_outlier(
#                 nb_points=self.l_n, radius=self.l_r)

#         pts_filtered = np.asarray(pcd.points)

#         cloud_filtered_msg = pc2.create_cloud_xyz32(header, pts_filtered)
#         self.pub_filtered.publish(cloud_filtered_msg)


# def main(args=None):
#     rclpy.init(args=args)
#     node = PointCloudFilterNode()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()


# if __name__ == '__main__':
#     main()




# #!/usr/bin/env python3
# """
# pointcloud_filter_node.py

# Implements Section III-A (consumed, not re-implemented -- depthai-ros's
# /oak/points already gives us the stereo-derived 3D point cloud) and
# Section III-B (Point Cloud Filtering) of:

#   Eppenberger et al., "Leveraging Stereo-Camera Data for Real-Time
#   Dynamic Obstacle Detection and Tracking", IROS 2020.
#   https://arxiv.org/abs/2007.10743

# Paper's filtering sequence (Sec III-B), applied here:
#   1. crop the point cloud at the depth limit l_d (trusted depth range)
#   2. crop at heights l_g / l_h to remove ground plane / ceiling points
#   3. voxel filter with leaf size l_l (down-sample, even 3D density)
#   4. radius outlier filter: remove points with < l_n neighbors within l_r

# Because steps 2-4 require a "height" that means something (z-up, robot
# frame), we first transform the incoming optical-frame cloud into
# `target_frame` (default: base_footprint) using tf2, BEFORE height
# cropping. The depth crop (step 1) is applied in the *camera* frame
# first, since it reflects a sensor-measurement trust limit, not a
# world-space quantity.

# Publishes two clouds, matching the paper's notation:
#   - <output_dense_topic>:    h^d  (cropped + transformed, NOT voxel/
#                                     outlier filtered -- kept dense on
#                                     purpose, needed later for occlusion
#                                     reasoning / voting in Section III-D)
#   - <output_filtered_topic>: h^s  (fully filtered -- used for
#                                     clustering in Section III-C)
# """

# import numpy as np
# import open3d as o3d

# import rclpy
# from rclpy.node import Node
# from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

# from sensor_msgs.msg import PointCloud2
# from sensor_msgs_py import point_cloud2 as pc2

# import tf2_ros
# from tf2_ros import TransformException


# def quat_to_rot_matrix(x, y, z, w):
#     """Quaternion -> 3x3 rotation matrix (no external dep needed)."""
#     n = x * x + y * y + z * z + w * w
#     if n < 1e-12:
#         return np.eye(3)
#     s = 2.0 / n
#     xx, yy, zz = x * x * s, y * y * s, z * z * s
#     xy, xz, yz = x * y * s, x * z * s, y * z * s
#     wx, wy, wz = w * x * s, w * y * s, w * z * s
#     return np.array([
#         [1.0 - (yy + zz), xy - wz, xz + wy],
#         [xy + wz, 1.0 - (xx + zz), yz - wx],
#         [xz - wy, yz + wx, 1.0 - (xx + yy)],
#     ])


# class PointCloudFilterNode(Node):

#     def __init__(self):
#         super().__init__('pointcloud_filter_node')

#         # ---- Parameters (names mirror the paper's symbols, Sec IV-A) ----
#         self.declare_parameter('input_topic', '/oak/points')
#         self.declare_parameter('output_dense_topic', '/manriix/points_dense')
#         self.declare_parameter('output_filtered_topic', '/manriix/points_filtered')
#         self.declare_parameter('target_frame', 'base_footprint')

#         self.declare_parameter('l_d', 5.0)     # depth trust limit [m], camera frame
#         self.declare_parameter('l_g', 0.15)    # ground crop height [m], target frame
#         self.declare_parameter('l_h', 1.8)     # ceiling crop height [m], target frame
#         self.declare_parameter('l_l', 0.05)    # voxel leaf size [m]
#         self.declare_parameter('l_n', 30)      # min neighbors for radius filter
#         self.declare_parameter('l_r', 0.5)     # radius for outlier filter [m]

#         self.input_topic = self.get_parameter('input_topic').value
#         self.output_dense_topic = self.get_parameter('output_dense_topic').value
#         self.output_filtered_topic = self.get_parameter('output_filtered_topic').value
#         self.target_frame = self.get_parameter('target_frame').value

#         self.l_d = float(self.get_parameter('l_d').value)
#         self.l_g = float(self.get_parameter('l_g').value)
#         self.l_h = float(self.get_parameter('l_h').value)
#         self.l_l = float(self.get_parameter('l_l').value)
#         self.l_n = int(self.get_parameter('l_n').value)
#         self.l_r = float(self.get_parameter('l_r').value)

#         self.get_logger().info(
#             f'Params: l_d={self.l_d} l_g={self.l_g} l_h={self.l_h} '
#             f'l_l={self.l_l} l_n={self.l_n} l_r={self.l_r} '
#             f'target_frame={self.target_frame}'
#         )

#         # ---- TF ----
#         self.tf_buffer = tf2_ros.Buffer()
#         self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

#         # ---- I/O ----
#         # /oak/points is published BEST_EFFORT / KEEP_LAST(5) -> match it.
#         qos = QoSProfile(
#             reliability=ReliabilityPolicy.BEST_EFFORT,
#             history=HistoryPolicy.KEEP_LAST,
#             depth=5,
#             durability=DurabilityPolicy.VOLATILE,
#         )

#         self.sub = self.create_subscription(
#             PointCloud2, self.input_topic, self.cloud_callback, qos)

#         self.pub_dense = self.create_publisher(
#             PointCloud2, self.output_dense_topic, qos)
#         self.pub_filtered = self.create_publisher(
#             PointCloud2, self.output_filtered_topic, qos)

#         # Cache the rigid transform matrix; re-looked-up periodically in
#         # case TF changes (cheap for static transforms, robust if you
#         # later switch to a moving base_link -> camera chain).
#         self._R = None
#         self._t = None
#         self._source_frame_cached = None

#         self.get_logger().info(
#             f'Subscribed to {self.input_topic}, publishing '
#             f'{self.output_dense_topic} (h^d) and '
#             f'{self.output_filtered_topic} (h^s)'
#         )

#     # ------------------------------------------------------------------
#     def _update_transform(self, source_frame: str) -> bool:
#         try:
#             tf = self.tf_buffer.lookup_transform(
#                 self.target_frame, source_frame, rclpy.time.Time())
#         except TransformException as ex:
#             self.get_logger().warn(
#                 f'TF {source_frame} -> {self.target_frame} not available: {ex}',
#                 throttle_duration_sec=2.0,
#             )
#             return False

#         q = tf.transform.rotation
#         t = tf.transform.translation
#         self._R = quat_to_rot_matrix(q.x, q.y, q.z, q.w)
#         self._t = np.array([t.x, t.y, t.z])
#         self._source_frame_cached = source_frame
#         return True

#     # ------------------------------------------------------------------
#     def cloud_callback(self, msg: PointCloud2):
#         source_frame = msg.header.frame_id

#         if self._R is None or self._source_frame_cached != source_frame:
#             if not self._update_transform(source_frame):
#                 return

#         # --- Read points (camera optical frame: x-right, y-down, z-fwd) ---
#         pts = pc2.read_points_numpy(
#             msg, field_names=('x', 'y', 'z'), skip_nans=True)
#         if pts.size == 0:
#             return
#         pts = pts.astype(np.float64)

#         # --- Step 1: crop by depth limit l_d (camera-frame z = depth) ---
#         z_cam = pts[:, 2]
#         keep = (z_cam > 0.0) & (z_cam <= self.l_d)
#         pts = pts[keep]
#         if pts.shape[0] == 0:
#             return

#         # --- Transform into target_frame (z-up) ---
#         pts_world = pts @ self._R.T + self._t

#         # --- Step 2: crop by height l_g (ground) / l_h (ceiling) ---
#         z_world = pts_world[:, 2]
#         keep = (z_world >= self.l_g) & (z_world <= self.l_h)
#         pts_dense = pts_world[keep]
#         if pts_dense.shape[0] == 0:
#             return

#         # Publish h^d: dense, cropped, transformed -- NOT downsampled.
#         header = msg.header
#         header.frame_id = self.target_frame
#         cloud_dense_msg = pc2.create_cloud_xyz32(header, pts_dense)
#         self.pub_dense.publish(cloud_dense_msg)

#         # --- Step 3 & 4: voxel downsample + radius outlier removal ---
#         pcd = o3d.geometry.PointCloud()
#         pcd.points = o3d.utility.Vector3dVector(pts_dense)

#         pcd = pcd.voxel_down_sample(voxel_size=self.l_l)

#         if len(pcd.points) > 0 and self.l_n > 0:
#             pcd, _ = pcd.remove_radius_outlier(
#                 nb_points=self.l_n, radius=self.l_r)

#         pts_filtered = np.asarray(pcd.points)

#         cloud_filtered_msg = pc2.create_cloud_xyz32(header, pts_filtered)
#         self.pub_filtered.publish(cloud_filtered_msg)


# def main(args=None):
#     rclpy.init(args=args)
#     node = PointCloudFilterNode()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()


# if __name__ == '__main__':
#     main()