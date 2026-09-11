#!/usr/bin/env python3
"""
occupancy_grid_node.py

Implements Section III-H (2D Occupancy grid) of:
  Eppenberger et al., "Leveraging Stereo-Camera Data for Real-Time
  Dynamic Obstacle Detection and Tracking", IROS 2020.

Three maps are maintained in parallel, as in the paper:
  STATIC    -- persistent costs from clusters classified 'static'.
               The paper clears erroneously-occupied space by
               raytracing; we approximate that with observation decay:
               static cells not re-observed lose confidence and release.
  DYNAMIC   -- rebuilt every frame from 'dynamic'/'person' objects.
               Costs are EXPANDED IN THE DIRECTION OF THE ESTIMATED
               VELOCITY (the object's footprint is swept along v*t for
               t in [0, predict_horizon]) so a planner avoids where the
               object is GOING, not just where it is.
  UNCERTAIN -- short-living costs for clusters not yet classified.

Outputs:
  - /manriix/grid/static, /manriix/grid/dynamic, /manriix/grid/uncertain
        (nav_msgs/OccupancyGrid) individual layers
  - /manriix/grid/aggregated (nav_msgs/OccupancyGrid) cell-wise max,
        ready for RViz / planners that take a grid
  - /manriix/obstacle_cloud (sensor_msgs/PointCloud2) all occupied cells
        as points at z=0.15 -- plugs directly into a Nav2 costmap
        obstacle layer as an observation source (marking, no clearing;
        keep your lidar/depth as the clearing source).

The grid is robot-centered ("rolling"), fixed size, in the map frame.
"""

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2

import tf2_ros
from tf2_ros import TransformException

from manriix_oak_obstacles_msgs.msg import ClusterTrackArray


DYNAMIC_LABELS = {'dynamic', 'person'}


class OccupancyGridNode(Node):

    def __init__(self):
        super().__init__('occupancy_grid_node')

        self.declare_parameter('tracks_topic', '/manriix/objects')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_footprint')
        # Frame for /manriix/obstacle_cloud. STVL/ObstacleLayer measure
        # obstacle_range and frustum geometry from the CLOUD's frame
        # origin; publishing in 'map' would measure range from the map
        # origin and break as the robot drives away. base_footprint keeps
        # ranges sensor-relative. Set equal to map_frame to disable the
        # extra transform (dev/standalone behavior).
        self.declare_parameter('obstacle_cloud_frame', 'base_footprint')

        self.declare_parameter('grid_size', 8.0)        # [m] side length
        self.declare_parameter('resolution', 0.05)       # [m/cell]
        self.declare_parameter('inflate_cells', 2)       # simple obstacle inflation

        self.declare_parameter('predict_horizon', 1.0)   # [s] velocity sweep length
        self.declare_parameter('predict_steps', 5)        # sweep sub-steps
        self.declare_parameter('static_hit', 30)          # confidence added per observation
        self.declare_parameter('static_decay', 2)         # confidence lost per frame
        self.declare_parameter('static_max', 100)
        self.declare_parameter('static_occupied_thresh', 50)

        self.tracks_topic = self.get_parameter('tracks_topic').value
        self.map_frame = self.get_parameter('map_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.obstacle_cloud_frame = self.get_parameter('obstacle_cloud_frame').value
        self.grid_size = float(self.get_parameter('grid_size').value)
        self.resolution = float(self.get_parameter('resolution').value)
        self.inflate_cells = int(self.get_parameter('inflate_cells').value)
        self.predict_horizon = float(self.get_parameter('predict_horizon').value)
        self.predict_steps = int(self.get_parameter('predict_steps').value)
        self.static_hit = int(self.get_parameter('static_hit').value)
        self.static_decay = int(self.get_parameter('static_decay').value)
        self.static_max = int(self.get_parameter('static_max').value)
        self.static_occupied_thresh = int(self.get_parameter('static_occupied_thresh').value)

        self.n_cells = int(round(self.grid_size / self.resolution))

        # Persistent static confidence grid + its world origin
        self.static_conf = np.zeros((self.n_cells, self.n_cells), dtype=np.int16)
        self.origin_xy = None  # world coords of cell (0,0); shifts as robot moves

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(
            ClusterTrackArray, self.tracks_topic, self._cb, qos)

        self.pub_static = self.create_publisher(OccupancyGrid, '/manriix/grid/static', 5)
        self.pub_dynamic = self.create_publisher(OccupancyGrid, '/manriix/grid/dynamic', 5)
        self.pub_uncertain = self.create_publisher(OccupancyGrid, '/manriix/grid/uncertain', 5)
        self.pub_agg = self.create_publisher(OccupancyGrid, '/manriix/grid/aggregated', 5)
        self.pub_cloud = self.create_publisher(PointCloud2, '/manriix/obstacle_cloud', 5)

        self.get_logger().info(
            f'{self.n_cells}x{self.n_cells} cells @ {self.resolution}m, '
            f'predict_horizon={self.predict_horizon}s')

    # ------------------------------------------------------------------
    def _robot_xy(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, rclpy.time.Time())
            return np.array([tf.transform.translation.x,
                             tf.transform.translation.y])
        except TransformException:
            return np.zeros(2)

    # ------------------------------------------------------------------
    def _roll_grid(self, new_origin):
        """Shift the persistent static grid so data survives robot motion."""
        if self.origin_xy is None:
            self.origin_xy = new_origin
            return
        shift = np.round((new_origin - self.origin_xy) / self.resolution).astype(int)
        if shift[0] == 0 and shift[1] == 0:
            return
        rolled = np.zeros_like(self.static_conf)
        sx, sy = shift
        src_x = slice(max(0, sx), self.n_cells + min(0, sx))
        dst_x = slice(max(0, -sx), self.n_cells + min(0, -sx))
        src_y = slice(max(0, sy), self.n_cells + min(0, sy))
        dst_y = slice(max(0, -sy), self.n_cells + min(0, -sy))
        rolled[dst_y, dst_x] = self.static_conf[src_y, src_x]
        self.static_conf = rolled
        self.origin_xy = self.origin_xy + shift * self.resolution

    # ------------------------------------------------------------------
    def _world_to_cell(self, pts_xy):
        """(N,2) world -> (N,2) int cell indices [col,row]; mask in-bounds."""
        rel = (pts_xy - self.origin_xy) / self.resolution
        idx = np.floor(rel).astype(int)
        ok = ((idx[:, 0] >= 0) & (idx[:, 0] < self.n_cells) &
              (idx[:, 1] >= 0) & (idx[:, 1] < self.n_cells))
        return idx[ok]

    def _mark(self, grid, idx, value=100):
        if idx.shape[0] == 0:
            return
        grid[idx[:, 1], idx[:, 0]] = np.maximum(grid[idx[:, 1], idx[:, 0]], value)

    def _inflate(self, grid):
        if self.inflate_cells <= 0:
            return grid
        out = grid.copy()
        occ = np.argwhere(grid >= 100)
        r = self.inflate_cells
        for y, x in occ:
            y0, y1 = max(0, y - r), min(self.n_cells, y + r + 1)
            x0, x1 = max(0, x - r), min(self.n_cells, x + r + 1)
            out[y0:y1, x0:x1] = np.maximum(out[y0:y1, x0:x1], 90)
        return out

    # ------------------------------------------------------------------
    def _cb(self, msg: ClusterTrackArray):
        robot_xy = self._robot_xy()
        center = robot_xy - self.grid_size / 2.0
        self._roll_grid(center)

        dynamic_grid = np.zeros((self.n_cells, self.n_cells), dtype=np.int8)
        uncertain_grid = np.zeros((self.n_cells, self.n_cells), dtype=np.int8)
        static_observed = np.zeros((self.n_cells, self.n_cells), dtype=bool)

        for trk in msg.tracks:
            pts = pc2.read_points_numpy(
                trk.points, field_names=('x', 'y', 'z'), skip_nans=True)
            if pts.shape[0] == 0:
                continue
            xy = pts[:, :2]

            if trk.classification == 'static':
                idx = self._world_to_cell(xy)
                if idx.shape[0]:
                    static_observed[idx[:, 1], idx[:, 0]] = True

            elif trk.classification in DYNAMIC_LABELS:
                # Paper: "expand the costs in the direction of the
                # estimated velocity of the objects" -- sweep the
                # footprint along v*t.
                v = np.array([trk.velocity.x, trk.velocity.y])
                steps = max(1, self.predict_steps)
                for k in range(steps + 1):
                    t = self.predict_horizon * k / steps
                    swept = xy + v * t
                    idx = self._world_to_cell(swept)
                    # full cost now, tapering into the future
                    cost = int(100 - (30 * k / steps))
                    self._mark(dynamic_grid, idx, cost)

            else:  # 'uncertain' / 'unknown' -- short-living by design:
                   # this layer is rebuilt from scratch every frame.
                idx = self._world_to_cell(xy)
                self._mark(uncertain_grid, idx, 80)

        # ---- static layer update: hits + decay (raytrace substitute) ----
        self.static_conf = np.maximum(self.static_conf - self.static_decay, 0)
        self.static_conf[static_observed] = np.minimum(
            self.static_conf[static_observed] + self.static_hit + self.static_decay,
            self.static_max)
        static_grid = np.where(
            self.static_conf >= self.static_occupied_thresh, 100, 0).astype(np.int8)

        static_grid = self._inflate(static_grid)
        dynamic_grid = self._inflate(dynamic_grid)

        agg = np.maximum(np.maximum(static_grid, dynamic_grid), uncertain_grid)

        stamp = msg.header.stamp
        self.pub_static.publish(self._to_msg(static_grid, stamp))
        self.pub_dynamic.publish(self._to_msg(dynamic_grid, stamp))
        self.pub_uncertain.publish(self._to_msg(uncertain_grid, stamp))
        self.pub_agg.publish(self._to_msg(agg, stamp))
        self._publish_cloud(agg, stamp)

    # ------------------------------------------------------------------
    def _to_msg(self, grid, stamp) -> OccupancyGrid:
        m = OccupancyGrid()
        m.header.stamp = stamp
        m.header.frame_id = self.map_frame
        m.info.resolution = self.resolution
        m.info.width = self.n_cells
        m.info.height = self.n_cells
        m.info.origin.position.x = float(self.origin_xy[0])
        m.info.origin.position.y = float(self.origin_xy[1])
        m.info.origin.orientation.w = 1.0
        m.data = grid.astype(np.int8).flatten().tolist()
        return m

    def _publish_cloud(self, agg, stamp):
        occ = np.argwhere(agg >= 80)  # rows=y, cols=x
        if occ.shape[0] == 0:
            pts = np.zeros((0, 3))
        else:
            xs = self.origin_xy[0] + (occ[:, 1] + 0.5) * self.resolution
            ys = self.origin_xy[1] + (occ[:, 0] + 0.5) * self.resolution
            zs = np.full(occ.shape[0], 0.15)
            pts = np.stack([xs, ys, zs], axis=1)

        out_frame = self.obstacle_cloud_frame
        if out_frame != self.map_frame and pts.shape[0] > 0:
            # Full 2D pose transform (translation + yaw), not just
            # translation -- the robot rotates.
            try:
                tf = self.tf_buffer.lookup_transform(
                    out_frame, self.map_frame, rclpy.time.Time())
                q = tf.transform.rotation
                t = tf.transform.translation
                # yaw from quaternion (planar assumption)
                siny = 2.0 * (q.w * q.z + q.x * q.y)
                cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
                yaw = np.arctan2(siny, cosy)
                c, s = np.cos(yaw), np.sin(yaw)
                xy = pts[:, :2] @ np.array([[c, -s], [s, c]]).T \
                    + np.array([t.x, t.y])
                pts = np.column_stack([xy, pts[:, 2] + t.z])
            except TransformException as ex:
                self.get_logger().warn(
                    f'obstacle_cloud TF {self.map_frame}->{out_frame} '
                    f'failed, publishing in {self.map_frame}: {ex}',
                    throttle_duration_sec=2.0)
                out_frame = self.map_frame

        from std_msgs.msg import Header
        h = Header()
        h.stamp = stamp
        h.frame_id = out_frame
        self.pub_cloud.publish(pc2.create_cloud_xyz32(h, pts))


def main(args=None):
    rclpy.init(args=args)
    node = OccupancyGridNode()
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
# occupancy_grid_node.py

# Implements Section III-H (2D Occupancy grid) of:
#   Eppenberger et al., "Leveraging Stereo-Camera Data for Real-Time
#   Dynamic Obstacle Detection and Tracking", IROS 2020.

# Three maps are maintained in parallel, as in the paper:
#   STATIC    -- persistent costs from clusters classified 'static'.
#                The paper clears erroneously-occupied space by
#                raytracing; we approximate that with observation decay:
#                static cells not re-observed lose confidence and release.
#   DYNAMIC   -- rebuilt every frame from 'dynamic'/'person' objects.
#                Costs are EXPANDED IN THE DIRECTION OF THE ESTIMATED
#                VELOCITY (the object's footprint is swept along v*t for
#                t in [0, predict_horizon]) so a planner avoids where the
#                object is GOING, not just where it is.
#   UNCERTAIN -- short-living costs for clusters not yet classified.

# Outputs:
#   - /manriix/grid/static, /manriix/grid/dynamic, /manriix/grid/uncertain
#         (nav_msgs/OccupancyGrid) individual layers
#   - /manriix/grid/aggregated (nav_msgs/OccupancyGrid) cell-wise max,
#         ready for RViz / planners that take a grid
#   - /manriix/obstacle_cloud (sensor_msgs/PointCloud2) all occupied cells
#         as points at z=0.15 -- plugs directly into a Nav2 costmap
#         obstacle layer as an observation source (marking, no clearing;
#         keep your lidar/depth as the clearing source).

# The grid is robot-centered ("rolling"), fixed size, in the map frame.
# """

# import numpy as np

# import rclpy
# from rclpy.node import Node
# from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

# from nav_msgs.msg import OccupancyGrid
# from sensor_msgs.msg import PointCloud2
# from sensor_msgs_py import point_cloud2 as pc2

# import tf2_ros
# from tf2_ros import TransformException

# from manriix_oak_obstacles_msgs.msg import ClusterTrackArray


# DYNAMIC_LABELS = {'dynamic', 'person'}


# class OccupancyGridNode(Node):

#     def __init__(self):
#         super().__init__('occupancy_grid_node')

#         self.declare_parameter('tracks_topic', '/manriix/objects')
#         self.declare_parameter('map_frame', 'map')
#         self.declare_parameter('base_frame', 'base_footprint')

#         self.declare_parameter('grid_size', 8.0)        # [m] side length
#         self.declare_parameter('resolution', 0.05)       # [m/cell]
#         self.declare_parameter('inflate_cells', 2)       # simple obstacle inflation

#         self.declare_parameter('predict_horizon', 1.0)   # [s] velocity sweep length
#         self.declare_parameter('predict_steps', 5)        # sweep sub-steps
#         self.declare_parameter('static_hit', 30)          # confidence added per observation
#         self.declare_parameter('static_decay', 2)         # confidence lost per frame
#         self.declare_parameter('static_max', 100)
#         self.declare_parameter('static_occupied_thresh', 50)

#         self.tracks_topic = self.get_parameter('tracks_topic').value
#         self.map_frame = self.get_parameter('map_frame').value
#         self.base_frame = self.get_parameter('base_frame').value
#         self.grid_size = float(self.get_parameter('grid_size').value)
#         self.resolution = float(self.get_parameter('resolution').value)
#         self.inflate_cells = int(self.get_parameter('inflate_cells').value)
#         self.predict_horizon = float(self.get_parameter('predict_horizon').value)
#         self.predict_steps = int(self.get_parameter('predict_steps').value)
#         self.static_hit = int(self.get_parameter('static_hit').value)
#         self.static_decay = int(self.get_parameter('static_decay').value)
#         self.static_max = int(self.get_parameter('static_max').value)
#         self.static_occupied_thresh = int(self.get_parameter('static_occupied_thresh').value)

#         self.n_cells = int(round(self.grid_size / self.resolution))

#         # Persistent static confidence grid + its world origin
#         self.static_conf = np.zeros((self.n_cells, self.n_cells), dtype=np.int16)
#         self.origin_xy = None  # world coords of cell (0,0); shifts as robot moves

#         self.tf_buffer = tf2_ros.Buffer()
#         self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

#         qos = QoSProfile(
#             reliability=ReliabilityPolicy.BEST_EFFORT,
#             history=HistoryPolicy.KEEP_LAST,
#             depth=5,
#             durability=DurabilityPolicy.VOLATILE,
#         )
#         self.create_subscription(
#             ClusterTrackArray, self.tracks_topic, self._cb, qos)

#         self.pub_static = self.create_publisher(OccupancyGrid, '/manriix/grid/static', 5)
#         self.pub_dynamic = self.create_publisher(OccupancyGrid, '/manriix/grid/dynamic', 5)
#         self.pub_uncertain = self.create_publisher(OccupancyGrid, '/manriix/grid/uncertain', 5)
#         self.pub_agg = self.create_publisher(OccupancyGrid, '/manriix/grid/aggregated', 5)
#         self.pub_cloud = self.create_publisher(PointCloud2, '/manriix/obstacle_cloud', 5)

#         self.get_logger().info(
#             f'{self.n_cells}x{self.n_cells} cells @ {self.resolution}m, '
#             f'predict_horizon={self.predict_horizon}s')

#     # ------------------------------------------------------------------
#     def _robot_xy(self):
#         try:
#             tf = self.tf_buffer.lookup_transform(
#                 self.map_frame, self.base_frame, rclpy.time.Time())
#             return np.array([tf.transform.translation.x,
#                              tf.transform.translation.y])
#         except TransformException:
#             return np.zeros(2)

#     # ------------------------------------------------------------------
#     def _roll_grid(self, new_origin):
#         """Shift the persistent static grid so data survives robot motion."""
#         if self.origin_xy is None:
#             self.origin_xy = new_origin
#             return
#         shift = np.round((new_origin - self.origin_xy) / self.resolution).astype(int)
#         if shift[0] == 0 and shift[1] == 0:
#             return
#         rolled = np.zeros_like(self.static_conf)
#         sx, sy = shift
#         src_x = slice(max(0, sx), self.n_cells + min(0, sx))
#         dst_x = slice(max(0, -sx), self.n_cells + min(0, -sx))
#         src_y = slice(max(0, sy), self.n_cells + min(0, sy))
#         dst_y = slice(max(0, -sy), self.n_cells + min(0, -sy))
#         rolled[dst_y, dst_x] = self.static_conf[src_y, src_x]
#         self.static_conf = rolled
#         self.origin_xy = self.origin_xy + shift * self.resolution

#     # ------------------------------------------------------------------
#     def _world_to_cell(self, pts_xy):
#         """(N,2) world -> (N,2) int cell indices [col,row]; mask in-bounds."""
#         rel = (pts_xy - self.origin_xy) / self.resolution
#         idx = np.floor(rel).astype(int)
#         ok = ((idx[:, 0] >= 0) & (idx[:, 0] < self.n_cells) &
#               (idx[:, 1] >= 0) & (idx[:, 1] < self.n_cells))
#         return idx[ok]

#     def _mark(self, grid, idx, value=100):
#         if idx.shape[0] == 0:
#             return
#         grid[idx[:, 1], idx[:, 0]] = np.maximum(grid[idx[:, 1], idx[:, 0]], value)

#     def _inflate(self, grid):
#         if self.inflate_cells <= 0:
#             return grid
#         out = grid.copy()
#         occ = np.argwhere(grid >= 100)
#         r = self.inflate_cells
#         for y, x in occ:
#             y0, y1 = max(0, y - r), min(self.n_cells, y + r + 1)
#             x0, x1 = max(0, x - r), min(self.n_cells, x + r + 1)
#             out[y0:y1, x0:x1] = np.maximum(out[y0:y1, x0:x1], 90)
#         return out

#     # ------------------------------------------------------------------
#     def _cb(self, msg: ClusterTrackArray):
#         robot_xy = self._robot_xy()
#         center = robot_xy - self.grid_size / 2.0
#         self._roll_grid(center)

#         dynamic_grid = np.zeros((self.n_cells, self.n_cells), dtype=np.int8)
#         uncertain_grid = np.zeros((self.n_cells, self.n_cells), dtype=np.int8)
#         static_observed = np.zeros((self.n_cells, self.n_cells), dtype=bool)

#         for trk in msg.tracks:
#             pts = pc2.read_points_numpy(
#                 trk.points, field_names=('x', 'y', 'z'), skip_nans=True)
#             if pts.shape[0] == 0:
#                 continue
#             xy = pts[:, :2]

#             if trk.classification == 'static':
#                 idx = self._world_to_cell(xy)
#                 if idx.shape[0]:
#                     static_observed[idx[:, 1], idx[:, 0]] = True

#             elif trk.classification in DYNAMIC_LABELS:
#                 # Paper: "expand the costs in the direction of the
#                 # estimated velocity of the objects" -- sweep the
#                 # footprint along v*t.
#                 v = np.array([trk.velocity.x, trk.velocity.y])
#                 steps = max(1, self.predict_steps)
#                 for k in range(steps + 1):
#                     t = self.predict_horizon * k / steps
#                     swept = xy + v * t
#                     idx = self._world_to_cell(swept)
#                     # full cost now, tapering into the future
#                     cost = int(100 - (30 * k / steps))
#                     self._mark(dynamic_grid, idx, cost)

#             else:  # 'uncertain' / 'unknown' -- short-living by design:
#                    # this layer is rebuilt from scratch every frame.
#                 idx = self._world_to_cell(xy)
#                 self._mark(uncertain_grid, idx, 80)

#         # ---- static layer update: hits + decay (raytrace substitute) ----
#         self.static_conf = np.maximum(self.static_conf - self.static_decay, 0)
#         self.static_conf[static_observed] = np.minimum(
#             self.static_conf[static_observed] + self.static_hit + self.static_decay,
#             self.static_max)
#         static_grid = np.where(
#             self.static_conf >= self.static_occupied_thresh, 100, 0).astype(np.int8)

#         static_grid = self._inflate(static_grid)
#         dynamic_grid = self._inflate(dynamic_grid)

#         agg = np.maximum(np.maximum(static_grid, dynamic_grid), uncertain_grid)

#         stamp = msg.header.stamp
#         self.pub_static.publish(self._to_msg(static_grid, stamp))
#         self.pub_dynamic.publish(self._to_msg(dynamic_grid, stamp))
#         self.pub_uncertain.publish(self._to_msg(uncertain_grid, stamp))
#         self.pub_agg.publish(self._to_msg(agg, stamp))
#         self._publish_cloud(agg, stamp)

#     # ------------------------------------------------------------------
#     def _to_msg(self, grid, stamp) -> OccupancyGrid:
#         m = OccupancyGrid()
#         m.header.stamp = stamp
#         m.header.frame_id = self.map_frame
#         m.info.resolution = self.resolution
#         m.info.width = self.n_cells
#         m.info.height = self.n_cells
#         m.info.origin.position.x = float(self.origin_xy[0])
#         m.info.origin.position.y = float(self.origin_xy[1])
#         m.info.origin.orientation.w = 1.0
#         m.data = grid.astype(np.int8).flatten().tolist()
#         return m

#     def _publish_cloud(self, agg, stamp):
#         occ = np.argwhere(agg >= 80)  # rows=y, cols=x
#         if occ.shape[0] == 0:
#             pts = np.zeros((0, 3))
#         else:
#             xs = self.origin_xy[0] + (occ[:, 1] + 0.5) * self.resolution
#             ys = self.origin_xy[1] + (occ[:, 0] + 0.5) * self.resolution
#             zs = np.full(occ.shape[0], 0.15)
#             pts = np.stack([xs, ys, zs], axis=1)
#         from std_msgs.msg import Header
#         h = Header()
#         h.stamp = stamp
#         h.frame_id = self.map_frame
#         self.pub_cloud.publish(pc2.create_cloud_xyz32(h, pts))


# def main(args=None):
#     rclpy.init(args=args)
#     node = OccupancyGridNode()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()


# if __name__ == '__main__':
#     main()