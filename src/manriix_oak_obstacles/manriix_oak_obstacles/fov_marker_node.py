#!/usr/bin/env python3
"""
fov_marker_node.py

Publishes the OAK-D's horizontal field of view as a white floor-level
wedge outline (visualization_msgs/Marker), matching the convention used
in the paper's Fig. 1 ("The robot's footprint and field of view are
shown in green and white, respectively").

Design:
  - Looks up the camera's pose relative to base_footprint via TF every
    tick, so the wedge automatically tracks the camera (and, in
    production, the robot's motion) with no manual placement.
  - Draws a simple triangular wedge (apex at the camera's floor
    position, two edges at +/- half the horizontal FOV, sampled far
    edge points for a slight arc) at z=0 in base_footprint.
  - Runs continuously and independently of the rest of the pipeline --
    intended to be launched alongside the camera itself (oak_camera*.
    launch.py), so the FOV is visible the moment the camera node is up,
    with no manual "add a display" step beyond adding the Marker/
    MarkerArray topic in RViz/Foxglove once.

This node has NO dependency on the obstacle pipeline nodes -- only on
TF and the camera's known horizontal FOV -- so it stays up (and stays
correct) even if the rest of the pipeline is restarted independently.
"""

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

import tf2_ros
from tf2_ros import TransformException

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point


def quat_to_yaw(x, y, z, w):
    """Extract yaw (rotation about world z) from a quaternion."""
    siny = 2.0 * (w * z + x * y)
    cosy = 1.0 - 2.0 * (y * y + z * z)
    return np.arctan2(siny, cosy)


class FovMarkerNode(Node):

    def __init__(self):
        super().__init__('fov_marker_node')

        self.declare_parameter('base_frame', 'base_footprint')
        # Default matches the dev launch's driver-component naming
        # (camera_name='oak' -> oak_rgb_camera_optical_frame). Override
        # to oak_d_pro_w_rgb_camera_optical_frame or similar in a
        # production params file if the deployed camera name differs.
        self.declare_parameter('camera_frame', 'oak_rgb_camera_optical_frame')
        self.declare_parameter('horizontal_fov_deg', 127.0)  # OAK-D Pro W
        self.declare_parameter('range_m', 3.0)  # match l_d for visual consistency
        self.declare_parameter('arc_segments', 12)
        self.declare_parameter('publish_rate_hz', 5.0)
        self.declare_parameter('marker_topic', '/manriix/camera_fov')
        self.declare_parameter('line_width', 0.02)

        self.base_frame = self.get_parameter('base_frame').value
        self.camera_frame = self.get_parameter('camera_frame').value
        self.hfov_rad = np.deg2rad(
            float(self.get_parameter('horizontal_fov_deg').value))
        self.range_m = float(self.get_parameter('range_m').value)
        self.arc_segments = int(self.get_parameter('arc_segments').value)
        self.line_width = float(self.get_parameter('line_width').value)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.pub = self.create_publisher(
            Marker, self.get_parameter('marker_topic').value, 5)

        rate = float(self.get_parameter('publish_rate_hz').value)
        self.timer = self.create_timer(1.0 / rate, self._tick)

        self.get_logger().info(
            f'FOV marker: {self.camera_frame} -> {self.base_frame}, '
            f'hfov={np.rad2deg(self.hfov_rad):.0f}deg, range={self.range_m}m'
        )

    def _tick(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.base_frame, self.camera_frame, rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().warn(
                f'FOV marker: TF {self.camera_frame}->{self.base_frame} '
                f'unavailable: {ex}', throttle_duration_sec=5.0)
            return

        t = tf.transform.translation
        q = tf.transform.rotation
        apex_x, apex_y = t.x, t.y
        # Camera optical frame convention (x-right, z-forward): the
        # "forward" direction in the parent frame is well-approximated
        # by the yaw of the transform for a forward-facing, ~level
        # mount. Using the transform's own yaw keeps this correct
        # automatically if the mount pitch/roll are small, matching how
        # the rest of the pipeline already treats the camera pose.
        yaw = quat_to_yaw(q.x, q.y, q.z, q.w) + np.pi / 2.0

        half = self.hfov_rad / 2.0
        angles = np.linspace(yaw - half, yaw + half, self.arc_segments)
        far_points = [
            (apex_x + self.range_m * np.cos(a),
             apex_y + self.range_m * np.sin(a))
            for a in angles
        ]

        pts = [Point(x=apex_x, y=apex_y, z=0.0)]
        for (x, y) in far_points:
            pts.append(Point(x=x, y=y, z=0.0))
        pts.append(Point(x=apex_x, y=apex_y, z=0.0))

        m = Marker()
        m.header.frame_id = self.base_frame
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'camera_fov'
        m.id = 0
        m.type = Marker.LINE_STRIP
        m.action = Marker.ADD
        m.pose.orientation.w = 1.0
        m.scale.x = self.line_width
        m.color.r = 1.0
        m.color.g = 1.0
        m.color.b = 1.0
        m.color.a = 0.9
        m.points = pts
        m.lifetime = Duration(seconds=1.0).to_msg()

        self.pub.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = FovMarkerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()