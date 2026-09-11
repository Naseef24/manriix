#!/usr/bin/env python3
"""
Bridge node: Detection3DArray → PointCloud2

Converts each Detection3D into a vertical patch of points on the FRONT
face of the bounding box (nearest face to the camera). This is what a
depth camera would actually see — and lets Nav2's raytracing clear
cells properly as the robot/obstacles move.

Topic:
  Subscribes  /oak_obstacles/detections     vision_msgs/Detection3DArray
  Publishes   /oak_obstacles/points         sensor_msgs/PointCloud2
"""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from vision_msgs.msg import Detection3DArray
from sensor_msgs.msg import PointCloud2, PointField


def make_pointcloud2(points, header):
    msg = PointCloud2()
    msg.header = header
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
    msg.data = points.astype(np.float32).tobytes()
    return msg


class DetectionToPointCloud(Node):
    def __init__(self):
        super().__init__('detection_to_pointcloud_node')

        self.declare_parameter('input_topic',
                               '/oak_obstacles/detections')
        self.declare_parameter('output_topic',
                               '/oak_obstacles/points')
        # How densely to sample the FRONT face of each box
        self.declare_parameter('grid_w', 8)
        self.declare_parameter('grid_h', 12)

        in_topic = self.get_parameter('input_topic').value
        out_topic = self.get_parameter('output_topic').value
        self.gw = self.get_parameter('grid_w').value
        self.gh = self.get_parameter('grid_h').value

        qos = QoSProfile(
            depth=5,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST)

        self.sub = self.create_subscription(
            Detection3DArray, in_topic, self.callback, qos)
        self.pub = self.create_publisher(PointCloud2, out_topic, qos)

        self.get_logger().info(
            f'Sub: {in_topic}  Pub: {out_topic}\n'
            f'Front-face grid: {self.gw}x{self.gh} per detection')

    def callback(self, msg: Detection3DArray):
        all_points = []
        for det in msg.detections:
            c = det.bbox.center.position
            s = det.bbox.size

            # In camera optical frame: Z is forward (depth from camera).
            # The "front face" of the box (nearest to camera) is at:
            #   z_front = center.z - size.z / 2
            # Front face spans:
            #   x in [center.x - sx/2, center.x + sx/2]
            #   y in [center.y - sy/2, center.y + sy/2]

            z_front = c.z - s.z / 2.0
            xs = np.linspace(c.x - s.x / 2.0,
                             c.x + s.x / 2.0, self.gw)
            ys = np.linspace(c.y - s.y / 2.0,
                             c.y + s.y / 2.0, self.gh)
            xx, yy = np.meshgrid(xs, ys)
            zz = np.full_like(xx, z_front)

            pts = np.stack([xx.flatten(),
                            yy.flatten(),
                            zz.flatten()], axis=1)
            all_points.append(pts)

        if not all_points:
            empty = np.zeros((0, 3), dtype=np.float32)
            cloud = make_pointcloud2(empty, msg.header)
        else:
            pts = np.concatenate(all_points, axis=0)
            cloud = make_pointcloud2(pts, msg.header)

        self.pub.publish(cloud)


def main(args=None):
    rclpy.init(args=args)
    node = DetectionToPointCloud()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()