import rclpy
from rclpy.node import Node

from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2

import numpy as np


class PassThroughFilter(Node):

    def __init__(self):
        super().__init__('passthrough_filter')

        from rclpy.qos import qos_profile_sensor_data

        self.sub = self.create_subscription(
            PointCloud2,
            '/oak/points',
            self.callback,
            qos_profile_sensor_data
        )

        self.pub = self.create_publisher(
            PointCloud2,
            '/oak/points_filtered',
            10
        )

        # paper values
        self.z_min = 0.7
        self.z_max = 5.0

        self.get_logger().info("PassThrough Filter Node Started")
    
    def callback(self, msg):

        points_list = list(pc2.read_points(
            msg,
            field_names=("x", "y", "z", "intensity"),
            skip_nans=True
        ))

        if len(points_list) == 0:
            return

        filtered = []

        for p in points_list:
            x = p[0]
            y = p[1]
            z = p[2]
            i = p[3]

            # -----------------------
            # FILTER LOGIC
            # -----------------------

            if z < 0 or z > 3.0:
                continue

            if y > 0.5 or y < -0.5:
                continue

            if abs(x) > 2.0:
                continue

            filtered.append((x, y, z, i))

        if len(filtered) == 0:
            return

        filtered_msg = pc2.create_cloud(
            msg.header,
            msg.fields,
            filtered
        )

        self.pub.publish(filtered_msg)

    # def callback(self, msg):

    #     points = pc2.read_points(
    #         msg,
    #         field_names=("x", "y", "z", "intensity"),
    #         skip_nans=True
    #     )

    #     filtered = []

    #     for x, y, z, i in points:

    #         # 1. forward range
    #         if z < 0 or z > 3.0:
    #             continue

    #         # 2. height filter
    #         if y > 0.5 or y < -0.5:
    #             continue

    #         # 3. lateral filter
    #         if abs(x) > 2.0:
    #             continue

    #         filtered.append((x, y, z, i))

    #     filtered_msg = pc2.create_cloud(
    #         msg.header,
    #         msg.fields,
    #         filtered
    #     )

    #     self.pub.publish(filtered_msg)


    # def callback(self, msg):

    #     points = list(pc2.read_points(
    #         msg,
    #         field_names=("x", "y", "z", "intensity"),
    #         skip_nans=True
    #     ))

    #     filtered = []

    #     for p in points:
    #         x, y, z, i = p

    #         # -----------------------------
    #         # FILTER RULES (NEW REQUIREMENT)
    #         # -----------------------------

    #         # 1. forward distance (use Z as depth)
    #         if z < 0 or z > 3.0:
    #             continue

    #         # 2. height constraint (above floor)
    #         # keep low obstacles (0 to 0.5m)
    #         if y > 0.5 or y < -0.5:
    #             continue

    #         # 3. lateral noise reduction (optional but useful)
    #         if abs(x) > 2.0:
    #             continue

    #         filtered.append((x, y, z, i))

    #     # convert back
    #     filtered_msg = pc2.create_cloud(
    #         msg.header,
    #         msg.fields,
    #         filtered
    #     )

    #     self.pub.publish(filtered_msg)


    # def callback(self, msg):

    #     points = list(pc2.read_points(
    #         msg,
    #         field_names=("x", "y", "z", "intensity"),
    #         skip_nans=True
    #     ))

    #     filtered = []

    #     for p in points:
    #         x, y, z, i = p

    #         # PASS-THROUGH FILTER (Z ONLY for now)
    #         if self.z_min <= z <= self.z_max:
    #             filtered.append([x, y, z, i])

    #     # convert back
    #     filtered_msg = pc2.create_cloud(
    #         msg.header,
    #         msg.fields,
    #         filtered
    #     )

    #     self.pub.publish(filtered_msg)


def main(args=None):
    rclpy.init(args=args)
    node = PassThroughFilter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()