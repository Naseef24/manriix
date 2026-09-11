import rclpy
from rclpy.node import Node

from vision_msgs.msg import Detection2DArray
from geometry_msgs.msg import PointStamped


class OAKObstacleFilter(Node):

    def __init__(self):
        super().__init__('oak_obstacle_filter')

        self.sub = self.create_subscription(
            Detection2DArray,
            '/oak/nn/spatial_detections',
            self.callback,
            10
        )

        self.pub = self.create_publisher(
            PointStamped,
            '/oak_obstacles/filtered',
            10
        )

        self.get_logger().info("OAK obstacle filter started")

    def callback(self, msg):

        for det in msg.detections:

            for res in det.results:

                score = res.hypothesis.score
                pos = det.results[0].pose.pose.position

                x = pos.x
                y = pos.y
                z = pos.z

                # -------------------------
                # ROBOT FILTER RULES
                # -------------------------

                if score < 0.6:
                    continue

                if z < 0.2 or z > 3.0:
                    continue

                if abs(x) > 2.0:
                    continue

                point = PointStamped()
                point.header = msg.header
                point.point.x = x
                point.point.y = y
                point.point.z = z

                self.pub.publish(point)


def main():
    rclpy.init()
    node = OAKObstacleFilter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()




# import rclpy
# from rclpy.node import Node

# from sensor_msgs.msg import Image
# from cv_bridge import CvBridge

# import cv2
# import numpy as np
# import torch

# from transformers import SegformerImageProcessor, SegformerForSemanticSegmentation


# class FusionNode(Node):

#     def __init__(self):
#         super().__init__('segformer_fusion_node')

#         self.bridge = CvBridge()

#         # RGB input
#         self.sub_rgb = self.create_subscription(
#             Image,
#             '/oak/rgb/image_raw',
#             self.rgb_callback,
#             10
#         )

#         # Depth input (change if your topic differs)
#         self.sub_depth = self.create_subscription(
#             Image,
#             '/oak/stereo/image_raw',
#             self.depth_callback,
#             10
#         )

#         # Output overlay
#         self.pub = self.create_publisher(
#             Image,
#             '/vision/safe_overlay',
#             10
#         )

#         self.get_logger().info("SegFormer + Depth Fusion Node Started")

#         # Model
#         self.processor = SegformerImageProcessor.from_pretrained(
#             "nvidia/segformer-b0-finetuned-cityscapes-1024-1024"
#         )

#         self.model = SegformerForSemanticSegmentation.from_pretrained(
#             "nvidia/segformer-b0-finetuned-cityscapes-1024-1024"
#         )

#         self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
#         self.model.to(self.device)

#         self.latest_depth = None

#     # -------------------------
#     # Depth callback
#     # -------------------------
#     def depth_callback(self, msg):
#         self.latest_depth = self.bridge.imgmsg_to_cv2(
#             msg,
#             desired_encoding="passthrough"
#         )

#     # -------------------------
#     # RGB callback
#     # -------------------------
#     def rgb_callback(self, msg):

#         if self.latest_depth is None:
#             return

#         frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
#         rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

#         # -------------------------
#         # Segmentation inference
#         # -------------------------
#         inputs = self.processor(images=rgb, return_tensors="pt").to(self.device)

#         with torch.no_grad():
#             outputs = self.model(**inputs)

#         seg = torch.argmax(outputs.logits.squeeze(), dim=0).cpu().numpy()

#         # IMPORTANT FIX: correct resize (NO corruption)
#         seg = cv2.resize(
#             seg.astype(np.uint8),
#             (frame.shape[1], frame.shape[0]),
#             interpolation=cv2.INTER_NEAREST
#         )

#         # -------------------------
#         # Depth processing
#         # -------------------------
#         depth = self.latest_depth

#         if depth.shape[:2] != seg.shape[:2]:
#             depth = cv2.resize(depth, (frame.shape[1], frame.shape[0]))

#         depth = depth.astype(np.float32)

#         # valid safe range (tune later)
#         depth_mask = (depth > 0.3) & (depth < 3.0)

#         # -------------------------
#         # MASK LOGIC (CORE FIX)
#         # -------------------------

#         # NOTE: class 0 ≠ guaranteed floor, but used as proxy
#         floor_mask = (seg == 0)

#         safe_mask = floor_mask & depth_mask
#         unsafe_mask = ~safe_mask

#         # -------------------------
#         # VISUALIZATION (NO PIXEL CONFLICT)
#         # -------------------------

#         overlay = frame.copy()

#         alpha = 0.5

#         # Safe region (GREEN)
#         green_layer = np.zeros_like(frame)
#         green_layer[:, :] = (0, 255, 0)

#         # Unsafe region (RED)
#         red_layer = np.zeros_like(frame)
#         red_layer[:, :] = (0, 0, 255)

#         # Apply masks safely (no overwrite conflict)
#         overlay = np.where(
#             np.repeat(safe_mask[:, :, None], 3, axis=2),
#             cv2.addWeighted(frame, 1 - alpha, green_layer, alpha, 0),
#             overlay
#         )

#         overlay = np.where(
#             np.repeat(unsafe_mask[:, :, None], 3, axis=2),
#             cv2.addWeighted(frame, 1 - alpha, red_layer, alpha, 0),
#             overlay
#         )

#         # -------------------------
#         # Publish
#         # -------------------------
#         out_msg = self.bridge.cv2_to_imgmsg(overlay, "bgr8")
#         out_msg.header = msg.header

#         self.pub.publish(out_msg)


# def main():
#     rclpy.init()
#     node = FusionNode()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()


# if __name__ == '__main__':
#     main()