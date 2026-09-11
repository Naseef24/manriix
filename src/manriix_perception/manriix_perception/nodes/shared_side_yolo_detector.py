#!/usr/bin/env python3
import json, threading
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String

class SharedSideYoloDetector(Node):
    def __init__(self):
        super().__init__("shared_side_yolo_detector")
        self.declare_parameter("model_path", "/home/hype/models/yolo11/yolo11n.onnx")
        self.declare_parameter("confidence", 0.25)
        self.declare_parameter("imgsz", 640)
        self.declare_parameter("max_inference_hz", 8.0)
        self.declare_parameter("device", "0")
        from ultralytics import YOLO
        self.model = YOLO(str(self.get_parameter("model_path").value))
        self.conf = float(self.get_parameter("confidence").value)
        self.imgsz = int(self.get_parameter("imgsz").value)
        self.device = str(self.get_parameter("device").value)
        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.latest = {"left": None, "right": None}
        self.done_stamp = {"left": -1.0, "right": -1.0}
        self.next_cam = "left"
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE,
                         history=HistoryPolicy.KEEP_LAST, depth=1)
        for cam in ("left", "right"):
            self.create_subscription(Image, f"/monitor/zedx_{cam}/image_raw",
                                     lambda m, c=cam: self.cb(c, m), qos)
        self.pub = self.create_publisher(String, "/web/camera_detections_side", 10)
        hz = float(self.get_parameter("max_inference_hz").value)
        self.create_timer(1.0 / max(1.0, hz), self.tick)
        self.get_logger().info("Shared latest-frame YOLO detector ready for left/right")

    def cb(self, cam, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().warning(f"{cam}: cv_bridge: {e}")
            return
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        with self.lock:
            self.latest[cam] = (stamp, frame)

    def tick(self):
        order = [self.next_cam, "right" if self.next_cam == "left" else "left"]
        selected = None
        for cam in order:
            with self.lock:
                item = self.latest[cam]
            if item and item[0] != self.done_stamp[cam]:
                selected = (cam, item[0], item[1])
                break
        if not selected:
            return
        cam, stamp, frame = selected
        self.done_stamp[cam] = stamp
        self.next_cam = "right" if cam == "left" else "left"
        try:
            result = self.model.predict(frame, conf=self.conf, imgsz=self.imgsz,
                                        device=self.device, verbose=False)[0]
        except Exception as e:
            self.get_logger().error(f"{cam}: inference failed: {e}")
            return
        dets = []
        if result.boxes is not None:
            xyxy = result.boxes.xyxy.detach().cpu().numpy()
            confs = result.boxes.conf.detach().cpu().numpy()
            classes = result.boxes.cls.detach().cpu().numpy().astype(int)
            for box, conf, cls_id in zip(xyxy, confs, classes):
                x1,y1,x2,y2 = map(float, box)
                dets.append({
                    "label": str(result.names.get(int(cls_id), cls_id)),
                    "label_id": int(cls_id),
                    "confidence": float(conf * 100.0),
                    "bbox": [[x1,y1],[x2,y1],[x2,y2],[x1,y2]],
                    "position": None, "distance": None,
                    "source": "shared_yolo_tensorrt"
                })
        msg = String()
        msg.data = json.dumps({cam: {"count": len(dets), "detections": dets, "stamp": stamp}})
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = SharedSideYoloDetector()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()

if __name__ == "__main__":
    main()