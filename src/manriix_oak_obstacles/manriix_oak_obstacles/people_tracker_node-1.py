# #!/usr/bin/env python3
# """
# people_tracker_node.py

# Implements Section III-E (2D People Detector) of Eppenberger et al.,
# IROS 2020 -- adapted to the OAK-D Pro W's on-device spatial NN.

# The MobileNet-SSD detector runs ON-DEVICE on the OAK's VPU
# (depthai_ros_driver, nn.i_nn_family: mobilenet, i_nn_type: spatial),
# so host inference cost is ~zero (the paper spent 42.4 ms/frame on this,
# Table I). Your driver version publishes the spatial detections as
# vision_msgs/Detection3DArray on /oak/nn/spatial_detections: each
# detection carries a 3D position (bbox.center.position) in the camera
# optical frame, fused on-device from the RGB-aligned depth.

# Because we get 3D positions directly, we do tracking-by-detection with
# association on 3D center distance (instead of image-plane IoU). The
# paper explicitly notes several valid alternatives for the 2D-3D
# association step; with this hardware the 3D route is simpler and makes
# Step-5 fusion (person <-> cluster) a direct nearest-centroid match.

# Publishes BoundingBoxTrackArray on /manriix/person_boxes; the box
# fields carry the 3D position (in the target/world frame after TF), and
# box pixel fields are left zero (not provided by Detection3DArray).
# """

# import numpy as np

# import rclpy
# from rclpy.node import Node
# from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
# from rclpy.time import Time

# import tf2_ros
# from tf2_ros import TransformException

# from geometry_msgs.msg import Point
# from vision_msgs.msg import Detection3DArray
# from manriix_oak_obstacles_msgs.msg import BoundingBoxTrack, BoundingBoxTrackArray


# PERSON_LABELS = {'15', 'person', 'Person', 'PERSON'}


# def quat_to_rot_matrix(x, y, z, w):
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


# class PersonTrack:
#     _next_id = 0

#     def __init__(self, position, conf, stamp):
#         self.box_id = PersonTrack._next_id
#         PersonTrack._next_id += 1
#         self.position = position   # np.array(3), world/target frame
#         self.confidence = conf
#         self.last_seen = stamp
#         self.hit_count = 1
#         self.missed_frames = 0


# class PeopleTrackerNode(Node):

#     def __init__(self):
#         super().__init__('people_tracker_node')

#         self.declare_parameter('detections_topic', '/oak/nn/spatial_detections')
#         self.declare_parameter('output_topic', '/manriix/person_boxes')
#         self.declare_parameter('target_frame', 'base_footprint')
#         self.declare_parameter('confidence_threshold', 0.5)
#         self.declare_parameter('max_assoc_dist', 0.7)   # [m] 3D association gate
#         self.declare_parameter('box_timeout', 0.7)       # [s]
#         self.declare_parameter('min_hits_to_confirm', 2)

#         self.detections_topic = self.get_parameter('detections_topic').value
#         self.output_topic = self.get_parameter('output_topic').value
#         self.target_frame = self.get_parameter('target_frame').value
#         self.confidence_threshold = float(self.get_parameter('confidence_threshold').value)
#         self.max_assoc_dist = float(self.get_parameter('max_assoc_dist').value)
#         self.box_timeout = float(self.get_parameter('box_timeout').value)
#         self.min_hits_to_confirm = int(self.get_parameter('min_hits_to_confirm').value)

#         self.tf_buffer = tf2_ros.Buffer()
#         self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
#         self._R = None
#         self._t = None
#         self._cached_source = None

#         qos = QoSProfile(
#             reliability=ReliabilityPolicy.BEST_EFFORT,
#             history=HistoryPolicy.KEEP_LAST,
#             depth=5,
#             durability=DurabilityPolicy.VOLATILE,
#         )

#         self.create_subscription(
#             Detection3DArray, self.detections_topic, self._cb, qos)
#         # Reliable publisher: downstream fusion + debugging tools expect it.
#         self.pub = self.create_publisher(BoundingBoxTrackArray, self.output_topic, 10)

#         self.tracks = {}

#         self.get_logger().info(
#             f'Listening on {self.detections_topic} as Detection3DArray '
#             f'(conf>={self.confidence_threshold}, gate={self.max_assoc_dist}m), '
#             f'positions output in {self.target_frame}'
#         )

#     # ------------------------------------------------------------------
#     def _update_tf(self, source_frame):
#         try:
#             tf = self.tf_buffer.lookup_transform(
#                 self.target_frame, source_frame, rclpy.time.Time())
#         except TransformException as ex:
#             self.get_logger().warn(
#                 f'TF {source_frame} -> {self.target_frame} unavailable: {ex}',
#                 throttle_duration_sec=2.0)
#             return False
#         q = tf.transform.rotation
#         t = tf.transform.translation
#         self._R = quat_to_rot_matrix(q.x, q.y, q.z, q.w)
#         self._t = np.array([t.x, t.y, t.z])
#         self._cached_source = source_frame
#         return True

#     # ------------------------------------------------------------------
#     def _cb(self, msg: Detection3DArray):
#         source_frame = msg.header.frame_id
#         if self._R is None or self._cached_source != source_frame:
#             if not self._update_tf(source_frame):
#                 return

#         now = Time.from_msg(msg.header.stamp).nanoseconds * 1e-9

#         dets = []
#         for det in msg.detections:
#             best = None
#             for r in det.results:
#                 class_id = getattr(getattr(r, 'hypothesis', r), 'class_id',
#                                    getattr(r, 'id', ''))
#                 score = getattr(getattr(r, 'hypothesis', r), 'score',
#                                 getattr(r, 'score', 0.0))
#                 if str(class_id) in PERSON_LABELS and score >= self.confidence_threshold:
#                     if best is None or score > best:
#                         best = score
#             if best is None:
#                 continue
#             p = det.bbox.center.position
#             pos_cam = np.array([p.x, p.y, p.z])
#             if np.linalg.norm(pos_cam) < 1e-6:
#                 continue  # no valid depth fused for this detection
#             pos_world = self._R @ pos_cam + self._t
#             dets.append((pos_world, best))

#         self._track_update(dets, now, msg)

#     # ------------------------------------------------------------------
#     def _track_update(self, dets, now, msg):
#         track_ids = list(self.tracks.keys())
#         matched_t, matched_d = set(), set()

#         pairs = []
#         for tid in track_ids:
#             for j, (pos, conf) in enumerate(dets):
#                 d = np.linalg.norm(self.tracks[tid].position - pos)
#                 if d <= self.max_assoc_dist:
#                     pairs.append((d, tid, j))
#         pairs.sort()

#         for d, tid, j in pairs:
#             if tid in matched_t or j in matched_d:
#                 continue
#             trk = self.tracks[tid]
#             trk.position, trk.confidence = dets[j]
#             trk.last_seen = now
#             trk.hit_count += 1
#             trk.missed_frames = 0
#             matched_t.add(tid)
#             matched_d.add(j)

#         for tid in track_ids:
#             if tid not in matched_t:
#                 self.tracks[tid].missed_frames += 1
#                 self.tracks[tid].hit_count = 0

#         for j, (pos, conf) in enumerate(dets):
#             if j not in matched_d:
#                 trk = PersonTrack(pos, conf, now)
#                 self.tracks[trk.box_id] = trk

#         for tid in [t for t, trk in self.tracks.items()
#                     if (now - trk.last_seen) > self.box_timeout]:
#             del self.tracks[tid]

#         out = BoundingBoxTrackArray()
#         out.header = msg.header
#         out.header.frame_id = self.target_frame
#         for trk in self.tracks.values():
#             if trk.hit_count < self.min_hits_to_confirm or trk.missed_frames > 0:
#                 continue
#             b = BoundingBoxTrack()
#             b.box_id = trk.box_id
#             b.confidence = float(trk.confidence)
#             b.hit_count = int(trk.hit_count)
#             b.missed_frames = int(trk.missed_frames)
#             b.position = Point(x=float(trk.position[0]),
#                                 y=float(trk.position[1]),
#                                 z=float(trk.position[2]))
#             out.boxes.append(b)
#         self.pub.publish(out)


# def main(args=None):
#     rclpy.init(args=args)
#     node = PeopleTrackerNode()
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
# people_tracker_node.py

# Implements Section III-E (2D People Detector) of Eppenberger et al.,
# IROS 2020 -- adapted to the OAK-D Pro W's on-device spatial NN.

# The MobileNet-SSD detector runs ON-DEVICE on the OAK's VPU
# (depthai_ros_driver, nn.i_nn_family: mobilenet, i_nn_type: spatial),
# so host inference cost is ~zero (the paper spent 42.4 ms/frame on this,
# Table I). Your driver version publishes the spatial detections as
# vision_msgs/Detection3DArray on /oak/nn/spatial_detections: each
# detection carries a 3D position (bbox.center.position) in the camera
# optical frame, fused on-device from the RGB-aligned depth.

# Because we get 3D positions directly, we do tracking-by-detection with
# association on 3D center distance (instead of image-plane IoU). The
# paper explicitly notes several valid alternatives for the 2D-3D
# association step; with this hardware the 3D route is simpler and makes
# Step-5 fusion (person <-> cluster) a direct nearest-centroid match.

# Publishes BoundingBoxTrackArray on /manriix/person_boxes; the box
# fields carry the 3D position (in the target/world frame after TF), and
# box pixel fields are left zero (not provided by Detection3DArray).
# """

# import numpy as np

# import rclpy
# from rclpy.node import Node
# from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
# from rclpy.time import Time

# import tf2_ros
# from tf2_ros import TransformException

# from geometry_msgs.msg import Point
# from vision_msgs.msg import Detection3DArray
# from manriix_oak_obstacles_msgs.msg import BoundingBoxTrack, BoundingBoxTrackArray


# PERSON_LABELS = {'15', 'person', 'Person', 'PERSON'}


# def quat_to_rot_matrix(x, y, z, w):
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


# class PersonTrack:
#     _next_id = 0

#     def __init__(self, position, conf, stamp):
#         self.box_id = PersonTrack._next_id
#         PersonTrack._next_id += 1
#         self.position = position   # np.array(3), world/target frame
#         self.confidence = conf
#         self.last_seen = stamp
#         self.hit_count = 1
#         self.missed_frames = 0


# class PeopleTrackerNode(Node):

#     def __init__(self):
#         super().__init__('people_tracker_node')

#         self.declare_parameter('detections_topic', '/oak/nn/spatial_detections')
#         self.declare_parameter('output_topic', '/manriix/person_boxes')
#         self.declare_parameter('target_frame', 'base_footprint')
#         self.declare_parameter('confidence_threshold', 0.5)
#         self.declare_parameter('max_assoc_dist', 0.7)   # [m] 3D association gate
#         self.declare_parameter('box_timeout', 0.7)       # [s]
#         self.declare_parameter('min_hits_to_confirm', 2)

#         self.detections_topic = self.get_parameter('detections_topic').value
#         self.output_topic = self.get_parameter('output_topic').value
#         self.target_frame = self.get_parameter('target_frame').value
#         self.confidence_threshold = float(self.get_parameter('confidence_threshold').value)
#         self.max_assoc_dist = float(self.get_parameter('max_assoc_dist').value)
#         self.box_timeout = float(self.get_parameter('box_timeout').value)
#         self.min_hits_to_confirm = int(self.get_parameter('min_hits_to_confirm').value)

#         self.tf_buffer = tf2_ros.Buffer()
#         self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
#         self._R = None
#         self._t = None
#         self._cached_source = None

#         qos = QoSProfile(
#             reliability=ReliabilityPolicy.BEST_EFFORT,
#             history=HistoryPolicy.KEEP_LAST,
#             depth=5,
#             durability=DurabilityPolicy.VOLATILE,
#         )

#         self.create_subscription(
#             Detection3DArray, self.detections_topic, self._cb, qos)
#         # Reliable publisher: downstream fusion + debugging tools expect it.
#         self.pub = self.create_publisher(BoundingBoxTrackArray, self.output_topic, 10)

#         self.tracks = {}

#         self.get_logger().info(
#             f'Listening on {self.detections_topic} as Detection3DArray '
#             f'(conf>={self.confidence_threshold}, gate={self.max_assoc_dist}m), '
#             f'positions output in {self.target_frame}'
#         )

#     # ------------------------------------------------------------------
#     def _update_tf(self, source_frame):
#         try:
#             tf = self.tf_buffer.lookup_transform(
#                 self.target_frame, source_frame, rclpy.time.Time())
#         except TransformException as ex:
#             self.get_logger().warn(
#                 f'TF {source_frame} -> {self.target_frame} unavailable: {ex}',
#                 throttle_duration_sec=2.0)
#             return False
#         q = tf.transform.rotation
#         t = tf.transform.translation
#         self._R = quat_to_rot_matrix(q.x, q.y, q.z, q.w)
#         self._t = np.array([t.x, t.y, t.z])
#         self._cached_source = source_frame
#         return True

#     # ------------------------------------------------------------------
#     def _cb(self, msg: Detection3DArray):
#         source_frame = msg.header.frame_id
#         if self._R is None or self._cached_source != source_frame:
#             if not self._update_tf(source_frame):
#                 return

#         now = Time.from_msg(msg.header.stamp).nanoseconds * 1e-9

#         dets = []
#         for det in msg.detections:
#             best_score = None
#             best_pos = None
#             for r in det.results:
#                 class_id = getattr(getattr(r, 'hypothesis', r), 'class_id',
#                                    getattr(r, 'id', ''))
#                 score = getattr(getattr(r, 'hypothesis', r), 'score',
#                                 getattr(r, 'score', 0.0))
#                 if str(class_id) in PERSON_LABELS and score >= self.confidence_threshold:
#                     if best_score is None or score > best_score:
#                         best_score = score
#                         # NOTE (driver quirk): in this depthai_ros_driver
#                         # version's Detection3DArray, the fused 3D position
#                         # (meters, camera optical frame) is in
#                         # results[].pose.pose.position, while
#                         # bbox.center.position holds the 2D PIXEL center.
#                         p = r.pose.pose.position
#                         best_pos = np.array([p.x, p.y, p.z])
#             if best_score is None or best_pos is None:
#                 continue
#             if np.linalg.norm(best_pos) < 1e-6:
#                 continue  # no valid depth fused for this detection
#             pos_world = self._R @ best_pos + self._t
#             dets.append((pos_world, best_score))

#         self._track_update(dets, now, msg)

#     # ------------------------------------------------------------------
#     def _track_update(self, dets, now, msg):
#         track_ids = list(self.tracks.keys())
#         matched_t, matched_d = set(), set()

#         pairs = []
#         for tid in track_ids:
#             for j, (pos, conf) in enumerate(dets):
#                 d = np.linalg.norm(self.tracks[tid].position - pos)
#                 if d <= self.max_assoc_dist:
#                     pairs.append((d, tid, j))
#         pairs.sort()

#         for d, tid, j in pairs:
#             if tid in matched_t or j in matched_d:
#                 continue
#             trk = self.tracks[tid]
#             trk.position, trk.confidence = dets[j]
#             trk.last_seen = now
#             trk.hit_count += 1
#             trk.missed_frames = 0
#             matched_t.add(tid)
#             matched_d.add(j)

#         for tid in track_ids:
#             if tid not in matched_t:
#                 self.tracks[tid].missed_frames += 1
#                 self.tracks[tid].hit_count = 0

#         for j, (pos, conf) in enumerate(dets):
#             if j not in matched_d:
#                 trk = PersonTrack(pos, conf, now)
#                 self.tracks[trk.box_id] = trk

#         for tid in [t for t, trk in self.tracks.items()
#                     if (now - trk.last_seen) > self.box_timeout]:
#             del self.tracks[tid]

#         out = BoundingBoxTrackArray()
#         out.header = msg.header
#         out.header.frame_id = self.target_frame
#         for trk in self.tracks.values():
#             if trk.hit_count < self.min_hits_to_confirm or trk.missed_frames > 0:
#                 continue
#             b = BoundingBoxTrack()
#             b.box_id = trk.box_id
#             b.confidence = float(trk.confidence)
#             b.hit_count = int(trk.hit_count)
#             b.missed_frames = int(trk.missed_frames)
#             b.position = Point(x=float(trk.position[0]),
#                                 y=float(trk.position[1]),
#                                 z=float(trk.position[2]))
#             out.boxes.append(b)
#         self.pub.publish(out)


# def main(args=None):
#     rclpy.init(args=args)
#     node = PeopleTrackerNode()
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
# people_tracker_node.py

# Implements Section III-E (2D People Detector) of Eppenberger et al.,
# IROS 2020 -- adapted to the OAK-D Pro W's on-device spatial NN.

# The MobileNet-SSD detector runs ON-DEVICE on the OAK's VPU
# (depthai_ros_driver, nn.i_nn_family: mobilenet, i_nn_type: spatial),
# so host inference cost is ~zero (the paper spent 42.4 ms/frame on this,
# Table I). Your driver version publishes the spatial detections as
# vision_msgs/Detection3DArray on /oak/nn/spatial_detections: each
# detection carries a 3D position (bbox.center.position) in the camera
# optical frame, fused on-device from the RGB-aligned depth.

# Because we get 3D positions directly, we do tracking-by-detection with
# association on 3D center distance (instead of image-plane IoU). The
# paper explicitly notes several valid alternatives for the 2D-3D
# association step; with this hardware the 3D route is simpler and makes
# Step-5 fusion (person <-> cluster) a direct nearest-centroid match.

# Publishes BoundingBoxTrackArray on /manriix/person_boxes; the box
# fields carry the 3D position (in the target/world frame after TF), and
# box pixel fields are left zero (not provided by Detection3DArray).
# """

# import numpy as np

# import rclpy
# from rclpy.node import Node
# from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
# from rclpy.time import Time

# import tf2_ros
# from tf2_ros import TransformException

# from geometry_msgs.msg import Point
# from vision_msgs.msg import Detection3DArray
# from sensor_msgs.msg import Image
# from manriix_oak_obstacles_msgs.msg import BoundingBoxTrack, BoundingBoxTrackArray

# try:
#     import cv2
#     from cv_bridge import CvBridge
#     HAVE_CV = True
# except ImportError:
#     HAVE_CV = False


# PERSON_LABELS = {'15', 'person', 'Person', 'PERSON'}


# def quat_to_rot_matrix(x, y, z, w):
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


# class PersonTrack:
#     _next_id = 0

#     def __init__(self, position, conf, stamp, pix_box=None):
#         self.box_id = PersonTrack._next_id
#         PersonTrack._next_id += 1
#         self.position = position   # np.array(3), world/target frame
#         self.confidence = conf
#         self.pix_box = pix_box      # (xmin, ymin, xmax, ymax) pixels, or None
#         self.last_seen = stamp
#         self.hit_count = 1
#         self.missed_frames = 0


# class PeopleTrackerNode(Node):

#     def __init__(self):
#         super().__init__('people_tracker_node')

#         self.declare_parameter('detections_topic', '/oak/nn/spatial_detections')
#         self.declare_parameter('output_topic', '/manriix/person_boxes')
#         self.declare_parameter('target_frame', 'base_footprint')
#         self.declare_parameter('confidence_threshold', 0.5)
#         self.declare_parameter('max_assoc_dist', 0.7)   # [m] 3D association gate
#         self.declare_parameter('box_timeout', 0.7)       # [s]
#         self.declare_parameter('min_hits_to_confirm', 2)
#         # Debug overlay: draw tracked person boxes + IDs on the NN
#         # passthrough image (paper Fig. 1/5 left panel).
#         self.declare_parameter('publish_debug_image', True)
#         self.declare_parameter('image_topic', '/oak/nn/passthrough/image_raw')
#         self.declare_parameter('debug_image_topic', '/manriix/detection_image')

#         self.detections_topic = self.get_parameter('detections_topic').value
#         self.output_topic = self.get_parameter('output_topic').value
#         self.target_frame = self.get_parameter('target_frame').value
#         self.confidence_threshold = float(self.get_parameter('confidence_threshold').value)
#         self.max_assoc_dist = float(self.get_parameter('max_assoc_dist').value)
#         self.box_timeout = float(self.get_parameter('box_timeout').value)
#         self.min_hits_to_confirm = int(self.get_parameter('min_hits_to_confirm').value)

#         self.tf_buffer = tf2_ros.Buffer()
#         self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
#         self._R = None
#         self._t = None
#         self._cached_source = None

#         qos = QoSProfile(
#             reliability=ReliabilityPolicy.BEST_EFFORT,
#             history=HistoryPolicy.KEEP_LAST,
#             depth=5,
#             durability=DurabilityPolicy.VOLATILE,
#         )

#         self.create_subscription(
#             Detection3DArray, self.detections_topic, self._cb, qos)
#         # Reliable publisher: downstream fusion + debugging tools expect it.
#         self.pub = self.create_publisher(BoundingBoxTrackArray, self.output_topic, 10)

#         # ---- debug annotated image ----
#         self.publish_debug_image = bool(
#             self.get_parameter('publish_debug_image').value) and HAVE_CV
#         if bool(self.get_parameter('publish_debug_image').value) and not HAVE_CV:
#             self.get_logger().warn(
#                 'publish_debug_image requested but cv2/cv_bridge missing '
#                 '(pip install opencv-python; apt ros-humble-cv-bridge)')
#         if self.publish_debug_image:
#             self.bridge = CvBridge()
#             self.pub_img = self.create_publisher(
#                 Image, self.get_parameter('debug_image_topic').value, 5)
#             self.create_subscription(
#                 Image, self.get_parameter('image_topic').value,
#                 self._image_cb, qos)

#         self.tracks = {}

#         self.get_logger().info(
#             f'Listening on {self.detections_topic} as Detection3DArray '
#             f'(conf>={self.confidence_threshold}, gate={self.max_assoc_dist}m), '
#             f'positions output in {self.target_frame}'
#         )

#     # ------------------------------------------------------------------
#     def _update_tf(self, source_frame):
#         try:
#             tf = self.tf_buffer.lookup_transform(
#                 self.target_frame, source_frame, rclpy.time.Time())
#         except TransformException as ex:
#             self.get_logger().warn(
#                 f'TF {source_frame} -> {self.target_frame} unavailable: {ex}',
#                 throttle_duration_sec=2.0)
#             return False
#         q = tf.transform.rotation
#         t = tf.transform.translation
#         self._R = quat_to_rot_matrix(q.x, q.y, q.z, q.w)
#         self._t = np.array([t.x, t.y, t.z])
#         self._cached_source = source_frame
#         return True

#     # ------------------------------------------------------------------
#     def _cb(self, msg: Detection3DArray):
#         source_frame = msg.header.frame_id
#         if self._R is None or self._cached_source != source_frame:
#             if not self._update_tf(source_frame):
#                 return

#         now = Time.from_msg(msg.header.stamp).nanoseconds * 1e-9

#         dets = []
#         for det in msg.detections:
#             best_score = None
#             best_pos = None
#             for r in det.results:
#                 class_id = getattr(getattr(r, 'hypothesis', r), 'class_id',
#                                    getattr(r, 'id', ''))
#                 score = getattr(getattr(r, 'hypothesis', r), 'score',
#                                 getattr(r, 'score', 0.0))
#                 if str(class_id) in PERSON_LABELS and score >= self.confidence_threshold:
#                     if best_score is None or score > best_score:
#                         best_score = score
#                         # NOTE (driver quirk): in this depthai_ros_driver
#                         # version's Detection3DArray, the fused 3D position
#                         # (meters, camera optical frame) is in
#                         # results[].pose.pose.position, while
#                         # bbox.center.position holds the 2D PIXEL center.
#                         p = r.pose.pose.position
#                         best_pos = np.array([p.x, p.y, p.z])
#             if best_score is None or best_pos is None:
#                 continue
#             if np.linalg.norm(best_pos) < 1e-6:
#                 continue  # no valid depth fused for this detection
#             pos_world = self._R @ best_pos + self._t
#             # bbox.center/size are PIXELS in this driver -- perfect for
#             # the debug overlay.
#             bc, bs = det.bbox.center.position, det.bbox.size
#             pix_box = (bc.x - bs.x / 2, bc.y - bs.y / 2,
#                        bc.x + bs.x / 2, bc.y + bs.y / 2)
#             dets.append((pos_world, best_score, pix_box))

#         self._track_update(dets, now, msg)

#     # ------------------------------------------------------------------
#     def _track_update(self, dets, now, msg):
#         track_ids = list(self.tracks.keys())
#         matched_t, matched_d = set(), set()

#         pairs = []
#         for tid in track_ids:
#             for j, (pos, conf, pix_box) in enumerate(dets):
#                 d = np.linalg.norm(self.tracks[tid].position - pos)
#                 if d <= self.max_assoc_dist:
#                     pairs.append((d, tid, j))
#         pairs.sort()

#         for d, tid, j in pairs:
#             if tid in matched_t or j in matched_d:
#                 continue
#             trk = self.tracks[tid]
#             trk.position, trk.confidence, trk.pix_box = dets[j]
#             trk.last_seen = now
#             trk.hit_count += 1
#             trk.missed_frames = 0
#             matched_t.add(tid)
#             matched_d.add(j)

#         for tid in track_ids:
#             if tid not in matched_t:
#                 self.tracks[tid].missed_frames += 1
#                 self.tracks[tid].hit_count = 0

#         for j, (pos, conf, pix_box) in enumerate(dets):
#             if j not in matched_d:
#                 trk = PersonTrack(pos, conf, now, pix_box)
#                 self.tracks[trk.box_id] = trk

#         for tid in [t for t, trk in self.tracks.items()
#                     if (now - trk.last_seen) > self.box_timeout]:
#             del self.tracks[tid]

#         out = BoundingBoxTrackArray()
#         out.header = msg.header
#         out.header.frame_id = self.target_frame
#         for trk in self.tracks.values():
#             if trk.hit_count < self.min_hits_to_confirm or trk.missed_frames > 0:
#                 continue
#             b = BoundingBoxTrack()
#             b.box_id = trk.box_id
#             b.confidence = float(trk.confidence)
#             b.hit_count = int(trk.hit_count)
#             b.missed_frames = int(trk.missed_frames)
#             b.position = Point(x=float(trk.position[0]),
#                                 y=float(trk.position[1]),
#                                 z=float(trk.position[2]))
#             out.boxes.append(b)
#         self.pub.publish(out)

#     # ------------------------------------------------------------------
#     #  Debug overlay (paper Fig. 1/5 left): boxes + confidence + IDs
#     # ------------------------------------------------------------------
#     def _image_cb(self, msg: Image):
#         try:
#             img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
#         except Exception:
#             return
#         for trk in self.tracks.values():
#             if trk.pix_box is None or trk.hit_count < self.min_hits_to_confirm:
#                 continue
#             x1, y1, x2, y2 = [int(v) for v in trk.pix_box]
#             cv2.rectangle(img, (x1, y1), (x2, y2), (0, 220, 255), 2)
#             cv2.putText(img, f'person:{trk.confidence:.2f}',
#                         (x1, max(14, y1 - 6)),
#                         cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 220, 255), 1)
#             cv2.putText(img, f'ID:{trk.box_id}',
#                         (x1, min(img.shape[0] - 4, y2 + 16)),
#                         cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
#         out_img = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
#         out_img.header = msg.header
#         self.pub_img.publish(out_img)


# def main(args=None):
#     rclpy.init(args=args)
#     node = PeopleTrackerNode()
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
# people_tracker_node.py

# Implements Section III-E (2D People Detector) of Eppenberger et al.,
# IROS 2020 -- adapted to the OAK-D Pro W's on-device spatial NN.

# The MobileNet-SSD detector runs ON-DEVICE on the OAK's VPU
# (depthai_ros_driver, nn.i_nn_family: mobilenet, i_nn_type: spatial),
# so host inference cost is ~zero (the paper spent 42.4 ms/frame on this,
# Table I). Your driver version publishes the spatial detections as
# vision_msgs/Detection3DArray on /oak/nn/spatial_detections: each
# detection carries a 3D position (bbox.center.position) in the camera
# optical frame, fused on-device from the RGB-aligned depth.

# Because we get 3D positions directly, we do tracking-by-detection with
# association on 3D center distance (instead of image-plane IoU). The
# paper explicitly notes several valid alternatives for the 2D-3D
# association step; with this hardware the 3D route is simpler and makes
# Step-5 fusion (person <-> cluster) a direct nearest-centroid match.

# Publishes BoundingBoxTrackArray on /manriix/person_boxes; the box
# fields carry the 3D position (in the target/world frame after TF), and
# box pixel fields are left zero (not provided by Detection3DArray).
# """

# import numpy as np

# import rclpy
# from rclpy.node import Node
# from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
# from rclpy.time import Time

# import tf2_ros
# from tf2_ros import TransformException

# from geometry_msgs.msg import Point
# from vision_msgs.msg import Detection3DArray
# from sensor_msgs.msg import Image
# from visualization_msgs.msg import Marker, MarkerArray
# from manriix_oak_obstacles_msgs.msg import BoundingBoxTrack, BoundingBoxTrackArray

# try:
#     import cv2
#     from cv_bridge import CvBridge
#     HAVE_CV = True
# except ImportError:
#     HAVE_CV = False


# PERSON_LABELS = {'15', 'person', 'Person', 'PERSON'}


# def quat_to_rot_matrix(x, y, z, w):
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


# class PersonTrack:
#     _next_id = 0

#     def __init__(self, position, conf, stamp, pix_box=None):
#         self.box_id = PersonTrack._next_id
#         PersonTrack._next_id += 1
#         self.position = position   # np.array(3), world/target frame
#         self.confidence = conf
#         self.pix_box = pix_box      # (xmin, ymin, xmax, ymax) pixels, or None
#         self.last_seen = stamp
#         self.hit_count = 1
#         self.missed_frames = 0


# class PeopleTrackerNode(Node):

#     def __init__(self):
#         super().__init__('people_tracker_node')

#         self.declare_parameter('detections_topic', '/oak/nn/spatial_detections')
#         self.declare_parameter('output_topic', '/manriix/person_boxes')
#         self.declare_parameter('target_frame', 'base_footprint')
#         self.declare_parameter('confidence_threshold', 0.5)
#         self.declare_parameter('max_assoc_dist', 0.7)   # [m] 3D association gate
#         self.declare_parameter('box_timeout', 0.7)       # [s]
#         self.declare_parameter('min_hits_to_confirm', 2)
#         # Debug overlay: draw tracked person boxes + IDs on the NN
#         # passthrough image (paper Fig. 1/5 left panel).
#         self.declare_parameter('publish_debug_image', True)
#         self.declare_parameter('image_topic', '/oak/nn/passthrough/image_raw')
#         self.declare_parameter('debug_image_topic', '/manriix/detection_image')
#         # DepthAI spatial coordinates use Y-UP while the message frame_id
#         # claims the optical convention (Y-DOWN). Symptom of the mismatch:
#         # person world z near 0 instead of torso height, and 3D person
#         # positions mirrored left/right of the true location. Default
#         # corrects Y; verify with the axis test in the tuning guide.
#         self.declare_parameter('spatial_sign_x', 1.0)
#         self.declare_parameter('spatial_sign_y', -1.0)
#         self.declare_parameter('spatial_sign_z', 1.0)

#         self.detections_topic = self.get_parameter('detections_topic').value
#         self.output_topic = self.get_parameter('output_topic').value
#         self.target_frame = self.get_parameter('target_frame').value
#         self.confidence_threshold = float(self.get_parameter('confidence_threshold').value)
#         self.max_assoc_dist = float(self.get_parameter('max_assoc_dist').value)
#         self.box_timeout = float(self.get_parameter('box_timeout').value)
#         self.min_hits_to_confirm = int(self.get_parameter('min_hits_to_confirm').value)
#         self.spatial_signs = np.array([
#             float(self.get_parameter('spatial_sign_x').value),
#             float(self.get_parameter('spatial_sign_y').value),
#             float(self.get_parameter('spatial_sign_z').value),
#         ])

#         self.tf_buffer = tf2_ros.Buffer()
#         self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
#         self._R = None
#         self._t = None
#         self._cached_source = None

#         qos = QoSProfile(
#             reliability=ReliabilityPolicy.BEST_EFFORT,
#             history=HistoryPolicy.KEEP_LAST,
#             depth=5,
#             durability=DurabilityPolicy.VOLATILE,
#         )

#         self.create_subscription(
#             Detection3DArray, self.detections_topic, self._cb, qos)
#         # Reliable publisher: downstream fusion + debugging tools expect it.
#         self.pub = self.create_publisher(BoundingBoxTrackArray, self.output_topic, 10)

#         # ---- debug annotated image ----
#         self.publish_debug_image = bool(
#             self.get_parameter('publish_debug_image').value) and HAVE_CV
#         if bool(self.get_parameter('publish_debug_image').value) and not HAVE_CV:
#             self.get_logger().warn(
#                 'publish_debug_image requested but cv2/cv_bridge missing '
#                 '(pip install opencv-python; apt ros-humble-cv-bridge)')
#         if self.publish_debug_image:
#             self.bridge = CvBridge()
#             self.pub_img = self.create_publisher(
#                 Image, self.get_parameter('debug_image_topic').value, 5)
#             self.create_subscription(
#                 Image, self.get_parameter('image_topic').value,
#                 self._image_cb, qos)

#         # DIAGNOSTIC: purple spheres at each tracked person's 3D world
#         # position. In RViz these must sit ON TOP of the human's cluster
#         # in /manriix/points_labeled -- if they're offset (mirrored,
#         # shifted, floating), the 3D geometry chain (spatial-detection
#         # axes / TF extrinsic / intrinsics) is wrong, and no amount of
#         # downstream tuning will fix classification or fusion.
#         self.pub_person_markers = self.create_publisher(
#             MarkerArray, '/manriix/person_markers', 10)

#         self.tracks = {}

#         self.get_logger().info(
#             f'Listening on {self.detections_topic} as Detection3DArray '
#             f'(conf>={self.confidence_threshold}, gate={self.max_assoc_dist}m), '
#             f'positions output in {self.target_frame}'
#         )

#     # ------------------------------------------------------------------
#     def _update_tf(self, source_frame):
#         try:
#             tf = self.tf_buffer.lookup_transform(
#                 self.target_frame, source_frame, rclpy.time.Time())
#         except TransformException as ex:
#             self.get_logger().warn(
#                 f'TF {source_frame} -> {self.target_frame} unavailable: {ex}',
#                 throttle_duration_sec=2.0)
#             return False
#         q = tf.transform.rotation
#         t = tf.transform.translation
#         self._R = quat_to_rot_matrix(q.x, q.y, q.z, q.w)
#         self._t = np.array([t.x, t.y, t.z])
#         self._cached_source = source_frame
#         return True

#     # ------------------------------------------------------------------
#     def _cb(self, msg: Detection3DArray):
#         source_frame = msg.header.frame_id
#         if self._R is None or self._cached_source != source_frame:
#             if not self._update_tf(source_frame):
#                 return

#         now = Time.from_msg(msg.header.stamp).nanoseconds * 1e-9

#         dets = []
#         for det in msg.detections:
#             best_score = None
#             best_pos = None
#             for r in det.results:
#                 class_id = getattr(getattr(r, 'hypothesis', r), 'class_id',
#                                    getattr(r, 'id', ''))
#                 score = getattr(getattr(r, 'hypothesis', r), 'score',
#                                 getattr(r, 'score', 0.0))
#                 if str(class_id) in PERSON_LABELS and score >= self.confidence_threshold:
#                     if best_score is None or score > best_score:
#                         best_score = score
#                         # NOTE (driver quirk): in this depthai_ros_driver
#                         # version's Detection3DArray, the fused 3D position
#                         # (meters, camera optical frame) is in
#                         # results[].pose.pose.position, while
#                         # bbox.center.position holds the 2D PIXEL center.
#                         p = r.pose.pose.position
#                         best_pos = np.array([p.x, p.y, p.z])
#             if best_score is None or best_pos is None:
#                 continue
#             if np.linalg.norm(best_pos) < 1e-6:
#                 continue  # no valid depth fused for this detection
#             best_pos = best_pos * self.spatial_signs  # convention fix
#             pos_world = self._R @ best_pos + self._t
#             # bbox.center/size are PIXELS in this driver -- perfect for
#             # the debug overlay.
#             bc, bs = det.bbox.center.position, det.bbox.size
#             pix_box = (bc.x - bs.x / 2, bc.y - bs.y / 2,
#                        bc.x + bs.x / 2, bc.y + bs.y / 2)
#             dets.append((pos_world, best_score, pix_box))

#         self._track_update(dets, now, msg)

#     # ------------------------------------------------------------------
#     def _track_update(self, dets, now, msg):
#         track_ids = list(self.tracks.keys())
#         matched_t, matched_d = set(), set()

#         pairs = []
#         for tid in track_ids:
#             for j, (pos, conf, pix_box) in enumerate(dets):
#                 d = np.linalg.norm(self.tracks[tid].position - pos)
#                 if d <= self.max_assoc_dist:
#                     pairs.append((d, tid, j))
#         pairs.sort()

#         for d, tid, j in pairs:
#             if tid in matched_t or j in matched_d:
#                 continue
#             trk = self.tracks[tid]
#             trk.position, trk.confidence, trk.pix_box = dets[j]
#             trk.last_seen = now
#             trk.hit_count += 1
#             trk.missed_frames = 0
#             matched_t.add(tid)
#             matched_d.add(j)

#         for tid in track_ids:
#             if tid not in matched_t:
#                 self.tracks[tid].missed_frames += 1
#                 self.tracks[tid].hit_count = 0

#         for j, (pos, conf, pix_box) in enumerate(dets):
#             if j not in matched_d:
#                 trk = PersonTrack(pos, conf, now, pix_box)
#                 self.tracks[trk.box_id] = trk

#         for tid in [t for t, trk in self.tracks.items()
#                     if (now - trk.last_seen) > self.box_timeout]:
#             del self.tracks[tid]

#         out = BoundingBoxTrackArray()
#         out.header = msg.header
#         out.header.frame_id = self.target_frame
#         for trk in self.tracks.values():
#             if trk.hit_count < self.min_hits_to_confirm or trk.missed_frames > 0:
#                 continue
#             b = BoundingBoxTrack()
#             b.box_id = trk.box_id
#             b.confidence = float(trk.confidence)
#             b.hit_count = int(trk.hit_count)
#             b.missed_frames = int(trk.missed_frames)
#             b.position = Point(x=float(trk.position[0]),
#                                 y=float(trk.position[1]),
#                                 z=float(trk.position[2]))
#             out.boxes.append(b)
#         self.pub.publish(out)

#         markers = MarkerArray()
#         mid = 0
#         for b in out.boxes:
#             m = Marker()
#             m.header = out.header
#             m.ns = 'person_pos'
#             m.id = mid
#             mid += 1
#             m.type = Marker.SPHERE
#             m.action = Marker.ADD
#             m.pose.position = b.position
#             m.pose.orientation.w = 1.0
#             m.scale.x = m.scale.y = m.scale.z = 0.25
#             m.color.r, m.color.g, m.color.b, m.color.a = 0.8, 0.2, 1.0, 0.9
#             m.lifetime = rclpy.duration.Duration(seconds=0.4).to_msg()
#             markers.markers.append(m)
#             t = Marker()
#             t.header = out.header
#             t.ns = 'person_id'
#             t.id = mid
#             mid += 1
#             t.type = Marker.TEXT_VIEW_FACING
#             t.action = Marker.ADD
#             t.pose.position.x = b.position.x
#             t.pose.position.y = b.position.y
#             t.pose.position.z = b.position.z + 0.3
#             t.pose.orientation.w = 1.0
#             t.scale.z = 0.18
#             t.color.r = t.color.g = t.color.b = t.color.a = 1.0
#             t.text = f'P{b.box_id}'
#             t.lifetime = rclpy.duration.Duration(seconds=0.4).to_msg()
#             markers.markers.append(t)
#         self.pub_person_markers.publish(markers)

#     # ------------------------------------------------------------------
#     #  Debug overlay (paper Fig. 1/5 left): boxes + confidence + IDs
#     # ------------------------------------------------------------------
#     def _image_cb(self, msg: Image):
#         try:
#             img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
#         except Exception:
#             return
#         for trk in self.tracks.values():
#             if trk.pix_box is None or trk.hit_count < self.min_hits_to_confirm:
#                 continue
#             x1, y1, x2, y2 = [int(v) for v in trk.pix_box]
#             cv2.rectangle(img, (x1, y1), (x2, y2), (0, 220, 255), 2)
#             cv2.putText(img, f'person:{trk.confidence:.2f}',
#                         (x1, max(14, y1 - 6)),
#                         cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 220, 255), 1)
#             cv2.putText(img, f'ID:{trk.box_id}',
#                         (x1, min(img.shape[0] - 4, y2 + 16)),
#                         cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
#         out_img = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
#         out_img.header = msg.header
#         self.pub_img.publish(out_img)


# def main(args=None):
#     rclpy.init(args=args)
#     node = PeopleTrackerNode()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()


# if __name__ == '__main__':
#     main()

#!/usr/bin/env python3
"""
people_tracker_node.py

Implements Section III-E (2D People Detector) of Eppenberger et al.,
IROS 2020 -- adapted to the OAK-D Pro W's on-device spatial NN.

The MobileNet-SSD detector runs ON-DEVICE on the OAK's VPU
(depthai_ros_driver, nn.i_nn_family: mobilenet, i_nn_type: spatial),
so host inference cost is ~zero (the paper spent 42.4 ms/frame on this,
Table I). Your driver version publishes the spatial detections as
vision_msgs/Detection3DArray on /oak/nn/spatial_detections: each
detection carries a 3D position (bbox.center.position) in the camera
optical frame, fused on-device from the RGB-aligned depth.

Because we get 3D positions directly, we do tracking-by-detection with
association on 3D center distance (instead of image-plane IoU). The
paper explicitly notes several valid alternatives for the 2D-3D
association step; with this hardware the 3D route is simpler and makes
Step-5 fusion (person <-> cluster) a direct nearest-centroid match.

Publishes BoundingBoxTrackArray on /manriix/person_boxes; the box
fields carry the 3D position (in the target/world frame after TF), and
box pixel fields are left zero (not provided by Detection3DArray).
"""

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.time import Time

import tf2_ros
from tf2_ros import TransformException

from geometry_msgs.msg import Point
from vision_msgs.msg import Detection3DArray
from sensor_msgs.msg import Image
from visualization_msgs.msg import Marker, MarkerArray
from manriix_oak_obstacles_msgs.msg import BoundingBoxTrack, BoundingBoxTrackArray

try:
    import cv2
    from cv_bridge import CvBridge
    HAVE_CV = True
except ImportError:
    HAVE_CV = False


PERSON_LABELS = {'15', 'person', 'Person', 'PERSON'}


def quat_to_rot_matrix(x, y, z, w):
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


class PersonTrack:
    _next_id = 0

    def __init__(self, position, conf, stamp, pix_box=None):
        self.box_id = PersonTrack._next_id
        PersonTrack._next_id += 1
        self.position = position   # np.array(3), world/target frame
        self.confidence = conf
        self.pix_box = pix_box      # (xmin, ymin, xmax, ymax) pixels, or None
        self.last_seen = stamp
        self.hit_count = 1
        self.missed_frames = 0


class PeopleTrackerNode(Node):

    def __init__(self):
        super().__init__('people_tracker_node')

        self.declare_parameter('detections_topic', '/oak/nn/spatial_detections')
        self.declare_parameter('output_topic', '/manriix/person_boxes')
        self.declare_parameter('target_frame', 'base_footprint')
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('max_assoc_dist', 0.7)   # [m] 3D association gate
        self.declare_parameter('box_timeout', 0.7)       # [s]
        self.declare_parameter('min_hits_to_confirm', 2)
        # Debug overlay: draw tracked person boxes + IDs on the NN
        # passthrough image (paper Fig. 1/5 left panel).
        self.declare_parameter('publish_debug_image', True)
        self.declare_parameter('image_topic', '/oak/nn/passthrough/image_raw')
        self.declare_parameter('debug_image_topic', '/manriix/detection_image')
        # DepthAI spatial coordinates use Y-UP while the message frame_id
        # claims the optical convention (Y-DOWN). Symptom of the mismatch:
        # person world z near 0 instead of torso height, and 3D person
        # positions mirrored left/right of the true location. Default
        # corrects Y; verify with the axis test in the tuning guide.
        self.declare_parameter('spatial_sign_x', 1.0)
        self.declare_parameter('spatial_sign_y', -1.0)
        self.declare_parameter('spatial_sign_z', 1.0)
        # The OAK's spatial ROI averaging can grab BACKGROUND depth
        # through/around a side-on or moving person, reporting them at
        # e.g. 6.5m on the correct bearing. Clusters only exist within
        # the filter's depth trust limit anyway, so detections beyond
        # this range are ghosts -- drop them.
        self.declare_parameter('max_person_range', 3.5)   # [m], camera-frame range

        self.detections_topic = self.get_parameter('detections_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.target_frame = self.get_parameter('target_frame').value
        self.confidence_threshold = float(self.get_parameter('confidence_threshold').value)
        self.max_assoc_dist = float(self.get_parameter('max_assoc_dist').value)
        self.box_timeout = float(self.get_parameter('box_timeout').value)
        self.min_hits_to_confirm = int(self.get_parameter('min_hits_to_confirm').value)
        self.spatial_signs = np.array([
            float(self.get_parameter('spatial_sign_x').value),
            float(self.get_parameter('spatial_sign_y').value),
            float(self.get_parameter('spatial_sign_z').value),
        ])
        self.max_person_range = float(self.get_parameter('max_person_range').value)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self._R = None
        self._t = None
        self._cached_source = None

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.create_subscription(
            Detection3DArray, self.detections_topic, self._cb, qos)
        # Reliable publisher: downstream fusion + debugging tools expect it.
        self.pub = self.create_publisher(BoundingBoxTrackArray, self.output_topic, 10)

        # ---- debug annotated image ----
        self.publish_debug_image = bool(
            self.get_parameter('publish_debug_image').value) and HAVE_CV
        if bool(self.get_parameter('publish_debug_image').value) and not HAVE_CV:
            self.get_logger().warn(
                'publish_debug_image requested but cv2/cv_bridge missing '
                '(pip install opencv-python; apt ros-humble-cv-bridge)')
        if self.publish_debug_image:
            self.bridge = CvBridge()
            self.pub_img = self.create_publisher(
                Image, self.get_parameter('debug_image_topic').value, 5)
            self.create_subscription(
                Image, self.get_parameter('image_topic').value,
                self._image_cb, qos)

        # DIAGNOSTIC: purple spheres at each tracked person's 3D world
        # position. In RViz these must sit ON TOP of the human's cluster
        # in /manriix/points_labeled -- if they're offset (mirrored,
        # shifted, floating), the 3D geometry chain (spatial-detection
        # axes / TF extrinsic / intrinsics) is wrong, and no amount of
        # downstream tuning will fix classification or fusion.
        self.pub_person_markers = self.create_publisher(
            MarkerArray, '/manriix/person_markers', 10)

        self.tracks = {}

        self.get_logger().info(
            f'Listening on {self.detections_topic} as Detection3DArray '
            f'(conf>={self.confidence_threshold}, gate={self.max_assoc_dist}m), '
            f'positions output in {self.target_frame}'
        )

    # ------------------------------------------------------------------
    def _update_tf(self, source_frame):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.target_frame, source_frame, rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().warn(
                f'TF {source_frame} -> {self.target_frame} unavailable: {ex}',
                throttle_duration_sec=2.0)
            return False
        q = tf.transform.rotation
        t = tf.transform.translation
        self._R = quat_to_rot_matrix(q.x, q.y, q.z, q.w)
        self._t = np.array([t.x, t.y, t.z])
        self._cached_source = source_frame
        return True

    # ------------------------------------------------------------------
    def _cb(self, msg: Detection3DArray):
        source_frame = msg.header.frame_id
        if self._R is None or self._cached_source != source_frame:
            if not self._update_tf(source_frame):
                return

        now = Time.from_msg(msg.header.stamp).nanoseconds * 1e-9

        dets = []
        for det in msg.detections:
            best_score = None
            best_pos = None
            for r in det.results:
                class_id = getattr(getattr(r, 'hypothesis', r), 'class_id',
                                   getattr(r, 'id', ''))
                score = getattr(getattr(r, 'hypothesis', r), 'score',
                                getattr(r, 'score', 0.0))
                if str(class_id) in PERSON_LABELS and score >= self.confidence_threshold:
                    if best_score is None or score > best_score:
                        best_score = score
                        # NOTE (driver quirk): in this depthai_ros_driver
                        # version's Detection3DArray, the fused 3D position
                        # (meters, camera optical frame) is in
                        # results[].pose.pose.position, while
                        # bbox.center.position holds the 2D PIXEL center.
                        p = r.pose.pose.position
                        best_pos = np.array([p.x, p.y, p.z])
            if best_score is None or best_pos is None:
                continue
            if np.linalg.norm(best_pos) < 1e-6:
                continue  # no valid depth fused for this detection
            best_pos = best_pos * self.spatial_signs  # convention fix
            pos_world = self._R @ best_pos + self._t
            # bbox.center/size are PIXELS in this driver -- perfect for
            # the debug overlay.
            bc, bs = det.bbox.center.position, det.bbox.size
            pix_box = (bc.x - bs.x / 2, bc.y - bs.y / 2,
                       bc.x + bs.x / 2, bc.y + bs.y / 2)
            dets.append((pos_world, best_score, pix_box))

        self._track_update(dets, now, msg)

    # ------------------------------------------------------------------
    def _track_update(self, dets, now, msg):
        track_ids = list(self.tracks.keys())
        matched_t, matched_d = set(), set()

        pairs = []
        for tid in track_ids:
            for j, (pos, conf, pix_box) in enumerate(dets):
                d = np.linalg.norm(self.tracks[tid].position - pos)
                if d <= self.max_assoc_dist:
                    pairs.append((d, tid, j))
        pairs.sort()

        for d, tid, j in pairs:
            if tid in matched_t or j in matched_d:
                continue
            trk = self.tracks[tid]
            trk.position, trk.confidence, trk.pix_box = dets[j]
            trk.last_seen = now
            trk.hit_count += 1
            trk.missed_frames = 0
            matched_t.add(tid)
            matched_d.add(j)

        for tid in track_ids:
            if tid not in matched_t:
                self.tracks[tid].missed_frames += 1
                self.tracks[tid].hit_count = 0

        for j, (pos, conf, pix_box) in enumerate(dets):
            if j not in matched_d:
                trk = PersonTrack(pos, conf, now, pix_box)
                self.tracks[trk.box_id] = trk

        for tid in [t for t, trk in self.tracks.items()
                    if (now - trk.last_seen) > self.box_timeout]:
            del self.tracks[tid]

        out = BoundingBoxTrackArray()
        out.header = msg.header
        out.header.frame_id = self.target_frame
        for trk in self.tracks.values():
            if trk.hit_count < self.min_hits_to_confirm or trk.missed_frames > 0:
                continue
            b = BoundingBoxTrack()
            b.box_id = trk.box_id
            b.confidence = float(trk.confidence)
            b.hit_count = int(trk.hit_count)
            b.missed_frames = int(trk.missed_frames)
            b.position = Point(x=float(trk.position[0]),
                                y=float(trk.position[1]),
                                z=float(trk.position[2]))
            out.boxes.append(b)
        self.pub.publish(out)

        markers = MarkerArray()
        mid = 0
        for b in out.boxes:
            m = Marker()
            m.header = out.header
            m.ns = 'person_pos'
            m.id = mid
            mid += 1
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position = b.position
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 0.25
            m.color.r, m.color.g, m.color.b, m.color.a = 0.8, 0.2, 1.0, 0.9
            m.lifetime = rclpy.duration.Duration(seconds=0.4).to_msg()
            markers.markers.append(m)
            t = Marker()
            t.header = out.header
            t.ns = 'person_id'
            t.id = mid
            mid += 1
            t.type = Marker.TEXT_VIEW_FACING
            t.action = Marker.ADD
            t.pose.position.x = b.position.x
            t.pose.position.y = b.position.y
            t.pose.position.z = b.position.z + 0.3
            t.pose.orientation.w = 1.0
            t.scale.z = 0.18
            t.color.r = t.color.g = t.color.b = t.color.a = 1.0
            t.text = f'P{b.box_id}'
            t.lifetime = rclpy.duration.Duration(seconds=0.4).to_msg()
            markers.markers.append(t)
        self.pub_person_markers.publish(markers)

    # ------------------------------------------------------------------
    #  Debug overlay (paper Fig. 1/5 left): boxes + confidence + IDs
    # ------------------------------------------------------------------
    def _image_cb(self, msg: Image):
        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception:
            return
        for trk in self.tracks.values():
            if trk.pix_box is None or trk.hit_count < self.min_hits_to_confirm:
                continue
            x1, y1, x2, y2 = [int(v) for v in trk.pix_box]
            cv2.rectangle(img, (x1, y1), (x2, y2), (0, 220, 255), 2)
            cv2.putText(img, f'person:{trk.confidence:.2f}',
                        (x1, max(14, y1 - 6)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 220, 255), 1)
            cv2.putText(img, f'ID:{trk.box_id}',
                        (x1, min(img.shape[0] - 4, y2 + 16)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        out_img = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
        out_img.header = msg.header
        self.pub_img.publish(out_img)


def main(args=None):
    rclpy.init(args=args)
    node = PeopleTrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()