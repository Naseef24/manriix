#!/usr/bin/env python3
"""
people_tracker_node.py  (v2 -- pixel-space only, no on-device 3D position)

Implements Section III-E (2D People Detector) of Eppenberger et al.,
IROS 2020, adapted for the OAK-D Pro W's on-device spatial NN.

WHY THIS VERSION EXISTS:
  DepthAI's on-device spatial-detection 3D position for a person proved
  unreliable in testing (verified via a tape-measured calibration check):
  reported distances swung from ~1m true distance to 7-9m depending on
  pose, likely due to the on-device spatial calculator averaging depth
  across the full rectangular bounding box (including background visible
  around/through gaps in the person's silhouette), compounded by the
  detector's NN input being a non-uniformly squashed full-FOV image
  (fix for the earlier center-crop defect) whose depth-frame ROI mapping
  may not hold cleanly across a differently-shaped depth stream.

  The 2D pixel bounding box itself was independently confirmed accurate
  (visually verified via /manriix/detection_image: a tight, correctly
  placed box). So: this node now does ONLY 2D pixel-space detection and
  tracking-by-detection (matching the paper's own scope for this
  module -- Fig. 5 shows exactly this: 2D boxes + IDs on the image
  plane). It does NOT attempt to derive or track any 3D position.

  3D position for a person now comes from fusion_node (Sec III-F),
  which associates each 2D box to the 3D cluster with the most
  projected points landing inside it -- the paper's own documented
  method -- and uses THAT cluster's already-verified-accurate centroid
  as the person's position. This fixes the depth problem at its root
  instead of patching around DepthAI's number.

Publishes BoundingBoxTrackArray with REAL pixel xmin/ymin/xmax/ymax
(previously left at 0.0 in the 3D-position-based version). The
message's `position` field is left at (0,0,0) -- vestigial, not
authoritative; consumers must get 3D position from the fused
ClusterTrack's centroid instead.
"""

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.time import Time

from sensor_msgs.msg import Image
from vision_msgs.msg import Detection3DArray
from manriix_oak_obstacles_msgs.msg import BoundingBoxTrack, BoundingBoxTrackArray

try:
    import cv2
    from cv_bridge import CvBridge
    HAVE_CV = True
except ImportError:
    HAVE_CV = False


PERSON_LABELS = {'15', 'person', 'Person', 'PERSON'}


def iou(box_a, box_b):
    """box = (xmin, ymin, xmax, ymax), pixel space."""
    ax1, ay1, ax2, ay2 = box_a
    bx1, by1, bx2, by2 = box_b
    ix1, iy1 = max(ax1, bx1), max(ay1, by1)
    ix2, iy2 = min(ax2, bx2), min(ay2, by2)
    iw, ih = max(0.0, ix2 - ix1), max(0.0, iy2 - iy1)
    inter = iw * ih
    if inter <= 0.0:
        return 0.0
    area_a = max(1e-6, (ax2 - ax1) * (ay2 - ay1))
    area_b = max(1e-6, (bx2 - bx1) * (by2 - by1))
    return inter / (area_a + area_b - inter)


class PersonTrack:
    _next_id = 0

    def __init__(self, box, conf, stamp):
        self.box_id = PersonTrack._next_id
        PersonTrack._next_id += 1
        self.box = box  # (xmin, ymin, xmax, ymax) pixels
        self.confidence = conf
        self.last_seen = stamp
        self.hit_count = 1
        self.missed_frames = 0


class PeopleTrackerNode(Node):

    def __init__(self):
        super().__init__('people_tracker_node')

        self.declare_parameter('detections_topic', '/oak/nn/spatial_detections')
        self.declare_parameter('output_topic', '/manriix/person_boxes')
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('iou_threshold', 0.3)   # min IoU to associate frame-to-frame
        self.declare_parameter('box_timeout', 0.7)      # [s]
        self.declare_parameter('min_hits_to_confirm', 2)
        self.declare_parameter('publish_debug_image', True)
        self.declare_parameter('image_topic', '/oak/nn/passthrough/image_raw')
        self.declare_parameter('debug_image_topic', '/manriix/detection_image')

        self.detections_topic = self.get_parameter('detections_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.confidence_threshold = float(self.get_parameter('confidence_threshold').value)
        self.iou_threshold = float(self.get_parameter('iou_threshold').value)
        self.box_timeout = float(self.get_parameter('box_timeout').value)
        self.min_hits_to_confirm = int(self.get_parameter('min_hits_to_confirm').value)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.create_subscription(
            Detection3DArray, self.detections_topic, self._cb, qos)
        # Reliable: fusion_node and debugging tools expect it.
        self.pub = self.create_publisher(BoundingBoxTrackArray, self.output_topic, 10)

        self.publish_debug_image = bool(
            self.get_parameter('publish_debug_image').value) and HAVE_CV
        if bool(self.get_parameter('publish_debug_image').value) and not HAVE_CV:
            self.get_logger().warn(
                'publish_debug_image requested but cv2/cv_bridge missing')
        if self.publish_debug_image:
            self.bridge = CvBridge()
            self.pub_img = self.create_publisher(
                Image, self.get_parameter('debug_image_topic').value, 5)
            self.create_subscription(
                Image, self.get_parameter('image_topic').value,
                self._image_cb, qos)

        self.tracks = {}  # box_id -> PersonTrack

        self.get_logger().info(
            f'Listening on {self.detections_topic} | pixel-space IoU tracking only, '
            f'NO on-device 3D position used (see module docstring)'
        )

    # ------------------------------------------------------------------
    def _cb(self, msg: Detection3DArray):
        now = Time.from_msg(msg.header.stamp).nanoseconds * 1e-9

        dets = []
        for det in msg.detections:
            best_score = None
            for r in det.results:
                class_id = getattr(getattr(r, 'hypothesis', r), 'class_id',
                                   getattr(r, 'id', ''))
                score = getattr(getattr(r, 'hypothesis', r), 'score',
                                getattr(r, 'score', 0.0))
                if str(class_id) in PERSON_LABELS and score >= self.confidence_threshold:
                    if best_score is None or score > best_score:
                        best_score = score
            if best_score is None:
                continue
            # bbox.center/size are PIXELS in this driver (confirmed via
            # visual check against /manriix/detection_image). This is
            # the ONLY geometry we trust from this message.
            bc, bs = det.bbox.center.position, det.bbox.size
            box = (bc.x - bs.x / 2, bc.y - bs.y / 2,
                   bc.x + bs.x / 2, bc.y + bs.y / 2)
            dets.append((box, best_score))

        self._track_update(dets, now, msg)

    # ------------------------------------------------------------------
    def _track_update(self, dets, now, msg):
        track_ids = list(self.tracks.keys())
        matched_t, matched_d = set(), set()

        # Greedy highest-IoU-first matching (small N per frame -- fine).
        pairs = []
        for tid in track_ids:
            for j, (box, conf) in enumerate(dets):
                v = iou(self.tracks[tid].box, box)
                if v >= self.iou_threshold:
                    pairs.append((v, tid, j))
        pairs.sort(reverse=True)

        for v, tid, j in pairs:
            if tid in matched_t or j in matched_d:
                continue
            trk = self.tracks[tid]
            trk.box, trk.confidence = dets[j]
            trk.last_seen = now
            trk.hit_count += 1
            trk.missed_frames = 0
            matched_t.add(tid)
            matched_d.add(j)

        for tid in track_ids:
            if tid not in matched_t:
                self.tracks[tid].missed_frames += 1
                self.tracks[tid].hit_count = 0

        for j, (box, conf) in enumerate(dets):
            if j not in matched_d:
                trk = PersonTrack(box, conf, now)
                self.tracks[trk.box_id] = trk

        for tid in [t for t, trk in self.tracks.items()
                    if (now - trk.last_seen) > self.box_timeout]:
            del self.tracks[tid]

        # Publish in the CAMERA frame -- these are pixel-space boxes,
        # not world positions. fusion_node reads header.frame_id
        # dynamically to know which camera frame to project into.
        out = BoundingBoxTrackArray()
        out.header = msg.header
        for trk in self.tracks.values():
            if trk.hit_count < self.min_hits_to_confirm or trk.missed_frames > 0:
                continue
            b = BoundingBoxTrack()
            b.box_id = trk.box_id
            b.xmin, b.ymin, b.xmax, b.ymax = [float(v) for v in trk.box]
            b.confidence = float(trk.confidence)
            b.hit_count = int(trk.hit_count)
            b.missed_frames = int(trk.missed_frames)
            # position left at (0,0,0): NOT authoritative in this
            # version. fusion_node derives real 3D position from the
            # associated cluster's centroid instead.
            out.boxes.append(b)
        self.pub.publish(out)

    # ------------------------------------------------------------------
    #  Debug overlay (paper Fig. 1/5 left): boxes + confidence + IDs
    # ------------------------------------------------------------------
    def _image_cb(self, msg: Image):
        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception:
            return
        for trk in self.tracks.values():
            if trk.hit_count < self.min_hits_to_confirm:
                continue
            x1, y1, x2, y2 = [int(v) for v in trk.box]
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