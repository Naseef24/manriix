#!/usr/bin/env python3
"""
check_alignment.py — live diagnostic for person<->cluster fusion.

Prints, once per second:
  person box (x, y)  |  nearest cluster centroid (x, y)  |  xy-distance
  |  inside fusion gate?  |  that cluster's label + dynamic_vote_ratio

Interpretation:
  - y sign DISAGREES between box and your cluster  -> lateral mirror:
        set people_tracker_node spatial_sign_x: -1.0
  - y agrees but distance > gate (0.7m)            -> depth/scale issue
  - distance < gate but label never 'person'       -> fusion/gamma issue
  - label 'static' while you WALK: watch vote_ratio -- that's the number
        l_rel_dyn must sit below.

Run (with workspace sourced, pipeline running):
    python3 check_alignment.py
"""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from manriix_oak_obstacles_msgs.msg import ClusterTrackArray, BoundingBoxTrackArray


class Checker(Node):
    def __init__(self):
        super().__init__('alignment_checker')
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=5)
        self.create_subscription(BoundingBoxTrackArray,
                                 '/manriix/person_boxes', self.box_cb, 10)
        self.create_subscription(ClusterTrackArray,
                                 '/manriix/cluster_tracks_fused', self.trk_cb, qos)
        self.boxes = []
        self.tracks = []
        self.create_timer(1.0, self.report)
        print(f'{"person(x,y)":>16} | {"cluster(x,y)":>16} | {"dist":>5} '
              f'| gate | {"label":>9} | vote')

    def box_cb(self, msg):
        self.boxes = [(b.box_id, np.array([b.position.x, b.position.y]))
                      for b in msg.boxes]

    def trk_cb(self, msg):
        self.tracks = [(t.track_id,
                        np.array([t.centroid.x, t.centroid.y]),
                        t.classification, t.dynamic_vote_ratio,
                        t.num_points) for t in msg.tracks]

    def report(self):
        if not self.boxes:
            print('-- no person box --')
            return
        if not self.tracks:
            print('-- no clusters --')
            return
        for bid, bxy in self.boxes:
            tid, cxy, label, ratio, npts = min(
                self.tracks, key=lambda t: np.linalg.norm(t[1] - bxy))
            d = np.linalg.norm(cxy - bxy)
            print(f'({bxy[0]:5.2f},{bxy[1]:6.2f})   | '
                  f'({cxy[0]:5.2f},{cxy[1]:6.2f})   | {d:5.2f} | '
                  f'{"YES" if d <= 0.7 else " no"} | {label:>9} | '
                  f'{ratio:.2f} ({npts}p)')


def main():
    rclpy.init()
    n = Checker()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == '__main__':
    main()