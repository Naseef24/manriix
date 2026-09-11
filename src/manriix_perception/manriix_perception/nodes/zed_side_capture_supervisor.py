#!/usr/bin/env python3
import json, signal, subprocess, time
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String

class SideCaptureSupervisor(Node):
    def __init__(self):
        super().__init__("zed_side_capture_supervisor")
        self.last={"front":None,"left":None,"right":None}; self.odom=None
        self.front_since=None; self.left_since=None
        self.children={"left":None,"right":None}; self.started={"left":False,"right":False}
        qos=QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                       durability=DurabilityPolicy.VOLATILE,
                       history=HistoryPolicy.KEEP_LAST, depth=1)
        for c in ("front","left","right"):
            self.create_subscription(Image,f"/monitor/zedx_{c}/image_raw",
                                     lambda m,cam=c:self.stream(cam),qos)
        self.create_subscription(Odometry,"/visual_slam/tracking/odometry",self.odom_cb,qos)
        self.pub=self.create_publisher(String,"/manriix/zed_side_capture_status",10)
        self.create_timer(0.5,self.tick)
    def stream(self,c): self.last[c]=time.monotonic()
    def odom_cb(self,m): self.odom=time.monotonic()
    def fresh(self,t): return t is not None and time.monotonic()-t <= 2.0
    def cmd(self,c):
        return ["ros2","run","manriix_perception","direct_zed_detection_node_modified.py",
        "--ros-args","-r",f"__node:=zedx_{c}_capture_driver",
        "-p","mode:=nav","-p",f"enabled_cameras:=zedx_{c}",
        "-p","camera_front_serial:=41911351","-p","camera_left_serial:=40487925",
        "-p","camera_right_serial:=46311875","-p","resolution:=SVGA","-p","fps:=15",
        "-p","front_depth_mode:=NEURAL_LIGHT","-p","left_depth_mode:=NONE",
        "-p","right_depth_mode:=NONE","-p","publish_rate:=15.0",
        "-p","publish_pointcloud:=False","-p","publish_compressed_monitoring:=False",
        "-p","publish_detection_images:=False","-p","publish_monitoring_thumbnails:=True",
        "-p","monitoring_publish_divisor:=3","-p","monitoring_width:=640"]
    def spawn(self,c):
        self.get_logger().warning(f"Starting lightweight {c.upper()} capture (NAV/no depth/no OD)")
        self.children[c]=subprocess.Popen(self.cmd(c),stdout=None,stderr=None,start_new_session=False)
        self.started[c]=True
    def tick(self):
        now=time.monotonic(); front=self.fresh(self.last["front"]) and self.fresh(self.odom)
        if front:
            if self.front_since is None:self.front_since=now
        else:self.front_since=None
        if not self.started["left"] and self.front_since and now-self.front_since>=5:
            self.spawn("left")
        lok=self.fresh(self.last["left"])
        if self.started["left"] and lok:
            if self.left_since is None:self.left_since=now
        else:self.left_since=None
        if not self.started["right"] and self.left_since and now-self.left_since>=3:
            self.spawn("right")
        s={"front_ready":front}
        for c in ("left","right"):
            p=self.children[c]
            s[c]={"started":self.started[c],"stream_fresh":self.fresh(self.last[c]),
                  "process_alive":p is not None and p.poll() is None,
                  "return_code":p.poll() if p is not None else None}
        m=String();m.data=json.dumps(s);self.pub.publish(m)
    def destroy_node(self):
        for p in self.children.values():
            if p is not None and p.poll() is None:
                try:p.send_signal(signal.SIGINT);p.wait(timeout=5)
                except Exception:pass
        super().destroy_node()
def main(args=None):
    rclpy.init(args=args);n=SideCaptureSupervisor()
    try:rclpy.spin(n)
    except KeyboardInterrupt:pass
    finally:n.destroy_node();rclpy.shutdown() if rclpy.ok() else None
if __name__=="__main__":main()