#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs
from geometry_msgs.msg import PoseStamped, Vector3
from sensor_msgs.msg import Imu
from dji_rs3pro_ros_controller.msg import EularAngle


class CheckAngleDetectionDelay(Node):
    def __init__(self):
        super().__init__('check_angle_detection_delay')
        
        self.get_logger().info("Starting CheckAngleDetectionDelay")
        
        self.imu_data = Imu()
        self.imu_angle = EularAngle()
        self.gimbal_angle = EularAngle()
        self.now_time = 0.0
        self.last_time = 0.0
        
        # TF2 setup
        self.tfBuffer = Buffer()
        self.listener = TransformListener(self.tfBuffer, self)
        
        # Subscribers
        self.sub_imu_data = self.create_subscription(
            Imu, "/imu/data", self.imu_data_callback, 10)
        self.sub_gimbal_angle = self.create_subscription(
            EularAngle, "/gimbal_angle", self.gimbal_angle_callback, 10)
        
        # Publisher
        self.pub_imu_angle = self.create_publisher(
            EularAngle, "/imu/correct_angle", 1)
        
        # Timer for main loop
        self.timer = self.create_timer(0.005, self.timer_callback)  # 200 Hz

    def imu_data_callback(self, msg):
        self.imu_data = msg
        
        pose_stamped = PoseStamped()
        pose_stamped.header = self.imu_data.header
        pose_stamped.header.frame_id = "imu_link"
        pose_stamped.pose.orientation = self.imu_data.orientation
        
        try:
            trans = self.tfBuffer.lookup_transform(
                'imu_link', 'end_effector', 
                rclpy.time.Time(), 
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
            
            pose_trans = tf2_geometry_msgs.do_transform_pose(pose_stamped, trans)
            
            roll, pitch, yaw = self.euler_from_quaternion(
                pose_trans.pose.orientation.x,
                pose_trans.pose.orientation.y,
                pose_trans.pose.orientation.z,
                pose_trans.pose.orientation.w
            )
            
            self.imu_angle.roll = yaw + math.pi
            self.imu_angle.pitch = -1.0 * roll
            self.imu_angle.yaw = pitch
            
        except Exception as e:
            self.get_logger().debug(f"TF lookup failed: {e}")

    def euler_from_quaternion(self, x, y, z, w):
        """Convert quaternion to Euler angles (roll, pitch, yaw)"""
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
        
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
        
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
        
        return roll_x, pitch_y, yaw_z

    def gimbal_angle_callback(self, msg):
        self.gimbal_angle = msg
        
        self.last_time = self.now_time
        self.now_time = self.get_clock().now().nanoseconds / 1e9
        dt = self.now_time - self.last_time
        
        # Log the comparison
        self.get_logger().info(
            f"Delta Time: {dt:.4f}s\n"
            f"Gimbal - roll: {self.gimbal_angle.roll:.4f}, "
            f"pitch: {self.gimbal_angle.pitch:.4f}, "
            f"yaw: {self.gimbal_angle.yaw:.4f}\n"
            f"IMU    - roll: {self.imu_angle.roll:.4f}, "
            f"pitch: {self.imu_angle.pitch:.4f}, "
            f"yaw: {self.imu_angle.yaw:.4f}"
        )
        
        # Publish corrected IMU angle
        self.imu_angle.header.stamp = self.get_clock().now().to_msg()
        self.pub_imu_angle.publish(self.imu_angle)

    def timer_callback(self):
        # Main loop runs at 200 Hz but doesn't do much
        pass


def main(args=None):
    rclpy.init(args=args)
    node = CheckAngleDetectionDelay()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
