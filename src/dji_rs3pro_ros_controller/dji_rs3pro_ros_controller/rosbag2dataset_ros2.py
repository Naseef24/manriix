#!/usr/bin/env python3

import csv
import os
import math
import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import Imu, Image
from dji_rs3pro_ros_controller.msg import EularAngle


class ROSBag2Dataset(Node):
    def __init__(self):
        super().__init__('rosbag2dataset')
        
        # Declare parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('save_data_directory', '/tmp/gimbal_dataset'),
                ('csv_name', 'dataset.csv'),
                ('gimbal_eular_angle_topic_name', '/gimbal_angle'),
                ('imu_eular_angle_topic_name', '/imu_correct_angle'),
                ('decompressed_image_topic_name', '/camera/image_raw'),
                ('imu_data_topic_name', '/imu/data'),
                ('ros_rate', 10),
            ]
        )
        
        # Get parameters
        self.save_data_directory = self.get_parameter('save_data_directory').value
        self.csv_name = self.get_parameter('csv_name').value
        self.gimbal_topic = self.get_parameter('gimbal_eular_angle_topic_name').value
        self.imu_angle_topic = self.get_parameter('imu_eular_angle_topic_name').value
        self.image_topic = self.get_parameter('decompressed_image_topic_name').value
        self.imu_data_topic = self.get_parameter('imu_data_topic_name').value
        ros_rate = self.get_parameter('ros_rate').value
        
        # Create save directory
        os.makedirs(self.save_data_directory, exist_ok=True)
        os.makedirs(os.path.join(self.save_data_directory, 'camera_image'), exist_ok=True)
        
        # Initialize CSV file with header
        csv_path = os.path.join(self.save_data_directory, self.csv_name)
        if not os.path.exists(csv_path):
            with open(csv_path, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(['counter', 'image_name', 'process_time', 'roll', 'pitch', 'yaw'])
        
        self.counter = 0
        self.save_data_checker_gimbal = False
        self.save_data_checker_imu = False
        self.save_data_checker_image = False
        
        # Data containers
        self.imu_data = Imu()
        self.gimbal_eular_angle = EularAngle()
        self.imu_eular_angle = EularAngle()
        self.integrated_angle = EularAngle()
        self.previous_angle = EularAngle()
        self.decompressed_image = Image()
        
        # OpenCV
        self.bridge = CvBridge()
        self.color_img_cv = np.empty(0)
        self.cropped_image = None
        
        # Timer for processing
        self.timer = self.create_timer(1.0 / ros_rate, self.timer_callback)
        self.init_time = self.get_clock().now()
        
        # Subscriptions
        self.sub_imu_data = self.create_subscription(
            Imu, self.imu_data_topic, self.imu_data_callback, 10)
        self.sub_gimbal_angle = self.create_subscription(
            EularAngle, self.gimbal_topic, self.gimbal_angle_callback, 10)
        self.sub_imu_angle = self.create_subscription(
            EularAngle, self.imu_angle_topic, self.imu_angle_callback, 10)
        self.sub_image = self.create_subscription(
            Image, self.image_topic, self.image_callback, 10)
        
        self.get_logger().info(f"ROSBag2Dataset initialized. Saving to: {self.save_data_directory}")

    def imu_data_callback(self, msg):
        self.imu_data = msg

    def gimbal_angle_callback(self, msg):
        self.previous_angle = self.gimbal_eular_angle
        self.gimbal_eular_angle = msg
        
        current_time = self.get_clock().now()
        self.process_time = (current_time - self.init_time).nanoseconds / 1e9
        
        # Calculate integrated angle (average of gimbal and IMU)
        self.integrated_angle.roll = (self.gimbal_eular_angle.roll + self.imu_eular_angle.roll) / 2.0
        self.integrated_angle.pitch = (self.gimbal_eular_angle.pitch + self.imu_eular_angle.pitch) / 2.0
        self.integrated_angle.yaw = (self.gimbal_eular_angle.yaw + self.imu_eular_angle.yaw) / 2.0
        
        self.save_data_checker_gimbal = True

    def imu_angle_callback(self, msg):
        self.imu_eular_angle = msg
        self.save_data_checker_imu = True

    def image_callback(self, msg):
        self.decompressed_image = msg
        try:
            self.color_img_cv = self.bridge.imgmsg_to_cv2(self.decompressed_image, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f"CV Bridge Error: {e}")
            return
        
        original_col = int(self.color_img_cv.shape[1])
        original_row = int(self.color_img_cv.shape[0])
        
        # Crop to square
        init_col = (original_col - original_row) // 2
        self.cropped_image = self.color_img_cv[0:original_row, init_col:init_col+original_row]
        
        # Resize to 224x224
        self.cropped_image = cv2.resize(self.cropped_image, dsize=(224, 224), interpolation=cv2.INTER_AREA)
        
        self.save_data_checker_image = True

    def timer_callback(self):
        if (self.save_data_checker_image and 
            self.save_data_checker_gimbal and 
            self.save_data_checker_imu):
            
            self.save_data()
            
            self.get_logger().info(
                f"Count: {self.counter}\n"
                f"Process Time: {self.process_time:.3f}s\n"
                f"Gimbal  - roll: {self.gimbal_eular_angle.roll:.4f}, "
                f"pitch: {self.gimbal_eular_angle.pitch:.4f}, "
                f"yaw: {self.gimbal_eular_angle.yaw:.4f}\n"
                f"IMU     - roll: {self.imu_eular_angle.roll:.4f}, "
                f"pitch: {self.imu_eular_angle.pitch:.4f}, "
                f"yaw: {self.imu_eular_angle.yaw:.4f}\n"
                f"Integrated - roll: {self.integrated_angle.roll:.4f}, "
                f"pitch: {self.integrated_angle.pitch:.4f}, "
                f"yaw: {self.integrated_angle.yaw:.4f}\n"
                f"Diff (deg) - roll: {(self.gimbal_eular_angle.roll - self.imu_eular_angle.roll)*180.0/math.pi:.2f}, "
                f"pitch: {(self.gimbal_eular_angle.pitch - self.imu_eular_angle.pitch)*180.0/math.pi:.2f}, "
                f"yaw: {(self.gimbal_eular_angle.yaw - self.imu_eular_angle.yaw)*180.0/math.pi:.2f}"
            )
            
            self.counter += 1

    def save_data(self):
        if self.cropped_image is None:
            return
        
        # Save image
        image_name = f"image{self.counter}.png"
        image_path = os.path.join(self.save_data_directory, "camera_image", image_name)
        cv2.imwrite(image_path, self.cropped_image)
        
        # Save CSV row
        csv_path = os.path.join(self.save_data_directory, self.csv_name)
        with open(csv_path, 'a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                self.counter,
                image_name,
                self.process_time,
                self.integrated_angle.roll,
                self.integrated_angle.pitch,
                self.integrated_angle.yaw
            ])


def main(args=None):
    rclpy.init(args=args)
    node = ROSBag2Dataset()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
