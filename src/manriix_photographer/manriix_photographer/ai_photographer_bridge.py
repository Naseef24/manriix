#!/usr/bin/env python3
"""
AI Photographer Bridge for DJI RS3 Pro Gimbal
Converts AI photographer commands to DJI gimbal format
Matches ROS1 gimbal_hw_interface.cpp behavior
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import Int32, String
from dji_rs3pro_ros_controller.msg import EularAngle, GimbalCmd
from dji_rs3pro_ros_controller.srv import (
    CameraControl,
    SetFocusPosition
)
import math

class AIPhotographerBridge(Node):
    def __init__(self):
        super().__init__('ai_photographer_bridge')
        
        self.get_logger().info("=" * 50)
        self.get_logger().info("AI Photographer Bridge for DJI RS3 Pro")
        self.get_logger().info("=" * 50)
        
        # Initialize state
        self.face_angle_position = Point()
        self.face_angle_position.x = 0.0
        self.face_angle_position.y = 0.0
        self.face_angle_position.z = 0.0
        
        self.is_recording = False
        
        # Conversion constants (matching ROS1)
        self.RAD_TO_DEG = 57.2957795131
        self.DEG_TO_RAD = 0.01745329251
        
        # --- Subscribers (FROM AI Photographer) ---
        self.face_angle_sub = self.create_subscription(
            Point,
            '/face_angle_position',
            self.face_angle_callback,
            10)
        
        self.focus_sub = self.create_subscription(
            Int32,
            '/set_focus_position',
            self.focus_callback,
            10)
        
        self.capture_sub = self.create_subscription(
            String,
            '/capture_image_topic',
            self.capture_callback,
            10)
        
        # --- Publishers (TO DJI Gimbal) ---
        # Matches ROS1: gimbal_cmd_pub.publish(gimbal_cmd)
        self.gimbal_cmd_pub = self.create_publisher(
            GimbalCmd,
            '/gimbalCmd',
            10)
        
        # --- Publishers (TO AI Photographer - Feedback) ---
        # Matches ROS1 telemetry feedback
        self.telemetry_pub = self.create_publisher(
            Point,
            '/gimbalTelemetry',
            10)
        
        # --- Subscribers (FROM DJI Gimbal - Feedback) ---
        # Matches ROS1: gimbal_telemetry_sub
        self.gimbal_state_sub = self.create_subscription(
            EularAngle,
            '/gimbal_angle',
            self.gimbal_state_callback,
            10)
        
        # --- Service Clients ---
        self.camera_control_client = self.create_client(
            CameraControl,
            '/camera_control')
        
        self.focus_client = self.create_client(
            SetFocusPosition,
            '/set_focus_position')
        
        # Telemetry feedback timer (10 Hz)
        self.create_timer(0.1, self.publish_telemetry)
        
        self.get_logger().info("Subscribed to:")
        self.get_logger().info("  - /face_angle_position (Point)")
        self.get_logger().info("  - /gimbal_angle (EularAngle feedback)")
        self.get_logger().info("Publishing:")
        self.get_logger().info("  - /gimbalCmd (GimbalCmd)")
        self.get_logger().info("  - /gimbalTelemetry (Point feedback)")
        self.get_logger().info("Ready!")

    def face_angle_callback(self, msg):
        """
        Convert Point from AI to GimbalCmd for DJI
        Matches ROS1 gimbal_hw_interface.cpp:faceAnglePositionCallback()
        
        Input (from AI):
          msg.x = yaw (radians)
          msg.y = roll (radians)  
          msg.z = pitch (radians)
        
        Output (to DJI):
          gimbal_cmd.angle[0] = 0.0 (reserved)
          gimbal_cmd.angle[1] = yaw (degrees)
          gimbal_cmd.angle[2] = pitch (degrees)
          gimbal_cmd.angle[3] = roll (degrees)
        """
        # Store received values
        self.face_angle_position = msg
        
        # Log received values
        self.get_logger().debug(
            f"Received face angle - x:{msg.x:.3f}, y:{msg.y:.3f}, z:{msg.z:.3f}")
        
        # Validate
        if (math.isnan(msg.x) or math.isnan(msg.y) or math.isnan(msg.z)):
            self.get_logger().warn("Received NaN values!")
            return
        
        # Create GimbalCmd (matches ROS1 write() function)
        gimbal_cmd = GimbalCmd()
        
        # Convert radians to degrees and map to gimbal axes
        # Matches ROS1 EXACTLY:
        #   gimbal_cmd.angle[1] = face_angle_position_.x * RAD_TO_DEG  (yaw)
        #   gimbal_cmd.angle[2] = face_angle_position_.z * RAD_TO_DEG  (pitch)
        #   gimbal_cmd.angle[3] = face_angle_position_.y * RAD_TO_DEG  (roll)
        
        gimbal_cmd.angle = [0.0] * 6  # Initialize array
        gimbal_cmd.angle[0] = 0.0
        gimbal_cmd.angle[1] = msg.x * self.RAD_TO_DEG  # yaw from x
        gimbal_cmd.angle[2] = msg.z * self.RAD_TO_DEG  # pitch from z
        gimbal_cmd.angle[3] = msg.y * self.RAD_TO_DEG  # roll from y
        
        # Velocity (zeros for position control)
        gimbal_cmd.vel = [0.0] * 6
        gimbal_cmd.accel = [0.0] * 6
        gimbal_cmd.current = [0.0] * 6
        gimbal_cmd.msg_ctr = 0
        
        # Publish to DJI gimbal
        self.gimbal_cmd_pub.publish(gimbal_cmd)
        
        # LOG CORRECTLY: show what we're sending in gimbal frame
        self.get_logger().info(
            f"Gimbal cmd -> yaw:{gimbal_cmd.angle[1]:.1f}° (from x:{msg.x:.3f}), "
            f"pitch:{gimbal_cmd.angle[2]:.1f}° (from z:{msg.z:.3f}), "
            f"roll:{gimbal_cmd.angle[3]:.1f}° (from y:{msg.y:.3f})")

    def gimbal_state_callback(self, msg):
        """
        Receive current gimbal state from DJI controller
        Matches ROS1: gimbalTelemetryCallback()
        """
        # Store current gimbal angles for feedback
        self.face_angle_position.x = msg.yaw
        self.face_angle_position.y = msg.roll
        self.face_angle_position.z = msg.pitch

    def focus_callback(self, msg):
        """Handle focus position commands"""
        if not self.focus_client.wait_for_service(timeout_sec=0.5):
            self.get_logger().warn('Focus service not available')
            return
        
        request = SetFocusPosition.Request()
        request.position = msg.data
        
        future = self.focus_client.call_async(request)
        self.get_logger().debug(f'Focus command: {msg.data}')

    def capture_callback(self, msg):
        """
        Handle photo/video capture commands
        'y' = take photo
        'v' = toggle video recording
        """
        command = msg.data.lower()
        
        if command == 'y':
            self.call_camera_control('photo')
            
        elif command == 'v':
            if not self.is_recording:
                self.call_camera_control('start_recording')
                self.is_recording = True
            else:
                self.call_camera_control('stop_recording')
                self.is_recording = False

    def call_camera_control(self, action):
        """Call camera control service"""
        if not self.camera_control_client.wait_for_service(timeout_sec=0.5):
            self.get_logger().warn('Camera control service not available')
            return
        
        request = CameraControl.Request()
        request.command = action
        
        future = self.camera_control_client.call_async(request)
        self.get_logger().info(f'Camera: {action}')

    def publish_telemetry(self):
        """
        Publish current gimbal position back to AI photographer
        Matches ROS1 telemetry feedback
        """
        msg = Point()
        msg.x = self.face_angle_position.x  # yaw
        msg.y = self.face_angle_position.y  # roll
        msg.z = self.face_angle_position.z  # pitch
        self.telemetry_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = AIPhotographerBridge()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()