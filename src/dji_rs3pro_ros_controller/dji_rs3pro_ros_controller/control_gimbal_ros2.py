#!/usr/bin/env python3

import struct
import numpy as np
import math
import time

import rclpy
import rclpy.logging
from rclpy.lifecycle import TransitionCallbackReturn
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import Point

from sensor_msgs.msg import Imu, Image
from std_msgs.msg import Int32
from can_msgs.msg import Frame
from dji_rs3pro_ros_controller.msg import GimbalCmd, GimbalTelemetry, EularAngle

# Import custom services
from dji_rs3pro_ros_controller.srv import (
    SetFocusPosition, CameraControl, CalibrateFocusMotor,
    AutoCalibrate, SetGimbalSpeed, SendJointSpeed
)
from std_srvs.srv import Trigger, SetBool

from .check_sum import calc_crc16, calc_crc32
from .can_bus_controller import GimbalBase


class GimbalController(GimbalBase):
    def __init__(self):
        super().__init__()
        
        # Enable colored logging
        rclpy.logging.set_logger_level(self.get_logger().name, rclpy.logging.LoggingSeverity.INFO)
        
        self.get_logger().info(
            f'\033[96m[GIMBAL CTRL] Initializing DJI RS3 Pro controller — '
            f'can3  send_id=0x{self.send_id:03X}  recv_id=0x{self.recv_id:03X}\033[0m')
        
        # Declare and get parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('print_status', True),
                ('roll_max', 45.0),
                ('roll_min', -45.0),
                ('pitch_max', 45.0),
                ('pitch_min', -45.0),
                ('yaw_max', 180.0),
                ('yaw_min', -180.0),
                ('angular_velocity_threshold', 4.0),
                ('target_angle_threshold', 2.5),
            ]
        )
        
        self.print_status_checker = self.get_parameter('print_status').value
        self.roll_max = self.get_parameter('roll_max').value
        self.roll_min = self.get_parameter('roll_min').value
        self.pitch_max = self.get_parameter('pitch_max').value
        self.pitch_min = self.get_parameter('pitch_min').value
        self.yaw_max = self.get_parameter('yaw_max').value
        self.yaw_min = self.get_parameter('yaw_min').value
        self.angular_velocity_threshold = self.get_parameter('angular_velocity_threshold').value
        self.target_angle_threshold = self.get_parameter('target_angle_threshold').value
        
        self.imu_angle = EularAngle()
        self.target_angle = EularAngle()
        self.img_data = Image()
        self.imu_data = Imu()
        
        self.target_roll = 0.0
        self.target_pitch = 0.0
        self.target_yaw = 0.0
        
        self.last_change_time = 0.0
        self.current_yaw_side = 1
        self.last_gimbal_cmd = None
        self._last_no_can_warn_time = 0.0    

        self.get_logger().info(
            f'\033[96m[GIMBAL CTRL] Node created — send_id=0x{self.send_id:03X}  recv_id=0x{self.recv_id:03X}\033[0m')

    def on_configure(self, state):
        # Call parent on_configure first — creates CAN publishers/subscriber
        ret = super().on_configure(state)
        if ret != TransitionCallbackReturn.SUCCESS:
            return ret

        # TF2
        self.tfBuffer = Buffer()
        self.listener = TransformListener(self.tfBuffer, self)

        # Subscriptions
        self.sub_imu_data = self.create_subscription(
            Imu, "/imu/data", self.imu_data_callback, 10)
        self.sub_img_data = self.create_subscription(
            Image, "/camera/image_raw", self.img_data_callback, 10)
        self.gimbal_cmd_sub = self.create_subscription(
            GimbalCmd, '/gimbalCmd', self.request_target_position, 10)
        self.focus_position_sub = self.create_subscription(
            Int32, 'set_focus_position', self.set_focus_position_callback, 1)

        # Publishers
        self.pub_imu_correct_angle = self.create_publisher(
            EularAngle, "/imu_correct_angle", 1)
        self.telemetry_pub = self.create_publisher(
            Point, '/gimbalTelemetry', 1)

        # Services
        self.focus_control_service = self.create_service(
            SetFocusPosition, 'set_focus_position_srv', self.set_focus_position_service)
        self.calibrate_focus_motor_service = self.create_service(
            CalibrateFocusMotor, 'calibrate_focus_motor', self.calibrate_focus_motor_service)
        self.camera_control_service = self.create_service(
            CameraControl, 'camera_control', self.camera_control_service)
        self.camera_status_service = self.create_service(
            Trigger, 'camera_status', self.camera_status_service)
        self.calibration_service = self.create_service(
            AutoCalibrate, 'calibrate_gimbal', self.calibration_service)
        self.active_track_service = self.create_service(
            SetBool, 'set_active_track', self.set_active_track_service)
        self.speed_service = self.create_service(
            SendJointSpeed, 'send_joint_speed_cmd', self.send_joint_speed_cmd)

        self.get_logger().info(
            f'\033[92m[GIMBAL CTRL] Configured — subscriptions and services ready\033[0m')
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state):
        ret = super().on_activate(state)
        if ret != TransitionCallbackReturn.SUCCESS:
            return ret
        # Start control loop and initialise gimbal
        self.timer = self.create_timer(0.1, self.controller_callback)
        self.set_hyperparams()
        self.get_logger().info(
            f'\033[92m[GIMBAL CTRL] Active — control loop started at 10Hz\033[0m')
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state):
        # Stop timer and return gimbal to neutral
        if hasattr(self, 'timer') and self.timer:
            self.timer.cancel()
            self.timer = None
        self.setPosControl(0, 0, 0)
        self.get_logger().info(
            f'\033[93m[GIMBAL CTRL] Deactivated — gimbal returned to 0,0,0\033[0m')
        return super().on_deactivate(state)

    def on_cleanup(self, state):
        if hasattr(self, 'timer') and self.timer:
            self.timer.cancel()
            self.timer = None
        self.get_logger().info(
            f'\033[93m[GIMBAL CTRL] Cleaned up\033[0m')
        return super().on_cleanup(state)

    def img_data_callback(self, msg):
        self.img_data = msg

    def imu_data_callback(self, msg):
        self.imu_data = msg
        
        # Convert quaternion to Euler
        eular = self.quaternion_to_euler(
            self.imu_data.orientation.x,
            self.imu_data.orientation.y,
            self.imu_data.orientation.z,
            self.imu_data.orientation.w
        )
        
        self.imu_angle.roll = eular[1] * -1.0
        self.imu_angle.pitch = (eular[0] + math.pi/2) * -1.0
        self.imu_angle.yaw = eular[2]
        
        self.pub_imu_correct_angle.publish(self.imu_angle)

    def quaternion_to_euler(self, x, y, z, w):
        """Convert quaternion to Euler angles"""
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)
        
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        return [roll, pitch, yaw]

    def print_status_func(self):
        if self.can_is_alive():
            deg_roll  = self.roll  / math.pi * 180.0
            deg_pitch = self.pitch / math.pi * 180.0
            deg_yaw   = self.yaw   / math.pi * 180.0
            self.get_logger().info(
                f'{self._C_GREEN}[GIMBAL OK]{self._C_RESET} '
                f'roll:{deg_roll:7.2f}°  '
                f'pitch:{deg_pitch:7.2f}°  '
                f'yaw:{deg_yaw:7.2f}°'
            )
        else:
            now = self.get_clock().now().nanoseconds / 1e9
            if now - self._last_no_can_warn_time >= 2.0:
                self._last_no_can_warn_time = now
                self.get_logger().warn(
                    f'{self._C_RED}[GIMBAL NO CAN]{self._C_RESET} '
                    f'No data from gimbal — check can3 cable and gimbal power'
                )
    def check_limit(self, angle, min_angle, max_angle):
        if angle > max_angle:
            return max_angle
        elif angle < min_angle:
            return min_angle
        return angle

    def request_target_position(self, gimbal_cmd):
        try:
            self.get_logger().info("Set New Target Angle")
            
            current_yaw = self.yaw * 180.0 / math.pi
            requested_yaw = gimbal_cmd.angle[1]
            requested_roll = gimbal_cmd.angle[2]    # FIX: angle[3] is roll
            requested_pitch = gimbal_cmd.angle[3]   # FIX: angle[2] is pitch
            
            # Normalize yaw
            while requested_yaw > 180.0:
                requested_yaw -= 360.0
            while requested_yaw < -180.0:
                requested_yaw += 360.0
            
            intermediate_yaw = requested_yaw
            
            # Handle zero crossing
            if (current_yaw > 0 and requested_yaw < 0):
                self.get_logger().info(f"Taking long path through +180 from {current_yaw} to {requested_yaw}")
                # intermediate_yaw = 180.0 # Rishi changed to remove zero crossing 
                self.last_gimbal_cmd = gimbal_cmd
            elif (current_yaw < 0 and requested_yaw > 0):
                self.get_logger().info(f"Taking long path through -180 from {current_yaw} to {requested_yaw}")
                # intermediate_yaw = -180.0 # Rishi changed to remove zero crossing 
                self.last_gimbal_cmd = gimbal_cmd
            else:
                self.last_gimbal_cmd = None
            
            self.target_yaw = intermediate_yaw
            self.target_roll = self.check_limit(requested_roll, self.roll_min, self.roll_max)      # FIX
            self.target_pitch = self.check_limit(requested_pitch, self.pitch_min, self.pitch_max)  # FIX

            self.get_logger().info(
                f"Target: yaw:{self.target_yaw:.2f}, pitch:{self.target_pitch:.2f}, roll:{self.target_roll:.2f}"
            )
            
            result_gimbal = Point(x=self.yaw, y=self.roll, z=self.pitch)
            self.telemetry_pub.publish(result_gimbal)
            
            self.set_attitude_control()
            
        except Exception as e:
            self.get_logger().error(f"Error in request_target_position: {e}")

    def reach_target_angle(self):
        tmp_target_roll = self.target_roll / 180.0 * math.pi
        tmp_target_pitch = self.target_pitch / 180.0 * math.pi
        tmp_target_yaw = self.target_yaw / 180.0 * math.pi
        
        threshold_rad = self.target_angle_threshold / 180.0 * math.pi
        
        if (abs(tmp_target_roll - self.roll) < threshold_rad and
            abs(tmp_target_pitch - self.pitch) < threshold_rad and
            abs(tmp_target_yaw - self.yaw) < threshold_rad):
            self.get_logger().info("Reached target angle")
            return True
        return False

    def set_attitude_control(self):
        yaw = int(self.target_yaw * 10)
        roll = int(self.target_roll * 10)
        pitch = int(self.target_pitch * 10)
        
        if -1800 <= yaw <= 1800 and -450 <= roll <= 450 and -450 <= pitch <= 450:
            self.setPosControl(yaw, roll, pitch)
        else:
            self.get_logger().warn(f"Target angles out of range")

    def setPosControl(self, yaw, roll, pitch):
        ctrl_byte = 0x01
        time_for_action = 0x01
        hex_data = struct.pack('<3h2B', yaw, roll, pitch, ctrl_byte, time_for_action)
        
        pack_data = ['{:02X}'.format(struct.unpack('<1B', i.to_bytes(1, 'big'))[0]) for i in hex_data]
        cmd_data = ':'.join(pack_data)
        
        cmd = self.assemble_can_msg(cmd_type='03', cmd_set='0E', cmd_id='00', data=cmd_data)
        self.send_cmd(cmd)
        return True

    # Focus control methods
    def set_focus_position(self, position):
        self.get_logger().info(f"Setting focus position to: {position}")
        if 0 <= position <= 4095:
            command_sub_id = 0x01
            control_type = 0x00
            data_length = 0x02
            
            hex_data = struct.pack('<BBBH', command_sub_id, control_type, data_length, position)
            pack_data = ['{:02X}'.format(b) for b in hex_data]
            cmd_data = ':'.join(pack_data)
            
            cmd = self.assemble_can_msg(cmd_type='00', cmd_set='0E', cmd_id='12', data=cmd_data)
            self.send_cmd(cmd)
            return True
        return False

    def set_focus_position_callback(self, msg):
        position = int(msg.data)
        self.set_focus_position(position)

    def set_focus_position_service(self, request, response):
        success = self.set_focus_position(request.position)
        response.success = success
        response.message = "Focus set" if success else "Failed"
        return response

    def calibrate_focus_motor(self, calibration_type):
        hex_data = struct.pack('<BBB', 0x02, 0x00, calibration_type)
        pack_data = ['{:02X}'.format(b) for b in hex_data]
        cmd_data = ':'.join(pack_data)
        
        cmd = self.assemble_can_msg(cmd_type='03', cmd_set='0E', cmd_id='12', data=cmd_data)
        self.send_cmd(cmd)
        return True

    def calibrate_focus_motor_service(self, request, response):
        success = self.calibrate_focus_motor(request.calibration_type)
        response.success = success
        response.message = "Calibration sent" if success else "Failed"
        return response

    # Camera control methods
    def camera_control(self, command):
        cmd_dict = {
            'shutter': 0x0001,
            'stop_shutter': 0x0002,
            'start_recording': 0x0003,
            'stop_recording': 0x0004,
            'center_focus': 0x0005,
            'end_center_focus': 0x000B
        }
        
        if command not in cmd_dict:
            return False
        
        hex_data = struct.pack('<H', cmd_dict[command])
        pack_data = ['{:02X}'.format(b) for b in hex_data]
        cmd_data = ':'.join(pack_data)
        
        cmd = self.assemble_can_msg(cmd_type='03', cmd_set='0D', cmd_id='00', data=cmd_data)
        self.send_cmd(cmd)
        return True

    def camera_control_service(self, request, response):
        success = self.camera_control(request.command)
        response.success = success
        response.message = f"Command '{request.command}' executed" if success else "Failed"
        return response

    def camera_status_service(self, request, response):
        response.success = True
        response.message = "Camera status: Unknown"
        return response

    # Calibration methods
    def set_auto_calibration(self, enable, calibration_type=0):
        tlv_id = 0x00
        enable_bit = 0x01 if enable else 0x00
        type_bits = (calibration_type & 0x7F) << 1
        value = enable_bit | type_bits
        length = 1
        
        tlv_data = struct.pack('<BBB', tlv_id, length, value)
        pack_data = ['{:02X}'.format(b) for b in tlv_data]
        cmd_data = ':'.join(pack_data)
        
        cmd = self.assemble_can_msg(cmd_type='03', cmd_set='0E', cmd_id='0F', data=cmd_data)
        self.send_cmd(cmd)
        return True

    def calibration_service(self, request, response):
        success = self.set_auto_calibration(True)
        response.success = success
        response.message = "Calibration started" if success else "Failed"
        return response

    # Active track
    def set_active_track(self, enable):
        ctrl_byte = 0x03
        hex_data = struct.pack('<B', ctrl_byte)
        pack_data = ['{:02X}'.format(b) for b in hex_data]
        cmd_data = ':'.join(pack_data)
        
        cmd = self.assemble_can_msg(cmd_type='03', cmd_set='0E', cmd_id='11', data=cmd_data)
        self.send_cmd(cmd)
        return True

    def set_active_track_service(self, request, response):
        success = self.set_active_track(request.data)
        response.success = success
        response.message = "ActiveTrack enabled" if request.data else "ActiveTrack disabled"
        return response

    # Speed control
    def send_joint_speed_cmd(self, request, response):
        yaw = int(request.yaw * 10)
        pitch = int(request.pitch * 10)
        roll = int(request.roll * 10)
        
        if -3600 <= yaw <= 3600 and -3600 <= roll <= 3600 and -3600 <= pitch <= 3600:
            self.setSpeedControl(yaw, roll, pitch)
            response.success = True
        else:
            response.success = False
        return response

    def setSpeedControl(self, yaw, roll, pitch, ctrl_byte=0x80):
        hex_data = struct.pack('<3hB', yaw, roll, pitch, ctrl_byte)
        pack_data = ['{:02X}'.format(b) for b in hex_data]
        cmd_data = ':'.join(pack_data)
        
        cmd = self.assemble_can_msg(cmd_type='03', cmd_set='0E', cmd_id='01', data=cmd_data)
        self.send_cmd(cmd)
        return True

    def controller_callback(self):
        if self.can_is_alive():
            self.request_current_position()
            self.publish_current_position()
        else:
            # Still send position request to detect when gimbal comes online
            self.request_current_position()

        if self.print_status_checker:
            self.print_status_func()
            
        # Check if reached target and handle multi-step movement
        if self.can_is_alive() and self.reach_target_angle():
            if self.last_gimbal_cmd is not None:
                self.get_logger().info(
                    f'{self._C_CYAN}[GIMBAL]{self._C_RESET} '
                    f'Reached intermediate — proceeding to final target')
                final_cmd = self.last_gimbal_cmd
                self.last_gimbal_cmd = None
                self.request_target_position(final_cmd)


def main(args=None):
    rclpy.init(args=args)
    node = GimbalController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
