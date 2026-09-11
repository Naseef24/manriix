#!/usr/bin/env python3
import struct
import numpy as np
import math

import rclpy
import rclpy.logging
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

from std_msgs.msg import String
from sensor_msgs.msg import JointState
from can_msgs.msg import Frame

from .check_sum import calc_crc16, calc_crc32
from dji_rs3pro_ros_controller.msg import EularAngle


class GimbalBase(Node):
    def __init__(self):
        super().__init__('rs3pro_gimbal_base')
        # Enable colored logging
        rclpy.logging.set_logger_level(self.get_logger().name, rclpy.logging.LoggingSeverity.INFO)        

        self.header = 0xAA
        self.enc = 0x00
        self.res1 = 0x00
        self.res2 = 0x00
        self.res3 = 0x00
        self.seq = 0x0002

        self.send_id = int("223", 16)  # 547
        self.recv_id = int("222", 16)  # 546

        self.FRAME_LEN = 8

        self.can_recv_msg_buffer = []
        self.can_recv_msg_len_buffer = []
        self.can_recv_buffer_len = 16

        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0

        self.roll_max_limit = int(45)
        self.pitch_max_limit = int(45)
        self.yaw_max_limit = int(180)

        # Initialize Transform Broadcaster
        self.br = TransformBroadcaster(self)
        
        # Set parameters
        self.set_param()

    def parse_position_response(self, data_frame):
        pos_data = data_frame[16:-4]
        yaw = int('0x' + pos_data[1] + pos_data[0], base=16)
        roll = int('0x' + pos_data[3] + pos_data[2], base=16)
        pitch = int('0x' + pos_data[5] + pos_data[4], base=16)
        
        if yaw > 1800:
            yaw -= 65538
        if roll > 1800:
            roll -= 65538
        if pitch > 1800:
            pitch -= 65538

        self.yaw = yaw * 0.1 * np.pi / 180
        self.roll = roll * 0.1 * np.pi / 180
        self.pitch = pitch * 0.1 * np.pi / 180

        # Publish transforms
        self.publish_transforms()

    def publish_transforms(self):
        # Transform from map to gimbal_base
        t1 = TransformStamped()
        t1.header.stamp = self.get_clock().now().to_msg()
        t1.header.frame_id = 'map'
        t1.child_frame_id = 'gimbal_base'
        t1.transform.translation.x = 0.0
        t1.transform.translation.y = 0.0
        t1.transform.translation.z = -1.0
        t1.transform.rotation.x = 0.0
        t1.transform.rotation.y = 0.0
        t1.transform.rotation.z = 0.0
        t1.transform.rotation.w = 1.0
        self.br.sendTransform(t1)

        # Transform from gimbal_base to end_effector with gimbal angles
        t2 = TransformStamped()
        t2.header.stamp = self.get_clock().now().to_msg()
        t2.header.frame_id = 'gimbal_base'
        t2.child_frame_id = 'end_effector'
        t2.transform.translation.x = 0.0
        t2.transform.translation.y = 0.0
        t2.transform.translation.z = 0.0
        
        # Convert Euler to Quaternion
        quat = self.euler_to_quaternion(self.roll, self.pitch, self.yaw)
        t2.transform.rotation.x = quat[0]
        t2.transform.rotation.y = quat[1]
        t2.transform.rotation.z = quat[2]
        t2.transform.rotation.w = quat[3]
        self.br.sendTransform(t2)

    def euler_to_quaternion(self, roll, pitch, yaw):
        """Convert Euler angles to quaternion"""
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy

        return [qx, qy, qz, qw]

    def can_callback(self, data):
        if data.id == self.recv_id:
            tmp_byte_data = []
            for i in data.data:
                tmp_data = '{:02X}'.format(i)
                tmp_byte_data.append(tmp_data)

            self.can_recv_msg_buffer.append(tmp_byte_data)
            self.can_recv_msg_len_buffer.append(data.dlc)

            if len(self.can_recv_msg_buffer) > self.can_recv_buffer_len:
                self.can_recv_msg_buffer.pop(0)
                self.can_recv_msg_len_buffer.pop(0)

            full_msg_frames = self.can_buffer_to_full_frame()

            for hex_data in full_msg_frames:
                if self.validate_api_call(hex_data):
                    request_data = ":".join(hex_data[12:14])
                    if request_data == "0E:02":
                        self.parse_position_response(hex_data)
                    if request_data == "0E:08":
                        pass  # parse_push_response if needed

    def can_buffer_to_full_frame(self):
        full_msg_frames = []
        full_frame_counter = 0
        for i in range(len(self.can_recv_msg_buffer)):
            msg = self.can_recv_msg_buffer[i]
            length = self.can_recv_msg_len_buffer[i]
            msg = msg[:length]
            if msg[0] == "AA":
                full_msg_frames.append(msg)
                full_frame_counter += 1
            if msg[0] != "AA" and (full_frame_counter > 0):
                for byte in msg:
                    full_msg_frames[-1].append(byte)
        return full_msg_frames

    def validate_api_call(self, data_frame):
        validated = False
        check_sum = ':'.join(data_frame[-4:])
        data = ':'.join(data_frame[:-4])
        if len(data_frame) >= 8:
            if check_sum == calc_crc32(data):
                header = ':'.join(data_frame[:10])
                header_check_sum = ':'.join(data_frame[10:12])
                if header_check_sum == calc_crc16(header):
                    validated = True
        return validated

    def seq_num(self):
        if self.seq >= 0xFFFD:
            self.seq = 0x0002
        self.seq += 1
        seq_str = "%04x" % self.seq
        return seq_str[2:] + ":" + seq_str[0:2]

    def set_hyperparams(self):
        return_code = int(str(0), base=10)
        limit_pitch_max = int(str(self.pitch_max_limit), base=10)
        limit_pitch_min = int(str(0), base=10)
        limit_yaw_max = int(str(self.yaw_max_limit), base=10)
        limit_yaw_min = int(str(0), base=10)
        limit_roll_max = int(str(self.roll_max_limit), base=10)
        limit_roll_min = int(str(0), base=10)

        self.get_logger().info(
            f"set_hyperparams: limit_roll_max:{self.roll_max_limit} "
            f"limit_pitch_max:{self.pitch_max_limit} limit_yaw_max:{self.yaw_max_limit}"
        )

        hex_data = struct.pack('<7B', return_code, limit_pitch_max, limit_pitch_min,
                               limit_yaw_max, limit_yaw_min, limit_roll_max, limit_roll_min)
        
        pack_data = ['{:02X}'.format(struct.unpack('<1B', i.to_bytes(1, 'big'))[0]) for i in hex_data]
        cmd_data = ':'.join(pack_data)
        cmd = self.assemble_can_msg(cmd_type='03', cmd_set='0E', cmd_id='03', data=cmd_data)
        
        self.send_cmd(cmd)
        return True

    def assemble_can_msg(self, cmd_type, cmd_set, cmd_id, data):
        if data == "":
            can_frame_data = "{prefix}" + ":{cmd_set}:{cmd_id}".format(
                cmd_set=cmd_set, cmd_id=cmd_id)
        else:
            can_frame_data = "{prefix}" + ":{cmd_set}:{cmd_id}:{data}".format(
                cmd_set=cmd_set, cmd_id=cmd_id, data=data)

        cmd_length = len(can_frame_data.split(":")) + 15

        seqnum = self.seq_num()
        can_frame_header = "{header:02x}".format(header=self.header)
        can_frame_header += ":" + ("%04x" % (cmd_length))[2:4]
        can_frame_header += ":" + ("%04x" % (cmd_length))[0:2]
        can_frame_header += ":" + "{cmd_type}".format(cmd_type=cmd_type)
        can_frame_header += ":" + "{enc:02x}".format(enc=self.enc)
        can_frame_header += ":" + "{res1:02x}".format(res1=self.res1)
        can_frame_header += ":" + "{res2:02x}".format(res2=self.res2)
        can_frame_header += ":" + "{res3:02x}".format(res3=self.res3)
        can_frame_header += ":" + seqnum
        can_frame_header += ":" + calc_crc16(can_frame_header)

        whole_can_frame = can_frame_data.format(prefix=can_frame_header)
        whole_can_frame += ":" + calc_crc32(whole_can_frame)
        whole_can_frame = whole_can_frame.upper()
        
        return whole_can_frame

    def send_cmd(self, cmd):
        data = [int(i, 16) for i in cmd.split(":")]
        self.send_data(self.send_id, data)

    def send_data(self, can_id, data):
        data_len = len(data)
        full_frame_num, left_len = divmod(data_len, self.FRAME_LEN)

        if left_len == 0:
            frame_num = full_frame_num
        else:
            frame_num = full_frame_num + 1

        send_msg_buffer = []
        data_offset = 0
        
        for i in range(full_frame_num):
            msg = Frame()
            msg.id = can_id
            msg.is_rtr = False
            msg.is_extended = False
            msg.is_error = False
            msg.dlc = 8
            msg.data = [0, 0, 0, 0, 0, 0, 0, 0]

            for j in range(self.FRAME_LEN):
                msg.data[j] = data[data_offset + j]
            data_offset += self.FRAME_LEN
            send_msg_buffer.append(msg)

        if left_len > 0:
            msg = Frame()
            msg.id = can_id
            msg.is_rtr = False
            msg.is_extended = False
            msg.is_error = False
            msg.dlc = left_len
            msg.data = [0, 0, 0, 0, 0, 0, 0, 0]

            for j in range(left_len):
                msg.data[j] = data[data_offset + j]
            send_msg_buffer.append(msg)

        for msg in send_msg_buffer:
            self.pub_can_command.publish(msg)

    def request_current_position(self):
        hex_data = [0x01]
        pack_data = ['{:02X}'.format(i) for i in hex_data]
        cmd_data = ':'.join(pack_data)
        cmd = self.assemble_can_msg(cmd_type='03', cmd_set='0E', cmd_id='02', data=cmd_data)
        self.send_cmd(cmd)

    def publish_current_position(self):
        pub_angle = EularAngle()
        pub_angle.header.stamp = self.get_clock().now().to_msg()
        pub_angle.header.frame_id = "end_effector"
        pub_angle.yaw = self.yaw
        pub_angle.pitch = self.pitch
        pub_angle.roll = self.roll
        self.pub_eular_angle.publish(pub_angle)

    def set_param(self):
        # Create timer for main loop (1000 Hz)
        self.timer = self.create_timer(0.02, self.timer_callback)
        
        # Create subscribers and publishers
        self.sub_can_data = self.create_subscription(
            Frame, 'from_can_bus', self.can_callback, 10)
        
        self.pub_can_command = self.create_publisher(Frame, 'to_can_bus', 10)
        self.pub_eular_angle = self.create_publisher(EularAngle, '/gimbal_angle', 10)
        
        # Set hyperparameters
        self.set_hyperparams()

    def timer_callback(self):
        self.request_current_position()
        self.publish_current_position()


def main(args=None):
    rclpy.init(args=args)
    node = GimbalBase()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
