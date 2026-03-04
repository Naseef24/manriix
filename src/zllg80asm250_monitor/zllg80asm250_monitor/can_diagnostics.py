#!/usr/bin/env python3
"""
ZLLG80ASM250 V1.0 - CAN Diagnostics Node (ROS2)
Publishes motor status data to ROS2 topics and provides services

Motor Specifications (ZLLG80ASM250-L-24V):
    - Torque Constant (Kt): 0.8 Nm/A
    - Max Torque: 19.15 Nm @ 23.08A

Author: Hype
Date: December 2024
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray, String
import can
import struct
import json
import time


class ZLLG80ASM250Diagnostics(Node):
    """
    ZLLG80ASM250 V1.0 CAN Diagnostics
    Publishes motor data to ROS2 topics
    """
    
    # Motor Constants
    KT_CONSTANT = 0.8       # Torque constant in Nm/A
    MAX_TORQUE = 19.15      # Maximum torque in Nm
    MAX_CURRENT = 23.082    # Maximum current in A
    
    # Fault code descriptions
    FAULT_CODES = {
        0x00000000: "No error",
        0x00000001: "Over-voltage",
        0x00000002: "Under-voltage",
        0x00000100: "EEPROM read/write error",
        0x00000004: "Over-current (left)",
        0x00040000: "Over-current (right)",
        0x00000008: "Overload (left)",
        0x00080000: "Overload (right)",
        0x00000020: "Encoder out of tolerance (left)",
        0x00200000: "Encoder out of tolerance (right)",
        0x00000080: "Reference voltage error (left)",
        0x00800000: "Reference voltage error (right)",
        0x00000200: "Hall error (left)",
        0x02000000: "Hall error (right)",
        0x00000400: "High motor temperature (left)",
        0x04000000: "High motor temperature (right)",
        0x00000800: "Encoder error (left)",
        0x08000000: "Encoder error (right)",
        0x00002000: "Speed setting error (left)",
        0x20000000: "Speed setting error (right)",
    }
    
    def __init__(self):
        super().__init__('zllg80asm250_diagnostics')
        
        # Declare parameters
        self.declare_parameter('can_channel', 'can2')
        self.declare_parameter('bitrate', 1000000)
        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('kt_constant', self.KT_CONSTANT)
        
        # Get parameters
        can_channel = self.get_parameter('can_channel').get_parameter_value().string_value
        bitrate = self.get_parameter('bitrate').get_parameter_value().integer_value
        publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        self.kt = self.get_parameter('kt_constant').get_parameter_value().double_value
        
        # CAN Bus setup
        try:
            self.bus = can.Bus(interface='socketcan', channel=can_channel, bitrate=bitrate)
            self.get_logger().info(f'CAN bus initialized on {can_channel} at {bitrate} bps')
        except Exception as e:
            self.get_logger().error(f'Failed to initialize CAN bus: {e}')
            raise
        
        # Driver IDs
        self.driver1_id = 0x601
        self.driver2_id = 0x602
        
        # Publishers
        self.pub_voltage = self.create_publisher(Float32, '/zllg80asm250/voltage', 10)
        self.pub_power = self.create_publisher(Float32, '/zllg80asm250/power', 10)
        self.pub_currents = self.create_publisher(Float32MultiArray, '/zllg80asm250/currents', 10)
        self.pub_torques = self.create_publisher(Float32MultiArray, '/zllg80asm250/torques_nm', 10)
        self.pub_speeds = self.create_publisher(Float32MultiArray, '/zllg80asm250/speeds', 10)
        self.pub_motor_temps = self.create_publisher(Float32MultiArray, '/zllg80asm250/motor_temps', 10)
        self.pub_driver_temps = self.create_publisher(Float32MultiArray, '/zllg80asm250/driver_temps', 10)
        self.pub_status = self.create_publisher(String, '/zllg80asm250/status_json', 10)
        self.pub_faults = self.create_publisher(String, '/zllg80asm250/faults', 10)
        
        # Timer for periodic publishing
        self.timer = self.create_timer(1.0 / publish_rate, self.publish_data)
        
        self.get_logger().info(f'ZLLG80ASM250 V1.0 Diagnostics initialized (Kt = {self.kt} Nm/A)')
    
    def send_sdo_read(self, driver_id, index, subindex):
        """Send SDO read request and get response"""
        data = [0x40, index & 0xFF, (index >> 8) & 0xFF, subindex, 0, 0, 0, 0]
        msg = can.Message(arbitration_id=driver_id, data=data, is_extended_id=False)
        
        try:
            self.bus.send(msg)
            response_id = 0x580 + (driver_id - 0x600)
            start = time.time()
            while time.time() - start < 0.1:
                recv_msg = self.bus.recv(timeout=0.05)
                if recv_msg and recv_msg.arbitration_id == response_id:
                    if recv_msg.data[1] == (index & 0xFF) and recv_msg.data[2] == ((index >> 8) & 0xFF):
                        return recv_msg.data
            return None
        except Exception as e:
            self.get_logger().warn(f'SDO read error: {e}')
            return None
    
    def current_to_torque(self, current_amps):
        """Convert current (A) to torque (Nm) using Kt"""
        return current_amps * self.kt
    
    def read_current(self, driver_id):
        """Read current in Amps (Index 6077h)"""
        left_data = self.send_sdo_read(driver_id, 0x6077, 0x01)
        right_data = self.send_sdo_read(driver_id, 0x6077, 0x02)
        
        left = struct.unpack('<h', bytes(left_data[4:6]))[0] / 10.0 if left_data and left_data[0] in [0x4B, 0x43, 0x4F] else 0
        right = struct.unpack('<h', bytes(right_data[4:6]))[0] / 10.0 if right_data and right_data[0] in [0x4B, 0x43, 0x4F] else 0
        return left, right
    
    def read_speed(self, driver_id):
        """Read speed in RPM (Index 606Ch)"""
        left_data = self.send_sdo_read(driver_id, 0x606C, 0x01)
        right_data = self.send_sdo_read(driver_id, 0x606C, 0x02)
        
        left = struct.unpack('<i', bytes(left_data[4:8]))[0] / 10.0 if left_data and left_data[0] in [0x4B, 0x43, 0x4F] else 0
        right = struct.unpack('<i', bytes(right_data[4:8]))[0] / 10.0 if right_data and right_data[0] in [0x4B, 0x43, 0x4F] else 0
        return left, right
    
    def read_voltage(self, driver_id):
        """Read voltage in V (Index 2035h)"""
        data = self.send_sdo_read(driver_id, 0x2035, 0x00)
        return struct.unpack('<H', bytes(data[4:6]))[0] / 100.0 if data and data[0] in [0x4B, 0x43, 0x4F] else 0
    
    def read_temperature(self, driver_id):
        """Read temperatures in °C (Index 2032h)"""
        left_data = self.send_sdo_read(driver_id, 0x2032, 0x01)
        right_data = self.send_sdo_read(driver_id, 0x2032, 0x02)
        driver_data = self.send_sdo_read(driver_id, 0x2032, 0x03)
        
        left = struct.unpack('<h', bytes(left_data[4:6]))[0] / 10.0 if left_data and left_data[0] in [0x4B, 0x43, 0x4F] else 0
        right = struct.unpack('<h', bytes(right_data[4:6]))[0] / 10.0 if right_data and right_data[0] in [0x4B, 0x43, 0x4F] else 0
        driver = struct.unpack('<h', bytes(driver_data[4:6]))[0] / 10.0 if driver_data and driver_data[0] in [0x4B, 0x43, 0x4F] else 0
        return left, right, driver
    
    def read_fault_code(self, driver_id):
        """Read fault code (Index 603Fh)"""
        data = self.send_sdo_read(driver_id, 0x603F, 0x00)
        return struct.unpack('<I', bytes(data[4:8]))[0] if data and data[0] in [0x4B, 0x43, 0x4F] else 0
    
    def get_fault_description(self, fault_code):
        """Convert fault code to description"""
        if fault_code == 0:
            return "No error"
        descriptions = [desc for code, desc in self.FAULT_CODES.items() if fault_code & code]
        return "; ".join(descriptions) if descriptions else f"Unknown: 0x{fault_code:08X}"
    
    def publish_data(self):
        """Read data and publish to topics"""
        try:
            # Read from both drivers
            lf_curr, lb_curr = self.read_current(self.driver1_id)
            rf_curr, rb_curr = self.read_current(self.driver2_id)
            
            lf_spd, lb_spd = self.read_speed(self.driver1_id)
            rf_spd, rb_spd = self.read_speed(self.driver2_id)
            
            voltage = self.read_voltage(self.driver1_id)
            
            l_temp, _, d1_temp = self.read_temperature(self.driver1_id)
            r_temp, _, d2_temp = self.read_temperature(self.driver2_id)
            
            fault1 = self.read_fault_code(self.driver1_id)
            fault2 = self.read_fault_code(self.driver2_id)
            
            # Convert current to torque (Nm)
            lf_torque = self.current_to_torque(lf_curr)
            lb_torque = self.current_to_torque(lb_curr)
            rf_torque = self.current_to_torque(rf_curr)
            rb_torque = self.current_to_torque(rb_curr)
            
            # Calculate power
            total_current = abs(lf_curr) + abs(lb_curr) + abs(rf_curr) + abs(rb_curr)
            power = voltage * total_current
            
            # Publish voltage
            voltage_msg = Float32()
            voltage_msg.data = voltage
            self.pub_voltage.publish(voltage_msg)
            
            # Publish power
            power_msg = Float32()
            power_msg.data = power
            self.pub_power.publish(power_msg)
            
            # Publish currents [LF, LB, RF, RB] in Amps
            currents_msg = Float32MultiArray()
            currents_msg.data = [lf_curr, lb_curr, rf_curr, rb_curr]
            self.pub_currents.publish(currents_msg)
            
            # Publish torques [LF, LB, RF, RB] in Nm
            torques_msg = Float32MultiArray()
            torques_msg.data = [lf_torque, lb_torque, rf_torque, rb_torque]
            self.pub_torques.publish(torques_msg)
            
            # Publish speeds [LF, LB, RF, RB] in RPM
            speeds_msg = Float32MultiArray()
            speeds_msg.data = [lf_spd, lb_spd, rf_spd, rb_spd]
            self.pub_speeds.publish(speeds_msg)
            
            # Publish motor temps [Left, Right] in °C
            motor_temps_msg = Float32MultiArray()
            motor_temps_msg.data = [l_temp, r_temp]
            self.pub_motor_temps.publish(motor_temps_msg)
            
            # Publish driver temps [D1, D2] in °C
            driver_temps_msg = Float32MultiArray()
            driver_temps_msg.data = [d1_temp, d2_temp]
            self.pub_driver_temps.publish(driver_temps_msg)
            
            # Publish complete status as JSON
            status = {
                'timestamp': time.time(),
                'motor_model': 'ZLLG80ASM250-L-24V',
                'kt_constant': self.kt,
                'voltage': voltage,
                'power': power,
                'currents_A': {'lf': lf_curr, 'lb': lb_curr, 'rf': rf_curr, 'rb': rb_curr},
                'torques_Nm': {'lf': lf_torque, 'lb': lb_torque, 'rf': rf_torque, 'rb': rb_torque},
                'speeds_RPM': {'lf': lf_spd, 'lb': lb_spd, 'rf': rf_spd, 'rb': rb_spd},
                'motor_temps_C': {'left': l_temp, 'right': r_temp},
                'driver_temps_C': {'d1': d1_temp, 'd2': d2_temp},
                'faults': {'d1': fault1, 'd2': fault2}
            }
            status_msg = String()
            status_msg.data = json.dumps(status)
            self.pub_status.publish(status_msg)
            
            # Publish faults if any
            if fault1 or fault2:
                fault_msg = String()
                fault_info = []
                if fault1:
                    fault_info.append(f"Driver1: {self.get_fault_description(fault1)}")
                if fault2:
                    fault_info.append(f"Driver2: {self.get_fault_description(fault2)}")
                fault_msg.data = "; ".join(fault_info)
                self.pub_faults.publish(fault_msg)
            
        except Exception as e:
            self.get_logger().warn(f'Error publishing data: {e}')
    
    def destroy_node(self):
        self.bus.shutdown()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = ZLLG80ASM250Diagnostics()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
