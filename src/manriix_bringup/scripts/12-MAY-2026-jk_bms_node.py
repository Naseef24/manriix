#!/usr/bin/env python3
"""
jk_bms_node.py — JK BMS monitor for Manriix
ROS2 Humble port of ROS1 jk_bms_monitor.py

Reads battery data from JK BMS over RS-485 serial and publishes:
  /battery/status        → sensor_msgs/BatteryState
  /battery/cell_voltages → std_msgs/Float32MultiArray
  /battery/temperature   → std_msgs/Float32

Connection: JK BMS RS-485 → USB-RS485 adapter → Jetson USB
Default port: /dev/ttyUSB2  baud: 115200
"""

import struct
import serial
import rclpy
import time
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Float32MultiArray, Float32

#ANSI colors for terminal output
C_RESET = "\033[0m"
C_RED = "\033[31m"
C_GREEN = "\033[32m"
C_YELLOW = "\033[33m"
C_CYAN = "\033[36m" 
C_BOLD = "\033[1m"

# ─────────────────────────────────────────────────────────────────────────────
# JKBMS — pure serial protocol class — unchanged from ROS1 version
# ─────────────────────────────────────────────────────────────────────────────
class JKBMS:
    def __init__(self, port='/dev/ttyUSB*', baudrate=115200):
        self.bms = None
        self.port = port
        self.baudrate = baudrate

    def connect(self):
        """Open serial connection to JK BMS"""
        try:
            self.bms = serial.Serial(self.port)
            self.bms.baudrate = self.baudrate
            self.bms.timeout = 0.2
            return True
        except serial.SerialException as e:
            print(f'{C_RED}[BMS] Serial connect failed: {e}{C_RESET}')
            self.bms = None
            return False

    def disconnect(self):
        """Close serial connection"""
        if self.bms and self.bms.is_open:
            self.bms.close()
            print(f'{C_YELLOW}[BMS] Serial disconnected{C_RESET}')
        self.bms = None

    def is_connected(self):
        """Check if serial connection is open"""
        return self.bms is not None and self.bms.is_open

    def send_bms_command(self, cmd_string):
        """Send hex command string byte by byte"""
        if not self.bms:
            return False
        try:
            cmd_bytes = bytearray.fromhex(cmd_string)
            for cmd_byte in cmd_bytes:
                self.bms.write(bytearray.fromhex('{0:02x}'.format(cmd_byte)))
            return True
        except serial.SerialException as e:
            print(f'{C_RED}[BMS] Send failed: {e}{C_RESET}')
            return False

    def read_bms_data(self):
        """Send read-all command and parse response into dict"""
        if not self.bms:
            return None
        try:
            if not self.send_bms_command('4E 57 00 13 00 00 00 00 06 03 00 00 00 00 00 00 68 00 00 01 29'):
                return None

            time.sleep(0.1)  # Wait for response

            if self.bms.in_waiting >= 4:
                if self.bms.read(1).hex() == '4e':
                    if self.bms.read(1).hex() == '57':
                        length = int.from_bytes(self.bms.read(2), byteorder='big')
                        length -= 2

                        available = self.bms.in_waiting
                        if available != length:
                            time.sleep(0.1)  # Wait for full response
                            available = self.bms.in_waiting
                            if available != length:
                                self.bms.reset_input_buffer()
                                raise Exception('Data length mismatch')
                            
                        b = bytearray.fromhex('4e57')
                        b += (length + 2).to_bytes(2, byteorder='big')
                        data = bytearray(self.bms.read(available))
                        data = b + data

                        crc_calc = sum(data[0:-4])
                        crc_lo = struct.unpack_from('>H', data[-2:])[0]
                        if crc_calc != crc_lo:
                            self.bms.reset_input_buffer()
                            raise Exception('CRC check failed')
                        data = data[11:length - 19]
                        return self._parse(data)
                    
            self.bms.reset_input_buffer()
            return None
        
        except Exception as e:
            print(f'{C_RED}[BMS] Read error: {e}{C_RESET}')
            self.bms.reset_input_buffer()
            return None
        
    def _parse(self, data):
        """Parse raw BMS data into dict."""
        try:    
            bytecount = data[1]
            cellcount = int(bytecount / 3)

            cell_voltages = []
            total_voltage = 0.0
            for i in range(cellcount):
                v = struct.unpack_from('>H', data, i * 3 + 2)[0] / 1000.0
                cell_voltages.append(v)
                total_voltage += v

            temp_fet = struct.unpack_from('>H', data, bytecount + 3)[0]
            if temp_fet > 100:
                temp_fet = -(temp_fet - 100)

            temp_1 = struct.unpack_from('>H', data, bytecount + 6)[0]
            if temp_1 > 100:
                temp_1 = -(temp_1 - 100)    

            temp_2 = struct.unpack_from('>H', data, bytecount + 9)[0]
            if temp_2 > 100:
                temp_2 = -(temp_2 - 100)    

            avg_temp = (temp_1 + temp_2) / 2.0

            battery_voltage = struct.unpack_from('>H', data, bytecount + 12)[0] / 100.0
            unsigned_current = struct.unpack_from('>H', data, bytecount + 15)[0]
            current = unsigned_current / 100.0
            if unsigned_current > 32767:
                current = (32767 - unsigned_current) / 100.0

            capacity = struct.unpack_from('>B', data, bytecount + 18)[0]

            return {
                'cell_voltages': cell_voltages,
                'total_voltage': total_voltage,
                'battery_voltage': battery_voltage,
                'current': current,
                'temperature': avg_temp,
                'capacity_percent': capacity,
                'temp_fet': temp_fet,
                'temp_1': temp_1,
                'temp_2': temp_2,
                'cell_count': cellcount,
            }             
        except Exception as e:
            print(f'{C_RED}[BMS] Parse error: {e}{C_RESET}')
            return None 
        
# ─────────────────────────────────────────────────────────────────────────────
# JKBMSNode — ROS2 Humble node
# ─────────────────────────────────────────────────────────────────────────────
class JKBMSNode(Node):
    def __init__(self):
        super().__init__('jk_bms_node')
        
        #Parameters
        self.declare_parameter('port', '/dev/ttyUSB1')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('update_rate', 0.1)
        self.declare_parameter('low_battery_threshold', 20.0)

        self.port = self.get_parameter('port').value
        self.baudrate = self.get_parameter('baudrate').value
        self.update_rate = self.get_parameter('update_rate').value
        self.low_battery = self.get_parameter('low_battery_threshold').value

        #Publishers
        self.battery_pub = self.create_publisher(BatteryState, '/battery/status', 10)
        self.cell_pub = self.create_publisher(Float32MultiArray, '/battery/cell_voltages', 10)  
        self.temp_pub = self.create_publisher(Float32, '/battery/temperature', 10)

        #BMS driver interface
        self.bms = JKBMS(port=self.port, baudrate=self.baudrate)
        self._connect()

        #Timer
        self.timer = self.create_timer(1.0 / self.update_rate, self._read_and_publish)

        self.get_logger().info(
            f'{C_GREEN}{C_BOLD}[BMS]{C_RESET} '
            f'JK BMS node started - port={self.port} '
            f'rate={self.update_rate}Hz '
            f'low_threshold={self.low_battery}%')
        
    def _connect(self):
        if self.bms.connect():
            self.get_logger().info(f'{C_GREEN}[BMS]{C_RESET} Connected to {self.port}')

        else:
            self.get_logger().warn(
                f'{C_YELLOW}[BMS]{C_RESET} '
                f'Not connected - will retry each cycle')     

    def _read_and_publish(self):
        '''Timer callback - read and publish battery data'''
        if not self.bms.is_connected():
            self.get_logger().warn(
                f'{C_YELLOW}[BMS]{C_RESET} Reconnecting...')
            self._connect()
            return

        data = self.bms.read_bms_data()
        
        if not data:
            self.get_logger().warn(
                f'{C_YELLOW}[BMS]{C_RESET} Failed to Read BMS data')
            return

        now = self.get_clock().now().to_msg()

        #Publish battery status
        msg = BatteryState()
        msg.header.stamp = now
        msg.header.frame_id = 'base_link'
        msg.voltage = data['battery_voltage']
        msg.current = data['current']
        msg.percentage = data['capacity_percent'] / 100.0
        msg.temperature = data['temperature']
        msg.design_capacity = float('nan')  # Not provided by BMS
        msg.charge = float('nan')           # Not provided by BMS
        msg.capacity = float('nan')         # Not provided by BMS
        msg.present = True
        msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LION 
        
        #Status
        if data['current'] > 0.1:
            msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        elif data['current'] < -0.1:
            msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_CHARGING
        else:msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_NOT_CHARGING

        #Health
        if (data['temperature'] > 50 or data['temperature'] < -10 or 
            data['battery_voltage'] > 60 or data['battery_voltage'] < 10):
            msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_OVERHEAT
        else:
            msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_GOOD
        
        self.battery_pub.publish(msg)

        # ── Cell voltages ─────────────────────────────────────────────────
        cell_msg = Float32MultiArray()
        cell_msg.data = [float(v) for v in data['cell_voltages']]
        self.cell_pub.publish(cell_msg)

        # ── Temperature ─────────────────────────────────────────────────
        temp_msg = Float32()
        temp_msg.data = float(data['temperature'])
        self.temp_pub.publish(temp_msg)

        # ── Logging ───────────────────────────────────────────────────────
        pct = data['capacity_percent']        
        color = C_RED if pct < self.low_battery else \
                C_YELLOW if pct < 40 else C_GREEN
        
        status_str = 'CHARGING' if data['current'] < -0.1 else \
                     'DISCHARGING' if data['current'] > 0.1 else 'IDLE'
        
        self.get_logger().info(
            f'{color}{C_BOLD}[BMS]{C_RESET} '
            f'{pct}% | {data["battery_voltage"]:.2f}V | '
            f'{data["current"]:.2f}A | '
            f'{data["temperature"]:.1f}°C | '
            f'{data["cell_count"]} cells | {status_str}')
        
        if pct < self.low_battery:
            self.get_logger().warn(
                f'{C_RED}{C_BOLD}[BMS LOW BATTERY]{C_RESET} '
                f'{pct}% - below threshold {self.low_battery}%')
            
    def shutdown(self):
        self.bms.disconnect()
        self.get_logger().info(f'{C_YELLOW}[BMS]{C_RESET} Disconnected')

def main(args=None):
    rclpy.init(args=args)
    node = JKBMSNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():  
            rclpy.shutdown()

if __name__ == '__main__':
    main()    