#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
import serial
import struct
import time

# Terminal Color Codes
class Colors:
    HEADER = '\033[95m'
    BLUE = '\033[94m'
    CYAN = '\033[96m'
    GREEN = '\033[92m'
    YELLOW = '\033[93m'
    RED = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'

class JKBMSNode(Node):
    def __init__(self):
        super().__init__('jk_bms_node')
        
        # Connection params
        self.declare_parameter('port', '/dev/bms')
        self.declare_parameter('baudrate', 115200)

        # Battery hardware spec (static, configured in BMS firmware)
        self.declare_parameter('cell_count', 8)
        self.declare_parameter('design_capacity_ah', 120.0)
        self.declare_parameter('location', 'manriix_base')

        # Cell voltage thresholds (mirror of BMS firmware config)
        # Used to compute health flags and populate BatteryState fields
        self.declare_parameter('cell_uvp_v', 2.60)   # Cell undervoltage protection
        self.declare_parameter('cell_ovp_v', 3.60)   # Cell overvoltage protection
        self.declare_parameter('soc_0_v', 2.62)      # Nominal 0% SOC voltage
        self.declare_parameter('soc_100_v', 3.56)    # Nominal 100% SOC voltage
        self.declare_parameter('cell_balance_threshold_v', 0.005)  # max acceptable cell spread
        
        port = self.get_parameter('port').get_parameter_value().string_value
        baud = self.get_parameter('baudrate').get_parameter_value().integer_value

        self.cell_count = self.get_parameter('cell_count').get_parameter_value().integer_value
        self.design_capacity_ah = self.get_parameter('design_capacity_ah').get_parameter_value().double_value
        self.location_str = self.get_parameter('location').get_parameter_value().string_value
        self.cell_uvp_v = self.get_parameter('cell_uvp_v').get_parameter_value().double_value
        self.cell_ovp_v = self.get_parameter('cell_ovp_v').get_parameter_value().double_value
        self.soc_0_v = self.get_parameter('soc_0_v').get_parameter_value().double_value
        self.soc_100_v = self.get_parameter('soc_100_v').get_parameter_value().double_value
        self.cell_balance_threshold_v = self.get_parameter(
            'cell_balance_threshold_v').get_parameter_value().double_value
                
        try:
            self.bms = serial.Serial(port, baud, timeout=0.2)
            print(f"{Colors.GREEN}{Colors.BOLD}[INFO] Connected to JK BMS on {port}{Colors.ENDC}")
        except Exception as e:
            print(f"{Colors.RED}{Colors.BOLD}[ERROR] Could not open serial port: {e}{Colors.ENDC}")
            raise e

        self.battery_pub = self.create_publisher(BatteryState, 'main_battery/state', 10)
        self.timer = self.create_timer(2.0, self.timer_callback)

    def _health_str(self, h):
        return {
            BatteryState.POWER_SUPPLY_HEALTH_UNKNOWN: 'UNKNOWN',
            BatteryState.POWER_SUPPLY_HEALTH_GOOD: 'GOOD',
            BatteryState.POWER_SUPPLY_HEALTH_OVERHEAT: 'OVERHEAT',
            BatteryState.POWER_SUPPLY_HEALTH_DEAD: 'DEAD',
            BatteryState.POWER_SUPPLY_HEALTH_OVERVOLTAGE: 'OVERVOLT',
            BatteryState.POWER_SUPPLY_HEALTH_UNSPEC_FAILURE: 'IMBALANCED',
        }.get(h, 'UNKNOWN')
    
    def timer_callback(self):
        try:
            # Request Data (Command 0x03)
            self.bms.write(bytearray.fromhex('4E 57 00 13 00 00 00 00 06 03 00 00 00 00 00 00 68 00 00 01 29'))
            time.sleep(0.1)

            if self.bms.in_waiting >= 4:
                if self.bms.read(1).hex() == '4e' and self.bms.read(1).hex() == '57':
                    length = int.from_bytes(self.bms.read(2), byteorder='big') - 2
                    
                    if self.bms.in_waiting < length:
                        time.sleep(0.1)
                    
                    raw_payload = self.bms.read(self.bms.in_waiting)
                    header = bytearray.fromhex("4e57") + (length + 2).to_bytes(2, byteorder='big')
                    full_frame = header + raw_payload
                    
                    # CRC Check
                    if sum(full_frame[0:-4]) != struct.unpack_from('>H', full_frame[-2:])[0]:
                        print(f"{Colors.YELLOW}[WARN] CRC Mismatch! Skipping frame.{Colors.ENDC}")
                        return

                    data = full_frame[11:length-19]
                    bytecount = data[1]
                    cellcount = int(bytecount / 3)

                    # Extract Voltages
                    cell_voltages = []
                    for i in range(cellcount):
                        v = struct.unpack_from('>xH', data, i * 3 + 2)[0] / 1000.0
                        cell_voltages.append(float(v))

                    # Extract Temperatures (Logic from original script)
                    temp_fet = struct.unpack_from('>H', data, bytecount + 3)[0]
                    if temp_fet > 100: temp_fet = -(temp_fet - 100)
                    
                    temp_1 = struct.unpack_from('>H', data, bytecount + 6)[0]
                    if temp_1 > 100: temp_1 = -(temp_1 - 100)
                    
                    temp_2 = struct.unpack_from('>H', data, bytecount + 9)[0]
                    if temp_2 > 100: temp_2 = -(temp_2 - 100)

                    # Extract Totals
                    total_v = struct.unpack_from('>H', data, bytecount + 12)[0] / 100.0
                    
                    raw_curr = struct.unpack_from('>H', data, bytecount + 15)[0]
                    current = (32767 - raw_curr) / 100.0 if raw_curr > 32767 else raw_curr / 100.0
                    
                    soc = float(struct.unpack_from('>B', data, bytecount + 18)[0])

                    # Current State Logic
                    if abs(current) < 0.1:
                        curr_color = Colors.BLUE
                        mode = "Idle/Standby"
                        status = BatteryState.POWER_SUPPLY_STATUS_NOT_CHARGING
                    elif current < 0:
                        curr_color = Colors.GREEN
                        mode = "Charging"
                        status = BatteryState.POWER_SUPPLY_STATUS_CHARGING
                    else:
                        curr_color = Colors.YELLOW
                        mode = "Discharging"
                        status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING

                    # --- Print to Terminal with Colors ---
                    print(f"\n{Colors.BLUE}{Colors.BOLD}--- JK BMS Status ---{Colors.ENDC}")
                    print(f"{Colors.CYAN}Voltage:{Colors.ENDC} {Colors.BOLD}{total_v}V{Colors.ENDC}")
                    print(f"{Colors.CYAN}Current:{Colors.ENDC} {curr_color}{current}A ({mode}){Colors.ENDC}")
                    print(f"{Colors.CYAN}Capacity:{Colors.ENDC} {Colors.GREEN}{soc}%{Colors.ENDC}")
                    print(f"{Colors.CYAN}Temps:   {Colors.ENDC} Probe1: {temp_1}°C | Probe2: {temp_2}°C | FET: {temp_fet}°C")
                    
                    print(f"{Colors.CYAN}Cells:{Colors.ENDC}", end=" ")
                    for i, v in enumerate(cell_voltages):
                        print(f"[{i+1}]: {v}V", end=" | ")
                    print("\n" + "-"*30)

                    # --- Populate ROS 2 Message ---
                    msg = BatteryState()
                    msg.header.stamp = self.get_clock().now().to_msg()
                    msg.header.frame_id = "bms_frame"
                    
                    # Derived from live cell voltages
                    cell_min_v = min(cell_voltages)
                    cell_max_v = max(cell_voltages)
                    cell_spread_v = cell_max_v - cell_min_v

                    # Power supply health classification
                    if cell_min_v < self.cell_uvp_v:
                        health = BatteryState.POWER_SUPPLY_HEALTH_DEAD
                    elif cell_max_v > self.cell_ovp_v:
                        health = BatteryState.POWER_SUPPLY_HEALTH_OVERVOLTAGE
                    elif temp_fet > 80 or temp_1 > 60 or temp_2 > 60:
                        health = BatteryState.POWER_SUPPLY_HEALTH_OVERHEAT
                    elif cell_spread_v > self.cell_balance_threshold_v * 4:
                        # 4x balance threshold = unhealthy spread
                        health = BatteryState.POWER_SUPPLY_HEALTH_UNSPEC_FAILURE
                    else:
                        health = BatteryState.POWER_SUPPLY_HEALTH_GOOD

                    # Build message
                    msg.voltage = float(total_v)
                    # ROS 2 standard: negative = discharging
                    msg.current = float(-current)
                    msg.percentage = soc / 100.0
                    msg.temperature = float((temp_1 + temp_2) / 2.0)
                    msg.cell_voltage = cell_voltages
                    msg.cell_temperature = [float(temp_1), float(temp_2), float(temp_fet)]

                    # Capacity (Ah) — design_capacity from params,
                    # current charge = design * SOC
                    msg.design_capacity = float(self.design_capacity_ah)
                    msg.capacity = float(self.design_capacity_ah)  # assume no aging info from JK
                    msg.charge = float(self.design_capacity_ah * (soc / 100.0))

                    msg.power_supply_status = status
                    msg.power_supply_health = health
                    msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LIFE
                    msg.present = True
                    msg.location = self.location_str
                    # Serial number unknown — leave default empty

                    self.battery_pub.publish(msg)

                    # Log min/max/spread for terminal too
                    print(f"{Colors.CYAN}Cell min:{Colors.ENDC} {cell_min_v:.3f}V  "
                          f"{Colors.CYAN}max:{Colors.ENDC} {cell_max_v:.3f}V  "
                          f"{Colors.CYAN}spread:{Colors.ENDC} {cell_spread_v*1000:.1f}mV  "
                          f"{Colors.CYAN}health:{Colors.ENDC} {self._health_str(health)}")

        except Exception as e:
            print(f"{Colors.RED}[ERROR] BMS Loop Fail: {e}{Colors.ENDC}")
            self.bms.reset_input_buffer()

def main(args=None):
    rclpy.init(args=args)
    node = JKBMSNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print(f"\n{Colors.YELLOW}Shutting down BMS Node...{Colors.ENDC}")
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
