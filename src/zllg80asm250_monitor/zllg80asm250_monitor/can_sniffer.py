#!/usr/bin/env python3
"""
ZLLG80ASM250 V1.0 - CAN Sniffer Node (ROS2)
Monitors and logs all CAN traffic for debugging

Author: Hype
Date: December 2024
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import can
import time
from collections import defaultdict


class CANSniffer(Node):
    """CAN Bus Sniffer for debugging"""
    
    def __init__(self):
        super().__init__('can_sniffer')
        
        # Declare parameters
        self.declare_parameter('can_channel', 'can2')
        self.declare_parameter('bitrate', 1000000)
        self.declare_parameter('show_extended', False)
        
        # Get parameters
        can_channel = self.get_parameter('can_channel').get_parameter_value().string_value
        bitrate = self.get_parameter('bitrate').get_parameter_value().integer_value
        self.show_extended = self.get_parameter('show_extended').get_parameter_value().bool_value
        
        # CAN Bus setup
        try:
            self.bus = can.Bus(interface='socketcan', channel=can_channel, bitrate=bitrate)
            self.get_logger().info(f'CAN Sniffer started on {can_channel} at {bitrate} bps')
        except Exception as e:
            self.get_logger().error(f'Failed to initialize CAN bus: {e}')
            raise
        
        # Statistics
        self.msg_count = defaultdict(int)
        self.start_time = time.time()
        
        # Publishers
        self.pub_raw = self.create_publisher(String, '/can/raw', 100)
        self.pub_stats = self.create_publisher(String, '/can/stats', 10)
        
        # Known ZLAC8015D message types
        self.known_ids = {
            0x000: "NMT",
            0x080: "SYNC",
            0x601: "SDO Tx Driver1",
            0x602: "SDO Tx Driver2",
            0x581: "SDO Rx Driver1",
            0x582: "SDO Rx Driver2",
            0x701: "Heartbeat Driver1",
            0x702: "Heartbeat Driver2",
            0x081: "Emergency Driver1",
            0x082: "Emergency Driver2",
        }
        
        # Timer for statistics
        self.stats_timer = self.create_timer(5.0, self.publish_statistics)
        
        # Timer for reading CAN messages
        self.read_timer = self.create_timer(0.001, self.read_can)
        
        self.get_logger().info('CAN Sniffer initialized - listening for messages...')
    
    def format_message(self, msg):
        """Format CAN message for display"""
        id_type = "EXT" if msg.is_extended_id else "STD"
        desc = self.known_ids.get(msg.arbitration_id, "")
        data_hex = ' '.join(f'{b:02X}' for b in msg.data)
        timestamp = time.time() - self.start_time
        return f"[{timestamp:8.3f}] {id_type} 0x{msg.arbitration_id:03X} [{len(msg.data)}] {data_hex:24s} {desc}"
    
    def decode_sdo_response(self, msg):
        """Decode SDO response message"""
        if len(msg.data) < 4:
            return ""
        
        cmd = msg.data[0]
        index = msg.data[1] | (msg.data[2] << 8)
        subindex = msg.data[3]
        
        cmd_names = {
            0x60: "Write OK",
            0x80: "ERROR",
            0x43: "Read 4B",
            0x47: "Read 3B",
            0x4B: "Read 2B",
            0x4F: "Read 1B",
        }
        
        cmd_name = cmd_names.get(cmd, f"CMD:{cmd:02X}")
        
        if cmd in [0x43, 0x47, 0x4B, 0x4F]:
            value = int.from_bytes(msg.data[4:8], 'little')
            return f" | {cmd_name} Index:0x{index:04X} Sub:{subindex} = {value}"
        
        return f" | {cmd_name} Index:0x{index:04X} Sub:{subindex}"
    
    def read_can(self):
        """Read CAN messages"""
        msg = self.bus.recv(timeout=0.001)
        if msg:
            # Filter extended IDs if not wanted
            if msg.is_extended_id and not self.show_extended:
                return
            
            # Update statistics
            self.msg_count[msg.arbitration_id] += 1
            
            # Format and publish
            formatted = self.format_message(msg)
            
            # Add SDO decoding for driver responses
            if msg.arbitration_id in [0x581, 0x582]:
                formatted += self.decode_sdo_response(msg)
            
            self.get_logger().info(formatted)
            
            raw_msg = String()
            raw_msg.data = formatted
            self.pub_raw.publish(raw_msg)
    
    def publish_statistics(self):
        """Publish message statistics"""
        stats_lines = ["=== CAN Bus Statistics ==="]
        stats_lines.append(f"Uptime: {time.time() - self.start_time:.1f}s")
        stats_lines.append(f"Total IDs seen: {len(self.msg_count)}")
        stats_lines.append("")
        stats_lines.append("ID           Count   Description")
        stats_lines.append("-" * 45)
        
        for arb_id in sorted(self.msg_count.keys()):
            count = self.msg_count[arb_id]
            desc = self.known_ids.get(arb_id, "Unknown")
            stats_lines.append(f"0x{arb_id:03X}        {count:6d}   {desc}")
        
        stats_str = '\n'.join(stats_lines)
        
        stats_msg = String()
        stats_msg.data = stats_str
        self.pub_stats.publish(stats_msg)
    
    def destroy_node(self):
        self.bus.shutdown()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = CANSniffer()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
