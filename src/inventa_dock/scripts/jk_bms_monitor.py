#!/usr/bin/env python3

import rospy
import serial
import struct
import time
from sensor_msgs.msg import BatteryState

class JKBMS:
    def __init__(self, port="/dev/ttyUSB0", baudrate=9600):
        """Initialize the JKBMS communication class"""
        self.ser = serial.Serial(
            port=port,
            baudrate=baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=1
        )
        
        # Constants from the protocol
        self.START_BYTE = 0xDD
        self.STOP_BYTE = 0x77
        self.READ_STATUS = 0xA5
        self.CMD_READ_BASIC_INFO = 0x03

    def calculate_checksum(self, data):
        """Calculate the checksum as per protocol (inverse of sum + 1)"""
        checksum = sum(data)
        inverse = (~checksum) & 0xFFFF  # 16-bit inverse
        result = (inverse + 1) & 0xFFFF  # Add 1 and keep 16 bits
        return [(result >> 8) & 0xFF, result & 0xFF]  # High byte first, low byte last

    def send_command(self, status, command, data=None):
        """Send a command to the BMS"""
        if data is None:
            data = []
        
        # Create message
        msg = [self.START_BYTE, status, command, len(data)]
        msg.extend(data)
        
        # Calculate checksum (using command + data length + data)
        checksum_data = [command, len(data)]
        checksum_data.extend(data)
        checksum = self.calculate_checksum(checksum_data)
        
        # Add checksum and stop byte
        msg.extend(checksum)
        msg.append(self.STOP_BYTE)
        
        # Send message
        self.ser.write(bytes(msg))
        
        # Wait for response
        time.sleep(0.1)
        
        # Read response
        return self.read_response()
    
    def read_response(self):
        """Read and parse the response from the BMS"""
        # Wait for start byte
        start_byte = self.ser.read(1)
        if not start_byte or start_byte[0] != self.START_BYTE:
            rospy.logwarn("Error: No valid start byte received")
            return None
        
        # Read command code
        cmd_code = self.ser.read(1)
        if not cmd_code:
            rospy.logwarn("Error: No command code received")
            return None
        
        # Read status
        status = self.ser.read(1)
        if not status:
            rospy.logwarn("Error: No status received")
            return None
        
        # Read data length
        data_len = self.ser.read(1)
        if not data_len:
            rospy.logwarn("Error: No data length received")
            return None
        
        data_length = data_len[0]
        
        # Read data content
        data = self.ser.read(data_length)
        if len(data) != data_length:
            rospy.logwarn(f"Error: Expected {data_length} bytes of data, got {len(data)}")
            return None
        
        # Read checksum
        checksum = self.ser.read(2)
        if len(checksum) != 2:
            rospy.logwarn("Error: No or incomplete checksum received")
            return None
        
        # Read stop byte
        stop_byte = self.ser.read(1)
        if not stop_byte or stop_byte[0] != self.STOP_BYTE:
            rospy.logwarn("Error: No valid stop byte received")
            return None
        
        # Return parsed data
        return {
            'command': cmd_code[0],
            'status': status[0],
            'data': data
        }
    
    def read_battery_status(self):
        """Read battery status from the BMS"""
        response = self.send_command(self.READ_STATUS, self.CMD_READ_BASIC_INFO)
        
        if not response or response['status'] != 0x00:
            rospy.logwarn("Error reading battery status")
            return None
        
        data = response['data']
        
        if len(data) < 23:  # Minimum expected data length
            rospy.logwarn(f"Error: Battery status data too short ({len(data)})")
            return None
        
        try:
            # Parse the data according to protocol
            total_voltage = struct.unpack('>H', data[0:2])[0] * 10  # 10mV unit -> mV
            current = struct.unpack('>h', data[2:4])[0] * 10  # 10mA unit -> mA, signed
            remaining_capacity = struct.unpack('>H', data[4:6])[0] * 10  # 10mAh unit -> mAh
            nominal_capacity = struct.unpack('>H', data[6:8])[0] * 10  # 10mAh unit -> mAh
            cycles = struct.unpack('>H', data[8:10])[0]
            
            # Skip some fields
            protection_status = struct.unpack('>H', data[16:18])[0]
            
            # Battery percentage is at position 19
            battery_percent = data[19]
            
            fet_status = data[20]
            charging_mosfet_on = bool(fet_status & 0x01)
            discharging_mosfet_on = bool(fet_status & 0x02)
            
            # Determine power supply status for BatteryState message
            if current > 0:
                # Charging
                power_supply_status = BatteryState.POWER_SUPPLY_STATUS_CHARGING
            elif current < 0:
                # Discharging
                power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
            elif charging_mosfet_on and discharging_mosfet_on:
                # Idle but connected
                power_supply_status = BatteryState.POWER_SUPPLY_STATUS_NOT_CHARGING
            else:
                # Disconnected
                power_supply_status = BatteryState.POWER_SUPPLY_STATUS_UNKNOWN
                
            # Determine health based on protection status
            if protection_status > 0:
                power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_DEAD
            else:
                power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_GOOD
            
            # Create a dictionary with the relevant data
            return {
                'voltage': total_voltage / 1000.0,             # Volts
                'current': current / 1000.0,                   # Amps
                'charge': remaining_capacity / 1000.0,         # Ah
                'capacity': nominal_capacity / 1000.0,         # Ah
                'percentage': battery_percent / 100.0,         # 0.0 to 1.0
                'power_supply_status': power_supply_status,    # For BatteryState message
                'power_supply_health': power_supply_health,    # For BatteryState message
                'charging': charging_mosfet_on,
                'discharging': discharging_mosfet_on,
                'cycles': cycles
            }
            
        except Exception as e:
            rospy.logerr(f"Error parsing battery data: {e}")
            return None
    
    def close(self):
        """Close the serial connection"""
        if self.ser and self.ser.is_open:
            self.ser.close()


class JKBMSNode:
    def __init__(self):
        """Initialize the ROS node"""
        rospy.init_node('jk_bms_node', anonymous=True)
        
        # Get parameters
        self.port = rospy.get_param('~port', '/dev/ttyUSB0')
        self.baud_rate = rospy.get_param('~baud_rate', 9600)
        self.update_rate = rospy.get_param('~update_rate', 1.0)  # Hz
        
        # Initialize BMS communication
        try:
            self.bms = JKBMS(port=self.port, baudrate=self.baud_rate)
            rospy.loginfo(f"Connected to JK BMS on {self.port}")
        except Exception as e:
            rospy.logerr(f"Failed to connect to JK BMS: {e}")
            raise
        
        # Create publisher for battery status
        self.battery_pub = rospy.Publisher('/battery_status', BatteryState, queue_size=10)
        
        # Log startup
        rospy.loginfo(f"JK BMS node initialized, publishing to /battery_status")
        
    def run(self):
        """Main loop for the node"""
        rate = rospy.Rate(self.update_rate)
        
        while not rospy.is_shutdown():
            try:
                # Get battery status from BMS
                status = self.bms.read_battery_status()
                
                if status:
                    # Create BatteryState message
                    msg = BatteryState()
                    msg.header.stamp = rospy.Time.now()
                    msg.voltage = status['voltage']
                    msg.current = status['current']
                    msg.charge = status['charge']
                    msg.capacity = status['capacity']
                    msg.design_capacity = status['capacity']
                    msg.percentage = status['percentage']
                    msg.power_supply_status = status['power_supply_status']
                    msg.power_supply_health = status['power_supply_health']
                    msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LION
                    msg.present = True
                    
                    # Publish the message
                    self.battery_pub.publish(msg)
                    
                    # Log battery status
                    rospy.loginfo(f"Battery: {status['percentage']*100:.1f}%, " + 
                                  f"{status['voltage']:.2f}V, {status['current']:.2f}A, " +
                                  f"{'Charging' if status['charging'] else 'Discharging' if status['discharging'] else 'Idle'}")
                else:
                    rospy.logwarn("Failed to read battery status")
                
            except Exception as e:
                rospy.logerr(f"Error in main loop: {e}")
            
            rate.sleep()
    
    def shutdown(self):
        """Clean shutdown"""
        rospy.loginfo("Shutting down JK BMS node")
        self.bms.close()


if __name__ == "__main__":
    try:
        node = JKBMSNode()
        rospy.on_shutdown(node.shutdown)
        node.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"Unexpected error: {e}")