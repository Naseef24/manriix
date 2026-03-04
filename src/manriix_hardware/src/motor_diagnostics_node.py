#!/usr/bin/env python3
import can
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger, SetBool
import time

class MotorDiagnosticsNode(Node):
    def __init__(self):
        super().__init__('motor_diagnostics')
        
        # Initialize CAN bus
        try:
            self.bus = can.Bus(interface='socketcan', channel='can2', bitrate=1000000)
            self.get_logger().info('CAN bus initialized on can2')
        except can.CanError as e:
            self.get_logger().error(f"Failed to initialize CAN bus: {e}")
            raise

        self.driver1_id = 0x601
        self.driver2_id = 0x602

        # Create services
        self.reset_service = self.create_service(
            Trigger, 'reset_driver', self.reset_driver_callback)
        self.initiate_service = self.create_service(
            Trigger, 'initiate_motor', self.initiate_motor_callback)        
        self.fault_service = self.create_service(
            Trigger, 'get_fault_info', self.get_fault_info_callback)
        self.halt_service = self.create_service(
            SetBool, 'halt_motor', self.halt_motor_callback)
        self.park_service = self.create_service(
            SetBool, 'set_park_mode', self.set_park_mode_callback)
        self.temp_service = self.create_service(
            Trigger, 'get_motor_temperature', self.get_temperature_callback)

        self.get_logger().info('Motor Diagnostics Node ready')
        self.get_logger().info('Services:')
        self.get_logger().info('  - /reset_driver')
        self.get_logger().info('  - /initiate_motor')
        self.get_logger().info('  - /get_fault_info')
        self.get_logger().info('  - /halt_motor')
        self.get_logger().info('  - /set_park_mode')
        self.get_logger().info('  - /get_motor_temperature')

    def send_can_message(self, cob_id, data):
        """Send CAN message"""
        msg = can.Message(arbitration_id=cob_id, data=data, is_extended_id=False)
        try:
            self.bus.send(msg)
            return True
        except can.CanError as e:
            self.get_logger().error(f"Failed to send CAN message: {e}")
            return False

    def reset_driver_callback(self, request, response):
        """Reset motor drivers and clear faults"""
        self.get_logger().info('Resetting drivers...')
        
        try:
            for driver_id in [self.driver1_id, self.driver2_id]:
                # Send reset command
                self.send_can_message(driver_id, [0x2B, 0x40, 0x60, 0x00, 0x80, 0x00, 0x00, 0x00])
                time.sleep(0.1)
                
                # NMT reset
                self.send_can_message(0x000, [0x81, driver_id & 0xFF])
                time.sleep(0.5)
                
                # Reinitialize
                self.send_can_message(driver_id, [0x2F, 0x60, 0x60, 0x00, 0x03, 0x00, 0x00, 0x00])
                time.sleep(0.1)
                self.send_can_message(driver_id, [0x2B, 0x40, 0x60, 0x00, 0x06, 0x00, 0x00, 0x00])
                time.sleep(0.1)
                self.send_can_message(driver_id, [0x2B, 0x40, 0x60, 0x00, 0x07, 0x00, 0x00, 0x00])
                time.sleep(0.1)
                self.send_can_message(driver_id, [0x2B, 0x40, 0x60, 0x00, 0x0F, 0x00, 0x00, 0x00])
                time.sleep(0.1)
            
            response.success = True
            response.message = "Drivers reset successfully"
            self.get_logger().info('Reset completed')
        except Exception as e:
            response.success = False
            response.message = f"Reset failed: {str(e)}"
            self.get_logger().error(f'Reset failed: {e}')
        
        return response

    def initiate_motor_callback(self, request, response):
        """Initiate motor drivers"""
        self.get_logger().info('Initiating motors...')
        
        try:
            for driver_id in [self.driver1_id, self.driver2_id]:
                self.send_can_message(driver_id, [0x2F, 0x60, 0x60, 0x00, 0x03, 0x00, 0x00, 0x00])
                time.sleep(0.1)
                self.send_can_message(driver_id, [0x2B, 0x40, 0x60, 0x00, 0x06, 0x00, 0x00, 0x00])
                time.sleep(0.1)
                self.send_can_message(driver_id, [0x2B, 0x40, 0x60, 0x00, 0x07, 0x00, 0x00, 0x00])
                time.sleep(0.1)
                self.send_can_message(driver_id, [0x2B, 0x40, 0x60, 0x00, 0x0F, 0x00, 0x00, 0x00])
                time.sleep(0.1)
            
            response.success = True
            response.message = "Motors initiated successfully"
            self.get_logger().info('Initiation completed')
        except Exception as e:
            response.success = False
            response.message = f"Initiation failed: {str(e)}"
            self.get_logger().error(f'Initiation failed: {e}')
        
        return response

    def get_fault_info_callback(self, request, response):
        """Read fault information from drivers"""
        self.get_logger().info('Reading fault info...')
        
        fault_descriptions = {
            0x00000000: "No error",
            0x00000001: "Overvoltage",
            0x00000002: "Under voltage",
            0x00000100: "EEPROM read/write error",
            0x00000004: "Left motor overcurrent",
            0x00000008: "Left motor overload",
            0x00000400: "Left motor temperature too high",
            0x00040000: "Right motor overcurrent",
            0x00080000: "Right motor overload",
            0x04000000: "Right motor temperature too high",
        }
        
        info_lines = []
        
        try:
            for node_id in [self.driver1_id, self.driver2_id]:
                # Read fault code
                self.send_can_message(node_id, [0x40, 0x3F, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00])
                fault_response = self.bus.recv(timeout=1.0)
                
                # Read status word
                self.send_can_message(node_id, [0x40, 0x41, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00])
                status_response = self.bus.recv(timeout=1.0)
                
                if fault_response and status_response:
                    fault_code = int.from_bytes(fault_response.data[-4:], byteorder='little')
                    status_word = int.from_bytes(status_response.data[-2:], byteorder='little')
                    
                    info_lines.append(f"\n=== Driver 0x{node_id:03X} ===")
                    info_lines.append(f"Fault Code: 0x{fault_code:08X}")
                    
                    if fault_code in fault_descriptions:
                        info_lines.append(f"Description: {fault_descriptions[fault_code]}")
                    else:
                        info_lines.append("Description: Unknown fault")
                    
                    info_lines.append(f"Status: 0x{status_word:04X}")
                    if status_word & 0x0008:
                        info_lines.append("  ⚠ FAULT ACTIVE")
                    if status_word & 0x0004:
                        info_lines.append("  ✓ Operation enabled")
            
            response.success = True
            response.message = "\n".join(info_lines)
            self.get_logger().info('Fault info retrieved')
        except Exception as e:
            response.success = False
            response.message = f"Failed to read fault info: {str(e)}"
            self.get_logger().error(f'Fault info read failed: {e}')
        
        return response

    def get_temperature_callback(self, request, response):
        """Read motor temperatures"""
        self.get_logger().info('Reading temperatures...')
        
        temp_lines = []
        
        try:
            for node_id in [self.driver1_id, self.driver2_id]:
                # Read left motor temperature
                self.send_can_message(node_id, [0x40, 0x32, 0x20, 0x01, 0x00, 0x00, 0x00, 0x00])
                temp_left_response = self.bus.recv(timeout=1.0)
                
                # Read right motor temperature
                self.send_can_message(node_id, [0x40, 0x32, 0x20, 0x02, 0x00, 0x00, 0x00, 0x00])
                temp_right_response = self.bus.recv(timeout=1.0)
                
                # Read bus voltage
                self.send_can_message(node_id, [0x40, 0x35, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00])
                voltage_response = self.bus.recv(timeout=1.0)
                
                if temp_left_response and temp_right_response:
                    temp_left = int.from_bytes(temp_left_response.data[-2:], byteorder='little', signed=True) * 0.1
                    temp_right = int.from_bytes(temp_right_response.data[-2:], byteorder='little', signed=True) * 0.1
                    
                    temp_lines.append(f"\n=== Driver 0x{node_id:03X} ===")
                    temp_lines.append(f"Motor 1 temp: {temp_left:.1f}°C")
                    temp_lines.append(f"Motor 2 temp: {temp_right:.1f}°C")
                    
                    if voltage_response:
                        voltage = int.from_bytes(voltage_response.data[-2:], byteorder='little') * 0.01
                        temp_lines.append(f"Bus voltage: {voltage:.1f}V")
            
            response.success = True
            response.message = "\n".join(temp_lines)
            self.get_logger().info('Temperature retrieved')
        except Exception as e:
            response.success = False
            response.message = f"Failed to read temperature: {str(e)}"
            self.get_logger().error(f'Temperature read failed: {e}')
        
        return response

    def halt_motor_callback(self, request, response):
        """Halt or resume motor control"""
        halt_mode = 1 if request.data else 0
        halt_command = [0x2B, 0x5D, 0x60, 0x00, halt_mode, 0x00, 0x00, 0x00]
        
        try:
            for driver_id in [self.driver1_id, self.driver2_id]:
                self.send_can_message(driver_id, halt_command)
            
            action = "halted" if halt_mode else "resumed"
            response.success = True
            response.message = f"Motors {action}"
            self.get_logger().info(f'Motors {action}')
        except Exception as e:
            response.success = False
            response.message = f"Halt failed: {str(e)}"
            self.get_logger().error(f'Halt failed: {e}')
        
        return response

    def set_park_mode_callback(self, request, response):
        """Enable or disable park mode"""
        park_mode = 1 if request.data else 0
        park_command = [0x2B, 0x26, 0x20, 0x04, park_mode, 0x00, 0x00, 0x00]
        save_command = [0x2B, 0x10, 0x20, 0x00, 0x01, 0x00, 0x00, 0x00]
        
        try:
            for driver_id in [self.driver1_id, self.driver2_id]:
                self.send_can_message(driver_id, park_command)
                self.send_can_message(driver_id, save_command)
            
            action = "enabled" if park_mode else "disabled"
            response.success = True
            response.message = f"Park mode {action}"
            self.get_logger().info(f'Park mode {action}')
        except Exception as e:
            response.success = False
            response.message = f"Park mode failed: {str(e)}"
            self.get_logger().error(f'Park mode failed: {e}')
        
        return response

    def shutdown(self):
        """Cleanup on shutdown"""
        self.get_logger().info('Shutting down diagnostics node')
        self.bus.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = MotorDiagnosticsNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
