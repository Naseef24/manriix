#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from dji_rs3pro_ros_controller.srv import SetFocusPosition

# Since we're using ros2_socketcan, we don't use python-can directly
# Instead, we'll publish to the CAN topic
from can_msgs.msg import Frame


class FocusController(Node):
    def __init__(self):
        super().__init__('focus_controller')
        
        # Current position
        self.current_position = 0
        
        # Create service
        self.focus_service = self.create_service(
            SetFocusPosition,
            'set_focus_position',
            self.handle_set_focus_position
        )
        
        # Publisher for CAN messages
        self.can_pub = self.create_publisher(Frame, 'to_can_bus', 10)
        
        # Timer to send messages at 100Hz
        self.timer = self.create_timer(0.01, self.timer_callback)  # 100 Hz
        
        self.get_logger().info("Focus Controller initialized")
    
    def handle_set_focus_position(self, request, response):
        position = request.position
        if 0 <= position <= 4095:
            self.current_position = position
            self.send_focus_command(self.current_position)
            self.get_logger().info(f"Focus position set to {position}")
            response.success = True
            response.message = "Focus position set successfully"
        else:
            self.get_logger().warn(f"Invalid focus position requested: {position}")
            response.success = False
            response.message = "Invalid focus position. Must be between 0 and 4095"
        
        return response
    
    def timer_callback(self):
        self.send_focus_command(self.current_position)
    
    def send_focus_command(self, position):
        # Prepare the command
        cmd_set = 0x0E
        cmd_id = 0x12
        sub_cmd_id = 0x01  # Focus Motor position control
        control_type = 0x00  # Focus control
        data_length = 0x02  # Two-byte length
        
        # Create the message data
        data = [
            cmd_set,
            cmd_id,
            sub_cmd_id,
            control_type,
            data_length,
            position & 0xFF,  # Low byte of position
            (position >> 8) & 0xFF,  # High byte of position
            0  # Padding to make 8 bytes
        ]
        
        # Create and send the CAN message
        msg = Frame()
        msg.id = 0x222
        msg.is_rtr = False
        msg.is_extended = False
        msg.is_error = False
        msg.dlc = 8
        msg.data = data
        
        try:
            self.can_pub.publish(msg)
            self.get_logger().debug(f"Sent focus position command: {position}")
        except Exception as e:
            self.get_logger().error(f"Failed to send focus position command: {e}")


def main(args=None):
    rclpy.init(args=args)
    controller = FocusController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
