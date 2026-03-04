#!/usr/bin/env python3
"""
ROS 2 Bridge Node - Translates between MQTT (vision system) and ROS 2

This node acts as a bridge between the standalone vision system (communicating via MQTT)
and the ROS 2 ecosystem. It subscribes to MQTT topics published by the vision system
and republishes them as ROS 2 topics, and vice versa.

MQTT Topics (Vision → ROS 2):
    vision/detection/results    -> /vision/detection/results (custom msg)
    vision/gimbal/target        -> /face_angle_position (Point)
    vision/photo/request        -> /capture_image_topic (String)
    vision/video/request        -> /capture_video_topic (String)
    vision/zoom/value           -> /set_focus_position (Int32)

ROS 2 Topics (ROS 2 → Vision):
    /gimbalTelemetry           -> ros2/gimbal/telemetry (MQTT)
    /camera_node/busy          -> ros2/camera/busy (MQTT)
    /photo/command             -> photography/control/capture (MQTT: 'yes'→'s', 'no'→'n')
    /human_clustering/start_scan -> ros2/mode/control (MQTT)
    /human_clustering/start_attn_photo_capture -> ros2/mode/control (MQTT)

Author: AI Photography System
Date: November 2025
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import Int32, String, Bool
import paho.mqtt.client as mqtt
import json
import threading
import time


class VisionBridgeNode(Node):
    """
    ROS 2 Bridge Node for Vision System
    
    Bridges communication between MQTT-based vision system and ROS 2 topics.
    Maintains backward compatibility with existing ROS 2 interfaces.
    """
    
    def __init__(self, mqtt_broker='localhost', mqtt_port=1883):
        super().__init__('vision_bridge_node')
        
        self.get_logger().info('🚀 Initializing Vision Bridge Node...')
        
        # ========================================
        # ROS 2 Publishers (Vision → ROS 2)
        # ========================================
        self.gimbal_target_pub = self.create_publisher(
            Point, '/face_angle_position', 10
        )
        self.get_logger().info('✅ Publisher: /face_angle_position (Point)')
        
        self.focus_pub = self.create_publisher(
            Int32, '/set_focus_position', 1
        )
        self.get_logger().info('✅ Publisher: /set_focus_position (Int32)')
        
        self.photo_pub = self.create_publisher(
            String, '/capture_image_topic', 1
        )
        self.get_logger().info('✅ Publisher: /capture_image_topic (String)')
        
        self.video_pub = self.create_publisher(
            String, '/capture_video_topic', 1
        )
        self.get_logger().info('✅ Publisher: /capture_video_topic (String)')
        
        self.scan_status_pub = self.create_publisher(
            String, '/scan_status', 1
        )
        self.get_logger().info('✅ Publisher: /scan_status (String)')
        
        # Optional: Publisher for detection results (if you want to expose them in ROS 2)
        self.detection_pub = self.create_publisher(
            String, '/vision/detection/results', 10
        )
        self.get_logger().info('✅ Publisher: /vision/detection/results (String)')
        
        # ========================================
        # ROS 2 Subscribers (ROS 2 → Vision)
        # ========================================
        self.gimbal_telemetry_sub = self.create_subscription(
            Point, '/gimbalTelemetry', 
            self.gimbal_telemetry_callback, 10
        )
        self.get_logger().info('✅ Subscriber: /gimbalTelemetry (Point)')
        
        self.start_scan_sub = self.create_subscription(
            String, '/human_clustering/start_scan', 
            self.start_scan_callback, 10
        )
        self.get_logger().info('✅ Subscriber: /human_clustering/start_scan (String)')
        
        self.start_attention_sub = self.create_subscription(
            String, '/human_clustering/start_attn_photo_capture',
            self.start_attention_callback, 10
        )
        self.get_logger().info('✅ Subscriber: /human_clustering/start_attn_photo_capture (String)')
        
        self.camera_busy_sub = self.create_subscription(
            Bool, '/camera_node/busy', 
            self.camera_busy_callback, 10
        )
        self.get_logger().info('✅ Subscriber: /camera_node/busy (Bool)')
        
        self.photo_command_sub = self.create_subscription(
            String, '/photo/command',
            self.photo_command_callback, 10
        )
        self.get_logger().info('✅ Subscriber: /photo/command (String)')
        
        # ========================================
        # MQTT Client Setup
        # ========================================
        self.mqtt_broker = mqtt_broker
        self.mqtt_port = mqtt_port
        self.mqtt_client = mqtt.Client(client_id="ros2_vision_bridge")
        self.mqtt_client.on_connect = self.on_mqtt_connect
        self.mqtt_client.on_message = self.on_mqtt_message
        self.mqtt_client.on_disconnect = self.on_mqtt_disconnect
        self.mqtt_connected = False
        
        # Start MQTT connection in separate thread
        self.mqtt_thread = threading.Thread(target=self._connect_mqtt, daemon=True)
        self.mqtt_thread.start()
        
        # Statistics
        self.mqtt_rx_count = 0
        self.mqtt_tx_count = 0
        self.ros2_rx_count = 0
        self.ros2_tx_count = 0
        
        # Create timer for status reporting
        self.status_timer = self.create_timer(10.0, self.report_status)
        
        self.get_logger().info('✅ ROS 2 Vision Bridge initialized')
    
    def _connect_mqtt(self):
        """Connect to MQTT broker (runs in separate thread)"""
        retry_delay = 5
        while rclpy.ok():
            try:
                self.get_logger().info(f'🔌 Connecting to MQTT broker at {self.mqtt_broker}:{self.mqtt_port}...')
                self.mqtt_client.connect(self.mqtt_broker, self.mqtt_port, 60)
                self.mqtt_client.loop_start()
                break
            except Exception as e:
                self.get_logger().error(f'❌ MQTT connection failed: {e}. Retrying in {retry_delay}s...')
                time.sleep(retry_delay)
    
    def on_mqtt_connect(self, client, userdata, flags, rc):
        """MQTT connection callback"""
        if rc == 0:
            self.mqtt_connected = True
            self.get_logger().info("✅ Connected to MQTT broker")
            
            # Subscribe to vision system output topics
            topics = [
                "vision/detection/results",
                "vision/gimbal/target",
                "vision/photo/request",
                "vision/video/request",
                "vision/zoom/value",
                "vision/status"
            ]
            
            for topic in topics:
                client.subscribe(topic)
                self.get_logger().info(f'✅ MQTT subscribed: {topic}')
                
        else:
            self.mqtt_connected = False
            self.get_logger().error(f"❌ MQTT connection failed with code {rc}")
    
    def on_mqtt_disconnect(self, client, userdata, rc):
        """MQTT disconnection callback"""
        self.mqtt_connected = False
        if rc != 0:
            self.get_logger().warning(f"⚠️ MQTT disconnected unexpectedly (code: {rc}). Reconnecting...")
            # Client will auto-reconnect
    
    def on_mqtt_message(self, client, userdata, msg):
        """Handle incoming MQTT messages from vision system"""
        try:
            topic = msg.topic
            payload = json.loads(msg.payload.decode())
            self.mqtt_rx_count += 1
            
            if topic == "vision/gimbal/target":
                self._handle_gimbal_target(payload)
            
            elif topic == "vision/photo/request":
                self._handle_photo_request(payload)
            
            elif topic == "vision/video/request":
                self._handle_video_request(payload)
            
            elif topic == "vision/zoom/value":
                self._handle_zoom_value(payload)
            
            elif topic == "vision/status":
                self._handle_vision_status(payload)
            
            elif topic == "vision/detection/results":
                self._handle_detection_results(payload)
                
        except json.JSONDecodeError as e:
            self.get_logger().error(f"❌ Failed to decode MQTT JSON: {e}")
        except Exception as e:
            self.get_logger().error(f"❌ Error handling MQTT message from {topic}: {e}")
    
    # ========================================
    # MQTT → ROS 2 Message Handlers
    # ========================================
    
    def _handle_gimbal_target(self, payload):
        """Publish gimbal target position to ROS 2 (zoom handled separately by _handle_zoom_value)"""
        try:
            # Publish gimbal position
            point_msg = Point()
            point_msg.x = float(payload.get('center_x', 0.0))
            point_msg.y = float(payload.get('center_y', 0.0))
            point_msg.z = 0.0
            self.gimbal_target_pub.publish(point_msg)
            
            # Zoom is handled separately by vision/zoom/value topic
            # to avoid duplicate/conflicting zoom commands
            
            self.ros2_tx_count += 1
            
        except Exception as e:
            self.get_logger().error(f"❌ Error handling gimbal target: {e}")
    
    def _handle_photo_request(self, payload):
        """Publish photo capture request to ROS 2"""
        try:
            photo_msg = String()
            photo_msg.data = payload.get('trigger', 'n')
            self.photo_pub.publish(photo_msg)
            self.ros2_tx_count += 1
            
            if photo_msg.data == 'y':
                self.get_logger().info('📸 Photo capture requested')
                
        except Exception as e:
            self.get_logger().error(f"❌ Error handling photo request: {e}")
    
    def _handle_video_request(self, payload):
        """Publish video recording request to ROS 2"""
        try:
            video_msg = String()
            video_msg.data = payload.get('trigger', 'n')
            self.video_pub.publish(video_msg)
            self.ros2_tx_count += 1
            
            if video_msg.data == 'y':
                self.get_logger().info('🎥 Video recording requested')
                
        except Exception as e:
            self.get_logger().error(f"❌ Error handling video request: {e}")
    
    def _handle_zoom_value(self, payload):
        """Publish zoom/focus value to ROS 2"""
        try:
            focus_msg = Int32()
            zoom_value = payload.get('zoom_value', 0)
            focus_msg.data = int(8.33 * zoom_value)
            self.focus_pub.publish(focus_msg)
            self.ros2_tx_count += 1
            
        except Exception as e:
            self.get_logger().error(f"❌ Error handling zoom value: {e}")
    
    def _handle_vision_status(self, payload):
        """Publish vision system status to ROS 2"""
        try:
            status_msg = String()
            status_msg.data = payload.get('scan_status', 'not_scanning')
            self.scan_status_pub.publish(status_msg)
            self.ros2_tx_count += 1
            
        except Exception as e:
            self.get_logger().error(f"❌ Error handling vision status: {e}")
    
    def _handle_detection_results(self, payload):
        """Publish detection results to ROS 2 (as JSON string)"""
        try:
            # Publish as JSON string for flexibility
            detection_msg = String()
            detection_msg.data = json.dumps(payload)
            self.detection_pub.publish(detection_msg)
            self.ros2_tx_count += 1
            
            # Log if match found
            if payload.get('found_match'):
                matching_phrase = payload.get('matching_phrase', 'unknown')
                self.get_logger().info(f'🎯 Detection match: {matching_phrase}')
                
        except Exception as e:
            self.get_logger().error(f"❌ Error handling detection results: {e}")
    
    # ========================================
    # ROS 2 → MQTT Callbacks
    # ========================================
    
    def gimbal_telemetry_callback(self, msg):
        """Forward gimbal telemetry from ROS 2 to vision system via MQTT"""
        try:
            payload = json.dumps({
                'yaw': float(msg.x),
                'pitch': float(msg.z),
                'roll': float(msg.y),
                'timestamp': time.time()
            })
            self.mqtt_client.publish("ros2/gimbal/telemetry", payload)
            self.mqtt_tx_count += 1
            self.ros2_rx_count += 1
            
        except Exception as e:
            self.get_logger().error(f"❌ Error forwarding gimbal telemetry: {e}")
    
    def start_scan_callback(self, msg):
        """Forward scan start command from ROS 2 to vision system via MQTT"""
        try:
            payload = json.dumps({
                'start_scan': msg.data,
                'start_attention': 'no',
                'timestamp': time.time()
            })
            self.mqtt_client.publish("ros2/mode/control", payload)
            self.mqtt_tx_count += 1
            self.ros2_rx_count += 1
            
            self.get_logger().info(f'🔄 Scan mode: {msg.data}')
            
        except Exception as e:
            self.get_logger().error(f"❌ Error forwarding scan command: {e}")
    
    def start_attention_callback(self, msg):
        """Forward attention mode command from ROS 2 to vision system via MQTT"""
        try:
            payload = json.dumps({
                'start_scan': 'no',
                'start_attention': msg.data,
                'timestamp': time.time()
            })
            self.mqtt_client.publish("ros2/mode/control", payload)
            self.mqtt_tx_count += 1
            self.ros2_rx_count += 1
            
            self.get_logger().info(f'🎯 Attention mode: {msg.data}')
            
        except Exception as e:
            self.get_logger().error(f"❌ Error forwarding attention command: {e}")
    
    def camera_busy_callback(self, msg):
        """Forward camera busy status from ROS 2 to vision system via MQTT"""
        try:
            payload = json.dumps({
                'busy': 'true' if msg.data else 'false',
                'timestamp': time.time()
            })
            self.mqtt_client.publish("ros2/camera/busy", payload)
            self.mqtt_tx_count += 1
            self.ros2_rx_count += 1
            
        except Exception as e:
            self.get_logger().error(f"❌ Error forwarding camera busy status: {e}")
    
    def photo_command_callback(self, msg):
        """
        Forward photo capture control command from ROS 2 to vision system via MQTT.
        
        ROS 2 Input: /photo/command (String) - 'yes' or 'no'
        MQTT Output: photography/control/capture - 's' (start) or 'n' (stop)
        
        Conversion:
            'yes' → 's' (start capture)
            'no'  → 'n' (stop capture)
        """
        try:
            ros_command = msg.data.lower().strip()
            
            # Convert ROS 2 command to MQTT command
            if ros_command == 'yes':
                mqtt_command = 's'
                action = "START"
            elif ros_command == 'no':
                mqtt_command = 'n'
                action = "STOP"
            else:
                self.get_logger().warning(f"⚠️ Unknown photo command: '{ros_command}' (expected 'yes' or 'no')")
                return
            
            # Publish to MQTT (raw string, not JSON)
            self.mqtt_client.publish("photography/control/capture", mqtt_command)
            self.mqtt_tx_count += 1
            self.ros2_rx_count += 1
            
            self.get_logger().info(f'📸 Capture control: {action} (ROS2: "{ros_command}" → MQTT: "{mqtt_command}")')
            
        except Exception as e:
            self.get_logger().error(f"❌ Error forwarding photo command: {e}")
    
    # ========================================
    # Status Reporting
    # ========================================
    
    def report_status(self):
        """Periodically report bridge status"""
        mqtt_status = "🟢 Connected" if self.mqtt_connected else "🔴 Disconnected"
        self.get_logger().info(
            f'📊 Bridge Status: MQTT {mqtt_status} | '
            f'RX: MQTT={self.mqtt_rx_count} ROS2={self.ros2_rx_count} | '
            f'TX: MQTT={self.mqtt_tx_count} ROS2={self.ros2_tx_count}'
        )
    
    def destroy_node(self):
        """Cleanup on shutdown"""
        self.get_logger().info('🛑 Shutting down Vision Bridge Node...')
        self.mqtt_client.loop_stop()
        self.mqtt_client.disconnect()
        super().destroy_node()


def main(args=None):
    """Main entry point for ROS 2 bridge node"""
    rclpy.init(args=args)
    
    # Create bridge node with configurable MQTT broker
    # You can pass these as ROS 2 parameters if needed
    node = VisionBridgeNode(
        mqtt_broker='localhost',
        mqtt_port=1883
    )
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('⚠️ Keyboard interrupt received')
    except Exception as e:
        node.get_logger().error(f'❌ Bridge node error: {e}')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
