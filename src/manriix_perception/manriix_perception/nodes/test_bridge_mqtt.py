#!/usr/bin/env python3
"""
Test script to monitor MQTT messages from photo_capture_bridge_node

Usage:
    python3 test_bridge_mqtt.py

Expected output:
    Received tracking data with detections from cameras
    
Author: Vision Team
Date: 2026-02-23
"""

import paho.mqtt.client as mqtt
import json
import time
from datetime import datetime


class BridgeTester:
    """Simple MQTT subscriber to test bridge output"""
    
    def __init__(self, mqtt_host='localhost', mqtt_port=1883):
        self.mqtt_host = mqtt_host
        self.mqtt_port = mqtt_port
        self.topic = '/photo_capture/tracking_results'
        
        self.message_count = 0
        self.last_message_time = 0
        self.detection_count = 0
        
        # Setup MQTT client
        self.client = mqtt.Client()
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message
        
    def on_connect(self, client, userdata, flags, rc):
        """Callback when connected to MQTT broker"""
        if rc == 0:
            print(f"✅ Connected to MQTT broker at {self.mqtt_host}:{self.mqtt_port}")
            client.subscribe(self.topic)
            print(f"📡 Subscribed to topic: {self.topic}")
            print("\nWaiting for messages... (Press Ctrl+C to stop)\n")
        else:
            print(f"❌ Connection failed with code {rc}")
    
    def on_message(self, client, userdata, msg):
        """Callback when message received"""
        try:
            # Parse JSON payload
            data = json.loads(msg.payload.decode())
            
            # Update stats
            self.message_count += 1
            self.last_message_time = time.time()
            
            # Extract info
            timestamp = data.get('timestamp', 0)
            total_dets = data.get('total_detections', 0)
            cameras = data.get('cameras', {})
            
            self.detection_count += total_dets
            
            # Print summary
            ts_str = datetime.fromtimestamp(timestamp).strftime('%H:%M:%S.%f')[:-3]
            print(f"[{self.message_count:4d}] {ts_str} | Detections: {total_dets:2d} | Cameras: {len(cameras)}")
            
            # Print detailed info for each camera
            for cam_name, cam_data in cameras.items():
                tracks = cam_data if isinstance(cam_data, list) else cam_data.get('tracks', [])
                if tracks:
                    print(f"  📷 {cam_name}: {len(tracks)} track(s)")
                    for track in tracks[:2]:  # Show first 2 tracks
                        tid = track.get('tracking_id', '?')
                        conf = track.get('confidence', 0)
                        dist = track.get('distance', 0)
                        depth_mean = track.get('depth_mean', 0)
                        has_matrix = track.get('camera_matrix') is not None
                        
                        print(f"    - ID:{tid} conf:{conf:.1f}% dist:{dist:.2f}m "
                              f"depth:{depth_mean:.2f}m matrix:{'✅' if has_matrix else '❌'}")
            
            print()  # Blank line between messages
            
        except json.JSONDecodeError as e:
            print(f"❌ Invalid JSON: {e}")
        except Exception as e:
            print(f"❌ Error processing message: {e}")
    
    def run(self):
        """Start the test subscriber"""
        print("=" * 70)
        print("  MQTT Bridge Tester")
        print("=" * 70)
        print(f"MQTT Broker: {self.mqtt_host}:{self.mqtt_port}")
        print(f"Topic: {self.topic}")
        print("=" * 70)
        print()
        
        try:
            # Connect and start loop
            self.client.connect(self.mqtt_host, self.mqtt_port, keepalive=60)
            self.client.loop_forever()
            
        except KeyboardInterrupt:
            print("\n\n" + "=" * 70)
            print("  Statistics")
            print("=" * 70)
            print(f"Messages received: {self.message_count}")
            print(f"Total detections:  {self.detection_count}")
            if self.message_count > 0:
                print(f"Avg detections/msg: {self.detection_count / self.message_count:.1f}")
            print("=" * 70)
            
        except Exception as e:
            print(f"❌ Connection error: {e}")
            print("\nTroubleshooting:")
            print("1. Check MQTT broker is running: systemctl status mosquitto")
            print("2. Check bridge node is running")
            print("3. Check detection node is publishing")
        
        finally:
            self.client.disconnect()
            print("\n✅ Disconnected. Goodbye!\n")


def main():
    """Main entry point"""
    tester = BridgeTester()
    tester.run()


if __name__ == '__main__':
    main()
