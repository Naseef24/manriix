#!/usr/bin/env python3
"""
Map Manager Node
Handles map save/load/list operations for waypoint navigation
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid

import os
import json
import subprocess
from pathlib import Path
from typing import List, Dict, Optional
from datetime import datetime


class MapManager(Node):
    def __init__(self):
        super().__init__('map_manager')
        
        self.get_logger().info("=" * 60)
        self.get_logger().info("Starting Map Manager Node")
        self.get_logger().info("=" * 60)
        
        # Declare parameters
        self.declare_parameter('maps_directory', 
            os.path.expanduser('~/manriix2_ws/src/manriix_navigation/maps'))
        # use_sim_time passed from launch file
        
        # Get parameters
        self.maps_dir = self.get_parameter('maps_directory').value
        
        # Ensure maps directory exists
        os.makedirs(self.maps_dir, exist_ok=True)
        
        # QoS
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        
        # Publishers
        self.map_list_pub = self.create_publisher(String, '/map_manager/list', reliable_qos)
        self.map_status_pub = self.create_publisher(String, '/map_manager/status', reliable_qos)
        
        # Subscribers
        self.save_map_sub = self.create_subscription(
            String, '/map_manager/save', self.save_map_callback, reliable_qos
        )
        self.load_map_sub = self.create_subscription(
            String, '/map_manager/load', self.load_map_callback, reliable_qos
        )
        self.delete_map_sub = self.create_subscription(
            String, '/map_manager/delete', self.delete_map_callback, reliable_qos
        )
        self.request_list_sub = self.create_subscription(
            String, '/map_manager/request_list', self.request_list_callback, reliable_qos
        )
        
        # Publish initial map list
        self.publish_map_list()
        
        self.get_logger().info(f"Maps directory: {self.maps_dir}")
        self.get_logger().info(f"Available maps: {len(self.get_available_maps())}")
        self.get_logger().info("Map Manager ready")

    def get_available_maps(self) -> List[Dict]:
        """Get list of available maps with metadata"""
        maps = []
        
        try:
            for file in os.listdir(self.maps_dir):
                if file.endswith('.yaml'):
                    map_name = file[:-5]  # Remove .yaml extension
                    pgm_file = os.path.join(self.maps_dir, f"{map_name}.pgm")
                    yaml_file = os.path.join(self.maps_dir, file)
                    
                    if os.path.exists(pgm_file):
                        # Get file modification time
                        mtime = os.path.getmtime(yaml_file)
                        modified = datetime.fromtimestamp(mtime).strftime('%Y-%m-%d %H:%M')
                        
                        # Get file size
                        size_bytes = os.path.getsize(pgm_file)
                        size_kb = size_bytes / 1024
                        
                        maps.append({
                            'name': map_name,
                            'yaml_file': yaml_file,
                            'pgm_file': pgm_file,
                            'modified': modified,
                            'size_kb': round(size_kb, 1)
                        })
        except Exception as e:
            self.get_logger().error(f"Error listing maps: {e}")
        
        # Sort by modification time (newest first)
        maps.sort(key=lambda x: x['modified'], reverse=True)
        return maps

    def publish_map_list(self):
        """Publish list of available maps as JSON"""
        maps = self.get_available_maps()
        
        msg = String()
        msg.data = json.dumps({
            'maps': maps,
            'count': len(maps),
            'directory': self.maps_dir
        })
        self.map_list_pub.publish(msg)
        
        self.get_logger().debug(f"Published map list: {len(maps)} maps")

    def request_list_callback(self, msg: String):
        """Handle request to publish map list"""
        self.get_logger().info("Map list requested")
        self.publish_map_list()

    def save_map_callback(self, msg: String):
        """Save current map with given name"""
        try:
            data = json.loads(msg.data)
            map_name = data.get('name', '').strip()
            
            if not map_name:
                self.publish_status('error', 'Map name is required')
                return
            
            # Sanitize map name
            map_name = "".join(c for c in map_name if c.isalnum() or c in '_-')
            
            if not map_name:
                self.publish_status('error', 'Invalid map name')
                return
            
            # Full path (without extension)
            map_path = os.path.join(self.maps_dir, map_name)
            
            self.get_logger().info(f"Saving map as: {map_name}")
            
            # Call map_saver_cli
            result = subprocess.run(
                ['ros2', 'run', 'nav2_map_server', 'map_saver_cli', '-f', map_path],
                capture_output=True,
                text=True,
                timeout=30
            )
            
            if result.returncode == 0:
                self.get_logger().info(f"Map saved successfully: {map_name}")
                self.publish_status('success', f"Map '{map_name}' saved successfully")
                self.publish_map_list()  # Update list
            else:
                error_msg = result.stderr or result.stdout or "Unknown error"
                self.get_logger().error(f"Map save failed: {error_msg}")
                self.publish_status('error', f"Failed to save map: {error_msg}")
                
        except json.JSONDecodeError:
            # Simple string format (just map name)
            map_name = msg.data.strip()
            if map_name:
                # Retry with simple name
                new_msg = String()
                new_msg.data = json.dumps({'name': map_name})
                self.save_map_callback(new_msg)
            else:
                self.publish_status('error', 'Invalid save request format')
        except subprocess.TimeoutExpired:
            self.publish_status('error', 'Map save timed out')
        except Exception as e:
            self.get_logger().error(f"Error saving map: {e}")
            self.publish_status('error', f"Error: {str(e)}")

    def load_map_callback(self, msg: String):
        """Load map by name (publishes path for localization launch)"""
        try:
            data = json.loads(msg.data)
            map_name = data.get('name', '').strip()
        except json.JSONDecodeError:
            map_name = msg.data.strip()
        
        if not map_name:
            self.publish_status('error', 'Map name is required')
            return
        
        # Find map
        yaml_path = os.path.join(self.maps_dir, f"{map_name}.yaml")
        pgm_path = os.path.join(self.maps_dir, f"{map_name}.pgm")
        
        if not os.path.exists(yaml_path) or not os.path.exists(pgm_path):
            self.publish_status('error', f"Map '{map_name}' not found")
            return
        
        self.get_logger().info(f"Map selected for loading: {map_name}")
        self.get_logger().info(f"  YAML: {yaml_path}")
        self.get_logger().info(f"  PGM: {pgm_path}")
        
        # Publish load status with path
        self.publish_status('load_ready', json.dumps({
            'name': map_name,
            'yaml_path': yaml_path,
            'pgm_path': pgm_path
        }))

    def delete_map_callback(self, msg: String):
        """Delete map by name"""
        try:
            data = json.loads(msg.data)
            map_name = data.get('name', '').strip()
        except json.JSONDecodeError:
            map_name = msg.data.strip()
        
        if not map_name:
            self.publish_status('error', 'Map name is required')
            return
        
        yaml_path = os.path.join(self.maps_dir, f"{map_name}.yaml")
        pgm_path = os.path.join(self.maps_dir, f"{map_name}.pgm")
        
        deleted = False
        
        try:
            if os.path.exists(yaml_path):
                os.remove(yaml_path)
                deleted = True
            if os.path.exists(pgm_path):
                os.remove(pgm_path)
                deleted = True
            
            if deleted:
                self.get_logger().info(f"Map deleted: {map_name}")
                self.publish_status('success', f"Map '{map_name}' deleted")
                self.publish_map_list()
            else:
                self.publish_status('error', f"Map '{map_name}' not found")
                
        except Exception as e:
            self.get_logger().error(f"Error deleting map: {e}")
            self.publish_status('error', f"Error deleting map: {str(e)}")

    def publish_status(self, status: str, message: str):
        """Publish status message"""
        msg = String()
        msg.data = json.dumps({
            'status': status,
            'message': message,
            'timestamp': datetime.now().isoformat()
        })
        self.map_status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MapManager()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
