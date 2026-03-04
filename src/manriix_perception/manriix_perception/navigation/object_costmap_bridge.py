#!/usr/bin/env python3
"""
Object to Costmap Bridge

Converts fused ZED detections to PointCloud2 obstacles for Nav2 costmap integration.
Based on the proven approach from friend's implementation.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import Header, ColorRGBA
from builtin_interfaces.msg import Time

import numpy as np
import struct
import time
import threading
from typing import List, Tuple, Dict, Any, Optional
from dataclasses import dataclass
from enum import IntEnum

# Local imports
from manriix_perception.fusion.spatial_fusion import FusedDetection


class ObjectClass(IntEnum):
    """ZED Object Class IDs"""
    PERSON = 0
    VEHICLE = 1
    BAG = 2
    ANIMAL = 3
    ELECTRONICS = 4
    FRUIT_VEGETABLE = 5
    SPORT = 6


@dataclass
class ClassConfig:
    """Configuration for each object class"""
    enabled: bool
    inflation_radius: float  # meters
    height: float  # obstacle height for point generation
    color: Tuple[float, float, float, float]  # RGBA
    cost_value: int  # Costmap cost value


class ObjectCostmapBridge(Node):
    """
    Bridge node that converts fused ZED detections to PointCloud2 for costmap integration
    
    Subscribes to: /zed_system/fused_detections (ZedDetectionArray)
    Publishes to: /object_obstacles (PointCloud2)
    Publishes to: /object_markers (MarkerArray) - for visualization
    
    The PointCloud2 can be used as an observation source in the
    voxel_layer or obstacle_layer of Nav2 costmaps.
    """
    
    def __init__(self):
        super().__init__('object_costmap_bridge')
        
        # Load configuration
        self._load_configuration()
        
        # Setup class configurations
        self._setup_class_configs()
        
        # QoS profiles
        self.sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=5
        )
        
        self.reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=5
        )
        
        # Subscribers
        self._setup_subscribers()
        
        # Publishers
        self._setup_publishers()
        
        # Statistics and monitoring
        self.processing_stats = {
            'objects_processed': 0,
            'points_generated': 0,
            'last_processing_time': 0.0,
            'average_processing_time': 0.0,
            'publish_rate_hz': 0.0,
            'last_publish_time': 0.0
        }
        
        # Thread safety
        self.processing_lock = threading.Lock()
        
        # Performance monitoring timer
        self.create_timer(10.0, self._log_performance)
        
        self.get_logger().info(
            f'Object to Costmap Bridge initialized\n'
            f'  Input:  {self.objects_topic}\n'
            f'  Output: {self.output_topic}\n'
            f'  Frame:  {self.output_frame_id}\n'
            f'  Min confidence: {self.min_confidence}'
        )
    
    def _load_configuration(self):
        """Load configuration parameters"""
        # Input/Output topics
        self.declare_parameter('objects_topic', '/zed_system/fused_detections')
        self.declare_parameter('output_topic', '/object_obstacles') 
        self.declare_parameter('markers_topic', '/object_markers')
        
        # Frame configuration
        self.declare_parameter('output_frame_id', 'base_link')
        
        # Filtering parameters
        self.declare_parameter('min_confidence', 50.0)
        self.declare_parameter('max_range', 8.0)
        self.declare_parameter('min_range', 0.3)
        
        # Point generation parameters
        self.declare_parameter('points_per_meter', 10.0)
        self.declare_parameter('obstacle_height', 0.5)
        
        # Class enable/disable flags
        self.declare_parameter('detect_persons', True)
        self.declare_parameter('detect_vehicles', True)
        self.declare_parameter('detect_animals', True)
        self.declare_parameter('detect_bags', False)
        self.declare_parameter('detect_electronics', False)
        self.declare_parameter('detect_sport', False)
        
        # Class-specific inflation radii
        self.declare_parameter('person_radius', 0.4)
        self.declare_parameter('vehicle_radius', 0.5)
        self.declare_parameter('animal_radius', 0.3)
        self.declare_parameter('bag_radius', 0.2)
        self.declare_parameter('default_radius', 0.3)
        
        # Visualization (configurable - can enable for development)
        self.declare_parameter('publish_markers', False)
        
        # Get parameter values
        self.objects_topic = self.get_parameter('objects_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.markers_topic = self.get_parameter('markers_topic').value
        self.output_frame_id = self.get_parameter('output_frame_id').value
        
        self.min_confidence = self.get_parameter('min_confidence').value
        self.max_range = self.get_parameter('max_range').value
        self.min_range = self.get_parameter('min_range').value
        
        self.points_per_meter = self.get_parameter('points_per_meter').value
        self.obstacle_height = self.get_parameter('obstacle_height').value
        
        self.publish_markers = self.get_parameter('publish_markers').value
    
    def _setup_class_configs(self):
        """Setup configuration for each object class"""
        self.class_configs: Dict[int, ClassConfig] = {
            ObjectClass.PERSON: ClassConfig(
                enabled=self.get_parameter('detect_persons').value,
                inflation_radius=self.get_parameter('person_radius').value,
                height=1.7,
                color=(0.0, 1.0, 0.0, 0.8),  # Green
                cost_value=200  # High cost but navigable
            ),
            ObjectClass.VEHICLE: ClassConfig(
                enabled=self.get_parameter('detect_vehicles').value,
                inflation_radius=self.get_parameter('vehicle_radius').value,
                height=1.5,
                color=(1.0, 0.0, 0.0, 0.8),  # Red
                cost_value=254  # Lethal - avoid completely
            ),
            ObjectClass.ANIMAL: ClassConfig(
                enabled=self.get_parameter('detect_animals').value,
                inflation_radius=self.get_parameter('animal_radius').value,
                height=0.8,
                color=(1.0, 0.5, 0.0, 0.8),  # Orange
                cost_value=180  # High cost
            ),
            ObjectClass.BAG: ClassConfig(
                enabled=self.get_parameter('detect_bags').value,
                inflation_radius=self.get_parameter('bag_radius').value,
                height=0.5,
                color=(0.5, 0.5, 0.5, 0.8),  # Gray
                cost_value=150  # Moderate cost
            ),
            ObjectClass.ELECTRONICS: ClassConfig(
                enabled=self.get_parameter('detect_electronics').value,
                inflation_radius=self.get_parameter('default_radius').value,
                height=0.3,
                color=(0.0, 0.0, 1.0, 0.8),  # Blue
                cost_value=120  # Low-moderate cost
            ),
            ObjectClass.SPORT: ClassConfig(
                enabled=self.get_parameter('detect_sport').value,
                inflation_radius=self.get_parameter('default_radius').value,
                height=0.3,
                color=(1.0, 1.0, 0.0, 0.8),  # Yellow
                cost_value=100  # Low cost
            ),
        }
    
    def _setup_subscribers(self):
        """Setup input subscribers"""
        from manriix_perception.msg import ZedDetectionArray
        
        self.objects_sub = self.create_subscription(
            ZedDetectionArray,
            self.objects_topic,
            self.objects_callback,
            self.sensor_qos
        )
    
    def _setup_publishers(self):
        """Setup output publishers"""
        # PointCloud2 publisher for costmap
        self.cloud_pub = self.create_publisher(
            PointCloud2,
            self.output_topic,
            self.sensor_qos
        )
        
        # Visualization markers (optional)
        if self.publish_markers:
            self.marker_pub = self.create_publisher(
                MarkerArray,
                self.markers_topic,
                self.reliable_qos
            )
    
    def objects_callback(self, msg):
        """
        Process incoming fused detection messages
        
        Args:
            msg: ZedDetectionArray message containing fused detections
        """
        with self.processing_lock:
            start_time = time.time()
            
            try:
                if len(msg.detections) == 0:
                    return
                
                all_points = []
                markers = MarkerArray() if self.publish_markers else None
                marker_id = 0
                objects_processed = 0
                
                for detection_msg in msg.detections:
                    # Filter by confidence
                    if detection_msg.confidence < self.min_confidence:
                        continue
                    
                    # Get class config
                    class_id = detection_msg.class_id
                    config = self.class_configs.get(class_id)
                    
                    if config is None or not config.enabled:
                        continue
                    
                    # Get object position
                    position = detection_msg.position
                    
                    # Filter by range
                    distance = np.sqrt(position.x**2 + position.y**2)
                    if distance < self.min_range or distance > self.max_range:
                        continue
                    
                    # Get object dimensions or use defaults
                    if len(detection_msg.dimensions_3d) >= 3:
                        width = max(detection_msg.dimensions_3d[0], 0.3)
                        depth = max(detection_msg.dimensions_3d[2], 0.3)
                    else:
                        width = config.inflation_radius * 2
                        depth = config.inflation_radius * 2
                    
                    # Add inflation
                    width += config.inflation_radius * 2
                    depth += config.inflation_radius * 2
                    
                    # Generate obstacle points
                    points = self._generate_obstacle_points(
                        position.x, position.y, width, depth, self.obstacle_height
                    )
                    all_points.extend(points)
                    
                    # Generate visualization marker
                    if self.publish_markers:
                        marker = self._create_marker(
                            msg.header, marker_id, detection_msg, config
                        )
                        markers.markers.append(marker)
                        marker_id += 1
                    
                    objects_processed += 1
                
                # Publish PointCloud2
                if all_points:
                    cloud_msg = self._create_pointcloud2(msg.header, all_points)
                    self.cloud_pub.publish(cloud_msg)
                
                # Publish markers
                if self.publish_markers and markers and markers.markers:
                    # Clear old markers first
                    self._publish_clear_markers(msg.header)
                    # Publish new markers
                    self.marker_pub.publish(markers)
                
                # Update statistics
                processing_time = (time.time() - start_time) * 1000  # ms
                self._update_processing_stats(objects_processed, len(all_points), processing_time)
                
            except Exception as e:
                self.get_logger().error(f"Error processing objects: {e}")
    
    def _generate_obstacle_points(self, cx: float, cy: float, width: float, 
                                depth: float, height: float) -> List[Tuple[float, float, float]]:
        """
        Generate points around object footprint for costmap
        
        Args:
            cx, cy: Center position
            width, depth: Object dimensions with inflation
            height: Height of generated points
            
        Returns:
            List of 3D points
        """
        points = []
        
        # Calculate number of points based on density
        n_width = max(3, int(width * self.points_per_meter))
        n_depth = max(3, int(depth * self.points_per_meter))
        
        # Generate grid of points within object footprint
        for dx in np.linspace(-width/2, width/2, n_width):
            for dy in np.linspace(-depth/2, depth/2, n_depth):
                points.append((cx + dx, cy + dy, height))
        
        # Add perimeter points for better obstacle definition
        perimeter_points = max(8, int((width + depth) * self.points_per_meter / 2))
        
        for dx in np.linspace(-width/2, width/2, perimeter_points):
            points.append((cx + dx, cy - depth/2, height))
            points.append((cx + dx, cy + depth/2, height))
        
        for dy in np.linspace(-depth/2, depth/2, perimeter_points):
            points.append((cx - width/2, cy + dy, height))
            points.append((cx + width/2, cy + dy, height))
        
        return points
    
    def _create_pointcloud2(self, header: Header, points: List[Tuple[float, float, float]]) -> PointCloud2:
        """
        Create PointCloud2 message from list of points
        
        Args:
            header: Source message header
            points: List of 3D points
            
        Returns:
            PointCloud2 message
        """
        # Create output header
        out_header = Header()
        out_header.stamp = self.get_clock().now().to_msg()
        out_header.frame_id = self.output_frame_id if self.output_frame_id else header.frame_id
        
        # Define point fields (x, y, z)
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        
        # Pack point data
        point_step = 12  # 3 floats * 4 bytes each
        data = bytearray(len(points) * point_step)
        
        for i, (x, y, z) in enumerate(points):
            struct.pack_into('fff', data, i * point_step, x, y, z)
        
        # Create PointCloud2 message
        cloud = PointCloud2()
        cloud.header = out_header
        cloud.height = 1
        cloud.width = len(points)
        cloud.fields = fields
        cloud.is_bigendian = False
        cloud.point_step = point_step
        cloud.row_step = point_step * len(points)
        cloud.data = bytes(data)
        cloud.is_dense = True
        
        return cloud
    
    def _create_marker(self, header: Header, marker_id: int, detection_msg, config: ClassConfig) -> Marker:
        """
        Create visualization marker for detected object
        
        Args:
            header: Source message header
            marker_id: Unique marker ID
            detection_msg: Detection message
            config: Class configuration
            
        Returns:
            Marker message
        """
        marker = Marker()
        marker.header = header
        marker.ns = 'detected_objects'
        marker.id = marker_id
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        
        # Position
        marker.pose.position = detection_msg.position
        marker.pose.orientation.w = 1.0
        
        # Size
        if len(detection_msg.dimensions_3d) >= 3:
            marker.scale.x = max(float(detection_msg.dimensions_3d[0]), 0.3)
            marker.scale.y = max(float(detection_msg.dimensions_3d[2]), 0.3) 
            marker.scale.z = max(float(detection_msg.dimensions_3d[1]), 0.3)
        else:
            marker.scale.x = config.inflation_radius * 2
            marker.scale.y = config.inflation_radius * 2
            marker.scale.z = config.height
        
        # Color
        marker.color = ColorRGBA(
            r=config.color[0],
            g=config.color[1], 
            b=config.color[2],
            a=config.color[3]
        )
        
        # Lifetime
        marker.lifetime.sec = 1
        marker.lifetime.nanosec = 0
        
        return marker
    
    def _publish_clear_markers(self, header: Header):
        """Publish marker to clear all previous markers"""
        if not self.publish_markers:
            return
        
        clear_marker = Marker()
        clear_marker.header = header
        clear_marker.action = Marker.DELETEALL
        
        clear_markers = MarkerArray()
        clear_markers.markers.append(clear_marker)
        
        self.marker_pub.publish(clear_markers)
    
    def _update_processing_stats(self, objects_processed: int, points_generated: int, 
                                processing_time: float):
        """Update processing statistics"""
        self.processing_stats['objects_processed'] += objects_processed
        self.processing_stats['points_generated'] += points_generated
        
        # Update average processing time
        current_avg = self.processing_stats['average_processing_time']
        self.processing_stats['average_processing_time'] = (current_avg + processing_time) / 2
        
        # Update publish rate
        current_time = time.time()
        last_time = self.processing_stats['last_publish_time']
        
        if last_time > 0:
            time_diff = current_time - last_time
            self.processing_stats['publish_rate_hz'] = 1.0 / time_diff if time_diff > 0 else 0.0
        
        self.processing_stats['last_publish_time'] = current_time
    
    def _log_performance(self):
        """Log performance statistics periodically"""
        stats = self.processing_stats
        
        self.get_logger().info(
            f"Costmap Bridge Performance: "
            f"Objects processed: {stats['objects_processed']}, "
            f"Points generated: {stats['points_generated']}, "
            f"Avg processing time: {stats['average_processing_time']:.1f}ms, "
            f"Publish rate: {stats['publish_rate_hz']:.1f}Hz"
        )


def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)
    
    try:
        node = ObjectCostmapBridge()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error in object costmap bridge: {e}")
    finally:
        if 'node' in locals():
            node.destroy_node()
        try:
            rclpy.shutdown()
        except:
            pass


if __name__ == '__main__':
    main()