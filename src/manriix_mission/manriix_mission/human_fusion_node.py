#!/usr/bin/env python3
import std_msgs.msg
"""
Human Fusion Node for Manriix Photography Robot

Combines body tracking data from 3 ZED X cameras into unified human clusters.
Publishes human positions in map frame for POI scoring.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import numpy as np
from typing import List, Dict, Tuple
import time

from geometry_msgs.msg import PoseStamped, Point, PointStamped
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Header, ColorRGBA

import tf2_ros
from tf2_geometry_msgs import do_transform_point

# ZED Messages
try:
    from zed_msgs.msg import ObjectsStamped
except ImportError:
    print("Warning: zed_msgs not found. Using mock message type.")
    ObjectsStamped = None


class HumanCluster:
    """Represents a cluster of detected humans"""
    def __init__(self, position: np.ndarray, count: int = 1):
        self.position = position  # [x, y, z] in map frame
        self.count = count
        self.last_seen = time.time()
        self.activity_score = 0.0  # Based on movement
        self.positions_history = [position.copy()]
        
    def update(self, new_position: np.ndarray):
        """Update cluster with new detection"""
        # Moving average for position
        alpha = 0.3
        self.position = alpha * new_position + (1 - alpha) * self.position
        self.last_seen = time.time()
        self.positions_history.append(new_position.copy())
        
        # Keep only last 10 positions for activity calculation
        if len(self.positions_history) > 10:
            self.positions_history.pop(0)
            
        # Calculate activity score based on movement
        if len(self.positions_history) > 2:
            movements = np.diff(self.positions_history, axis=0)
            self.activity_score = np.mean(np.linalg.norm(movements, axis=1))
    
    def is_stale(self, timeout: float = 2.0) -> bool:
        """Check if cluster hasn't been updated recently"""
        return (time.time() - self.last_seen) > timeout


class HumanFusionNode(Node):
    """
    Fuses human detections from multiple ZED cameras.
    
    Subscriptions:
        - /zedx_front/zed_node/body_trk/skeletons
        - /zedx_left/zed_node/body_trk/skeletons
        - /zedx_right/zed_node/body_trk/skeletons
    
    Publications:
        - /human_clusters (MarkerArray) - Visualization
        - /human_clusters/poses (PoseArray) - For POI manager
        - /human_clusters/count (Int32) - Total human count
    """
    
    def __init__(self):
        super().__init__('human_fusion_node')
        
        # Parameters
        self.declare_parameter('cluster_distance_threshold', 1.0)  # meters
        self.declare_parameter('stale_timeout', 2.0)  # seconds
        self.declare_parameter('publish_rate', 10.0)  # Hz
        self.declare_parameter('min_detection_confidence', 0.5)
        
        self.cluster_threshold = self.get_parameter('cluster_distance_threshold').value
        self.stale_timeout = self.get_parameter('stale_timeout').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.min_confidence = self.get_parameter('min_detection_confidence').value
        
        # TF Buffer
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Camera frame names
        self.camera_frames = {
            'front': 'zedx_front_left_camera_frame',
            'left': 'zedx_left_left_camera_frame',
            'right': 'zedx_right_left_camera_frame'
        }
        
        # Human clusters storage
        self.clusters: List[HumanCluster] = []
        self.raw_detections: Dict[str, List[np.ndarray]] = {
            'front': [],
            'left': [],
            'right': []
        }
        
        # QoS for ZED topics
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        
        # Subscribers for ZED body tracking
        if ObjectsStamped is not None:
            self.sub_front = self.create_subscription(
                ObjectsStamped,
                '/zedx_front/zed_node/body_trk/skeletons',
                lambda msg: self.body_callback(msg, 'front'),
                qos
            )
            self.sub_left = self.create_subscription(
                ObjectsStamped,
                '/zedx_left/zed_node/body_trk/skeletons',
                lambda msg: self.body_callback(msg, 'left'),
                qos
            )
            self.sub_right = self.create_subscription(
                ObjectsStamped,
                '/zedx_right/zed_node/body_trk/skeletons',
                lambda msg: self.body_callback(msg, 'right'),
                qos
            )
        else:
            self.get_logger().warn("ZED messages not available - running in mock mode")
        
        # Publishers
        self.pub_markers = self.create_publisher(
            MarkerArray, 
            '/human_clusters/markers', 
            10
        )
        self.pub_count = self.create_publisher(
            std_msgs.msg.Int32,
            '/human_clusters/count',
            10
        )
        
        # Timer for periodic processing and publishing
        self.timer = self.create_timer(
            1.0 / self.publish_rate, 
            self.process_and_publish
        )
        
        self.get_logger().info("Human Fusion Node initialized")
        self.get_logger().info(f"  Cluster threshold: {self.cluster_threshold}m")
        self.get_logger().info(f"  Stale timeout: {self.stale_timeout}s")
        
    def body_callback(self, msg: ObjectsStamped, camera: str):
        """Process body tracking data from a camera"""
        frame_id = msg.header.frame_id
        detections = []
        
        for obj in msg.objects:
            # Check confidence (if available)
            if hasattr(obj, 'confidence') and obj.confidence < self.min_confidence:
                continue
                
            # Get position (head or center)
            if hasattr(obj, 'head_position') and len(obj.head_position) >= 3:
                pos = np.array([
                    obj.head_position[0],
                    obj.head_position[1],
                    obj.head_position[2]
                ])
            elif hasattr(obj, 'position'):
                pos = np.array([
                    obj.position.x,
                    obj.position.y,
                    obj.position.z
                ])
            else:
                continue
            
            # Transform to map frame
            try:
                map_pos = self.transform_to_map(pos, frame_id)
                if map_pos is not None:
                    detections.append(map_pos)
            except Exception as e:
                self.get_logger().debug(f"Transform failed: {e}")
                
        self.raw_detections[camera] = detections
        
    def transform_to_map(self, position: np.ndarray, source_frame: str) -> np.ndarray:
        """Transform position from camera frame to map frame"""
        try:
            # Create PointStamped
            point_stamped = PointStamped()
            point_stamped.header.frame_id = source_frame
            point_stamped.header.stamp = self.get_clock().now().to_msg()
            point_stamped.point.x = float(position[0])
            point_stamped.point.y = float(position[1])
            point_stamped.point.z = float(position[2])
            
            # Get transform
            transform = self.tf_buffer.lookup_transform(
                'map',
                source_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
            
            # Transform point
            transformed = do_transform_point(point_stamped, transform)
            
            return np.array([
                transformed.point.x,
                transformed.point.y,
                transformed.point.z
            ])
            
        except Exception as e:
            self.get_logger().debug(f"Transform error: {e}")
            return None
    
    def process_and_publish(self):
        """Fuse detections and publish clusters"""
        # Collect all current detections
        all_detections = []
        for camera, detections in self.raw_detections.items():
            all_detections.extend(detections)
        
        # Update existing clusters or create new ones
        self.update_clusters(all_detections)
        
        # Remove stale clusters
        self.clusters = [c for c in self.clusters if not c.is_stale(self.stale_timeout)]
        
        # Publish
        self.publish_markers()
        self.publish_count()
        
        # Clear raw detections
        for camera in self.raw_detections:
            self.raw_detections[camera] = []
    
    def update_clusters(self, detections: List[np.ndarray]):
        """Update cluster list with new detections"""
        for detection in detections:
            # Find nearest existing cluster
            min_dist = float('inf')
            nearest_cluster = None
            
            for cluster in self.clusters:
                dist = np.linalg.norm(detection[:2] - cluster.position[:2])  # 2D distance
                if dist < min_dist:
                    min_dist = dist
                    nearest_cluster = cluster
            
            # Update existing or create new cluster
            if nearest_cluster is not None and min_dist < self.cluster_threshold:
                nearest_cluster.update(detection)
            else:
                self.clusters.append(HumanCluster(detection))
    
    def publish_markers(self):
        """Publish visualization markers for clusters"""
        marker_array = MarkerArray()
        
        # Delete old markers
        delete_marker = Marker()
        delete_marker.header.frame_id = 'map'
        delete_marker.action = Marker.DELETEALL
        marker_array.markers.append(delete_marker)
        
        # Create marker for each cluster
        for i, cluster in enumerate(self.clusters):
            # Sphere marker for position
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'human_clusters'
            marker.id = i
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            
            marker.pose.position.x = cluster.position[0]
            marker.pose.position.y = cluster.position[1]
            marker.pose.position.z = 0.5  # Half height
            marker.pose.orientation.w = 1.0
            
            marker.scale.x = 0.5  # Diameter
            marker.scale.y = 0.5
            marker.scale.z = 1.0  # Height
            
            # Color based on activity (green = still, red = active)
            marker.color.r = min(1.0, cluster.activity_score * 5)
            marker.color.g = max(0.0, 1.0 - cluster.activity_score * 5)
            marker.color.b = 0.2
            marker.color.a = 0.7
            
            marker_array.markers.append(marker)
            
            # Text marker for count
            text_marker = Marker()
            text_marker.header.frame_id = 'map'
            text_marker.header.stamp = self.get_clock().now().to_msg()
            text_marker.ns = 'human_labels'
            text_marker.id = i + 1000
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            
            text_marker.pose.position.x = cluster.position[0]
            text_marker.pose.position.y = cluster.position[1]
            text_marker.pose.position.z = 1.5
            
            text_marker.scale.z = 0.3
            text_marker.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
            text_marker.text = f"Human {i+1}"
            
            marker_array.markers.append(text_marker)
        
        self.pub_markers.publish(marker_array)
    
    def publish_count(self):
        """Publish total human count"""
        from std_msgs.msg import Int32
        msg = Int32()
        msg.data = len(self.clusters)
        self.pub_count.publish(msg)
        
    def get_clusters_info(self) -> List[Dict]:
        """Get cluster information for POI manager"""
        return [
            {
                'position': cluster.position.tolist(),
                'count': cluster.count,
                'activity': cluster.activity_score,
                'age': time.time() - cluster.last_seen
            }
            for cluster in self.clusters
        ]


def main(args=None):
    rclpy.init(args=args)
    node = HumanFusionNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
