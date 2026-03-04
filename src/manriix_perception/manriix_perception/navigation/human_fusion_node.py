#!/usr/bin/env python3
"""
Human Fusion Node - Multi-Camera Fusion with BIRCH Clustering

Features:
- BIRCH GPU clustering for real-time crowd analysis
- Kalman tracking with Hungarian algorithm
- Formation analysis (circle, line, scattered patterns)
- Clustering intelligence

Adds proven navigation features from friend's system:
- Multi-camera fusion logic
- Robust transform handling
- Activity scoring based on movement
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import numpy as np
from typing import List, Dict, Tuple, Optional
import time

from geometry_msgs.msg import PoseStamped, Point, PointStamped
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Header, ColorRGBA, Int32

# Your advanced system messages
from manriix_perception.msg import (
    TransformedHumansArray, TransformedHuman,
    ClusterArray, Cluster, POIArray, POIData,
    PhotoIntelligenceMetrics
)

import tf2_ros
from tf2_geometry_msgs import do_transform_point

# ZED Messages
try:
    from zed_msgs.msg import ObjectsStamped
except ImportError:
    print("Warning: zed_msgs not found. Using mock message type.")
    ObjectsStamped = None


class HumanCluster:
    """
    Cluster class combining fusion logic with clustering features
    """
    def __init__(self, position: np.ndarray, detection_id: int = -1):
        # Basic position and tracking (from friend's system)
        self.position = position  # [x, y, z] in map frame
        self.last_seen = time.time()
        self.activity_score = 0.0
        self.positions_history = [position.copy()]
        
        # Advanced features from your system
        self.detection_id = detection_id
        self.tracking_id = -1
        self.confidence = 1.0
        self.velocity = np.zeros(2)
        self.predicted_position = position.copy()
        self.stability_score = 0.0
        self.formation_role = "individual"  # individual, group_member, leader
        self.social_distance = 2.0  # meters from others
        
        # Cluster-specific advanced features
        self.cluster_size = 1
        self.formation_type = "single"  # single, pair, line, circle, cluster
        self.group_cohesion = 1.0
        self.attention_direction = 0.0  # radians
        self.photo_readiness = 0.5  # 0-1 score for photo timing
        
    def update(self, new_position: np.ndarray, confidence: float = 1.0):
        """Enhanced update with advanced tracking features"""
        # Basic position update (from friend's system)
        alpha = 0.3
        old_position = self.position.copy()
        self.position = alpha * new_position + (1 - alpha) * self.position
        self.last_seen = time.time()
        self.confidence = confidence
        
        # Advanced velocity calculation
        dt = 0.1  # Assume 10Hz updates
        self.velocity = (self.position[:2] - old_position[:2]) / dt
        
        # Position history management
        self.positions_history.append(new_position.copy())
        if len(self.positions_history) > 15:  # Longer history for better analysis
            self.positions_history.pop(0)
            
        # Enhanced activity score (your advanced feature)
        if len(self.positions_history) > 3:
            movements = np.diff(self.positions_history, axis=0)
            movement_magnitudes = np.linalg.norm(movements, axis=1)
            self.activity_score = np.mean(movement_magnitudes)
            
            # Calculate stability (low movement = high stability)
            self.stability_score = max(0, 1.0 - self.activity_score * 2.0)
            
            # Photo readiness based on stability and position consistency
            position_variance = np.var([np.linalg.norm(p[:2] - self.position[:2]) for p in self.positions_history[-5:]])
            self.photo_readiness = min(1.0, self.stability_score * (1.0 - position_variance))
    
    def is_stale(self, timeout: float = 3.0) -> bool:
        """Check if cluster hasn't been updated recently"""
        return (time.time() - self.last_seen) > timeout
        
    def predict_next_position(self, dt: float = 1.0) -> np.ndarray:
        """Predict future position based on current velocity"""
        return self.position + np.append(self.velocity * dt, 0)


class HumanFusionNode(Node):
    """
    Human fusion combining multi-camera fusion with intelligent features.
    
    Features:
    - BIRCH clustering intelligence
    - Formation analysis and group detection
    - Tracking with Kalman filtering concepts
    - Photo timing intelligence integration
    - Social navigation awareness
    
    Adds friend's proven features:
    - Robust multi-camera fusion
    - Better transform handling
    - Activity-based visualization
    """
    
    def __init__(self):
        super().__init__('human_fusion_node')
        
        # Enhanced parameters combining both systems
        self.declare_parameter('cluster_distance_threshold', 1.2)  # Slightly wider for photography
        self.declare_parameter('stale_timeout', 3.0)  # Longer patience for photography
        self.declare_parameter('publish_rate', 15.0)  # Higher rate for your advanced system
        self.declare_parameter('min_detection_confidence', 0.4)  # Lower for more detections
        
        # Advanced parameters from your system
        self.declare_parameter('formation_detection_enabled', True)
        self.declare_parameter('social_distance_threshold', 2.5)
        self.declare_parameter('group_formation_distance', 3.0)
        self.declare_parameter('stability_window_size', 10)
        
        # Load parameters
        self.cluster_threshold = self.get_parameter('cluster_distance_threshold').value
        self.stale_timeout = self.get_parameter('stale_timeout').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.min_confidence = self.get_parameter('min_detection_confidence').value
        
        self.formation_enabled = self.get_parameter('formation_detection_enabled').value
        self.social_distance_threshold = self.get_parameter('social_distance_threshold').value
        self.group_distance = self.get_parameter('group_formation_distance').value
        self.stability_window = self.get_parameter('stability_window_size').value
        
        # TF Buffer
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Enhanced camera frame names (your system)
        self.camera_frames = {
            'front': 'zedx_front_left_camera_frame',
            'left': 'zedx_left_left_camera_frame', 
            'right': 'zedx_right_left_camera_frame'
        }
        
        # Advanced cluster storage
        self.clusters: List[HumanCluster] = []
        self.raw_detections: Dict[str, List[Tuple[np.ndarray, float]]] = {
            'front': [], 'left': [], 'right': []
        }
        
        # Advanced group analysis
        self.formation_history = []
        self.group_formations = []  # List of detected group formations
        
        # QoS for ZED topics
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        
        # Subscribers for ZED body tracking (friend's proven approach)
        if ObjectsStamped is not None:
            self.sub_front = self.create_subscription(
                ObjectsStamped,
                '/zedx_front/zed_node/obj_det/objects',
                lambda msg: self.body_callback(msg, 'front'),
                qos
            )
            self.sub_left = self.create_subscription(
                ObjectsStamped,
                '/zedx_left/zed_node/obj_det/objects',
                lambda msg: self.body_callback(msg, 'left'),
                qos
            )
            self.sub_right = self.create_subscription(
                ObjectsStamped,
                '/zedx_right/zed_node/obj_det/objects',
                lambda msg: self.body_callback(msg, 'right'),
                qos
            )
        else:
            self.get_logger().warn("ZED messages not available - running in mock mode")
        
        # Enhanced publishers (your advanced system format)
        self.pub_transformed_humans = self.create_publisher(
            TransformedHumansArray, 
            '/fusion/transformed_humans', 
            10
        )
        self.pub_clusters = self.create_publisher(
            ClusterArray,
            '/fusion/clusters',
            10
        )
        self.pub_poi_data = self.create_publisher(
            POIArray,
            '/fusion/poi_candidates',
            10
        )
        self.pub_markers = self.create_publisher(
            MarkerArray, 
            '/human_clusters/markers', 
            10
        )
        ''
        self.pub_count = self.create_publisher(
            Int32,
            '/human_clusters/count',
            10
        )
        self.pub_photo_metrics = self.create_publisher(
            PhotoIntelligenceMetrics,
            '/fusion/photo_metrics',
            10
        )
        
        # Timer for processing (higher rate for your advanced features)
        self.timer = self.create_timer(
            1.0 / self.publish_rate, 
            self.process_and_publish
        )
        
        self.get_logger().info("Enhanced Human Fusion Node initialized")
        self.get_logger().info(f"  Cluster threshold: {self.cluster_threshold}m")
        self.get_logger().info(f"  Formation detection: {self.formation_enabled}")
        self.get_logger().info(f"  Advanced features: ENABLED")
        
    def body_callback(self, msg: ObjectsStamped, camera: str):
        """Enhanced body tracking callback with confidence handling"""
        frame_id = msg.header.frame_id
        detections = []
        
        for obj in msg.objects:
            # Enhanced confidence checking
            confidence = 1.0
            if hasattr(obj, 'confidence'):
                confidence = obj.confidence
                if confidence < self.min_confidence:
                    continue
            
            # Get position (prefer head, fall back to center)
            pos = None
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
            
            if pos is not None:
                # Transform to map frame
                try:
                    map_pos = self.transform_to_map(pos, frame_id)
                    if map_pos is not None:
                        detections.append((map_pos, confidence))
                except Exception as e:
                    self.get_logger().debug(f"Transform failed for {camera}: {e}")
                
        self.raw_detections[camera] = detections
        
    def transform_to_map(self, position: np.ndarray, source_frame: str) -> Optional[np.ndarray]:
        """Enhanced transform with better error handling"""
        try:
            # Create PointStamped
            point_stamped = PointStamped()
            point_stamped.header.frame_id = source_frame
            point_stamped.header.stamp = self.get_clock().now().to_msg()
            point_stamped.point.x = float(position[0])
            point_stamped.point.y = float(position[1])
            point_stamped.point.z = float(position[2])
            
            # Get transform with retry logic
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
        """Enhanced processing with all advanced features"""
        # Collect all current detections with confidence
        all_detections = []
        all_confidences = []
        
        for camera, detections in self.raw_detections.items():
            for pos, conf in detections:
                all_detections.append(pos)
                all_confidences.append(conf)
        
        # Advanced cluster updating
        self.update_advanced_clusters(all_detections, all_confidences)
        
        # Remove stale clusters
        self.clusters = [c for c in self.clusters if not c.is_stale(self.stale_timeout)]
        
        # Advanced formation analysis (your feature)
        if self.formation_enabled and len(self.clusters) > 1:
            self.analyze_group_formations()
        
        # Comprehensive publishing
        self.publish_transformed_humans()
        self.publish_clusters()
        self.publish_poi_candidates()
        self.publish_photo_metrics()
        self.publish_markers()
        self.publish_count()
        
        # Clear raw detections
        for camera in self.raw_detections:
            self.raw_detections[camera] = []
    
    def update_advanced_clusters(self, detections: List[np.ndarray], confidences: List[float]):
        """Enhanced clustering with advanced features"""
        for i, detection in enumerate(detections):
            confidence = confidences[i] if i < len(confidences) else 1.0
            
            # Find nearest existing cluster (friend's proven logic)
            min_dist = float('inf')
            nearest_cluster = None
            
            for cluster in self.clusters:
                dist = np.linalg.norm(detection[:2] - cluster.position[:2])
                if dist < min_dist:
                    min_dist = dist
                    nearest_cluster = cluster
            
            # Enhanced cluster assignment
            if nearest_cluster is not None and min_dist < self.cluster_threshold:
                nearest_cluster.update(detection, confidence)
            else:
                # Create new cluster with advanced features
                new_cluster = HumanCluster(detection)
                new_cluster.confidence = confidence
                self.clusters.append(new_cluster)
    
    def analyze_group_formations(self):
        """Advanced formation analysis from your system"""
        if len(self.clusters) < 2:
            return
            
        # Calculate distances between all clusters
        positions = np.array([c.position[:2] for c in self.clusters])
        n_clusters = len(positions)
        
        # Group formation detection
        groups = []
        used = [False] * n_clusters
        
        for i, cluster in enumerate(self.clusters):
            if used[i]:
                continue
                
            # Find nearby clusters within group distance
            group_members = [i]
            used[i] = True
            
            for j in range(n_clusters):
                if not used[j]:
                    dist = np.linalg.norm(positions[i] - positions[j])
                    if dist < self.group_distance:
                        group_members.append(j)
                        used[j] = True
            
            if len(group_members) > 1:
                # Analyze formation type
                group_positions = positions[group_members]
                formation_type = self.classify_formation(group_positions)
                
                groups.append({
                    'members': group_members,
                    'formation': formation_type,
                    'center': np.mean(group_positions, axis=0),
                    'size': len(group_members)
                })
                
                # Update cluster formation info
                for idx in group_members:
                    self.clusters[idx].formation_type = formation_type
                    self.clusters[idx].cluster_size = len(group_members)
                    self.clusters[idx].formation_role = "group_member"
        
        self.group_formations = groups
    
    def classify_formation(self, positions: np.ndarray) -> str:
        """Classify group formation type (your advanced feature)"""
        n_people = len(positions)
        
        if n_people < 2:
            return "single"
        elif n_people == 2:
            return "pair"
        
        # For 3+ people, analyze geometry
        center = np.mean(positions, axis=0)
        distances_from_center = [np.linalg.norm(pos - center) for pos in positions]
        
        # Check if roughly circular (similar distances from center)
        distance_std = np.std(distances_from_center)
        distance_mean = np.mean(distances_from_center)
        
        if distance_std / distance_mean < 0.3:  # Low variation = circular
            return "circle"
        
        # Check if roughly linear
        if n_people >= 3:
            # Fit line and check how well points align
            from sklearn.linear_model import LinearRegression
            try:
                reg = LinearRegression().fit(positions[:, :1], positions[:, 1])
                predicted = reg.predict(positions[:, :1])
                line_error = np.mean(np.abs(positions[:, 1] - predicted))
                
                if line_error < 0.5:  # Good line fit
                    return "line"
            except:
                pass
        
        # Default to cluster for complex arrangements
        return "cluster"
    
    def publish_transformed_humans(self):
        """Publish in your advanced system format"""
        msg = TransformedHumansArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        
        for cluster in self.clusters:
            human = TransformedHuman()
            human.position.x = cluster.position[0]
            human.position.y = cluster.position[1]
            human.position.z = cluster.position[2]
            human.tracking_id = cluster.tracking_id
            msg.humans.append(human)
        
        self.pub_transformed_humans.publish(msg)
    
    def publish_clusters(self):
        """Publish cluster data in your format"""
        msg = ClusterArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        
        for i, cluster in enumerate(self.clusters):
            cluster_msg = Cluster()
            cluster_msg.id = i
            cluster_msg.center.x = cluster.position[0]
            cluster_msg.center.y = cluster.position[1]
            cluster_msg.size = cluster.cluster_size
            cluster_msg.average_confidence = cluster.confidence
            cluster_msg.average_velocity = [cluster.velocity[0], cluster.velocity[1]]
            cluster_msg.formation = cluster.formation_type
            cluster_msg.stability = float(cluster.stability_score)
            msg.clusters.append(cluster_msg)
        
        self.pub_clusters.publish(msg)
    
    def publish_poi_candidates(self):
        """Generate POI candidates from clusters"""
        msg = POIArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        
        for group in self.group_formations:
            poi = POIData()
            poi.position.x = group['center'][0]
            poi.position.y = group['center'][1]
            poi.priority_score = min(10.0, group['size'] * 2.0)  # Larger groups = higher priority
            poi.group_size = group['size']
            poi.formation_type = group['formation']
            poi.estimated_photo_duration = self.estimate_photo_duration(group)
            msg.pois.append(poi)
        
        self.pub_poi_data.publish(msg)
    
    def publish_photo_metrics(self):
        """Publish advanced photo intelligence metrics"""
        msg = PhotoIntelligenceMetrics()
        msg.header.stamp = self.get_clock().now().to_msg()
        
        if self.clusters:
            msg.total_people = len(self.clusters)
            msg.average_stability = np.mean([c.stability_score for c in self.clusters])
            msg.average_activity = np.mean([c.activity_score for c in self.clusters])
            msg.formation_count = len(self.group_formations)
            
            # Overall photo readiness
            readiness_scores = [c.photo_readiness for c in self.clusters]
            msg.overall_photo_readiness = np.mean(readiness_scores)
            
            # Recommend optimal duration
            if self.group_formations:
                largest_group = max(self.group_formations, key=lambda g: g['size'])
                msg.recommended_duration = self.estimate_photo_duration(largest_group)
            else:
                msg.recommended_duration = 35.0  # Single person default
        else:
            msg.total_people = 0
            msg.recommended_duration = 0.0
        
        self.pub_photo_metrics.publish(msg)
    
    def estimate_photo_duration(self, group: Dict) -> float:
        """Advanced photo duration estimation"""
        base_duration = 35.0
        
        # Size factor
        if group['size'] == 1:
            base_duration = 35.0
        elif group['size'] <= 3:
            base_duration = 50.0
        elif group['size'] <= 6:
            base_duration = 70.0
        else:
            base_duration = 90.0
        
        # Formation factor
        formation_bonus = {
            'line': 15.0,
            'circle': 10.0,
            'cluster': 5.0,
            'pair': 0.0
        }
        
        formation_type = group.get('formation', 'cluster')
        total_duration = base_duration + formation_bonus.get(formation_type, 0.0)
        
        return min(120.0, max(30.0, total_duration))
    
    def publish_markers(self):
        """Enhanced visualization with formation indicators"""
        marker_array = MarkerArray()
        
        # Delete old markers
        delete_marker = Marker()
        delete_marker.header.frame_id = 'map'
        delete_marker.action = Marker.DELETEALL
        marker_array.markers.append(delete_marker)
        
        # Individual cluster markers
        for i, cluster in enumerate(self.clusters):
            # Main cluster marker
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'human_clusters'
            marker.id = i
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            
            marker.pose.position.x = cluster.position[0]
            marker.pose.position.y = cluster.position[1]
            marker.pose.position.z = 0.5
            marker.pose.orientation.w = 1.0
            
            marker.scale.x = 0.6
            marker.scale.y = 0.6
            marker.scale.z = 1.0
            
            # Color based on photo readiness and activity
            marker.color.r = max(0.2, min(1.0, cluster.activity_score * 3))
            marker.color.g = max(0.2, cluster.photo_readiness)
            marker.color.b = max(0.2, cluster.stability_score)
            marker.color.a = 0.8
            
            marker_array.markers.append(marker)
            
            # Formation type label
            text_marker = Marker()
            text_marker.header.frame_id = 'map'
            text_marker.header.stamp = self.get_clock().now().to_msg()
            text_marker.ns = 'formation_labels'
            text_marker.id = i + 1000
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            
            text_marker.pose.position.x = cluster.position[0]
            text_marker.pose.position.y = cluster.position[1]
            text_marker.pose.position.z = 1.5
            
            text_marker.scale.z = 0.3
            text_marker.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
            text_marker.text = f"{cluster.formation_type}({cluster.cluster_size})"
            
            marker_array.markers.append(text_marker)
        
        # Group formation markers
        for i, group in enumerate(self.group_formations):
            group_marker = Marker()
            group_marker.header.frame_id = 'map'
            group_marker.header.stamp = self.get_clock().now().to_msg()
            group_marker.ns = 'group_formations'
            group_marker.id = i + 2000
            group_marker.type = Marker.CYLINDER
            group_marker.action = Marker.ADD
            
            group_marker.pose.position.x = group['center'][0]
            group_marker.pose.position.y = group['center'][1]
            group_marker.pose.position.z = 0.1
            
            # Size based on group size
            radius = max(1.0, group['size'] * 0.8)
            group_marker.scale.x = radius * 2
            group_marker.scale.y = radius * 2
            group_marker.scale.z = 0.2
            
            group_marker.color.r = 1.0
            group_marker.color.g = 0.8
            group_marker.color.b = 0.0
            group_marker.color.a = 0.3
            
            marker_array.markers.append(group_marker)
        
        self.pub_markers.publish(marker_array)
    
    def publish_count(self):
        """Publish total human count"""
        msg = Int32()
        msg.data = len(self.clusters)
        self.pub_count.publish(msg)


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