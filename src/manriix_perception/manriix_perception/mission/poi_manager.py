#!/usr/bin/env python3

"""
POIManager - Point of Interest Management
================================================

Intelligent POI (Point of Interest) manager that extracts position optimization
from the monolithic clustering node and provides intelligent features:

Features:
- Social navigation principles (respect personal space)
- Multi-angle position calculation for groups
- Dynamic priority scoring based on photo potential
- Environmental constraint handling (obstacles, map boundaries)
- Formation-aware positioning (circle, line, cluster patterns)
- Aesthetic composition rules (rule of thirds, symmetry)
- Distance optimization for different lens types
- Real-time position updates as crowds change

This replaces the simple position optimizer with production-grade POI management.
"""

import rclpy
from rclpy.lifecycle import LifecycleNode, State, TransitionCallbackReturn
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import numpy as np
from typing import Dict, List, Optional, Tuple
import time
import math
import yaml
import os
from tf_transformations import euler_from_quaternion  # ADD
from dataclasses import dataclass
from enum import Enum

from std_msgs.msg import String, Header
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from manriix_perception.msg import ClusterArray, Cluster, OptimalPosition, POIArray, POIData
from sensor_msgs.msg import PointCloud2
from manriix_perception.clustering.obstacle_analyzer import ObstacleAnalyzer
from manriix_perception.clustering.position_evaluator import PositionEvaluator
from manriix_perception.clustering.map_handler import MapHandler
from manriix_perception.clustering.fallback_manager import FallbackManager


class FormationType(Enum):
    """Formation types aligned with cluster_detector.py output"""
    SINGLE_PERSON = "single_person"
    PAIR = "pair"                    # 2 people
    SMALL_GROUP = "small_group"      # 3-4 people
    LINE = "line"                    # Linear formation
    CIRCLE = "circle"                # Circular formation
    LARGE_GROUP = "large_group"      # 9+ people
    SCATTERED = "scattered"          # No clear pattern
    FORMAL_POSE = "formal_pose"      # Organized posed formation (any event type)


@dataclass
class POI:
    """Point of Interest representation"""
    position: np.ndarray           # [x, y] position in map frame
    orientation: float             # Robot orientation for this position
    priority_score: float          # Overall priority (0.0-1.0)
    cluster_id: int               # Associated cluster ID
    formation_type: FormationType  # Type of formation being photographed
    social_distance: float        # Distance from humans (for social nav)
    aesthetic_score: float        # Aesthetic composition score
    accessibility_score: float    # How accessible this position is
    stability_score: float        # How stable this position is over time
    multi_angle_set: List         # Multiple angles for same cluster
    

@dataclass
class SocialNavConstraints:
    """Social navigation constraints"""
    personal_space_radius: float = 1.2    # Minimum distance from people
    intimate_space_radius: float = 0.45   # Intimate space (avoid completely)
    social_space_radius: float = 2.5      # Social interaction space
    public_space_radius: float = 3.5      # Public/formal space
    approach_angle_preference: float = 45.0  # Preferred approach angle (degrees)


@dataclass
class HighDensityPosition:
    """Stored high-density position for intelligent fallback"""
    position: np.ndarray           # [x, y] position in map frame
    max_density: float            # Maximum crowd density observed here (people/m²)
    max_group_size: int           # Largest group size seen at this position
    success_count: int            # Number of successful photos taken here
    last_successful_time: float   # Timestamp of last successful photo
    formation_type: str           # Most common formation seen here
    avg_photo_duration: float     # Average photo session duration
    accessibility_score: float    # How easy it is to reach this position (0.0-1.0)
    total_photos_taken: int       # Total number of photos taken from this position
    

class POIManager(LifecycleNode):
    """
    POI manager for intelligent position optimization.
    
    Capabilities:
    - Social navigation with personal space respect
    - Formation-aware positioning for different group types
    - Aesthetic composition optimization (rule of thirds, symmetry)
    - Multi-angle calculation for comprehensive coverage
    - Dynamic priority scoring with multiple factors
    - Environmental constraint handling (obstacles, boundaries)
    - Real-time updates as crowd dynamics change
    """
    
    def __init__(self):
        super().__init__('poi_manager')
        self.declare_parameter('personal_space_radius', 1.2)
        self.declare_parameter('social_space_radius', 2.5)
        self.declare_parameter('photo_distance_min', 2.0)
        self.declare_parameter('photo_distance_max', 8.0)
        self.declare_parameter('photo_distance_optimal', 3.5)
        self.declare_parameter('rule_of_thirds_weight', 0.3)
        self.declare_parameter('symmetry_weight', 0.2)
        self.declare_parameter('formation_weight', 0.25)
        self.declare_parameter('accessibility_weight', 0.25)
        self.declare_parameter('poi_update_rate', 2.0)
        self.declare_parameter('position_stability_time', 3.0)
        self.declare_parameter('config_path', '')

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        """Load config, build evaluators, create pubs/subs. Wait for /map before returning."""
        self.get_logger().info("Configuring POIManager...")
        try:
            # Read parameters
            self.social_constraints = SocialNavConstraints(
                personal_space_radius=self.get_parameter('personal_space_radius').value,
                social_space_radius=self.get_parameter('social_space_radius').value
            )
            self.photo_distance_min = self.get_parameter('photo_distance_min').value
            self.photo_distance_max = self.get_parameter('photo_distance_max').value
            self.photo_distance_optimal = self.get_parameter('photo_distance_optimal').value
            self.rule_thirds_weight = self.get_parameter('rule_of_thirds_weight').value
            self.symmetry_weight = self.get_parameter('symmetry_weight').value
            self.formation_weight = self.get_parameter('formation_weight').value
            self.accessibility_weight = self.get_parameter('accessibility_weight').value

            # State
            self.current_clusters: List[Dict] = []
            self.current_pois: List[POI] = []
            self.robot_position: Optional[np.ndarray] = None
            self.robot_orientation: Optional[float] = None
            self.map_data: Optional[OccupancyGrid] = None
            self.costmap_data: Optional[OccupancyGrid] = None
            self.poi_history = {}
            self.high_density_positions: List[HighDensityPosition] = []
            self.max_stored_positions = 20
            self.min_density_threshold = 2.0
            self.min_success_threshold = 2

            # Load clustering config
            _clustering_config_path = self.get_parameter('config_path').value \
                if self.has_parameter('config_path') else ''
            _yaml_config = {}
            if _clustering_config_path and os.path.exists(_clustering_config_path):
                try:
                    with open(_clustering_config_path, 'r') as f:
                        _raw = yaml.safe_load(f)
                    if 'human_clustering_node' in _raw:
                        _yaml_config = _raw['human_clustering_node'].get('ros__parameters', {})
                    else:
                        _yaml_config = _raw
                    self.get_logger().info(f"POIManager loaded config from {_clustering_config_path}")
                except Exception as e:
                    self.get_logger().warn(f"POIManager could not load clustering config: {e} — using defaults")

            config = {
                'map': _yaml_config.get('map', {
                    'free_threshold': 20,
                    'occupied_threshold': 80,
                    'unknown_cell_strategy': 'cautious'
                }),
                'obstacle': _yaml_config.get('obstacle', {
                    'min_safety_margin': 2,
                    'max_safety_margin': 5,
                    'density_radius': 10,
                    'complexity_calculation': 'entropy'
                }),
                'position': {
                    'reachability_check': _yaml_config.get('position', {}).get('reachability_check', True),
                    'history_length': _yaml_config.get('position', {}).get('history_length', 10),
                    'composition_history_length': 10,
                    'formation_scores': _yaml_config.get('position', {}).get('formation_scores', {
                        'single_person': 0.7, 'pair': 0.8, 'small_group': 0.85,
                        'line': 1.0, 'circle': 0.95, 'large_group': 0.75,
                        'scattered': 0.6, 'formal_pose': 1.0, 'unknown': 0.5
                    }),
                    'weights': _yaml_config.get('position', {}).get('weights', {
                        'size': 0.08, 'density': 0.05, 'formation': 0.10,
                        'stability': 0.10, 'obstacle_proximity': 0.12,
                        'environment_complexity': 0.05, 'path_feasibility': 0.16,
                        'temporal_consistency': 0.05
                    })
                }
            }

            self.map_handler = MapHandler(config, node=self)
            self.obstacle_analyzer = ObstacleAnalyzer(config, self.map_handler, node=self)
            self.position_evaluator = PositionEvaluator(config, self.map_handler, self.obstacle_analyzer, node=self)
            self.fallback_manager = FallbackManager(config, node=self)

            # QoS profiles
            qos_reliable = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, depth=5)
            qos_best_effort = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=3)
            qos_map = QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                depth=1
            )
            qos_costmap = QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                depth=1
            )

            # Subscribers
            self.clusters_sub = self.create_subscription(
                ClusterArray, '/human_clustering/clusters',
                self.clusters_callback, qos_reliable)
            self.robot_pose_sub = self.create_subscription(
                Odometry, '/odometry/filtered',
                self.robot_pose_callback, qos_best_effort)
            self.map_sub = self.create_subscription(
                OccupancyGrid, '/map',
                self.map_callback, qos_map)
            self.costmap_sub = self.create_subscription(
                OccupancyGrid, '/local_costmap/costmap',
                self.costmap_callback, qos_costmap)

            # Publishers
            self.optimal_position_pub = self.create_publisher(
                OptimalPosition, '/poi_manager/optimal_positions', qos_reliable)
            self.all_pois_pub = self.create_publisher(
                String, '/poi_manager/all_pois', qos_best_effort)
            self.poi_array_pub = self.create_publisher(
                POIArray, '/poi_manager/poi_array', qos_best_effort)
            self.poi_metrics_pub = self.create_publisher(
                String, '/poi_manager/metrics', qos_best_effort)

            self.get_logger().info("POIManager configured — /map will arrive via TRANSIENT_LOCAL subscription")
            return TransitionCallbackReturn.SUCCESS

        except Exception as e:
            self.get_logger().error(f"Configuration failed: {e}")
            return TransitionCallbackReturn.FAILURE

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        """Start POI update loop."""
        self.get_logger().info("Activating POIManager...")
        try:
            update_rate = self.get_parameter('poi_update_rate').value
            self.control_timer = self.create_timer(1.0 / update_rate, self.poi_update_loop)
            self.get_logger().info(f"POIManager active — update loop at {update_rate}Hz")
            return TransitionCallbackReturn.SUCCESS
        except Exception as e:
            self.get_logger().error(f"Activation failed: {e}")
            return TransitionCallbackReturn.FAILURE

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        """Stop POI update loop."""
        self.get_logger().info("Deactivating POIManager...")
        try:
            if hasattr(self, 'control_timer') and self.control_timer:
                self.control_timer.cancel()
                self.control_timer = None
            return TransitionCallbackReturn.SUCCESS
        except Exception as e:
            self.get_logger().error(f"Deactivation failed: {e}")
            return TransitionCallbackReturn.FAILURE

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        """Release all resources."""
        self.get_logger().info("Cleaning up POIManager...")
        try:
            if hasattr(self, 'control_timer') and self.control_timer:
                self.control_timer.cancel()
            self.current_clusters = []
            self.current_pois = []
            self.map_data = None
            self.costmap_data = None
            return TransitionCallbackReturn.SUCCESS
        except Exception as e:
            self.get_logger().error(f"Cleanup failed: {e}")
            return TransitionCallbackReturn.FAILURE

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        """Final cleanup."""
        self.get_logger().info("Shutting down POIManager...")
        try:
            if hasattr(self, 'control_timer') and self.control_timer:
                self.control_timer.cancel()
            return TransitionCallbackReturn.SUCCESS
        except Exception as e:
            self.get_logger().error(f"Shutdown failed: {e}")
            return TransitionCallbackReturn.FAILURE        
    def _validate_position(self, position: np.ndarray) -> bool:
        """Validate position array for NaN/Inf values"""
        try:
            if position is None or len(position) < 2:
                return False
            x, y = float(position[0]), float(position[1])
            if np.isnan(x) or np.isnan(y) or np.isinf(x) or np.isinf(y):
                return False
            if abs(x) > 1000 or abs(y) > 1000:
                return False
            return True
        except (TypeError, ValueError, IndexError):
            return False

    def clusters_callback(self, msg: ClusterArray):
        """Process cluster updates and recalculate POIs with NaN validation"""
        try:
            clusters = []
            for cluster_msg in msg.clusters:
                # Validate center coordinates before storing
                center_x = cluster_msg.center.x
                center_y = cluster_msg.center.y

                # NaN/Inf safety net
                if np.isnan(center_x) or np.isnan(center_y):
                    self.get_logger().warn(f"Skipping cluster {cluster_msg.id}: NaN center")
                    continue
                if np.isinf(center_x) or np.isinf(center_y):
                    self.get_logger().warn(f"Skipping cluster {cluster_msg.id}: Inf center")
                    continue
                if abs(center_x) > 1000 or abs(center_y) > 1000:
                    self.get_logger().warn(f"Skipping cluster {cluster_msg.id}: unreasonable center")
                    continue

                # Validate radius
                radius = cluster_msg.radius
                if np.isnan(radius) or np.isinf(radius) or radius <= 0:
                    radius = 0.5  # Default safe radius

                cluster = {
                    'id': cluster_msg.id,
                    'track_id': cluster_msg.id,
                    'center': np.array([center_x, center_y]),
                    'radius': radius,
                    'size': max(1, cluster_msg.size),  # Ensure positive size
                    'formation': cluster_msg.formation,
                    'stability': cluster_msg.stability if not np.isnan(cluster_msg.stability) else 0.5,
                    'density': cluster_msg.density if not np.isnan(cluster_msg.density) else 1.0
                }
                clusters.append(cluster)

            self.current_clusters = clusters
            self.get_logger().info(f"POI Manager received {len(clusters)} clusters")

        except Exception as e:
            self.get_logger().error(f"Error processing clusters: {e}")
            
            
    def robot_pose_callback(self, msg: Odometry):
        """Update robot position for distance calculations"""
        try:
            self.robot_position = np.array([
                msg.pose.pose.position.x,
                msg.pose.pose.position.y
            ])
            
            # Extract orientation
            # from tf_transformations import euler_from_quaternion
            q = msg.pose.pose.orientation
            _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
            self.robot_orientation = yaw
            
        except Exception as e:
            self.get_logger().error(f"Error processing robot pose: {e}")
            
            
    def map_callback(self, msg: OccupancyGrid):
        """Update map for obstacle avoidance"""
        self.map_data = msg
        
    def costmap_callback(self, msg: OccupancyGrid):
        """Live costmap — includes dynamic obstacles (people, moving objects)"""
        self.costmap_data = msg

    def poi_update_loop(self):
            """Main POI calculation and update loop"""
            try:
                # ADD THIS DEBUG LINE:
                self.get_logger().debug(f"POI update loop - clusters: {len(self.current_clusters) if hasattr(self, 'current_clusters') else 'not initialized'}")
                
                if not self.current_clusters:
                    return
                    
                # Calculate POIs for all clusters
                new_pois = []
                for cluster in self.current_clusters:
                    cluster_pois = self.calculate_cluster_pois(cluster)
                    new_pois.extend(cluster_pois)
                    
                # Update stability scores based on history
                self.update_poi_stability(new_pois)
                
                # Sort by priority
                new_pois.sort(key=lambda p: p.priority_score, reverse=True)
                
                self.current_pois = new_pois
                
                # Publish best POI
                if new_pois:
                    self.publish_optimal_position(new_pois[0])
                    
                # Publish all POIs for visualization
                self.publish_all_pois()
                self.publish_poi_array()  # Added: POIArray for exploration node
                self.publish_poi_metrics()
                
            except Exception as e:
                self.get_logger().error(f"Error in POI update loop: {e}")
            
            
    def calculate_cluster_pois(self, cluster: Dict) -> List[POI]:
        """Calculate POIs for a single cluster with NaN validation"""
        try:
            cluster_center = cluster['center']

            # ADD THIS DEBUG LINE:
            self.get_logger().debug(f"Calculating POIs for cluster {cluster.get('id', '?')}: center={cluster_center}, size={cluster.get('size')}, formation={cluster.get('formation')}")

            # Validate cluster center before processing
            if not self._validate_position(cluster_center):
                self.get_logger().debug(f"Skipping cluster {cluster.get('id', '?')}: invalid center")
                return []

            cluster_radius = cluster['radius']
            cluster_size = cluster['size']
            formation_type = FormationType(cluster.get('formation', 'scattered'))
            
            pois = []
            
            # Determine optimal distances based on cluster size
            optimal_distance = self.calculate_optimal_distance(cluster_size, formation_type)
            
            # Generate candidate positions around the cluster
            candidate_positions = self.generate_candidate_positions(
                cluster_center, cluster_radius, optimal_distance, formation_type
            )
            
            for position in candidate_positions:
                # Calculate orientation to face cluster center
                orientation = self.calculate_optimal_orientation(position, cluster_center)
                
                # Score this POI
                poi = POI(
                    position=position,
                    orientation=orientation,
                    priority_score=0.0,  # Will be calculated
                    cluster_id=cluster['id'],
                    formation_type=formation_type,
                    social_distance=np.linalg.norm(position - cluster_center),
                    aesthetic_score=0.0,  # Will be calculated
                    accessibility_score=0.0,  # Will be calculated
                    stability_score=0.0,  # Will be updated from history
                    multi_angle_set=[]
                )
                
                # Calculate scores - aesthetic scoring removed, using sophisticated evaluation
                poi.accessibility_score = self.calculate_accessibility_score(poi)
                poi.priority_score = self.calculate_priority_score(poi, cluster)
                
                # ADD THIS DEBUG:
                self.get_logger().debug(f"  Final check: access={poi.accessibility_score:.2f}, social_dist={poi.social_distance:.2f}m, min={self.social_constraints.personal_space_radius:.2f}m, PASS={poi.accessibility_score > 0.3 and poi.social_distance >= self.social_constraints.personal_space_radius}")

                # Only keep POIs that meet minimum requirements
                if (poi.accessibility_score >= 0.3 and 
                    poi.social_distance >= self.social_constraints.personal_space_radius):
                    pois.append(poi)
            
            # ADD THIS DEBUG LINE BEFORE RETURN:
            self.get_logger().debug(f"Generated {len(pois)} POIs for cluster {cluster.get('id', '?')}")
            return pois
            
        except Exception as e:
            self.get_logger().error(f"Error calculating cluster POIs: {e}")
            return []
            
            
    def calculate_optimal_distance(self, cluster_size: int, formation_type: FormationType) -> float:
        """Calculate optimal photo distance based on cluster size and formation"""
        try:
            # Base distance from cluster size
            if cluster_size == 1:
                base_distance = 2.5  # Closer for portraits
            elif cluster_size <= 3:
                base_distance = 3.0  # Small groups
            elif cluster_size <= 6:
                base_distance = 4.0  # Medium groups
            else:
                base_distance = 5.0  # Large groups need distance

            # Adjust for formation type - all types from cluster_detector
            if formation_type == FormationType.SINGLE_PERSON:
                base_distance -= 0.3  # Can get closer for single person
            elif formation_type == FormationType.PAIR:
                base_distance += 0.0  # Pairs are optimal at base distance
            elif formation_type == FormationType.SMALL_GROUP:
                base_distance += 0.2  # Slight increase for 3-4 people
            elif formation_type == FormationType.LINE:
                base_distance += 0.5  # Lines need more distance to capture width
            elif formation_type == FormationType.CIRCLE:
                base_distance += 0.3  # Circles need slight distance increase
            elif formation_type == FormationType.LARGE_GROUP:
                base_distance += 0.8  # Large groups need more distance
            elif formation_type == FormationType.FORMAL_POSE:
                base_distance += 1.0  # Formal poses need more distance for framing
            elif formation_type == FormationType.SCATTERED:
                base_distance += 0.5  # Scattered needs distance to capture all

            # Clamp to reasonable bounds
            return max(self.photo_distance_min, min(self.photo_distance_max, base_distance))

        except Exception:
            return self.photo_distance_optimal
            
            
    def generate_candidate_positions(self, cluster_center: np.ndarray, cluster_radius: float, 
                                   optimal_distance: float, formation_type: FormationType) -> List[np.ndarray]:
        """Generate candidate positions around cluster"""
        try:
            positions = []
            
            # Number of positions to generate based on formation type
            if formation_type == FormationType.SINGLE_PERSON:
                num_positions = 8  # 360° around person in 45° increments
            elif formation_type == FormationType.PAIR:
                num_positions = 8  # Multiple angles for couples/pairs
            elif formation_type == FormationType.SMALL_GROUP:
                num_positions = 10  # Good coverage for small groups
            elif formation_type == FormationType.LINE:
                num_positions = 6  # Focus on perpendicular positions
            elif formation_type == FormationType.CIRCLE:
                num_positions = 12  # Many angles for circle compositions
            elif formation_type == FormationType.LARGE_GROUP:
                num_positions = 12  # More options for large groups
            elif formation_type == FormationType.FORMAL_POSE:
                num_positions = 6  # Formal poses typically have preferred front angle
            elif formation_type == FormationType.SCATTERED:
                num_positions = 8  # Default coverage for scattered
            else:
                num_positions = 8  # Default
                
            # Generate positions in a circle around cluster
            for i in range(num_positions):
                angle = 2 * np.pi * i / num_positions
                
                # Calculate position at optimal distance
                position = cluster_center + optimal_distance * np.array([
                    np.cos(angle), np.sin(angle)
                ])
                
                # For line formations, prefer perpendicular angles
                if formation_type == FormationType.LINE:
                    # Bias towards perpendicular angles (0°, 90°, 180°, 270°)
                    preferred_angles = [0, np.pi/2, np.pi, 3*np.pi/2]
                    closest_preferred = min(preferred_angles, key=lambda a: abs(angle - a))
                    if abs(angle - closest_preferred) > np.pi/4:
                        continue  # Skip non-perpendicular positions
                        
                positions.append(position)

            # ADD THIS DEBUG LINE:
            self.get_logger().debug(f"Generated {len(positions)} candidate positions at distance {optimal_distance:.2f}m")
                
            return positions
            
        except Exception as e:
            self.get_logger().error(f"Error generating candidate positions: {e}")
            return []
            
            
    def calculate_optimal_orientation(self, position: np.ndarray, target: np.ndarray) -> float:
        """Calculate optimal orientation to face target"""
        try:
            direction_vector = target - position
            return math.atan2(direction_vector[1], direction_vector[0])
        except Exception:
            return 0.0
            
            
    def calculate_aesthetic_score(self, poi: POI, cluster: Dict) -> float:
        """Calculate aesthetic composition score"""
        try:
            score = 0.0
            
            # Rule of thirds score
            # Check if cluster center aligns with rule of thirds grid
            cluster_center = cluster['center']
            distance = poi.social_distance
            
            # Simple rule of thirds approximation
            # Ideal positions would place subjects at 1/3 and 2/3 of frame
            angle_to_center = math.atan2(
                cluster_center[1] - poi.position[1],
                cluster_center[0] - poi.position[0]
            )
            
            # Score based on distance (optimal framing)
            distance_score = 1.0 - abs(distance - self.photo_distance_optimal) / self.photo_distance_optimal
            distance_score = max(0.0, min(1.0, distance_score))
            
            # Formation-specific scoring - all types from cluster_detector
            formation_score = 0.5  # Default for unknown
            if poi.formation_type == FormationType.SINGLE_PERSON:
                formation_score = 0.6  # Single person can be well-framed
            elif poi.formation_type == FormationType.PAIR:
                formation_score = 0.7  # Pairs create natural compositions
            elif poi.formation_type == FormationType.SMALL_GROUP:
                formation_score = 0.75  # Small groups are easy to compose
            elif poi.formation_type == FormationType.LINE:
                formation_score = 0.7  # Lines create good compositions
            elif poi.formation_type == FormationType.CIRCLE:
                formation_score = 0.8  # Circles are naturally aesthetic
            elif poi.formation_type == FormationType.LARGE_GROUP:
                formation_score = 0.65  # Large groups harder to compose well
            elif poi.formation_type == FormationType.SCATTERED:
                formation_score = 0.4  # Scattered lacks visual cohesion
            elif poi.formation_type == FormationType.FORMAL_POSE:
                formation_score = 0.9  # Formal poses are highly aesthetic
                
            # Symmetry score (prefer positions that create symmetric compositions)
            symmetry_score = 0.5  # Placeholder for more complex symmetry calculation
            
            # Combine scores with weights
            score = (distance_score * self.rule_thirds_weight +
                    formation_score * self.formation_weight +
                    symmetry_score * self.symmetry_weight)
                    
            return max(0.0, min(1.0, score))
            
        except Exception as e:
            self.get_logger().error(f"Error calculating aesthetic score: {e}")
            return 0.5
            
            
    def calculate_accessibility_score(self, poi: POI) -> float:
        """Calculate how accessible this position is for the robot"""
        try:
            if self.map_data is None:
                return 0.7  # Default moderate accessibility
                
            # # Check if position is in free space
            # is_free = self.is_position_free(poi.position)

            # Check if position is a permanent obstacle (walls only — static map)
            # Do NOT use live costmap here: a bystander walking through the
            # photography position momentarily should NOT reject the POI entirely
            is_free = self.is_position_free(poi.position, static_only=True)
                        
            # ADD THIS DEBUG:
            self.get_logger().debug(f"  POI at [{poi.position[0]:.2f}, {poi.position[1]:.2f}]: is_free={is_free}")
            
            if not is_free:
                return 0.0  # Position is blocked
                
            # Check path accessibility from robot position
            if self.robot_position is not None:
                path_clear = self.check_path_clearance(self.robot_position, poi.position)
                if not path_clear:
                    return 0.3  # Path is partially blocked
                    
            # Distance penalty - very far positions are less accessible
            if self.robot_position is not None:
                distance = np.linalg.norm(poi.position - self.robot_position)

                # ADD: hard reject — POI too far to be useful
                # Stops 20m POIs that slipped through clustering filter
                if distance > 15.0:
                    return 0.0
                                
                distance_penalty = min(1.0, distance / 20.0)  # Penalty starts at 20m
                return max(0.3, 1.0 - distance_penalty)
                
            return 0.8  # Good default accessibility
            
        except Exception as e:
            self.get_logger().error(f"Error calculating accessibility score: {e}")
            return 0.5

    # REPLACE WITH:
    def is_position_free(self, position: np.ndarray, static_only: bool = False) -> bool:
        try:
            if static_only or self.costmap_data is None:
                map_to_check = self.map_data
            else:
                map_to_check = self.costmap_data    
            if map_to_check is None:
                return True
            
            # Validate position before using
            if not self._validate_position(position):
                self.get_logger().debug("is_position_free: invalid position (NaN/Inf)")
                return False

            # Convert world coordinates to map coordinates
            pos_x = float(position[0])
            pos_y = float(position[1])
            # resolution = float(self.map_data.info.resolution)
            origin_x = float(map_to_check.info.origin.position.x)
            origin_y = float(map_to_check.info.origin.position.y)
            resolution = float(map_to_check.info.resolution)

            if resolution <= 0:
                return True

            map_x = int((pos_x - origin_x) / resolution)
            map_y = int((pos_y - origin_y) / resolution)
            
            if (map_x < 0 or map_x >= map_to_check.info.width or
                map_y < 0 or map_y >= map_to_check.info.height):
                return False
        
            # Check occupancy
            index = map_y * map_to_check.info.width + map_x
            if index < len(map_to_check.data):
                occupancy = map_to_check.data[index]            
                # return occupancy < 50  # Free if less than 50% occupied
                return 0 <= occupancy < 50
            return False

        except (TypeError, ValueError) as e:
            self.get_logger().debug(f"is_position_free: conversion error: {e}")
            return False
        except Exception:
            return True  # Default to free if can't check
            
            
    def check_path_clearance(self, start: np.ndarray, end: np.ndarray) -> bool:
        """Basic line-of-sight check between start and end positions with NaN validation"""
        try:
            # Validate positions first
            if not self._validate_position(start) or not self._validate_position(end):
                return False

            # Simple line check - sample points along path
            distance = np.linalg.norm(end - start)

            # Validate distance (prevent NaN propagation)
            if np.isnan(distance) or np.isinf(distance):
                return False

            num_samples = int(distance / 0.2)  # Sample every 20cm

            for i in range(num_samples + 1):
                t = i / max(num_samples, 1)
                sample_point = start + t * (end - start)
                if not self.is_position_free(sample_point, static_only=True):
                    return False

            return True

        except Exception:
            return True  # Default to clear if can't check
            
            
    def calculate_priority_score(self, poi: POI, cluster: Dict) -> float:
        """Calculate sophisticated priority score using integrated position evaluation"""
        try:
            # Convert POI to position evaluation format
            # position_dict = {
            #     'position': poi.position,
            #     'cluster_size': cluster['size'],
            #     'formation': poi.formation_type.value,
            #     'track_id': cluster.get('track_id', -1),
            #     'stability': cluster.get('stability', poi.stability_score),
            #     'density': cluster.get('density', self.calculate_crowd_density(cluster)),
            #     'is_backup': False
            # }

            # ADD one line:
            position_dict = {
                'position': poi.position,
                'cluster_center': cluster['center'],    # ← ADD THIS
                'cluster_size': cluster['size'],
                'formation': poi.formation_type.value,
                'track_id': cluster.get('track_id', -1),
                'stability': cluster.get('stability', poi.stability_score),
                'density': cluster.get('density', self.calculate_crowd_density(cluster)),
                'is_backup': False
            }

            # Use sophisticated 10-factor scoring from position evaluator
            robot_pos = self.robot_position if self.robot_position is not None else np.zeros(2)
            sophisticated_score = self.position_evaluator.calculate_priority_score(position_dict, robot_pos)
            
            # Add social navigation bonus
            social_bonus = 0.1 if poi.social_distance >= self.social_constraints.personal_space_radius else 0.0
            
            # Add formation awareness bonus - organized formations get bonus
            high_value_formations = [FormationType.CIRCLE, FormationType.LINE, FormationType.FORMAL_POSE]
            medium_value_formations = [FormationType.PAIR, FormationType.SMALL_GROUP]
            if poi.formation_type in high_value_formations:
                formation_bonus = 0.05
            elif poi.formation_type in medium_value_formations:
                formation_bonus = 0.03
            else:
                formation_bonus = 0.0
            
            final_score = sophisticated_score + social_bonus + formation_bonus
            
            return max(0.0, min(1.0, final_score))
            
        except Exception as e:
            self.get_logger().error(f"Error calculating priority score: {e}")
            return 0.5
            
            
    def update_poi_stability(self, pois: List[POI]):
        """Update stability scores based on POI history"""
        try:
            current_time = time.time()
            stability_time = self.get_parameter('position_stability_time').value
            
            for poi in pois:
                # Create key for this POI location
                poi_key = f"{poi.cluster_id}_{poi.position[0]:.1f}_{poi.position[1]:.1f}"
                
                if poi_key not in self.poi_history:
                    self.poi_history[poi_key] = []
                    
                # Add current observation
                self.poi_history[poi_key].append(current_time)
                
                # Remove old observations
                cutoff_time = current_time - stability_time * 2  # Keep 2x stability window
                self.poi_history[poi_key] = [
                    t for t in self.poi_history[poi_key] if t > cutoff_time
                ]
                
                # Calculate stability score
                observations = self.poi_history[poi_key]
                if observations:
                    stability_duration = current_time - observations[0]
                    poi.stability_score = min(1.0, stability_duration / stability_time)
                else:
                    poi.stability_score = 0.0
                    
        except Exception as e:
            self.get_logger().error(f"Error updating POI stability: {e}")
            
            
    def publish_optimal_position(self, poi: POI):
        """Publish the best POI as optimal position"""
        try:
            msg = OptimalPosition()
            msg.x = float(poi.position[0])
            msg.y = float(poi.position[1])
            msg.orientation = float(poi.orientation)
            msg.priority_score = float(poi.priority_score)
            
            self.optimal_position_pub.publish(msg)
            
        except Exception as e:
            self.get_logger().error(f"Error publishing optimal position: {e}")
            
            
    def publish_all_pois(self):
        """Publish all POIs for visualization"""
        try:
            import json
            
            pois_data = []
            for poi in self.current_pois[:10]:  # Limit to top 10 for visualization
                poi_dict = {
                    'position': poi.position.tolist(),
                    'orientation': poi.orientation,
                    'priority_score': poi.priority_score,
                    'cluster_id': poi.cluster_id,
                    'formation_type': poi.formation_type.value,
                    'social_distance': poi.social_distance,
                    'aesthetic_score': poi.aesthetic_score,
                    'accessibility_score': poi.accessibility_score,
                    'stability_score': poi.stability_score
                }
                pois_data.append(poi_dict)
                
            msg = String()
            msg.data = json.dumps(pois_data)
            self.all_pois_pub.publish(msg)
            
        except Exception as e:
            self.get_logger().error(f"Error publishing all POIs: {e}")
            
            
    def publish_poi_array(self):
        """Publish POIArray for exploration node integration"""
        try:
            from std_msgs.msg import Header
            import builtin_interfaces.msg
            
            msg = POIArray()
            msg.header = Header()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'map'
            
            # Convert POIs to POIData messages
            poi_messages = []
            for poi in self.current_pois[:20]:  # Limit to top 20 POIs
                poi_msg = POIData()
                
                # Position and orientation
                poi_msg.position = Point()
                poi_msg.position.x = float(poi.position[0])
                poi_msg.position.y = float(poi.position[1])
                poi_msg.position.z = 0.0
                poi_msg.orientation = float(poi.orientation)
                
                # Scoring breakdown
                poi_msg.priority_score = float(poi.priority_score)
                poi_msg.aesthetic_score = float(poi.aesthetic_score)
                poi_msg.accessibility_score = float(poi.accessibility_score)
                poi_msg.stability_score = float(poi.stability_score)
                poi_msg.social_distance_score = float(min(1.0, poi.social_distance / 3.0))  # Normalize to 0-1
                
                # Associated cluster information
                poi_msg.cluster_id = int(poi.cluster_id)
                poi_msg.formation_type = str(poi.formation_type.value)
                poi_msg.group_size = int(getattr(poi, 'group_size', 1))
                poi_msg.social_distance = float(poi.social_distance)
                
                # Composition analysis (derived from aesthetic score)
                poi_msg.rule_of_thirds_score = float(poi.aesthetic_score * 0.8)  # Estimate
                poi_msg.symmetry_score = float(poi.aesthetic_score * 0.6)  # Estimate
                poi_msg.depth_score = float(poi.aesthetic_score * 0.7)  # Estimate
                poi_msg.framing_score = float(poi.aesthetic_score)
                
                # Accessibility details
                poi_msg.path_clear = bool(poi.accessibility_score > 0.5)
                poi_msg.navigation_difficulty = float(1.0 - poi.accessibility_score)
                poi_msg.distance_from_robot = float(np.linalg.norm(poi.position - self.robot_position) if self.robot_position is not None else 0.0)
                poi_msg.obstacles_detected = "Clear path" if poi.accessibility_score > 0.7 else "Some obstacles"
                
                # Temporal information
                poi_msg.stability_duration = float(getattr(poi, 'stability_duration', 0.0))
                poi_msg.observation_count = int(getattr(poi, 'observation_count', 1))
                
                # Timestamps
                current_time = self.get_clock().now().to_msg()
                poi_msg.first_observed = current_time  # Simplified
                poi_msg.last_updated = current_time
                
                poi_messages.append(poi_msg)
            
            msg.pois = poi_messages
            
            # Summary statistics
            if poi_messages:
                msg.total_pois_calculated = len(self.current_pois)
                msg.average_priority_score = float(np.mean([poi.priority_score for poi in self.current_pois]))
                msg.average_aesthetic_score = float(np.mean([poi.aesthetic_score for poi in self.current_pois]))
                msg.average_accessibility_score = float(np.mean([poi.accessibility_score for poi in self.current_pois]))
                msg.clusters_analyzed = len(set(poi.cluster_id for poi in self.current_pois))
                msg.formation_types_detected = ",".join(set(poi.formation_type.value for poi in self.current_pois))
            else:
                msg.total_pois_calculated = 0
                msg.average_priority_score = 0.0
                msg.average_aesthetic_score = 0.0
                msg.average_accessibility_score = 0.0
                msg.clusters_analyzed = 0
                msg.formation_types_detected = ""
            
            self.poi_array_pub.publish(msg)
            
        except Exception as e:
            self.get_logger().error(f"Error publishing POI array: {e}")
            
            
    def publish_poi_metrics(self):
        """Publish POI calculation metrics as JSON for web interface"""
        try:
            import json

            if not self.current_pois:
                # Publish empty metrics
                msg = String()
                msg.data = json.dumps({'num_pois': 0})
                self.poi_metrics_pub.publish(msg)
                return

            num_pois = len(self.current_pois)
            avg_priority = float(np.mean([p.priority_score for p in self.current_pois]))
            avg_aesthetic = float(np.mean([p.aesthetic_score for p in self.current_pois]))
            avg_accessibility = float(np.mean([p.accessibility_score for p in self.current_pois]))
            avg_stability = float(np.mean([p.stability_score for p in self.current_pois]))
            avg_social = float(np.mean([min(1.0, p.social_distance / 3.0) for p in self.current_pois]))

            # Get formation distribution
            formations = {}
            for poi in self.current_pois:
                f_type = poi.formation_type.value
                formations[f_type] = formations.get(f_type, 0) + 1

            # Build comprehensive metrics JSON matching the 10-factor scoring system
            metrics_data = {
                'num_pois': num_pois,
                'average_priority_score': avg_priority,

                # POI-level scores (simplified for display)
                'average_aesthetic_score': avg_aesthetic,
                'average_accessibility_score': avg_accessibility,
                'average_stability_score': avg_stability,
                'average_social_distance_score': avg_social,

                # # Position evaluator 10-factor breakdown (from config weights)
                # 'scoring_factors': {
                #     'distance': {'weight': 0.20, 'description': 'Proximity to robot'},
                #     'size': {'weight': 0.10, 'description': 'Cluster size'},
                #     'density': {'weight': 0.10, 'description': 'Crowd density'},
                #     'formation': {'weight': 0.10, 'description': 'Group arrangement'},
                #     'stability': {'weight': 0.10, 'description': 'Temporal stability'},
                #     'composition': {'weight': 0.15, 'description': 'Group consistency'},
                #     'obstacle_proximity': {'weight': 0.15, 'description': 'Clear of obstacles'},
                #     'environment_complexity': {'weight': 0.10, 'description': 'Navigation ease'},
                #     'path_feasibility': {'weight': 0.10, 'description': 'Path clearance'},
                #     'temporal_consistency': {'weight': 0.05, 'description': 'Position history'}
                # },

                # Position evaluator 10-factor breakdown (actual weights from config)
                'scoring_factors': {
                    k: {'weight': float(v), 'description': k.replace('_', ' ').title()}
                    for k, v in self.position_evaluator.weights.items()
                },

                # Formation distribution
                'formations': formations
            }

            msg = String()
            msg.data = json.dumps(metrics_data)
            self.poi_metrics_pub.publish(msg)

        except Exception as e:
            self.get_logger().error(f"Error publishing POI metrics: {e}")
            
            
    # ═══════════════════════════════════════════════════════════════
    # HIGH-DENSITY POSITION MEMORY SYSTEM
    # ═══════════════════════════════════════════════════════════════
    
    def record_successful_position(self, position: np.ndarray, cluster: Dict, 
                                 photo_duration: float):
        """Record a successful photo position for future fallback"""
        try:
            density = self.calculate_crowd_density(cluster)
            group_size = cluster.get('size', 0)
            
            # Only store high-value positions
            if density < self.min_density_threshold or group_size < 3:
                return
                
            formation_type = cluster.get('formation', 'unknown')
            current_time = time.time()
            
            # Check if we already have a position nearby
            existing_position = None
            for stored_pos in self.high_density_positions:
                distance = np.linalg.norm(stored_pos.position - position)
                if distance < 1.5:  # Within 1.5m
                    existing_position = stored_pos
                    break
                    
            if existing_position:
                # Update existing position
                existing_position.max_density = max(existing_position.max_density, density)
                existing_position.max_group_size = max(existing_position.max_group_size, group_size)
                existing_position.success_count += 1
                existing_position.last_successful_time = current_time
                existing_position.total_photos_taken += 1
                
                # Update average photo duration
                total_duration = (existing_position.avg_photo_duration * 
                                (existing_position.success_count - 1) + photo_duration)
                existing_position.avg_photo_duration = total_duration / existing_position.success_count
                
                self.get_logger().info(f"Updated high-density position: density={density:.1f}, "
                                     f"success_count={existing_position.success_count}")
            else:
                # Create new high-density position
                # accessibility = self.calculate_accessibility_score(position)

                # REPLACE WITH:
                # calculate_accessibility_score expects a POI object, not np.ndarray
                _temp_poi = POI(
                    position=position,
                    orientation=0.0,
                    priority_score=0.0,
                    cluster_id=-1,
                    formation_type=FormationType.SCATTERED,
                    social_distance=0.0,
                    aesthetic_score=0.0,
                    accessibility_score=0.0,
                    stability_score=0.0,
                    multi_angle_set=[]
                )
                accessibility = self.calculate_accessibility_score(_temp_poi)

                new_position = HighDensityPosition(
                    position=position.copy(),
                    max_density=density,
                    max_group_size=group_size,
                    success_count=1,
                    last_successful_time=current_time,
                    formation_type=formation_type,
                    avg_photo_duration=photo_duration,
                    accessibility_score=accessibility,
                    total_photos_taken=1
                )
                
                self.high_density_positions.append(new_position)
                self.get_logger().info(f"Stored new high-density position: "
                                     f"density={density:.1f}, group_size={group_size}")
                
            # Sort by success score and keep only top positions
            self.high_density_positions.sort(
                key=lambda x: self._calculate_position_score(x), 
                reverse=True
            )
            
            if len(self.high_density_positions) > self.max_stored_positions:
                removed = self.high_density_positions[self.max_stored_positions:]
                self.high_density_positions = self.high_density_positions[:self.max_stored_positions]
                self.get_logger().debug(f"Pruned {len(removed)} old high-density positions")
                
        except Exception as e:
            self.get_logger().error(f"Error recording successful position: {e}")
            
            
    def get_fallback_positions(self, current_position: Optional[np.ndarray] = None,
                             min_score_threshold: float = 0.6) -> List[HighDensityPosition]:
        """Get high-density positions for fallback when no current targets"""
        try:
            if not self.high_density_positions:
                self.get_logger().warn("No stored high-density positions available for fallback")
                return []
                
            # Filter positions by score threshold and recency
            current_time = time.time()
            valid_positions = []
            
            for pos in self.high_density_positions:
                # Calculate time decay (prefer more recent positions)
                time_decay = min(1.0, 7200.0 / max(1.0, current_time - pos.last_successful_time))  # 2-hour window
                score = self._calculate_position_score(pos) * time_decay
                
                if score >= min_score_threshold:
                    # Check accessibility if we have current position
                    if current_position is not None:
                        distance = np.linalg.norm(pos.position - current_position)
                        if distance > 20.0:  # Too far to be useful fallback
                            continue
                            
                    valid_positions.append(pos)
                    
            # Sort by score (already sorted, but apply time decay)
            valid_positions.sort(
                key=lambda x: self._calculate_position_score(x) * 
                             min(1.0, 7200.0 / max(1.0, current_time - x.last_successful_time)),
                reverse=True
            )
            
            self.get_logger().info(f"Found {len(valid_positions)} valid fallback positions")
            return valid_positions[:5]  # Return top 5 fallback positions
            
        except Exception as e:
            self.get_logger().error(f"Error getting fallback positions: {e}")
            return []
            
            
    def _calculate_position_score(self, position: HighDensityPosition) -> float:
        """Calculate overall score for a stored position"""
        try:
            # Normalize factors to 0-1 range
            density_score = min(1.0, position.max_density / 5.0)  # Max expected: 5 people/m²
            size_score = min(1.0, position.max_group_size / 15.0)  # Max expected: 15 people
            success_score = min(1.0, position.success_count / 10.0)  # Max expected: 10 successes
            photo_score = min(1.0, position.total_photos_taken / 50.0)  # Max expected: 50 photos
            
            # Weighted combination
            total_score = (
                density_score * 0.3 +      # Crowd density importance
                size_score * 0.2 +         # Group size importance  
                success_score * 0.3 +      # Success rate importance
                photo_score * 0.1 +        # Photo volume importance
                position.accessibility_score * 0.1  # Accessibility importance
            )
            
            return total_score
            
        except Exception:
            return 0.0
            
            
    def calculate_crowd_density(self, cluster: Dict) -> float:
        """Calculate crowd density (people per square meter)"""
        try:
            size = cluster.get('size', 1)
            radius = cluster.get('radius', 0.5)
            area = np.pi * (radius ** 2)
            return size / max(area, 0.1)  # Avoid division by zero
        except Exception:
            return 0.0


def main(args=None):
    rclpy.init(args=args)
    node = POIManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()