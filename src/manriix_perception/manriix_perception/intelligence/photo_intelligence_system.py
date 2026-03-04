#!/usr/bin/env python3

"""
PhotoIntelligenceSystem - Advanced Dynamic Photo Timing
========================================================

Intelligent photo timing system that replaces fixed 60s duration with dynamic 
30s-120s timing based on:
- Crowd density and size
- Group formation and stability  
- Movement patterns and attention
- Multi-shot sequencing for large groups

This extracts photo control logic from the monolithic clustering node.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
import numpy as np
from typing import Dict, List, Optional, Tuple
import time
import yaml
import os
from dataclasses import dataclass
from enum import Enum



from std_msgs.msg import String, Bool, Float32
from geometry_msgs.msg import Point
from manriix_perception.msg import ClusterArray, Cluster
from manriix_perception.utils.parameter_validator import ParameterValidator, ConfigValidator


class PhotoState(Enum):
    IDLE = "idle"
    EVALUATING = "evaluating"
    ACTIVE = "active"
    MULTI_SHOT = "multi_shot"
    COOLDOWN = "cooldown"


@dataclass
class PhotoIntelligence:
    """Intelligence metrics for photo timing decisions"""
    crowd_density: float = 0.0      # People per square meter
    group_size: int = 0             # Number of people in target group
    formation_stability: float = 0.0 # 0.0-1.0 how stable the formation is
    attention_score: float = 0.0    # 0.0-1.0 how engaged/posed the group is
    movement_variance: float = 0.0  # Amount of movement in group
    optimal_duration: float = 60.0  # Calculated optimal photo duration
    shot_sequence: int = 1          # Number of shots to take
    
    
@dataclass
class GroupFormationAnalysis:
    """Analysis of group formation patterns"""
    pattern_type: str = "scattered"  # single_person, pair, small_group, line, circle, large_group, scattered, formal_pose
    compactness: float = 0.0        # How tightly grouped (0.0-1.0)
    symmetry: float = 0.0           # Formation symmetry (0.0-1.0) 
    facing_alignment: float = 0.0   # How aligned people are facing (0.0-1.0)
    stability_duration: float = 0.0 # How long formation has been stable
    

class PhotoIntelligenceSystem(Node):
    """
    Advanced photo timing system with crowd intelligence.
    
    Replaces fixed 60s timing with dynamic 30s-120s based on:
    - Single person: 30-45s (quick, candid shots)
    - Small groups (2-5): 45-75s (allow posing time)
    - Large groups (6+): 75-120s (formation setup, multiple shots)
    - Wedding/formal events: +20s bonus for posing
    - High movement: -15s (capture before they move)
    - Stable formation: +10s (people are ready)
    """
    
    def __init__(self):
        super().__init__('photo_intelligence_system')
        
        # Initialize parameter validator and load validated configuration
        validator = ParameterValidator(self)
        
        # Load and validate photo intelligence configuration
        config = ConfigValidator.validate_photo_intelligence_config(validator)
        
        # Set validated parameters
        self.min_duration = config['min_duration']
        self.max_duration = config['max_duration']
        self.base_duration = config['base_duration']
        self.evaluation_time = config['evaluation_time']
        self.cooldown_time = config['cooldown_time']
        self.single_duration = config['single_person_duration']
        self.small_group_duration = config['small_group_duration']
        self.circle_bonus = config['circle_formation_bonus']
        self.enable_multi_shot = config['enable_multi_shot']
        self.multi_shot_threshold = config['multi_shot_threshold']
        
        # Additional parameters with individual validation
        self.medium_group_duration = validator.declare_and_validate(
            'medium_group_duration', 70.0, min_val=30.0, max_val=120.0,
            description="Medium group photo duration"
        )
        self.large_group_duration = validator.declare_and_validate(
            'large_group_duration', 90.0, min_val=40.0, max_val=120.0,
            description="Large group photo duration"
        )
        self.line_bonus = validator.declare_and_validate(
            'line_formation_bonus', 15.0, min_val=0.0, max_val=30.0,
            description="Line formation time bonus"
        )
        self.formal_bonus = validator.declare_and_validate(
            'formal_pose_bonus', 20.0, min_val=0.0, max_val=40.0,
            description="Formal/posed formation time bonus"
        )
        self.stable_reduction = validator.declare_and_validate(
            'stable_group_reduction', 10.0, min_val=0.0, max_val=30.0,
            description="Time reduction for stable groups"
        )
        self.shot_interval = validator.declare_and_validate(
            'shot_interval', 5.0, min_val=1.0, max_val=15.0,
            description="Interval between multi-shots"
        )
        self.max_shots = validator.declare_and_validate(
            'max_shots', 3, min_val=1, max_val=5,
            description="Maximum shots per sequence"
        )
        self.stability_threshold = validator.declare_and_validate(
            'stability_threshold', 2.0, min_val=0.5, max_val=10.0,
            description="Formation stability threshold"
        )
        
        # Additional timing adjustments with validation
        self.unstable_addition = validator.declare_and_validate(
            'unstable_group_addition', 15.0, min_val=0.0, max_val=30.0
        )
        self.high_movement_reduction = validator.declare_and_validate(
            'high_movement_reduction', 15.0, min_val=0.0, max_val=30.0
        )
        self.low_movement_addition = validator.declare_and_validate(
            'low_movement_addition', 10.0, min_val=0.0, max_val=20.0
        )
        self.high_attention_bonus = validator.declare_and_validate(
            'high_attention_bonus', 10.0, min_val=0.0, max_val=20.0
        )
        self.low_attention_reduction = validator.declare_and_validate(
            'low_attention_reduction', 10.0, min_val=0.0, max_val=20.0
        )
        self.high_density_addition = validator.declare_and_validate(
            'high_density_addition', 10.0, min_val=0.0, max_val=20.0
        )
        
        # State management
        self.current_state = PhotoState.IDLE
        self.photo_start_time = None
        self.evaluation_start_time = None
        self.current_intelligence = PhotoIntelligence()
        self.formation_history = []  # Track formation changes over time
        self.last_clusters = []
        self.stability_start_time = None
        
        # QoS profiles
        qos_reliable = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, depth=5)
        qos_best_effort = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=3)
        
        # Subscribers - input from perception system
        self.clusters_sub = self.create_subscription(
            ClusterArray,
            '/human_clustering/clusters',
            self.clusters_callback,
            qos_reliable
        )
        
        self.photo_trigger_sub = self.create_subscription(
            Bool,
            '/photo/intelligence/trigger',  # Mission controller triggers photo session
            self.photo_trigger_callback,
            qos_reliable
        )
        
        # Publishers - output commands
        self.photo_command_pub = self.create_publisher(
            String,
            '/photo/command',  # Compatible with friend's Canon R6 system
            qos_reliable
        )
        
        self.photo_status_pub = self.create_publisher(
            String,
            '/photo/intelligence/status',  # For mission controller
            qos_reliable
        )
        
        self.intelligence_metrics_pub = self.create_publisher(
            String,
            '/photo/intelligence/metrics',  # Debug info
            qos_best_effort
        )
        
        # Control timer for state machine
        self.control_timer = self.create_timer(0.5, self.control_loop)
        self.cooldown_timer = None  # One-shot timer for cooldown period

        self.get_logger().info("PhotoIntelligenceSystem initialized with CONFIGURABLE timing:")
        self.get_logger().info(f"  Timing range: {self.min_duration}s - {self.max_duration}s")
        self.get_logger().info(f"  Base duration: {self.base_duration}s")
        self.get_logger().info(f"  Group sizes: Single:{self.single_duration}s Small:{self.small_group_duration}s Med:{self.medium_group_duration}s Large:{self.large_group_duration}s")
        self.get_logger().info(f"  Multi-shot: {'Enabled' if self.enable_multi_shot else 'Disabled'} (threshold: {self.multi_shot_threshold} people)")
        self.get_logger().info(f"  Formation bonuses: Circle:+{self.circle_bonus}s Line:+{self.line_bonus}s Formal:+{self.formal_bonus}s")
        
        
        
    def clusters_callback(self, msg: ClusterArray):
        """Process cluster information for intelligence analysis"""
        try:
            clusters = []
            for cluster_msg in msg.clusters:
                cluster = {
                    'id': cluster_msg.id,
                    'center': np.array([cluster_msg.center.x, cluster_msg.center.y]),
                    'radius': cluster_msg.radius,
                    'size': cluster_msg.size,
                    'formation': cluster_msg.formation
                }
                clusters.append(cluster)
                
            # Update intelligence analysis
            self.analyze_crowd_intelligence(clusters)
            self.last_clusters = clusters
            
        except Exception as e:
            self.get_logger().error(f"Error processing clusters: {e}")
            
            
    def photo_trigger_callback(self, msg: Bool):
        """Triggered by mission controller when robot reaches optimal position"""
        try:
            if msg.data and self.current_state == PhotoState.IDLE:
                self.get_logger().info("Photo session triggered - starting evaluation")
                self.current_state = PhotoState.EVALUATING
                self.evaluation_start_time = time.time()
                
                # Publish status
                status_msg = String()
                status_msg.data = "evaluating"
                self.photo_status_pub.publish(status_msg)
                
            elif not msg.data and self.current_state in [PhotoState.ACTIVE, PhotoState.MULTI_SHOT]:
                self.get_logger().info("Photo session interrupted - stopping")
                self.stop_photo_session()
                
        except Exception as e:
            self.get_logger().error(f"Error handling photo trigger: {e}")
            
            
    def analyze_crowd_intelligence(self, clusters: List[Dict]):
        """Analyze crowd for intelligent photo timing decisions"""
        try:
            if not clusters:
                return
                
            # Find largest/most important cluster for photo
            target_cluster = max(clusters, key=lambda c: c['size'])
            
            # Basic metrics
            group_size = target_cluster['size']
            cluster_radius = target_cluster['radius']
            
            # Calculate crowd density (people per square meter)
            cluster_area = np.pi * (cluster_radius ** 2)
            crowd_density = group_size / max(cluster_area, 0.1)  # Avoid division by zero
            
            # Analyze formation
            formation_analysis = self.analyze_formation(target_cluster)
            
            # Calculate movement variance (stability)
            movement_variance = self.calculate_movement_variance(target_cluster)
            
            # Estimate attention score (how posed/ready they are)
            attention_score = self.estimate_attention_score(formation_analysis, movement_variance)
            
            # Calculate optimal duration
            optimal_duration, shot_sequence = self.calculate_optimal_timing(
                group_size, crowd_density, formation_analysis, movement_variance, attention_score
            )
            
            # Update intelligence
            self.current_intelligence = PhotoIntelligence(
                crowd_density=crowd_density,
                group_size=group_size,
                formation_stability=formation_analysis.stability_duration,
                attention_score=attention_score,
                movement_variance=movement_variance,
                optimal_duration=optimal_duration,
                shot_sequence=shot_sequence
            )
            
            # Publish metrics for debugging
            self.publish_intelligence_metrics()
            
        except Exception as e:
            self.get_logger().error(f"Error analyzing crowd intelligence: {e}")
            
            
    def analyze_formation(self, cluster: Dict) -> GroupFormationAnalysis:
        """Analyze group formation patterns"""
        try:
            formation_type = cluster.get('formation', 'scattered')
            size = cluster['size']
            radius = cluster['radius']
            
            # Calculate compactness (how tightly grouped)
            # Smaller radius for same size = more compact
            expected_radius = 0.3 * np.sqrt(size)  # Expected radius for loose group
            compactness = max(0.0, min(1.0, expected_radius / max(radius, 0.1)))
            
            # Formation-specific analysis - all types from cluster_detector
            if formation_type == 'single_person':
                symmetry = 1.0  # Single person is perfectly symmetric
                facing_alignment = 0.4  # Unknown facing direction
            elif formation_type == 'pair':
                symmetry = 0.85  # Pairs are often symmetric
                facing_alignment = 0.7  # Pairs often face similar direction
            elif formation_type == 'small_group':
                symmetry = 0.7  # Small groups have moderate symmetry
                facing_alignment = 0.6  # Moderate alignment
            elif formation_type == 'line':
                symmetry = 0.9  # Lines are very symmetric
                facing_alignment = 0.8  # People in lines typically face same direction
            elif formation_type == 'circle':
                symmetry = 0.8  # Circles are naturally symmetric
                facing_alignment = 0.6  # People in circles face inward
            elif formation_type == 'large_group':
                symmetry = 0.5  # Large groups harder to be symmetric
                facing_alignment = 0.4  # Varied facing directions
            elif formation_type == 'formal_pose':
                symmetry = 0.95  # Formal poses are intentionally symmetric
                facing_alignment = 0.9  # Everyone faces camera
            else:  # scattered
                symmetry = 0.3  # Random groupings lack symmetry
                facing_alignment = 0.2  # Random facing directions
                
            # Check stability duration
            stability_duration = 0.0
            if self.formation_history:
                # Check if formation has been stable
                current_time = time.time()
                for hist_time, hist_formation in reversed(self.formation_history):
                    if hist_formation == formation_type:
                        stability_duration = current_time - hist_time
                    else:
                        break
                        
            # Record current formation
            self.formation_history.append((time.time(), formation_type))
            # Keep only last 30 seconds of history
            cutoff_time = time.time() - 30.0
            self.formation_history = [h for h in self.formation_history if h[0] > cutoff_time]
            
            return GroupFormationAnalysis(
                pattern_type=formation_type,
                compactness=compactness,
                symmetry=symmetry,
                facing_alignment=facing_alignment,
                stability_duration=stability_duration
            )
            
        except Exception as e:
            self.get_logger().error(f"Error analyzing formation: {e}")
            return GroupFormationAnalysis()
            
            
    def calculate_movement_variance(self, cluster: Dict) -> float:
        """Calculate how much the group is moving (lower = more stable)"""
        try:
            # This is a simplified calculation - in reality you'd track
            # position changes over time. For now, use formation stability.
            formation_type = cluster.get('formation', 'scattered')

            # More organized formations tend to be more stable
            if formation_type == 'single_person':
                return 0.2  # Single person can move easily
            elif formation_type == 'pair':
                return 0.15  # Pairs tend to move together
            elif formation_type in ['circle', 'line', 'formal_pose']:
                return 0.1  # Organized formations are more stable
            elif formation_type == 'small_group':
                return 0.2  # Small groups moderate stability
            elif formation_type == 'large_group':
                return 0.3  # Large groups harder to coordinate
            else:  # scattered
                return 0.4  # Scattered groups move more

        except Exception:
            return 0.3  # Default moderate movement
            
            
    def estimate_attention_score(self, formation: GroupFormationAnalysis, movement: float) -> float:
        """Estimate how ready/posed the group is for photos"""
        try:
            # Base score from formation quality
            base_score = (formation.symmetry + formation.facing_alignment) / 2.0
            
            # Bonus for stable formations
            if formation.stability_duration > self.stability_threshold:
                base_score += 0.2
                
            # Penalty for high movement
            movement_penalty = movement * 0.5
            
            # Special cases - all formation types
            if formation.pattern_type == 'formal_pose':
                base_score += 0.25  # Formal poses are ready for photos
            elif formation.pattern_type == 'line':
                base_score += 0.15  # Lines suggest posed/formal arrangement
            elif formation.pattern_type == 'circle':
                base_score += 0.1  # Circles suggest social interaction
            elif formation.pattern_type == 'pair':
                base_score += 0.1  # Pairs often ready for photos
            elif formation.pattern_type == 'small_group':
                base_score += 0.05  # Small groups moderate readiness
                
            return max(0.0, min(1.0, base_score - movement_penalty))
            
        except Exception:
            return 0.5  # Default moderate attention
            
            
    def calculate_optimal_timing(self, group_size: int, density: float, 
                               formation: GroupFormationAnalysis, movement: float, 
                               attention: float) -> Tuple[float, int]:
        """Calculate optimal photo duration and number of shots (NOW FULLY CONFIGURABLE!)"""
        try:
            # Base duration from group size (configurable)
            if group_size == 1:
                base_duration = self.single_duration
            elif group_size <= 3:
                base_duration = self.small_group_duration
            elif group_size <= 6:
                base_duration = self.medium_group_duration
            else:
                base_duration = self.large_group_duration
                
            # Formation adjustments (configurable bonuses)
            if formation.pattern_type == 'formal_pose':
                base_duration += self.formal_bonus
            elif formation.pattern_type == 'line':
                base_duration += self.line_bonus
            elif formation.pattern_type == 'circle':
                base_duration += self.circle_bonus
                
            # Stability adjustments (configurable)
            if formation.stability_duration > self.stability_threshold:
                base_duration -= self.stable_reduction  # Already stable
            else:
                base_duration += self.unstable_addition  # Need time to stabilize
                
            # Movement adjustments (configurable)
            if movement > 0.3:
                base_duration -= self.high_movement_reduction  # High movement - capture quickly
            elif movement < 0.1:
                base_duration += self.low_movement_addition  # Very stable - can take time
                
            # Attention/quality adjustments (configurable)
            if attention > 0.7:
                base_duration += self.high_attention_bonus  # Well-posed group
            elif attention < 0.3:
                base_duration -= self.low_attention_reduction  # Candid/unposed
                
            # Density adjustments (configurable)
            if density > 1.5:  # High density
                base_duration += self.high_density_addition
                
            # Multi-shot sequence logic (configurable)
            shot_sequence = 1  # Default single shot
            if self.enable_multi_shot and group_size >= self.multi_shot_threshold:
                shot_sequence = min(self.max_shots, max(2, group_size // 4))  # Scale with group size
                
            # Clamp to configurable min/max bounds
            final_duration = max(self.min_duration, min(self.max_duration, base_duration))
            
            return final_duration, shot_sequence
            
        except Exception as e:
            self.get_logger().error(f"Error calculating optimal timing: {e}")
            return self.base_duration, 1
            
            
    def publish_intelligence_metrics(self):
        """Publish intelligence metrics for debugging"""
        try:
            intel = self.current_intelligence
            metrics = f"Group:{intel.group_size} Density:{intel.crowd_density:.1f} " \
                     f"Stability:{intel.formation_stability:.1f} Attention:{intel.attention_score:.1f} " \
                     f"Duration:{intel.optimal_duration:.1f}s Shots:{intel.shot_sequence}"
                     
            msg = String()
            msg.data = metrics
            self.intelligence_metrics_pub.publish(msg)
            
        except Exception as e:
            self.get_logger().error(f"Error publishing metrics: {e}")
            
            
    def control_loop(self):
        """Main state machine control loop"""
        try:
            current_time = time.time()
            
            if self.current_state == PhotoState.EVALUATING:
                # Evaluation phase - analyze crowd for optimal timing
                if self.evaluation_start_time and \
                   (current_time - self.evaluation_start_time) >= self.evaluation_time:
                    
                    self.get_logger().info(
                        f"Evaluation complete - Duration: {self.current_intelligence.optimal_duration:.1f}s, "
                        f"Shots: {self.current_intelligence.shot_sequence}"
                    )
                    
                    # Start photo session
                    self.start_photo_session()
                    
            elif self.current_state == PhotoState.ACTIVE:
                # Active photo session
                if self.photo_start_time and \
                   (current_time - self.photo_start_time) >= self.current_intelligence.optimal_duration:
                    
                    if self.current_intelligence.shot_sequence > 1:
                        self.get_logger().info("First shot complete - starting multi-shot sequence")
                        self.current_state = PhotoState.MULTI_SHOT
                        self.shot_count = 1
                    else:
                        self.get_logger().info("Photo session complete")
                        self.stop_photo_session()
                        
            elif self.current_state == PhotoState.MULTI_SHOT:
                # Multi-shot sequence with brief pauses
                if hasattr(self, 'shot_count'):
                    shot_interval = 5.0  # 5 second pause between shots
                    if (current_time - self.photo_start_time) >= \
                       (self.current_intelligence.optimal_duration + (self.shot_count * shot_interval)):
                        
                        self.shot_count += 1
                        if self.shot_count >= self.current_intelligence.shot_sequence:
                            self.get_logger().info(f"Multi-shot sequence complete ({self.shot_count} shots)")
                            self.stop_photo_session()
                        else:
                            self.get_logger().info(f"Shot {self.shot_count + 1} of {self.current_intelligence.shot_sequence}")
                            
        except Exception as e:
            self.get_logger().error(f"Error in control loop: {e}")
            
            
    def start_photo_session(self):
        """Start active photo session"""
        try:
            self.current_state = PhotoState.ACTIVE
            self.photo_start_time = time.time()
            
            # Send START command to Canon R6
            command_msg = String()
            command_msg.data = "START"
            self.photo_command_pub.publish(command_msg)
            
            # Update status
            status_msg = String()
            status_msg.data = "active"
            self.photo_status_pub.publish(status_msg)
            
            self.get_logger().info(f"Photo session started - Duration: {self.current_intelligence.optimal_duration:.1f}s")
            
        except Exception as e:
            self.get_logger().error(f"Error starting photo session: {e}")
            
            
    def stop_photo_session(self):
        """Stop photo session and enter cooldown"""
        try:
            # Send STOP command to Canon R6
            command_msg = String()
            command_msg.data = "STOP"
            self.photo_command_pub.publish(command_msg)
            
            # Enter cooldown state
            self.current_state = PhotoState.COOLDOWN
            self.cooldown_start_time = time.time()
            
            # Update status
            status_msg = String()
            status_msg.data = "completed"
            self.photo_status_pub.publish(status_msg)
            
            # Schedule return to idle (store timer to cancel after firing)
            self.cooldown_timer = self.create_timer(self.cooldown_time, self._cooldown_timer_callback)

            self.get_logger().info("Photo session stopped - entering cooldown")
            
        except Exception as e:
            self.get_logger().error(f"Error stopping photo session: {e}")
            
            
    def _cooldown_timer_callback(self):
        """One-shot timer callback for cooldown - cancels itself after firing"""
        self.return_to_idle()
        # Cancel timer so it only fires once
        if hasattr(self, 'cooldown_timer') and self.cooldown_timer:
            self.cooldown_timer.cancel()
            self.cooldown_timer = None

    def return_to_idle(self):
        """Return to idle state after cooldown"""
        self.current_state = PhotoState.IDLE

        status_msg = String()
        status_msg.data = "idle"
        self.photo_status_pub.publish(status_msg)

        self.get_logger().info("Photo system returned to idle - ready for next session")


def main(args=None):
    rclpy.init(args=args)
    node = PhotoIntelligenceSystem()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()