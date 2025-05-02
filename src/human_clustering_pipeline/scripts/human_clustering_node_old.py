#!/usr/bin/env python3

import rospy
import yaml
import os
import numpy as np
from typing import Dict, List
import threading
from collections import deque

import sys
sys.path.append("/home/hype/Desktop/lasa/depth_cam/depth_ws/src/human_clustering_pipeline")

from human_detection_pipeline.msg import HumansList
from human_clustering_pipeline.msg import OptimalPosition

from src.world_transform import WorldTransform
from src.cluster_detector import ClusterDetector
from src.cluster_tracker import ClusterTracker
from src.position_optimizer import PositionOptimizer
from src.viz_2d import TopView2DVisualizer
from src.performance_utils import MemoryManager, ParallelProcessor

class HumanClusteringNode:
    """Main node with performance optimizations"""
    
    def __init__(self):
        """Initialize the ROS node and components"""
        # Initialize node
        rospy.init_node('human_clustering_node')
        rospy.loginfo("Starting Human Clustering Node...")
        
        # Load configuration
        config_path = rospy.get_param('~config_path', 'config/config.yaml')
        self.config = self.load_config(config_path)
        
        if not self.config:
            rospy.logerr("Failed to load config file")
            return
        
        # Initialize performance optimizations
        self.memory_manager = MemoryManager(max_history=100)
        self.parallel_processor = ParallelProcessor()
        
        # Register memory buffers
        self.memory_manager.register_buffer('humans')
        self.memory_manager.register_buffer('clusters')
        self.memory_manager.register_buffer('optimal_positions')
        
        # Initialize components
        try:
            rospy.loginfo("Initializing World Transform...")
            self.world_transform = WorldTransform(self.config)
            
            rospy.loginfo("Initializing Cluster Detector...")
            self.cluster_detector = ClusterDetector(self.config)
            
            rospy.loginfo("Initializing Cluster Tracker...")
            self.cluster_tracker = ClusterTracker(self.config)
            
            rospy.loginfo("Initializing Position Optimizer...")
            self.position_optimizer = PositionOptimizer(self.config)
            
            rospy.loginfo("Initializing 2D Visualizer...")
            self.visualizer = TopView2DVisualizer(
                max_range=self.config['visualization']['max_range']
            )
            
            # Initialize publisher for optimal position
            self.optimal_pos_pub = rospy.Publisher(
                'optimal_position',
                OptimalPosition,
                queue_size=10
            )
            
        except Exception as e:
            rospy.logerr(f"Failed to initialize components: {e}")
            return
        
        # Thread-safe state variables
        self.state_lock = threading.Lock()
        self.current_humans = {}
        self.current_clusters = []
        self.optimal_positions = []
        self.robot_position = None
        self.robot_orientation = None
        
        # Movement monitor variables
        self.is_moving = False
        self.target_reached = False
        self.last_position_time = None
        self.wait_time = rospy.Duration(10)  # 1 minute wait
        self.current_optimal_position = None

        #State control
        self.position_locked = False 
        self.next_position_calculated = None
        self.publish_allowed = True
        
        # Performance monitoring
        self.frame_count = 0
        self.process_every_n_frames = 2
        self.processing_times = deque(maxlen=100)
        self.last_cleanup_time = rospy.Time.now()
        
        # Initialize subscribers
        self.human_sub = rospy.Subscriber(
            'human_detection/human_position',
            HumansList,
            self.human_callback,
            queue_size=10
        )
        
        rospy.loginfo("Human clustering node initialized successfully")
    
    def check_target_reached(self):
        """Check if robot has reached target position"""
        try:
            if self.current_optimal_position is None or self.robot_position is None:
                return False

            # Calculate distance to target
            target_pos = self.current_optimal_position['position'][:2]
            current_pos = self.robot_position[:2]
            distance = np.linalg.norm(target_pos - current_pos)

            # Calculate orientation difference
            target_orient = self.current_optimal_position['orientation']
            orient_diff = abs(target_orient - self.robot_orientation)
            
            reached = distance < 0.1 and orient_diff < np.radians(5)

            if reached:
                rospy.loginfo(f"Target position reached !")
            return reached

            
        except Exception as e:
            rospy.logerr(f"Error checking target reached: {e}")
            return False
            
    # def publish_optimal_position(self):
    #     """Publish the optimal position message"""
    #     try:
    #         if not self.optimal_positions or not self.publish_allowed:
    #             return

    #         # Create message
    #         optimal_pos_msg = OptimalPosition()
    #         optimal_pos_msg.x = self.current_optimal_position['position'][0]
    #         optimal_pos_msg.y = self.current_optimal_position['position'][1]
    #         optimal_pos_msg.orientation = self.current_optimal_position['orientation']
    #         optimal_pos_msg.priority_score = self.current_optimal_position['priority_score']

    #         # Publish and lock
    #         self.optimal_pos_pub.publish(optimal_pos_msg)
    #         self.publish_allowed = False  # Prevent further publishing
    #         rospy.loginfo(f"Published optimal position: ({optimal_pos_msg.x:.2f}, {optimal_pos_msg.y:.2f})")
            
    #     except Exception as e:
    #         rospy.logerr(f"Error publishing optimal position: {e}")

    def publish_optimal_position(self):
        """Publish the optimal position message"""
        try:
            if self.current_optimal_position is None:
                return

            # Create message
            optimal_pos_msg = OptimalPosition()
            optimal_pos_msg.x = self.current_optimal_position['position'][0]
            optimal_pos_msg.y = self.current_optimal_position['position'][1]
            
            # Calculate orientation to face cluster center
            cluster_center = self.current_optimal_position['cluster_center']
            pos = self.current_optimal_position['position']
            orientation = np.arctan2(cluster_center[1] - pos[1], 
                                cluster_center[0] - pos[0])
            
            optimal_pos_msg.orientation = orientation
            optimal_pos_msg.priority_score = self.current_optimal_position['priority_score']

            # Always publish for now (debug)
            self.optimal_pos_pub.publish(optimal_pos_msg)
            rospy.loginfo(f"Published optimal position: ({optimal_pos_msg.x:.2f}, {optimal_pos_msg.y:.2f})")
            
        except Exception as e:
            rospy.logerr(f"Error publishing optimal position: {e}")

    def load_config(self, config_path: str) -> Dict:
        """Load configuration from yaml file"""
        try:
            with open(config_path, 'r') as f:
                config = yaml.safe_load(f)
                rospy.loginfo(f"Loaded configuration from {config_path}")
                return config
        except Exception as e:
            rospy.logerr(f"Error loading config: {e}")
            return {}

    def process_human_detections(self, humans_msg: List) -> Dict:
        """Process human detections in parallel chunks"""
        try:
            if not humans_msg or not humans_msg.humans:
                return {}
                
            # Process humans in parallel for large groups
            if len(humans_msg.humans) > 10:
                chunks = list(self.parallel_processor.process_chunks(
                    humans_msg.humans, 
                    chunk_size=5
                ))
                
                def process_chunk(chunk):
                    results = {}
                    for human in chunk:
                        map_pos = self.world_transform.transform_to_map(
                            [(human.position.x, human.position.y, human.position.z)],
                            self.config['transform']['camera_frame']
                        )
                        
                        if map_pos:
                            results[human.tracking_id] = {
                                'map_position': map_pos[0],
                                'robot_relative': (
                                    human.position.x,
                                    human.position.y,
                                    human.position.z
                                )
                            }
                    return results
                
                chunk_results = self.parallel_processor.map_async(
                    process_chunk,
                    chunks
                )
                
                human_positions = {}
                for result in chunk_results:
                    human_positions.update(result)
                
                return human_positions
                
            else:
                human_positions = {}
                for human in humans_msg.humans:
                    map_pos = self.world_transform.transform_to_map(
                        [(human.position.x, human.position.y, human.position.z)],
                        self.config['transform']['camera_frame']
                    )
                    
                    if map_pos:
                        human_positions[human.tracking_id] = {
                            'map_position': map_pos[0],
                            'robot_relative': (
                                human.position.x,
                                human.position.y,
                                human.position.z
                            )
                        }
                
                return human_positions
                
        except Exception as e:
            rospy.logerr(f"Error processing human detections: {e}")
            return {}

   

    def human_callback(self, msg: HumansList):
        """Handle incoming human detection messages"""
        try:
            callback_start = rospy.Time.now()
            
            # Skip frames if needed for efficiency
            self.frame_count += 1
            if self.frame_count % self.process_every_n_frames != 0:
                return
            
            # Get robot's current pose
            robot_pose = self.world_transform.get_robot_pose()
            if robot_pose is None:
                rospy.logwarn("No robot pose available")
                return
                
            with self.state_lock:
                self.robot_position, self.robot_orientation = robot_pose
                self.robot_position = self.robot_position[:2]  # Ensure 2D
            
            # Process human detections
            human_positions = self.process_human_detections(msg)
            rospy.loginfo(f"Processing {len(human_positions)} humans")
            
            with self.state_lock:
                self.current_humans = human_positions
            
            self.memory_manager.store_data('humans', human_positions)
            
            if human_positions:
                try:
                    # Handle single person case differently
                    if len(human_positions) == 1:
                        # Get the single person's position
                        person_id, person_data = next(iter(human_positions.items()))
                        person_position = person_data['map_position']
                        
                        # Create simple cluster for single person
                        clusters = [{
                            'center': np.array(person_position),
                            'radius': 0.5,
                            'points': np.array([person_position]),
                            'ids': [person_id],
                            'size': 1,
                            'required_distance': 2.0,
                            'formation': 'single_person',
                            'density': 1.0,
                            'spread': 0.0
                        }]
                    else:
                        # Multiple people - use clustering
                        clusters = self.cluster_detector.detect_clusters(human_positions)
                        if clusters:
                            clusters = self.cluster_tracker.update_clusters(clusters)
                    
                    if clusters:
                        with self.state_lock:
                            self.current_clusters = clusters
                        
                        # Calculate optimal positions but don't use them yet
                        optimal_positions = self.position_optimizer.prioritize_positions(
                            clusters,
                            self.robot_position
                        )
                        
                        if self.is_moving:
                            # Just check if reached, don't recalculate position
                            if self.check_target_reached():
                                rospy.loginfo("Target reached, starting wait timer")
                                self.is_moving = False
                                self.target_reached = True
                                self.last_position_time = rospy.Time.now()
                                
                            # Keep using current position while moving
                            if self.current_optimal_position:
                                self.publish_optimal_position()
                        
                        elif self.target_reached:
                            # During wait time, store but don't use new positions
                            time_elapsed = (rospy.Time.now() - self.last_position_time).to_sec()
                            time_remaining = self.wait_time.to_sec() - time_elapsed
                            
                            if optimal_positions:
                                # Just store the next position
                                self.next_position_calculated = optimal_positions[0]
                                rospy.loginfo(f"Stored next position while waiting ({time_remaining:.1f}s remaining)")
                            
                            # Check wait time completion
                            if time_elapsed > self.wait_time.to_sec():
                                if self.next_position_calculated:
                                    rospy.loginfo("Wait complete, moving to next position")
                                    self.current_optimal_position = self.next_position_calculated
                                    self.next_position_calculated = None
                                    self.target_reached = False
                                    self.is_moving = True
                                    self.publish_optimal_position()
                        
                        else:
                            # Initial position or new position needed
                            if optimal_positions and not self.current_optimal_position:
                                rospy.loginfo("Setting initial position")
                                self.current_optimal_position = optimal_positions[0]
                                self.is_moving = True
                                self.publish_optimal_position()

                        # Update visualization
                        vis_positions = {
                            track_id: {'x': data['map_position'][0], 'y': data['map_position'][1]}
                            for track_id, data in human_positions.items()
                        }
                        
                        # Only visualize current position, not calculated next position
                        vis_optimal = [self.current_optimal_position] if self.current_optimal_position else []
                        
                        self.visualizer.update_positions(
                            positions=vis_positions,
                            clusters=self.current_clusters,
                            optimal_positions=vis_optimal,
                            robot_position=self.robot_position,
                            robot_orientation=self.robot_orientation
                        )
                        
                except Exception as e:
                    rospy.logerr(f"Error in cluster processing: {e}")
            
            # Performance monitoring
            process_time = (rospy.Time.now() - callback_start).to_sec()
            self.processing_times.append(process_time)
            
            if (rospy.Time.now() - self.last_cleanup_time).to_sec() > 10.0:
                self.memory_manager.cleanup_old_data()
                avg_process_time = np.mean(self.processing_times)
                rospy.loginfo(f"Performance: {avg_process_time:.3f}s")
                self.last_cleanup_time = rospy.Time.now()
            
            # Debug info
            if self.config['debug']['print_positions']:
                self.print_debug_info()
                
        except Exception as e:
            rospy.logerr(f"Error in human callback: {e}")

    def print_debug_info(self):
        """Print debug information about current state"""
        try:
            rospy.loginfo("\n========== Debug Information ==========")
            
            # Print robot pose
            if self.robot_position is not None:
                rospy.loginfo(
                    f"\nRobot Pose (map frame):"
                    f"\n  Position: ({self.robot_position[0]:.2f}, "
                    f"{self.robot_position[1]:.2f})"
                    f"\n  Orientation: {np.degrees(self.robot_orientation):.1f}°"
                )
            
            # Print human positions
            rospy.loginfo(f"\nDetected Humans ({len(self.current_humans)}):")
            for track_id, data in self.current_humans.items():
                rospy.loginfo(
                    f"ID: {track_id}"
                    f"\n  Map position: ({data['map_position'][0]:.2f}, "
                    f"{data['map_position'][1]:.2f})"
                    f"\n  Robot relative: ({data['robot_relative'][0]:.2f}, "
                    f"{data['robot_relative'][1]:.2f}, "
                    f"{data['robot_relative'][2]:.2f})"
                )
            
            # Print clusters with enhanced information
            rospy.loginfo(f"\nDetected Clusters ({len(self.current_clusters)}):")
            for i, cluster in enumerate(self.current_clusters):
                rospy.loginfo(
                    f"Cluster {i+1}:"
                    f"\n  Size: {cluster['size']} humans"
                    f"\n  Center: ({cluster['center'][0]:.2f}, "
                    f"{cluster['center'][1]:.2f})"
                    f"\n  Radius: {cluster['radius']:.2f}m"
                    f"\n  Formation: {cluster.get('pattern', 'unknown')}"
                    f"\n  Stability: {cluster.get('stability', 0.0):.2f}"
                    f"\n  Confidence: {cluster.get('confidence', 0.0):.2f}"
                    f"\n  Required Distance: {cluster['required_distance']:.2f}m"
                    f"\n  Member IDs: {cluster['ids']}"
                )
            
            # Print optimal positions
            rospy.loginfo(f"\nOptimal Positions ({len(self.optimal_positions)}):")
            for i, pos in enumerate(self.optimal_positions):
                rospy.loginfo(
                    f"Position {i+1}:"
                    f"\n  Location: ({pos['position'][0]:.2f}, "
                    f"{pos['position'][1]:.2f})"
                    f"\n  Orientation: {np.degrees(pos['orientation']):.1f}°"
                    f"\n  Distance: {pos['distance']:.2f}m"
                    f"\n  Priority Score: {pos['priority_score']:.2f}"
                )
                
            rospy.loginfo("\n=====================================")
            
        except Exception as e:
            rospy.logerr(f"Error in debug printing: {e}")
            
    def run(self):
        """Main run loop"""
        try:
            rospy.loginfo("Human clustering node is running")
            self.visualizer.start_visualization()
            rospy.spin()
        except KeyboardInterrupt:
            rospy.loginfo("Shutting down...")
        except Exception as e:
            rospy.logerr(f"Error in run loop: {e}")
            
if __name__ == '__main__':
    try:
        node = HumanClusteringNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
