#!/usr/bin/env python3

import rospy
import yaml
import os
import numpy as np
from typing import Dict, List
import threading
from collections import deque
from datetime import datetime
import sys
# sys.path.append("/home/hype/Desktop/lasa/depth_cam/depth_ws/src/human_clustering_pipeline")
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from human_detection_pipeline.msg import HumansList
from human_clustering_pipeline.msg import OptimalPosition
from std_msgs.msg import String
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
            ##############33
            self.start_scan_pub = rospy.Publisher(
                'start_scan',
                String,
                queue_size=10
            )
            

            ####################3
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
        # self.last_position_time = None
        self.wait_time = rospy.Duration(10)  # 1 minute wait
        self.current_optimal_position = None

        #State control
        self.position_locked = False 
        self.next_position_calculated = None
        self.publish_allowed = True

#####Editeed 
        self.is_scanning = False
        self.scan_region_radius = 0.5 
        self.scan_start_time = None
        self.scan_duration = rospy.Duration(10)  
        self.robot_stopped = False
        self.last_position = None
        self.position_stable_duration = rospy.Duration(1.0)  
        self.last_position_time = None
   ######################   Recheck up      
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
    
    def publish_start_scan(self, message):
        # Create a String message
        msg = String()
        msg.data = str(message)
        # Publish the message
        self.start_scan_pub.publish(msg)
        rospy.loginfo(f"Published message: {msg.data}")

            
    def publish_optimal_position(self):
        """Publish the optimal position message"""
        try:
            if self.current_optimal_position is None:
                rospy.logwarn("Cannot publish: current_optimal_position is None")
                return

            # Create message
            optimal_pos_msg = OptimalPosition()
            optimal_pos_msg.x = self.current_optimal_position['position'][0]
            optimal_pos_msg.y = self.current_optimal_position['position'][1]
            optimal_pos_msg.orientation = self.current_optimal_position['orientation']
            optimal_pos_msg.priority_score = self.current_optimal_position['priority_score']

            # Publish the message
            self.optimal_pos_pub.publish(optimal_pos_msg)
            rospy.loginfo(f"Published optimal position: ({optimal_pos_msg.x:.2f}, {optimal_pos_msg.y:.2f})")
            
        except Exception as e:
            rospy.logerr(f"Error publishing optimal position: {e}")


   ############## New chcek ##################3
    def check_target_reached(self):
        """Check if robot has reached target position and is fully stopped"""
        try:
            if self.current_optimal_position is None or self.robot_position is None:
                return False

            # If already scanning, check if the 2-minute timer has expired
            if self.is_scanning:
                time_elapsed = rospy.Time.now() - self.scan_start_time
                if time_elapsed > self.scan_duration:
                    rospy.loginfo("Scan time complete (2 minutes). Changing to 'no'")
                    self.is_scanning = False
                    
                    # Publish "no" to start_scan topic
                    self.publish_start_scan("no")
                    
                    return False  # Allow movement to a new position
                else:
                    remaining = self.scan_duration - time_elapsed
                    rospy.loginfo_throttle(10.0, f"Scanning in progress. {remaining.to_sec():.1f} seconds remaining.")
                    return True  # Still scanning, keep holding position

            # Calculate distance to target
            target_pos = self.current_optimal_position['position'][:2]
            current_pos = self.robot_position[:2]
            distance = np.linalg.norm(target_pos - current_pos)
            
            # Check if within region radius
            in_region = distance < self.scan_region_radius
            
            if not in_region:
                # Reset robot stopped tracking when not in region
                self.robot_stopped = False
                self.last_position = None
                self.last_position_time = None
                return False
                
            # If in region, check if robot has stopped moving
            current_time = rospy.Time.now()
            
            if self.last_position is None:
                # First time in region, initialize tracking
                self.last_position = current_pos.copy()
                self.last_position_time = current_time
                return False
                
            # Check position change to determine if robot has stopped
            position_change = np.linalg.norm(current_pos - self.last_position)
            time_stationary = current_time - self.last_position_time
            
            if position_change < 0.01:  # 1cm threshold for movement
                # Position hasn't changed significantly
                if time_stationary > self.position_stable_duration and not self.robot_stopped:
                    # Robot has been stationary long enough
                    self.robot_stopped = True
                    rospy.loginfo("Robot has fully stopped. Publishing 'yes' to start_scan topic and starting 2-minute timer.")
                    
                    # Start scanning timer and publish "yes"
                    self.is_scanning = True
                    self.scan_start_time = rospy.Time.now()
                    
                    # Publish "yes" to start_scan topic
                    self.publish_start_scan("yes")
            else:
                # Still moving, update tracking
                self.last_position = current_pos.copy()
                self.last_position_time = current_time
                self.robot_stopped = False
                
            return self.robot_stopped
            
        except Exception as e:
            rospy.logerr(f"Error checking target reached: {e}")
            return False

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

##########################EDITED two minutes method#################### 
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
                        
                        # Calculate optimal positions but don't use them yet if scanning
                        if not self.is_scanning:
                            optimal_positions = self.position_optimizer.prioritize_positions(
                                clusters,
                                self.robot_position
                            )
                            # Store the calculated positions in the class variable
                            self.optimal_positions = optimal_positions
                            
                            rospy.loginfo(f"Calculated {len(optimal_positions)} optimal positions")
                            if optimal_positions:
                                rospy.loginfo(f"Top position score: {optimal_positions[0]['priority_score']:.2f}")
                        else:
                            # Just maintain current optimal_positions while scanning
                            optimal_positions = self.optimal_positions
                        
                        if self.is_moving:
                            # Publish "no" while moving
                            self.publish_start_scan("no")
                            
                            # Check if reached and stopped
                            if self.check_target_reached():
                                rospy.loginfo("Target reached and robot stopped, scan in progress")
                                self.is_moving = False
                                self.target_reached = True
                            
                            # Keep using current position while moving
                            if self.current_optimal_position:
                                rospy.loginfo("About to publish position while moving")
                                self.publish_optimal_position()
                        
                        elif self.is_scanning:
                            # We're currently scanning, just check if the timer is complete
                            still_scanning = self.check_target_reached()
                            if not still_scanning:
                                # Scan completed, calculate next position
                                if self.next_position_calculated:
                                    rospy.loginfo("Scan complete, moving to next position")
                                    self.current_optimal_position = self.next_position_calculated
                                    self.next_position_calculated = None
                                    self.target_reached = False
                                    self.is_moving = True
                                    rospy.loginfo("About to publish next position")
                                    self.publish_optimal_position()
                        
                        elif self.target_reached:
                            # During scanning, store but don't use new positions
                            if optimal_positions:
                                # Just store the next position
                                self.next_position_calculated = optimal_positions[0]
                                rospy.loginfo(f"Stored next position while waiting for scan to complete")
                        
                        else:
                            # Initial position or new position needed
                            if optimal_positions and not self.current_optimal_position:
                                rospy.loginfo("Setting initial position")
                                self.current_optimal_position = optimal_positions[0]
                                self.is_moving = True
                                rospy.loginfo("About to publish initial position")
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
                    import traceback
                    rospy.logerr(traceback.format_exc())
            
            # Performance monitoring
            process_time = (rospy.Time.now() - callback_start).to_sec()
            self.processing_times.append(process_time)
            
            if (rospy.Time.now() - self.last_cleanup_time).to_sec() > 10.0:
                self.memory_manager.cleanup_old_data()
                avg_process_time = np.mean(self.processing_times)
                rospy.loginfo(f"Performance: {avg_process_time:.3f}s")
                self.last_cleanup_time = rospy.Time.now()
            
            # Debug info
            # if self.config['debug']['print_positions']:
            #     self.print_debug_info()
                
        except Exception as e:
            rospy.logerr(f"Error in human callback: {e}")
            import traceback
            rospy.logerr(traceback.format_exc())


###############3

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
            # rospy.loginfo(f"\nDetected Humans ({len(self.current_humans)}):")
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
            
            # Wait a moment for initial data
            rospy.sleep(2.0)
            
            # Force publish initial position if available
            if self.current_optimal_position:
                rospy.loginfo("Forcing initial position publish")
                self.publish_optimal_position()
                
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