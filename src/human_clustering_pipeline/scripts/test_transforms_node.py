#!/usr/bin/env python3

import rospy
import yaml
import numpy as np
from typing import Dict
from human_detection_pipeline.msg import HumansList

import sys
sys.path.append("/home/hype/Desktop/lasa/depth_cam/depth_ws/src/human_clustering_pipeline")
from src.world_transform import WorldTransform
from src.viz_2d import TopView2DVisualizer



class TransformTestNode:
    """Test node for verifying coordinate transforms and visualization"""
    
    def __init__(self):
        """Initialize the test node"""
        # Initialize node
        rospy.init_node('transform_test_node')
        rospy.loginfo("Starting Transform Test Node...")
        
        # Load configuration
        config_path = rospy.get_param('~config_path', 'config/config.yaml')
        self.config = self.load_config(config_path)
        
        if not self.config:
            rospy.logerr("Failed to load config file")
            return
            
        # Initialize components
        try:
            self.world_transform = WorldTransform(self.config)
            self.visualizer = TopView2DVisualizer(
                max_range=self.config['visualization']['max_range']
            )
            
            # Initialize state variables
            self.current_humans = {}  # Store human positions
            self.robot_position = None
            self.robot_orientation = None
            
            # Subscribe to human detection
            self.human_sub = rospy.Subscriber(
                'human_detection/human_position',
                HumansList,
                self.human_callback,
                queue_size=10
            )
            
        except Exception as e:
            rospy.logerr(f"Failed to initialize components: {e}")
            return
            
        rospy.loginfo("Transform test node initialized successfully")
        
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
            
    def human_callback(self, msg: HumansList):
        """Handle incoming human detection messages"""
        try:
            # Get robot's current pose in map frame
            robot_pose = self.world_transform.get_robot_pose()
            if robot_pose:
                self.robot_position, self.robot_orientation = robot_pose
            
            # Clear previous human positions
            self.current_humans.clear()
            
            # Extract human positions
            human_points = [
                (human.position.x, human.position.y, human.position.z)
                for human in msg.humans
            ]
            
            # Transform to map frame
            map_points = self.world_transform.transform_to_map(
                human_points,
                self.config['transform']['camera_frame']
            )
            
            # Update human positions
            for i, (x, y) in enumerate(map_points):
                self.current_humans[msg.humans[i].tracking_id] = {
                    'x': x,
                    'y': y
                }
            
            # Update visualization
            if self.robot_position is not None:
                self.visualizer.update_positions(
                    positions=self.current_humans,
                    robot_position=self.robot_position[:2],  # 2D only
                    robot_orientation=self.robot_orientation
                )
            
        except Exception as e:
            rospy.logerr(f"Error in human callback: {e}")
            
    def run(self):
        """Main run loop"""
        try:
            rospy.loginfo("Transform test node is running")
            
            # Start visualization
            self.visualizer.start_visualization()
            
            rospy.spin()
            
        except KeyboardInterrupt:
            rospy.loginfo("Shutting down...")
        except Exception as e:
            rospy.logerr(f"Error in run loop: {e}")
            
if __name__ == '__main__':
    try:
        node = TransformTestNode()
        node.run()
    except rospy.ROSInterruptException:
        pass