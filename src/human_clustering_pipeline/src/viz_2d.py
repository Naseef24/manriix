#!/usr/bin/env python3

import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np
import rospy
from queue import Queue
import threading

class TopView2DVisualizer:
    def __init__(self, max_range=6.0):
        """Initialize the 2D coordinate visualizer"""
        self.max_range = max_range
        
        # Create figure with two subplots side by side
        self.fig, (self.ax_robot, self.ax_map) = plt.subplots(1, 2, figsize=(20, 10))
        self.fig.tight_layout(pad=3.0)
        
        # Store current data
        self.current_positions = {}
        self.current_clusters = []
        self.optimal_positions = []
        self.robot_position = None
        self.robot_orientation = None
        self.positions_lock = threading.Lock()
        
        # Colors
        self.colors = {
            'human': '#4ECDC4',
            'robot': '#FF6B6B',
            'cluster': '#45B7D1',
            'optimal': '#FFD93D',
            'fov': '#66CC66'
        }
        
        # Initial plot setup
        self.setup_plots()
        
        # Animation
        self.ani = None

    def setup_plots(self):
        """Set up both coordinate system plots"""
        try:
            self.ax_robot.clear()
            self.ax_map.clear()
            
            # Common style settings
            for ax, title in [(self.ax_robot, 'Robot Frame View'), 
                            (self.ax_map, 'Fixed Map Frame View')]:
                # Colors
                ax.set_facecolor('#282C34')
                
                # Title and labels
                ax.set_title(title, pad=20, color='white', fontsize=14)
                ax.set_xlabel('X (m)', color='white')
                ax.set_ylabel('Y (m)', color='white')
                
                # Set limits
                ax.set_xlim(-self.max_range, self.max_range)
                ax.set_ylim(-self.max_range, self.max_range)
                
                # Grid
                ax.grid(True, linestyle='--', alpha=0.2, color='white')
                
                # Equal aspect ratio
                ax.set_aspect('equal')
                
                # Axes lines
                ax.axhline(y=0, color='white', linestyle='-', alpha=1.0, linewidth=1)
                ax.axvline(x=0, color='white', linestyle='-', alpha=1.0, linewidth=1)
                
                # Ticks
                ax.tick_params(axis='both', colors='white')
                
                # Borders
                for spine in ax.spines.values():
                    spine.set_color('white')
                
                # Origin marker
                ax.plot(0, 0, '+', color='white', markersize=10, markeredgewidth=2)
            
            self.fig.patch.set_facecolor('#282C34')
            
        except Exception as e:
            rospy.logerr(f"Error in plot setup: {e}")

    def draw_robot(self, ax, position, orientation):
        """Draw robot triangle at given position and orientation"""
        try:
            if position is None or orientation is None:
                return

            # Ensure position is numpy array and 2D
            position = np.array(position[:2])  # Take only x, y coordinates
            
            triangle_vertices = np.array([[0, 0.2], [-0.1, -0.1], [0.1, -0.1]])
            
            # Rotate vertices
            cos_theta = np.cos(orientation)
            sin_theta = np.sin(orientation)
            rotation_matrix = np.array([[cos_theta, -sin_theta],
                                      [sin_theta, cos_theta]])
            rotated_vertices = np.dot(triangle_vertices, rotation_matrix.T)
            
            # Translate vertices
            translated_vertices = rotated_vertices + position.reshape(1, 2)
            
            # Draw robot
            robot = plt.Polygon(translated_vertices, color=self.colors['robot'])
            ax.add_artist(robot)
            
            # Draw heading indicator
            heading_length = 0.5
            heading_end = position + heading_length * np.array([np.cos(orientation), np.sin(orientation)])
            ax.plot([position[0], heading_end[0]], [position[1], heading_end[1]], 
                    '--', color=self.colors['robot'], alpha=0.5)
            
        except Exception as e:
            rospy.logerr(f"Error drawing robot: {e}")

    def draw_fov(self, ax, position, orientation, distance):
        """Draw camera FOV from given position"""
        try:
            if position is None or orientation is None:
                return

            # Ensure 2D position
            position = position[:2] if len(position) > 2 else position
                
            fov_angle = np.radians(73.7)  # From config
            
            # Calculate FOV angles
            angle_left = orientation - fov_angle/2
            angle_right = orientation + fov_angle/2
            
            # Calculate FOV points
            left_point = position + distance * np.array([
                np.cos(angle_left),
                np.sin(angle_left)
            ])
            right_point = position + distance * np.array([
                np.cos(angle_right),
                np.sin(angle_right)
            ])
            
            # Draw FOV triangle
            fov_points = np.array([position, left_point, right_point])
            fov = plt.Polygon(fov_points, color=self.colors['fov'], alpha=0.2)
            ax.add_artist(fov)

            # Draw FOV lines
            ax.plot([position[0], left_point[0]], [position[1], left_point[1]], 
                    '--', color=self.colors['fov'], alpha=0.5)
            ax.plot([position[0], right_point[0]], [position[1], right_point[1]], 
                    '--', color=self.colors['fov'], alpha=0.5)
            
        except Exception as e:
            rospy.logerr(f"Error drawing FOV: {e}")

    def update_frame(self, frame):
        """Update function for animation"""
        try:
            # Get data safely
            with self.positions_lock:
                positions = self.current_positions.copy()
                clusters = self.current_clusters.copy()
                optimal_positions = self.optimal_positions.copy()
                robot_pos = self.robot_position
                robot_orient = self.robot_orientation
            
            # Clear and setup plots
            self.setup_plots()
            
            # Draw in robot frame (left plot)
            self.draw_robot(self.ax_robot, np.array([0, 0]), 0)  # Robot always at origin
            
            # Draw in map frame (right plot)
            if robot_pos is not None and robot_orient is not None:
                self.draw_robot(self.ax_map, robot_pos, robot_orient)
                # self.draw_fov(self.ax_map, robot_pos, robot_orient, 6.0)
            
            # Draw humans in both frames
            for track_id, pos in positions.items():
                # Map frame (right)
                self.ax_map.plot(pos['x'], pos['y'], 'o', 
                               markersize=8, color=self.colors['human'])
                self.ax_map.annotate(f'ID:{track_id}', (pos['x'], pos['y']),
                                   xytext=(5, 5), textcoords='offset points',
                                   fontsize=8, color='white')
                
                # Transform to robot frame for left plot
                if robot_pos is not None and robot_orient is not None:
                    # 1. Translate: subtract robot position
                    dx = pos['x'] - robot_pos[0]
                    dy = pos['y'] - robot_pos[1]
                    
                    # 2. Rotate: apply inverse rotation matrix
                    x_robot = dx * np.cos(-robot_orient) - dy * np.sin(-robot_orient)
                    y_robot = dx * np.sin(-robot_orient) + dy * np.cos(-robot_orient)
                    
                    # Draw in robot frame (left)
                    self.ax_robot.plot(x_robot, y_robot, 'o', 
                                     markersize=8, color=self.colors['human'])
                    self.ax_robot.annotate(f'ID:{track_id}', (x_robot, y_robot),
                                         xytext=(5, 5), textcoords='offset points',
                                         fontsize=8, color='white')
            
            # Draw clusters in both frames
            if clusters and robot_pos is not None and robot_orient is not None:
                for cluster in clusters:
                    # Draw in map frame (right)
                    circle_map = plt.Circle(
                        cluster['center'],
                        cluster['radius'],
                        fill=False,
                        color=self.colors['cluster'],
                        linestyle='--',
                        alpha=0.8
                    )
                    self.ax_map.add_artist(circle_map)
                    
                    # Transform cluster to robot frame
                    # 1. Translate center
                    dx = cluster['center'][0] - robot_pos[0]
                    dy = cluster['center'][1] - robot_pos[1]
                    
                    # 2. Rotate center
                    center_x_robot = dx * np.cos(-robot_orient) - dy * np.sin(-robot_orient)
                    center_y_robot = dx * np.sin(-robot_orient) + dy * np.cos(-robot_orient)
                    
                    # Draw in robot frame (left)
                    circle_robot = plt.Circle(
                        (center_x_robot, center_y_robot),
                        cluster['radius'],
                        fill=False,
                        color=self.colors['cluster'],
                        linestyle='--',
                        alpha=0.8
                    )
                    self.ax_robot.add_artist(circle_robot)

            # Draw optimal positions in both frames
            if optimal_positions and robot_pos is not None and robot_orient is not None:
                for pos in optimal_positions:
                    # Draw in map frame (right)
                    self.ax_map.plot(
                        pos['position'][0],
                        pos['position'][1],
                        '*',
                        color=self.colors['optimal'],
                        markersize=12
                    )
                    
                    # Transform to robot frame
                    dx = pos['position'][0] - robot_pos[0]
                    dy = pos['position'][1] - robot_pos[1]
                    x_robot = dx * np.cos(-robot_orient) - dy * np.sin(-robot_orient)
                    y_robot = dx * np.sin(-robot_orient) + dy * np.cos(-robot_orient)
                    
                    # Draw in robot frame (left)
                    self.ax_robot.plot(
                        x_robot,
                        y_robot,
                        '*',
                        color=self.colors['optimal'],
                        markersize=12
                    )
                    
                    # Draw lines to cluster centers
                    if 'cluster_center' in pos:
                        # Map frame line
                        self.ax_map.plot(
                            [pos['position'][0], pos['cluster_center'][0]],
                            [pos['position'][1], pos['cluster_center'][1]],
                            '--',
                            color=self.colors['optimal'],
                            alpha=0.5
                        )
                        
                        # Transform cluster center to robot frame
                        dx_center = pos['cluster_center'][0] - robot_pos[0]
                        dy_center = pos['cluster_center'][1] - robot_pos[1]
                        x_center_robot = dx_center * np.cos(-robot_orient) - dy_center * np.sin(-robot_orient)
                        y_center_robot = dx_center * np.sin(-robot_orient) + dy_center * np.cos(-robot_orient)
                        
                        # Robot frame line
                        self.ax_robot.plot(
                            [x_robot, x_center_robot],
                            [y_robot, y_center_robot],
                            '--',
                            color=self.colors['optimal'],
                            alpha=0.5
                        )

            # Update legend for both plots
            legend_elements = [
                plt.Line2D([], [], marker='o', color=self.colors['human'], 
                          linestyle='None', label='Human'),
                plt.Line2D([], [], marker='s', color=self.colors['robot'],
                          linestyle='None', label='Robot'),
                plt.Line2D([], [], marker='o', color=self.colors['cluster'],
                          linestyle='--', fillstyle='none', label='Cluster'),
                plt.Line2D([], [], marker='*', color=self.colors['optimal'],
                          linestyle='None', label='Optimal Position')
            ]
            
            # Add legends
            for ax, title in [(self.ax_robot, 'Robot Frame'), 
                            (self.ax_map, 'Map Frame')]:
                legend = ax.legend(handles=legend_elements,
                                 title=title,
                                 loc='upper center',
                                 bbox_to_anchor=(0.5, -0.1),
                                 ncol=4)
                plt.setp(legend.get_texts(), color='white')
                plt.setp(legend.get_title(), color='white')
            
        except Exception as e:
            rospy.logerr(f"Error in updating frame: {e}")




    def update_positions(self, positions, clusters=None, optimal_positions=None,
                        robot_position=None, robot_orientation=None):
        """Thread-safe state update"""
        try:
            with self.positions_lock:
                self.current_positions = positions
                if clusters is not None:
                    self.current_clusters = clusters
                if optimal_positions is not None:
                    # rospy.loginfo(f"Updating visualization with new optimal positions: {optimal_positions}")
                    self.optimal_positions = optimal_positions
                if robot_position is not None:
                    self.robot_position = np.array(robot_position)
                if robot_orientation is not None:
                    self.robot_orientation = robot_orientation
        except Exception as e:
            rospy.logerr(f"Error in updating positions: {e}")

    def start_visualization(self):
        """Start the visualization"""
        try:
            self.ani = FuncAnimation(
                self.fig,
                self.update_frame,
                interval=100,
                blit=False,
                cache_frame_data=False
            )
            plt.show()
        except Exception as e:
            rospy.logerr(f"Error starting visualization: {e}")