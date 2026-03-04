#!/usr/bin/env python3
"""
2D Matplotlib Visualization - Configuration-based enable/disable

Original matplotlib visualization implementation preserved.
Enabled/disabled via config file for resource optimization.
"""

import yaml
import os
import numpy as np
from rclpy.node import Node
from queue import Queue
import threading

# Conditional imports based on configuration
def _load_viz_config():
    try:
        config_path = os.path.join(os.path.dirname(__file__), '../../config/system_config.yaml')
        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)
            return config.get('visualization', {}).get('enable_matplotlib', False)
    except:
        return False

_matplotlib_enabled = _load_viz_config()

if _matplotlib_enabled:
    import matplotlib
    matplotlib.use('TkAgg')
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation
else:
    # Mock matplotlib for resource savings
    class MockMatplotlib:
        def __init__(self): pass
        def __getattr__(self, name): return lambda *args, **kwargs: None
    plt = MockMatplotlib()
    FuncAnimation = MockMatplotlib


class TopView2DVisualizer:
    def __init__(self, max_range=6.0, node: Node = None):
        """Initialize the 2D coordinate visualizer"""
        self.max_range = max_range
        self.node = node
        self.enabled = _matplotlib_enabled
        
        if not self.enabled:
            if node:
                node.get_logger().info("Matplotlib visualization DISABLED via configuration")
            return
            
        if node:
            node.get_logger().info("Matplotlib visualization ENABLED")
        
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
            'robot': '#66CC66',
            'cluster': '#45B7D1',
            'optimal': '#FFD93D',
            'fov': '#66CC66',
            'background': '#282C34',  
            'free': '#3E4451',        
            'occupied': '#FF6B6B'
        }
        
        # Initial plot setup
        self.setup_plots()
        
        # Animation
        self.ani = None 

        self.map_data = None

    def setup_plots(self):
        """Set up both coordinate system plots"""
        if not self.enabled:
            return
            
        # Robot-centric view
        self.ax_robot.set_xlim(-self.max_range, self.max_range)
        self.ax_robot.set_ylim(-self.max_range, self.max_range)
        self.ax_robot.set_aspect('equal')
        self.ax_robot.set_title('Robot-Centric View (Live)', fontsize=14, color='white')
        self.ax_robot.set_xlabel('X (meters)', fontsize=12, color='white')
        self.ax_robot.set_ylabel('Y (meters)', fontsize=12, color='white')
        
        # Map-based view 
        self.ax_map.set_xlim(-10, 10)
        self.ax_map.set_ylim(-10, 10) 
        self.ax_map.set_aspect('equal')
        self.ax_map.set_title('Map View (Global)', fontsize=14, color='white')
        self.ax_map.set_xlabel('X (meters)', fontsize=12, color='white')
        self.ax_map.set_ylabel('Y (meters)', fontsize=12, color='white')
        
        # Styling
        for ax in [self.ax_robot, self.ax_map]:
            ax.set_facecolor(self.colors['background'])
            ax.grid(True, alpha=0.3, color='white')
            ax.tick_params(colors='white')
            
        self.fig.patch.set_facecolor('#1E1E1E')

    def update_positions(self, humans_dict, robot_pos=None, robot_orientation=None):
        """Update human positions and robot pose"""
        if not self.enabled:
            return
            
        with self.positions_lock:
            self.current_positions = humans_dict.copy()
            if robot_pos is not None:
                self.robot_position = robot_pos
            if robot_orientation is not None:
                self.robot_orientation = robot_orientation

    def update_clusters(self, clusters_list):
        """Update cluster data"""
        if not self.enabled:
            return
            
        with self.positions_lock:
            self.current_clusters = clusters_list.copy()

    def update_optimal_positions(self, optimal_positions_list):
        """Update optimal positions"""
        if not self.enabled:
            return
            
        with self.positions_lock:
            self.optimal_positions = optimal_positions_list.copy()

    def update_map_data(self, map_msg):
        """Update map data"""
        if not self.enabled:
            return
            
        with self.positions_lock:
            self.map_data = map_msg

    def animate_frame(self, frame):
        """Animation frame update function"""
        if not self.enabled:
            return
            
        with self.positions_lock:
            # Clear plots
            self.ax_robot.clear()
            self.ax_map.clear()
            
            # Re-setup
            self.setup_plots()
            
            # Draw robot-centric view
            self.draw_robot_view()
            
            # Draw map view
            self.draw_map_view()

    def draw_robot_view(self):
        """Draw robot-centric coordinate view"""
        if not self.enabled:
            return
            
        # Draw robot at origin
        if self.robot_position is not None:
            self.ax_robot.scatter(0, 0, c=self.colors['robot'], s=150, marker='^', 
                                label='Robot', edgecolors='white', linewidths=2)
            
            # Draw robot orientation
            if self.robot_orientation is not None:
                dx = np.cos(self.robot_orientation) * 1.0
                dy = np.sin(self.robot_orientation) * 1.0
                self.ax_robot.arrow(0, 0, dx, dy, head_width=0.2, head_length=0.2, 
                                  fc=self.colors['robot'], ec=self.colors['robot'])

        # Draw humans relative to robot
        if self.current_positions and self.robot_position is not None:
            for human_id, pos in self.current_positions.items():
                # Convert to robot-relative coordinates
                rel_x = pos[0] - self.robot_position[0]
                rel_y = pos[1] - self.robot_position[1]
                
                if abs(rel_x) <= self.max_range and abs(rel_y) <= self.max_range:
                    self.ax_robot.scatter(rel_x, rel_y, c=self.colors['human'], s=100, 
                                        alpha=0.8, label='Human' if human_id == list(self.current_positions.keys())[0] else "")

        # Draw clusters relative to robot
        if self.current_clusters and self.robot_position is not None:
            for cluster in self.current_clusters:
                rel_x = cluster.get('x', 0) - self.robot_position[0]
                rel_y = cluster.get('y', 0) - self.robot_position[1]
                
                if abs(rel_x) <= self.max_range and abs(rel_y) <= self.max_range:
                    size = max(50, cluster.get('size', 1) * 30)
                    self.ax_robot.scatter(rel_x, rel_y, c=self.colors['cluster'], s=size, 
                                        alpha=0.7, marker='o', edgecolors='white')

        self.ax_robot.legend(loc='upper right')

    def draw_map_view(self):
        """Draw global map view"""
        if not self.enabled:
            return
            
        # Draw map if available
        if self.map_data is not None:
            # Convert occupancy grid to image
            map_array = np.array(self.map_data.data).reshape(
                self.map_data.info.height, self.map_data.info.width)
            
            # Create color map: free=light, occupied=dark, unknown=medium
            colored_map = np.zeros((map_array.shape[0], map_array.shape[1], 3))
            colored_map[map_array == 0] = [0.24, 0.27, 0.32]  # Free - light gray
            colored_map[map_array == 100] = [1.0, 0.42, 0.42]  # Occupied - red
            colored_map[map_array == -1] = [0.17, 0.17, 0.17]  # Unknown - dark gray
            
            # Calculate extent
            resolution = self.map_data.info.resolution
            origin_x = self.map_data.info.origin.position.x
            origin_y = self.map_data.info.origin.position.y
            
            extent = [
                origin_x,
                origin_x + self.map_data.info.width * resolution,
                origin_y,
                origin_y + self.map_data.info.height * resolution
            ]
            
            self.ax_map.imshow(colored_map, extent=extent, origin='lower', alpha=0.6)

        # Draw robot
        if self.robot_position is not None:
            self.ax_map.scatter(self.robot_position[0], self.robot_position[1], 
                              c=self.colors['robot'], s=200, marker='^', 
                              label='Robot', edgecolors='white', linewidths=2, zorder=5)

        # Draw humans
        if self.current_positions:
            xs = [pos[0] for pos in self.current_positions.values()]
            ys = [pos[1] for pos in self.current_positions.values()]
            self.ax_map.scatter(xs, ys, c=self.colors['human'], s=100, 
                              alpha=0.8, label='Humans', zorder=4)

        # Draw clusters
        if self.current_clusters:
            for cluster in self.current_clusters:
                x, y = cluster.get('x', 0), cluster.get('y', 0)
                size = max(100, cluster.get('size', 1) * 50)
                self.ax_map.scatter(x, y, c=self.colors['cluster'], s=size, 
                                  alpha=0.7, marker='o', edgecolors='white', zorder=3)

        # Draw optimal positions
        if self.optimal_positions:
            for opt_pos in self.optimal_positions:
                x, y = opt_pos.get('x', 0), opt_pos.get('y', 0)
                self.ax_map.scatter(x, y, c=self.colors['optimal'], s=150, 
                                  marker='*', alpha=0.9, edgecolors='black', zorder=6)

        self.ax_map.legend(loc='upper right')

    def start_visualization(self):
        """Start the visualization animation"""
        if not self.enabled:
            return
            
        self.ani = FuncAnimation(self.fig, self.animate_frame, interval=100, blit=False, cache_frame_data=False)
        plt.show(block=False)

    def stop_visualization(self):
        """Stop the visualization"""
        if not self.enabled:
            return
            
        if self.ani:
            self.ani.event_source.stop()
        plt.close(self.fig)

    def save_frame(self, filename):
        """Save current frame to file"""
        if not self.enabled:
            return
            
        self.animate_frame(None)
        self.fig.savefig(filename, dpi=150, bbox_inches='tight', 
                        facecolor=self.colors['background'], edgecolor='none')