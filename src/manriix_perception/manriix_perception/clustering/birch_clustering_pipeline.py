#!/usr/bin/env python3
import numpy as np
from sklearn.cluster import Birch
from scipy.optimize import linear_sum_assignment
from typing import List, Dict, Tuple, Optional
from rclpy.node import Node
from collections import deque, defaultdict
from dataclasses import dataclass
import time
import threading
from concurrent.futures import ThreadPoolExecutor

# Try to import CuPy for GPU acceleration
try:
    import cupy as cp
    CUPY_AVAILABLE = True
except ImportError:
    CUPY_AVAILABLE = False

# Import Kalman filter
try:
    from filterpy.kalman import KalmanFilter
    from filterpy.common import Q_discrete_white_noise
    FILTERPY_AVAILABLE = True
except ImportError:
    FILTERPY_AVAILABLE = False


@dataclass
class ClusterState:
    """Data class for cluster state information"""
    id: int
    center: np.ndarray
    radius: float
    size: int
    kalman_filter: Optional[object] = None
    age: int = 0
    last_seen: float = 0.0
    stability: float = 0.0
    velocity: np.ndarray = None
    formation: str = "unknown"


class BIRCHClusteringPipeline:
    """
    High-performance BIRCH clustering pipeline optimized for Jetson Orin.
    Handles 50+ people in real-time using GPU acceleration.
    """
    
    def __init__(self, config: Dict, node: Node = None):
        """Initialize BIRCH clustering with GPU optimization"""
        self.config = config
        self.node = node
        
        if self.node:
            self.node.get_logger().info("Initializing BIRCH Clustering Pipeline for Jetson AGX Orin")
        
        # Hardware specs from analysis
        self.cuda_cores = 2048
        self.tensor_cores = 64
        self.compute_capability = 8.7
        self.max_threads_per_block = 1024
        self.warp_size = 32
        self.memory_bandwidth = 204.8  # GB/s
        
        # Camera parameters
        camera_config = config.get('camera', {})
        fov_config = camera_config.get('fov', {})
        working_distance = camera_config.get('working_distance', {})
        
        self.fov_horizontal = np.radians(fov_config.get('horizontal', 69.0))
        self.min_distance = working_distance.get('min', 1.0)
        self.max_distance = working_distance.get('max', 10.0)
        
        # BIRCH parameters optimized for human clustering
        birch_config = config.get('birch', {})
        self.birch_params = {
            'n_clusters': None,  # Dynamic cluster count
            'threshold': birch_config.get('threshold', 1.5),
            'branching_factor': birch_config.get('branching_factor', 50),
            'compute_labels': True,
            'copy': False
        }
        
        # Initialize BIRCH model
        self.birch = Birch(**self.birch_params)
        self.is_fitted = False
        
        # Cluster tracking
        self.cluster_states = {}  # id -> ClusterState
        self.next_cluster_id = 0
        self.max_cluster_age = 10  # frames
        
        # Performance optimization
        self.use_gpu = self._check_gpu_availability()
        self.memory_pool = None
        self.cuda_streams = []
        self.thread_pool = ThreadPoolExecutor(max_workers=4)
        
        # Initialize GPU resources
        if self.use_gpu:
            self._initialize_gpu_resources()
        
        # Adaptive parameters
        self.adaptive_threshold = self.birch_params['threshold']
        self.crowd_density_threshold = 10  # people per 10m²
        
        # Performance monitoring
        self.frame_times = deque(maxlen=30)
        self.last_process_time = time.time()
        self.fps = 0
        
        # Hungarian algorithm parameters
        self.max_match_distance = 3.0  # meters
        self.iou_threshold = 0.3
        
        if self.node:
            self.node.get_logger().info(f"BIRCH Pipeline initialized (GPU: {self.use_gpu})")
    
    def _check_gpu_availability(self) -> bool:
        """Check if GPU acceleration is available"""
        if not CUPY_AVAILABLE:
            if self.node:
                self.node.get_logger().warn("CuPy not available - GPU acceleration disabled")
            return False
            
        try:
            # Test GPU allocation
            test_array = cp.zeros((100, 100))
            del test_array
            if self.node:
                self.node.get_logger().info("GPU acceleration available via CuPy")
            return True
        except Exception as e:
            if self.node:
                self.node.get_logger().warn(f"GPU acceleration not available: {e}")
            return False
    
    def _initialize_gpu_resources(self):
        """Initialize CUDA resources for GPU acceleration"""
        try:
            # Create memory pool for efficient allocation
            mempool = cp.get_default_memory_pool()
            mempool.set_limit(size=2 * 1024**3)  # 2GB limit # TODO: fix after testing

            self.memory_pool = mempool
            
            # Create CUDA streams for parallel operations
            self.cuda_streams = [cp.cuda.Stream() for _ in range(3)]
            
            # Pre-allocate GPU arrays for common operations
            self.gpu_distance_matrix = cp.zeros((100, 100), dtype=cp.float32)
            self.gpu_points_buffer = cp.zeros((100, 2), dtype=cp.float32)
            
            if self.node:
                self.node.get_logger().info("GPU resources initialized successfully")
            
        except Exception as e:
            if self.node:
                self.node.get_logger().error(f"Failed to initialize GPU resources: {e}")
            self.use_gpu = False
    
    def compute_distance_matrix_gpu(self, points: np.ndarray) -> np.ndarray:
        """
        Compute pairwise distance matrix using GPU acceleration.
        Optimized for Jetson Orin's Ampere architecture.
        """
        if not self.use_gpu or len(points) < 10:
            # Fall back to CPU for small arrays
            return self.compute_distance_matrix_cpu(points)
        
        try:
            with self.cuda_streams[0]:
                # Transfer to GPU using unified memory
                points_gpu = cp.asarray(points, dtype=cp.float32)
                
                # Efficient distance computation using broadcasting
                diff = points_gpu[:, np.newaxis, :] - points_gpu[np.newaxis, :, :]
                distances_gpu = cp.sqrt(cp.sum(diff ** 2, axis=2))
                
                # Transfer back to CPU
                distances = cp.asnumpy(distances_gpu)
                
            return distances
            
        except Exception as e:
            if self.node:
                self.node.get_logger().warn(f"GPU distance computation failed, falling back to CPU: {e}")
            return self.compute_distance_matrix_cpu(points)
    
    def compute_distance_matrix_cpu(self, points: np.ndarray) -> np.ndarray:
        """CPU fallback for distance matrix computation"""
        n = len(points)
        distances = np.zeros((n, n))
        
        for i in range(n):
            for j in range(i+1, n):
                dist = np.linalg.norm(points[i] - points[j])
                distances[i, j] = distances[j, i] = dist
        
        return distances
    
    def adapt_parameters(self, positions: np.ndarray):
        """Dynamically adapt BIRCH parameters based on crowd density"""
        if len(positions) < 3:
            return
        
        # Calculate crowd density
        area = self._calculate_convex_hull_area(positions)
        if area > 0:
            density = len(positions) / area
            
            # Adjust threshold based on density
            if density > self.crowd_density_threshold:
                # Tighter clustering in dense crowds
                self.adaptive_threshold = self.birch_params['threshold'] * 0.7
            elif density < self.crowd_density_threshold / 2:
                # Looser clustering in sparse crowds
                self.adaptive_threshold = self.birch_params['threshold'] * 1.3
            else:
                self.adaptive_threshold = self.birch_params['threshold']
            
            # Update BIRCH threshold
            self.birch.set_params(threshold=self.adaptive_threshold)
    
    def _calculate_convex_hull_area(self, points: np.ndarray) -> float:
        """Calculate area of convex hull for density estimation"""
        if len(points) < 3:
            return 0.0
        
        try:
            from scipy.spatial import ConvexHull
            hull = ConvexHull(points)
            return hull.volume  # In 2D, volume is actually area
        except:
            return 0.0
    
    def create_kalman_filter(self, initial_position: np.ndarray):
        """Create Kalman filter for cluster tracking"""
        if not FILTERPY_AVAILABLE:
            return None
            
        kf = KalmanFilter(dim_x=4, dim_z=2)
        
        # State: [x, y, vx, vy]
        kf.x = np.array([initial_position[0], initial_position[1], 0., 0.])
        
        # State transition matrix (constant velocity model)
        dt = 0.033  # 30 FPS
        kf.F = np.array([
            [1, 0, dt, 0],
            [0, 1, 0, dt],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])
        
        # Measurement function
        kf.H = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0]
        ])
        
        # Measurement noise
        kf.R = np.eye(2) * 0.1
        
        # Process noise
        kf.Q = Q_discrete_white_noise(dim=2, dt=dt, var=0.1, block_size=2)
        
        # Initial covariance
        kf.P *= 10
        
        return kf
    
    def incremental_clustering(self, positions: np.ndarray) -> np.ndarray:
        """
        Perform incremental BIRCH clustering.
        Returns cluster labels for each position.
        """
        if len(positions) == 0:
            return np.array([])
        
        # Adapt parameters based on crowd density
        self.adapt_parameters(positions)
        
        # Incremental fit
        if not self.is_fitted:
            # First fit
            self.birch.fit(positions)
            self.is_fitted = True
        else:
            # Incremental update
            self.birch.partial_fit(positions)
        
        # Get cluster labels
        labels = self.birch.predict(positions)
        
        return labels
    
    def match_clusters(self, current_clusters: List[Dict], 
                      previous_states: Dict[int, ClusterState]) -> Dict[int, int]:
        """
        Match current clusters to previous clusters using Hungarian algorithm.
        Optimized for real-time performance on Jetson Orin.
        """
        if not previous_states or not current_clusters:
            return {}
        
        n_current = len(current_clusters)
        n_previous = len(previous_states)
        
        # Build cost matrix
        cost_matrix = np.zeros((n_current, n_previous))
        prev_ids = list(previous_states.keys())
        
        for i, curr_cluster in enumerate(current_clusters):
            curr_center = curr_cluster['center']
            
            for j, prev_id in enumerate(prev_ids):
                prev_state = previous_states[prev_id]
                
                # Get predicted position from Kalman filter
                if prev_state.kalman_filter:
                    predicted_pos = prev_state.kalman_filter.x[:2]
                else:
                    predicted_pos = prev_state.center
                
                # Calculate distance cost
                dist = np.linalg.norm(curr_center - predicted_pos)
                
                # Combined cost
                cost = dist
                
                # Penalize if distance exceeds threshold
                if dist > self.max_match_distance:
                    cost += 1000
                
                cost_matrix[i, j] = cost
        
        # Solve assignment problem
        row_indices, col_indices = linear_sum_assignment(cost_matrix)
        
        # Build mapping
        matches = {}
        for row, col in zip(row_indices, col_indices):
            if cost_matrix[row, col] < 1000:  # Valid match
                matches[row] = prev_ids[col]
        
        return matches
    
    def update_kalman_filters(self, clusters: List[Dict], matches: Dict[int, int]):
        """Update Kalman filters for tracked clusters"""
        current_time = time.time()
        updated_ids = set()
        
        for idx, cluster in enumerate(clusters):
            if idx in matches:
                # Update existing cluster
                cluster_id = matches[idx]
                cluster['id'] = cluster_id
                
                if cluster_id in self.cluster_states:
                    state = self.cluster_states[cluster_id]
                    if state.kalman_filter:
                        state.kalman_filter.update(cluster['center'])
                    state.center = cluster['center']
                    state.size = cluster['size']
                    state.age += 1
                    state.last_seen = current_time
                    state.stability = min(1.0, state.age / 10.0)
                    if state.kalman_filter:
                        state.velocity = state.kalman_filter.x[2:4]
                else:
                    # Create new state for matched cluster
                    kf = self.create_kalman_filter(cluster['center'])
                    self.cluster_states[cluster_id] = ClusterState(
                        id=cluster_id,
                        center=cluster['center'],
                        radius=cluster.get('radius', 0.5),
                        size=cluster['size'],
                        kalman_filter=kf,
                        last_seen=current_time
                    )
                
                updated_ids.add(cluster_id)
            else:
                # New cluster
                cluster_id = self.next_cluster_id
                self.next_cluster_id += 1
                cluster['id'] = cluster_id
                
                kf = self.create_kalman_filter(cluster['center'])
                self.cluster_states[cluster_id] = ClusterState(
                    id=cluster_id,
                    center=cluster['center'],
                    radius=cluster.get('radius', 0.5),
                    size=cluster['size'],
                    kalman_filter=kf,
                    last_seen=current_time
                )
                
                updated_ids.add(cluster_id)
        
        # Predict next state for all filters
        for state in self.cluster_states.values():
            if state.kalman_filter:
                state.kalman_filter.predict()
        
        # Remove stale clusters
        stale_ids = []
        for cluster_id, state in self.cluster_states.items():
            if cluster_id not in updated_ids:
                if current_time - state.last_seen > self.max_cluster_age / 30.0:
                    stale_ids.append(cluster_id)
        
        for cluster_id in stale_ids:
            del self.cluster_states[cluster_id]
    
    def analyze_formation(self, points: np.ndarray) -> str:
        """Analyze cluster formation using eigenvalue analysis"""
        if len(points) < 3:
            return "small_group"
        
        # Center points
        centered = points - np.mean(points, axis=0)
        
        # Covariance matrix
        cov = np.cov(centered.T)
        eigenvalues, _ = np.linalg.eig(cov)
        
        # Ratio of eigenvalues indicates shape
        if eigenvalues[1] > 1e-10:
            ratio = eigenvalues[0] / eigenvalues[1]
        else:
            ratio = 10.0
        
        if ratio > 3.0:
            return "line"
        elif ratio < 1.5:
            return "circle"
        else:
            return "scattered"
    
    def calculate_required_distance(self, radius: float) -> float:
        """Calculate optimal camera distance for cluster"""
        distance = radius / np.tan(self.fov_horizontal / 2)
        distance *= 1.3  # Add 30% margin
        return np.clip(distance, self.min_distance, self.max_distance)
    
    def process_positions(self, positions: Dict[int, Dict]) -> List[Dict]:
        """
        Main processing pipeline for human positions.
        Returns list of tracked clusters with predictions.
        """
        start_time = time.time()
        
        if not positions:
            return self._get_predicted_clusters()
        
        # Extract position array
        points = np.array([
            [pos['map_position'][0], pos['map_position'][1]] 
            for pos in positions.values()
        ])
        person_ids = list(positions.keys())
        
        # Perform incremental BIRCH clustering
        labels = self.incremental_clustering(points)
        
        # Build clusters from labels
        current_clusters = []
        unique_labels = np.unique(labels)
        
        for label in unique_labels:
            if label == -1:  # Skip noise
                continue
            
            mask = labels == label
            cluster_points = points[mask]
            cluster_person_ids = [person_ids[i] for i in range(len(person_ids)) if mask[i]]
            
            # Calculate cluster properties
            center = np.mean(cluster_points, axis=0)
            radius = self._calculate_adaptive_radius(cluster_points)
            
            cluster = {
                'center': center,
                'radius': radius,
                'points': cluster_points,
                'person_ids': cluster_person_ids,
                'size': len(cluster_points),
                'formation': self.analyze_formation(cluster_points),
                'density': len(cluster_points) / (np.pi * radius**2) if radius > 0 else 0,
                'required_distance': self.calculate_required_distance(radius)
            }
            
            current_clusters.append(cluster)
        
        # Match with previous clusters
        matches = self.match_clusters(current_clusters, self.cluster_states)
        
        # Update Kalman filters
        self.update_kalman_filters(current_clusters, matches)
        
        # Add tracking information to clusters
        for cluster in current_clusters:
            cluster_id = cluster.get('id')
            if cluster_id and cluster_id in self.cluster_states:
                state = self.cluster_states[cluster_id]
                cluster['stability'] = state.stability
                cluster['velocity'] = state.velocity
                cluster['predicted_center'] = state.kalman_filter.x[:2] if state.kalman_filter else cluster['center']
                cluster['confidence'] = min(1.0, state.stability * 0.7 + 0.3)
        
        # Update performance metrics
        process_time = time.time() - start_time
        self.frame_times.append(process_time)
        if len(self.frame_times) == 30:
            self.fps = 1.0 / np.mean(self.frame_times)
        
        if self.node:
            self.node.get_logger().debug(f"BIRCH clustering completed in {process_time*1000:.2f}ms, FPS: {self.fps:.1f}")
        
        return current_clusters
    
    def _calculate_adaptive_radius(self, points: np.ndarray) -> float:
        """Calculate adaptive radius based on cluster size"""
        if len(points) == 0:
            return 0.0
        if len(points) == 1:
            return 0.5  # Personal space
        
        center = np.mean(points, axis=0)
        distances = np.linalg.norm(points - center, axis=1)
        
        # Use 90th percentile for robustness
        radius = np.percentile(distances, 90)
        
        # Add margin based on cluster size
        margin = 0.2 * np.log2(len(points) + 1)
        return radius * (1 + margin)
    
    def _get_predicted_clusters(self) -> List[Dict]:
        """Get predicted cluster positions when no new data available"""
        predicted_clusters = []
        
        for cluster_id, state in self.cluster_states.items():
            if state.kalman_filter:
                predicted_cluster = {
                    'id': cluster_id,
                    'center': state.kalman_filter.x[:2],
                    'predicted_center': state.kalman_filter.x[:2],
                    'velocity': state.velocity,
                    'size': state.size,
                    'formation': state.formation,
                    'stability': state.stability * 0.8,  # Reduce confidence
                    'confidence': 0.5,
                    'is_predicted': True
                }
                predicted_clusters.append(predicted_cluster)
        
        return predicted_clusters
    
    def cleanup(self):
        """Clean up GPU resources"""
        if self.use_gpu and self.memory_pool:
            self.memory_pool.free_all_blocks()
        
        if self.thread_pool:
            self.thread_pool.shutdown()
        
        if self.node:
            self.node.get_logger().info("BIRCH clustering pipeline cleaned up")
