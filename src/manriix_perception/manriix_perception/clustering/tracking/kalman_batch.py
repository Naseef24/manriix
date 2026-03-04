#!/usr/bin/env python3
"""
Batch Kalman Filter for multiple cluster tracking
GPU-accelerated for real-time performance on Jetson
"""

import numpy as np
from rclpy.node import Node
from typing import Dict, List, Tuple, Optional
import time

try:
    import cupy as cp
    CUPY_AVAILABLE = True
except ImportError:
    CUPY_AVAILABLE = False


class KalmanFilter:
    """Single Kalman filter for one cluster"""
    
    def __init__(self, dim_x: int = 6, dim_z: int = 3):
        """
        Initialize Kalman filter
        
        State vector: [x, y, z, vx, vy, vz] (position and velocity)
        Measurement vector: [x, y, z] (position only)
        
        Args:
            dim_x: State dimension (6 for 3D position + velocity)
            dim_z: Measurement dimension (3 for 3D position)
        """
        self.dim_x = dim_x
        self.dim_z = dim_z
        
        # State vector
        self.x = np.zeros((dim_x, 1))  # [x, y, z, vx, vy, vz]
        
        # State covariance matrix
        self.P = np.eye(dim_x) * 1000  # Large initial uncertainty
        
        # State transition matrix (constant velocity model)
        self.F = np.eye(dim_x)
        
        # Measurement matrix (observe position only)
        self.H = np.zeros((dim_z, dim_x))
        self.H[0, 0] = 1  # x
        self.H[1, 1] = 1  # y
        self.H[2, 2] = 1  # z
        
        # Process noise covariance
        self.Q = np.eye(dim_x) * 0.1
        self.Q[3:, 3:] *= 10  # Higher noise for velocity
        
        # Measurement noise covariance
        self.R = np.eye(dim_z) * 0.5  # 0.5m measurement noise
        
        # Identity matrix for efficiency
        self.I = np.eye(dim_x)
        
        # Track age and confidence
        self.age = 0
        self.hits = 0
        self.time_since_update = 0
        self.confidence = 0.0
    
    def update_dt(self, dt: float):
        """Update state transition matrix with time step"""
        # Update F matrix for constant velocity model
        self.F[0, 3] = dt  # x += vx * dt
        self.F[1, 4] = dt  # y += vy * dt
        self.F[2, 5] = dt  # z += vz * dt
        
        # Adjust process noise based on dt
        q = 0.1  # Process noise intensity
        self.Q = np.array([
            [dt**4/4, 0, 0, dt**3/2, 0, 0],
            [0, dt**4/4, 0, 0, dt**3/2, 0],
            [0, 0, dt**4/4, 0, 0, dt**3/2],
            [dt**3/2, 0, 0, dt**2, 0, 0],
            [0, dt**3/2, 0, 0, dt**2, 0],
            [0, 0, dt**3/2, 0, 0, dt**2]
        ]) * q
    
    def predict(self, dt: float = 0.033):
        """
        Predict next state
        
        Args:
            dt: Time step (default 33ms for 30 FPS)
        """
        # Update matrices with time step
        self.update_dt(dt)
        
        # Predict state: x = F @ x
        self.x = self.F @ self.x
        
        # Predict covariance: P = F @ P @ F.T + Q
        self.P = self.F @ self.P @ self.F.T + self.Q
        
        self.age += 1
        self.time_since_update += 1
        
        # Decrease confidence without measurements
        self.confidence *= 0.95
    
    def update(self, z: np.ndarray):
        """
        Update with measurement
        
        Args:
            z: Measurement vector [x, y, z]
        """
        # Convert to column vector
        z = z.reshape((self.dim_z, 1))
        
        # Innovation: y = z - H @ x
        y = z - self.H @ self.x
        
        # Innovation covariance: S = H @ P @ H.T + R
        S = self.H @ self.P @ self.H.T + self.R
        
        # Kalman gain: K = P @ H.T @ inv(S)
        K = self.P @ self.H.T @ np.linalg.inv(S)
        
        # Update state: x = x + K @ y
        self.x = self.x + K @ y
        
        # Update covariance: P = (I - K @ H) @ P
        self.P = (self.I - K @ self.H) @ self.P
        
        self.hits += 1
        self.time_since_update = 0
        
        # Increase confidence with measurements
        self.confidence = min(1.0, self.confidence + 0.1)
    
    def get_state(self) -> np.ndarray:
        """Get current position estimate"""
        return self.x[:3].flatten()  # Return [x, y, z]
    
    def get_velocity(self) -> np.ndarray:
        """Get current velocity estimate"""
        return self.x[3:].flatten()  # Return [vx, vy, vz]
    
    def get_prediction(self, dt: float = 1.0) -> np.ndarray:
        """Get predicted position after dt seconds"""
        pos = self.x[:3].flatten()
        vel = self.x[3:].flatten()
        return pos + vel * dt


class BatchKalmanGPU:
    """
    Batch Kalman filter for tracking multiple clusters
    GPU-accelerated for real-time performance
    """
    
    def __init__(self, max_tracks: int = 100, use_gpu: bool = True, node: Node = None):
        """
        Initialize batch Kalman filter
        
        Args:
            max_tracks: Maximum number of tracks to maintain
            use_gpu: Use GPU acceleration if available
            node: ROS2 node for logging
        """
        self.max_tracks = max_tracks
        self.use_gpu = use_gpu and CUPY_AVAILABLE
        self.node = node
        
        if not CUPY_AVAILABLE and self.node:
            self.node.get_logger().warn("CuPy not available - Kalman filter will use CPU")
        
        # Track storage
        self.tracks = {}  # track_id -> KalmanFilter
        self.next_track_id = 0
        
        # Track management parameters
        self.max_age = 10  # Delete track after 10 frames without update
        self.min_hits = 3   # Minimum hits to confirm track
        self.min_confidence = 0.3  # Minimum confidence to keep track
        
        if self.use_gpu:
            self._init_gpu_batch()
        
        if self.node:
            self.node.get_logger().info(f"BatchKalmanGPU initialized (GPU: {self.use_gpu}, max_tracks: {max_tracks})")
    
    def _init_gpu_batch(self):
        """Initialize GPU batch processing structures"""
        
        try:
            # Pre-allocate GPU arrays for batch processing
            self.gpu_states = cp.zeros((self.max_tracks, 6), dtype=cp.float32)
            self.gpu_covariances = cp.zeros((self.max_tracks, 6, 6), dtype=cp.float32)
            self.gpu_measurements = cp.zeros((self.max_tracks, 3), dtype=cp.float32)
            
            # State transition matrix (same for all tracks)
            self.gpu_F = cp.eye(6, dtype=cp.float32)
            
            # Measurement matrix
            self.gpu_H = cp.zeros((3, 6), dtype=cp.float32)
            self.gpu_H[0, 0] = 1
            self.gpu_H[1, 1] = 1
            self.gpu_H[2, 2] = 1
            
            # Noise matrices
            self.gpu_Q = cp.eye(6, dtype=cp.float32) * 0.1
            self.gpu_R = cp.eye(3, dtype=cp.float32) * 0.5
            
            if self.node:
                self.node.get_logger().info("GPU batch Kalman structures initialized")
            
        except Exception as e:
            if self.node:
                self.node.get_logger().warn(f"Failed to initialize GPU batch: {e}")
            self.use_gpu = False
    
    def predict_all(self, dt: float = 0.033):
        """
        Predict all tracks
        
        Args:
            dt: Time step
        """
        if not self.tracks:
            return
        
        if self.use_gpu and len(self.tracks) > 10:
            self._predict_batch_gpu(dt)
        else:
            self._predict_batch_cpu(dt)
    
    def _predict_batch_cpu(self, dt: float):
        """CPU batch prediction"""
        
        for track_id, kalman in self.tracks.items():
            kalman.predict(dt)
    
    def _predict_batch_gpu(self, dt: float):
        """GPU batch prediction"""
        
        try:
            n_tracks = len(self.tracks)
            
            # Update F matrix with dt
            self.gpu_F[0, 3] = dt
            self.gpu_F[1, 4] = dt
            self.gpu_F[2, 5] = dt
            
            # Copy states to GPU
            track_ids = list(self.tracks.keys())
            for i, track_id in enumerate(track_ids[:self.max_tracks]):
                self.gpu_states[i] = cp.asarray(self.tracks[track_id].x.flatten())
                self.gpu_covariances[i] = cp.asarray(self.tracks[track_id].P)
            
            # Batch predict states: X = F @ X
            # Using broadcasting for efficiency
            self.gpu_states[:n_tracks] = cp.matmul(
                self.gpu_F, 
                self.gpu_states[:n_tracks].T
            ).T
            
            # Batch predict covariances: P = F @ P @ F.T + Q
            for i in range(n_tracks):
                self.gpu_covariances[i] = (
                    self.gpu_F @ self.gpu_covariances[i] @ self.gpu_F.T + 
                    self.gpu_Q
                )
            
            # Copy back to CPU
            for i, track_id in enumerate(track_ids[:n_tracks]):
                self.tracks[track_id].x = self.gpu_states[i].get().reshape((6, 1))
                self.tracks[track_id].P = self.gpu_covariances[i].get()
                self.tracks[track_id].age += 1
                self.tracks[track_id].time_since_update += 1
                self.tracks[track_id].confidence *= 0.95
            
        except Exception as e:
            if self.node:
                self.node.get_logger().warn(f"GPU batch predict failed: {e}, using CPU")
            self._predict_batch_cpu(dt)
    
    def update_tracks(self, track_assignments: Dict[int, np.ndarray]):
        """
        Update tracks with measurements
        
        Args:
            track_assignments: Dict of track_id -> measurement [x, y, z]
        """
        for track_id, measurement in track_assignments.items():
            if track_id in self.tracks:
                self.tracks[track_id].update(measurement)
    
    def create_track(self, measurement: np.ndarray) -> int:
        """
        Create new track
        
        Args:
            measurement: Initial position [x, y, z]
            
        Returns:
            New track ID
        """
        # Check if we've reached max tracks
        if len(self.tracks) >= self.max_tracks:
            # Remove oldest unconfirmed track
            self._prune_tracks()
        
        # Create new Kalman filter
        kalman = KalmanFilter()
        
        # Initialize with measurement
        kalman.x[:3] = measurement.reshape((3, 1))
        kalman.x[3:] = 0  # Zero initial velocity
        kalman.update(measurement)
        
        # Add to tracks
        track_id = self.next_track_id
        self.tracks[track_id] = kalman
        self.next_track_id += 1
        
        return track_id
    
    def get_predictions(self, dt: float = 1.0) -> Dict[int, np.ndarray]:
        """
        Get predictions for all tracks
        
        Args:
            dt: Prediction time horizon (seconds)
            
        Returns:
            Dict of track_id -> predicted position
        """
        predictions = {}
        
        for track_id, kalman in self.tracks.items():
            if kalman.confidence > self.min_confidence:
                predictions[track_id] = kalman.get_prediction(dt)
        
        return predictions
    
    def get_velocities(self) -> Dict[int, np.ndarray]:
        """Get velocities for all tracks"""
        
        velocities = {}
        
        for track_id, kalman in self.tracks.items():
            if kalman.confidence > self.min_confidence:
                velocities[track_id] = kalman.get_velocity()
        
        return velocities
    
    def _prune_tracks(self):
        """Remove old or low-confidence tracks"""
        
        tracks_to_remove = []
        
        for track_id, kalman in self.tracks.items():
            # Remove if too old without updates
            if kalman.time_since_update > self.max_age:
                tracks_to_remove.append(track_id)
            # Remove if low confidence and not enough hits
            elif kalman.confidence < self.min_confidence and kalman.hits < self.min_hits:
                tracks_to_remove.append(track_id)
        
        for track_id in tracks_to_remove:
            del self.tracks[track_id]
        
        if tracks_to_remove and self.node:
            self.node.get_logger().debug(f"Pruned {len(tracks_to_remove)} tracks")
    
    def get_track_states(self) -> Dict[int, Dict]:
        """Get state information for all tracks"""
        
        states = {}
        
        for track_id, kalman in self.tracks.items():
            states[track_id] = {
                'position': kalman.get_state(),
                'velocity': kalman.get_velocity(),
                'confidence': kalman.confidence,
                'age': kalman.age,
                'hits': kalman.hits,
                'time_since_update': kalman.time_since_update
            }
        
        return states
    
    def reset(self):
        """Reset all tracks"""
        
        self.tracks.clear()
        self.next_track_id = 0
        
        if self.use_gpu:
            self.gpu_states.fill(0)
            self.gpu_covariances.fill(0)
        
        if self.node:
            self.node.get_logger().info("Batch Kalman filter reset")
