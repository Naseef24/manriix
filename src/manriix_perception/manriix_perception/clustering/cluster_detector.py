#!/usr/bin/env python3
"""
Optimized Cluster Detector with JIT and GPU acceleration
Detects clusters of humans using BIRCH clustering with performance optimizations
"""

import numpy as np
from typing import Dict, List, Optional, Tuple
from sklearn.cluster import Birch
import rclpy
from rclpy.node import Node
import time

# Import optimization components
try:
    from manriix_perception.clustering.optimization import (
        OptimizedOperations,
        QualityController,
        QualityLevel
    )
    OPTIMIZATION_AVAILABLE = True
except ImportError:
    OPTIMIZATION_AVAILABLE = False

try:
    import cupy as cp
    CUPY_AVAILABLE = True
except ImportError:
    CUPY_AVAILABLE = False


class ClusterDetector:
    """Detect clusters of humans using BIRCH clustering with optimizations"""

    def __init__(self, config: Dict, node: Node = None):
        """Initialize cluster detector with optimization support"""
        self.config = config
        self.node = node

        # BIRCH clustering parameters
        birch_config = config.get('birch', {})
        self.threshold = birch_config.get('threshold', 1.5)
        self.branching_factor = birch_config.get('branching_factor', 50)
        self.n_clusters = birch_config.get('n_clusters', None)
        self.adaptive_threshold = birch_config.get('adaptive_threshold', True)
        self.min_threshold = birch_config.get('min_threshold', 1.0)
        self.max_threshold = birch_config.get('max_threshold', 2.0)

        # GPU and JIT configuration
        gpu_config = config.get('gpu', {})
        jit_config = config.get('jit', {})
        self.use_gpu = gpu_config.get('enabled', False) and CUPY_AVAILABLE
        self.use_jit = jit_config.get('enabled', False)

        # Initialize BIRCH clusterer
        self.clusterer = Birch(
            threshold=self.threshold,
            branching_factor=self.branching_factor,
            n_clusters=self.n_clusters
        )

        # Initialize optimization layer
        self.optimized_ops = None
        if OPTIMIZATION_AVAILABLE and (self.use_jit or self.use_gpu):
            try:
                self.optimized_ops = OptimizedOperations(
                    use_jit=self.use_jit,
                    use_gpu=self.use_gpu,
                    node=node
                )
                if self.node:
                    self.node.get_logger().info(
                        f"ClusterDetector: Optimization layer enabled "
                        f"(JIT: {self.use_jit}, GPU: {self.use_gpu})"
                    )
            except Exception as e:
                if self.node:
                    self.node.get_logger().warn(f"Failed to initialize optimizations: {e}")
                self.optimized_ops = None

        # Performance tracking
        self.processing_times = []
        self.last_cluster_count = 0

        if self.node:
            self.node.get_logger().info(
                f"ClusterDetector initialized (threshold: {self.threshold}, "
                f"adaptive: {self.adaptive_threshold})"
            )

    def detect_clusters(self, human_positions: Dict) -> List[Dict]:
        """
        Detect clusters from human positions with NaN validation and optimization

        Args:
            human_positions: Dict of {id: {'map_position': (x, y)}}

        Returns:
            List of cluster dictionaries
        """
        start_time = time.perf_counter()

        if not human_positions or len(human_positions) < 1:
            return []

        try:
            # Extract and validate positions
            positions, ids = self._extract_and_validate_positions(human_positions)

            if len(positions) < 1:
                return []

            # Single person case - no clustering needed
            if len(positions) == 1:
                return [self._create_single_person_cluster(positions[0], ids[0])]

            # Adapt threshold based on spatial distribution
            if self.adaptive_threshold:
                self._adapt_threshold(positions)

            # Perform clustering with optimization
            labels = self._cluster_positions(positions)

            # Process clusters
            clusters = self._process_clusters(positions, ids, labels)

            # Track performance
            elapsed = (time.perf_counter() - start_time) * 1000  # ms
            self.processing_times.append(elapsed)
            if len(self.processing_times) > 100:
                self.processing_times = self.processing_times[-100:]

            self.last_cluster_count = len(clusters)

            return clusters

        except Exception as e:
            if self.node:
                self.node.get_logger().error(f"Error detecting clusters: {e}")
            return []

    def _extract_and_validate_positions(self, human_positions: Dict) -> Tuple[np.ndarray, List]:
        """
        Extract positions and validate for NaN/Inf values

        Args:
            human_positions: Dict of human positions

        Returns:
            Tuple of (valid_positions_array, valid_ids_list)
        """
        positions = []
        ids = []
        invalid_count = 0

        for human_id, data in human_positions.items():
            if 'map_position' not in data:
                continue

            pos = data['map_position']

            # Validate position values
            try:
                x = float(pos[0])
                y = float(pos[1])

                # Check for NaN or Inf
                if np.isnan(x) or np.isnan(y) or np.isinf(x) or np.isinf(y):
                    invalid_count += 1
                    continue

                # Check for unreasonable values (>1km from origin)
                if abs(x) > 1000 or abs(y) > 1000:
                    invalid_count += 1
                    continue

                positions.append([x, y])
                ids.append(human_id)

            except (TypeError, ValueError, IndexError):
                invalid_count += 1
                continue

        if invalid_count > 0 and self.node:
            self.node.get_logger().debug(f"Filtered {invalid_count} invalid positions")

        if not positions:
            return np.array([]), []

        return np.array(positions, dtype=np.float32), ids

    def _adapt_threshold(self, positions: np.ndarray):
        """
        Adapt BIRCH threshold based on spatial distribution

        Args:
            positions: Nx2 array of positions
        """
        if len(positions) < 2:
            return

        # Calculate spatial spread
        if self.optimized_ops:
            # Use optimized pairwise distances
            center = np.mean(positions, axis=0)
            positions_3d = np.column_stack([positions, np.zeros(len(positions))])
            center_3d = np.array([center[0], center[1], 0], dtype=np.float32)
            distances = self.optimized_ops.compute_distances(positions_3d, center_3d)
        else:
            # Fallback to numpy
            center = np.mean(positions, axis=0)
            distances = np.linalg.norm(positions - center, axis=1)

        avg_spread = np.mean(distances)

        # Adjust threshold: tighter for dense crowds, looser for sparse
        if avg_spread < 1.0:
            # Dense crowd - use smaller threshold
            new_threshold = max(self.min_threshold, self.threshold * 0.8)
        elif avg_spread > 3.0:
            # Sparse distribution - use larger threshold
            new_threshold = min(self.max_threshold, self.threshold * 1.2)
        else:
            new_threshold = self.threshold

        # Update clusterer if threshold changed significantly
        if abs(new_threshold - self.clusterer.threshold) > 0.1:
            self.clusterer = Birch(
                threshold=new_threshold,
                branching_factor=self.branching_factor,
                n_clusters=self.n_clusters
            )

    def _cluster_positions(self, positions: np.ndarray) -> np.ndarray:
        """
        Perform clustering on positions

        Args:
            positions: Nx2 array of positions

        Returns:
            Array of cluster labels
        """
        # Use sklearn BIRCH (already optimized internally)
        # GPU acceleration is applied to distance calculations in _adapt_threshold
        labels = self.clusterer.fit_predict(positions)
        return labels

    def _process_clusters(self, positions: np.ndarray, ids: List,
                         labels: np.ndarray) -> List[Dict]:
        """
        Process clustering results into cluster dictionaries

        Args:
            positions: Nx2 array of positions
            ids: List of human IDs
            labels: Array of cluster labels

        Returns:
            List of cluster dictionaries
        """
        clusters = []
        unique_labels = np.unique(labels)

        for label in unique_labels:
            if label == -1:  # Noise points - create individual clusters
                noise_mask = labels == -1
                noise_positions = positions[noise_mask]
                noise_ids = [ids[i] for i, m in enumerate(noise_mask) if m]

                # Create individual clusters for noise points
                for i, (pos, pid) in enumerate(zip(noise_positions, noise_ids)):
                    clusters.append(self._create_single_person_cluster(pos, pid))
                continue

            mask = labels == label
            cluster_positions = positions[mask]
            cluster_ids = [ids[i] for i, m in enumerate(mask) if m]

            if len(cluster_ids) == 0:
                continue

            # Calculate cluster properties using optimizations if available
            cluster = self._calculate_cluster_properties(
                cluster_positions, cluster_ids
            )
            clusters.append(cluster)

        return clusters

    def _calculate_cluster_properties(self, positions: np.ndarray,
                                      ids: List) -> Dict:
        """
        Calculate cluster properties with optimization support

        Args:
            positions: Cluster member positions
            ids: Cluster member IDs

        Returns:
            Cluster dictionary
        """
        n_members = len(ids)

        # Calculate center
        center = np.mean(positions, axis=0)

        # Calculate radius using optimized operations if available
        if self.optimized_ops and n_members > 5:
            positions_3d = np.column_stack([positions, np.zeros(n_members)])
            center_3d = np.array([center[0], center[1], 0], dtype=np.float32)
            distances = self.optimized_ops.compute_distances(positions_3d, center_3d)
        else:
            distances = np.linalg.norm(positions - center, axis=1)

        radius = float(np.max(distances)) if len(distances) > 0 else 0.5
        radius = max(radius, 0.5)  # Minimum radius

        # Calculate spread (std of distances)
        spread = float(np.std(distances)) if len(distances) > 1 else 0.0

        # Determine formation
        formation = self._determine_formation(positions)

        # Calculate density
        area = np.pi * radius ** 2
        density = n_members / area if area > 0 else 1.0

        return {
            'center': center,
            'radius': radius,
            'points': positions,
            'ids': ids,
            'size': n_members,
            'required_distance': self._calculate_required_distance(n_members),
            'formation': formation,
            'density': density,
            'spread': spread
        }

    def _create_single_person_cluster(self, position: np.ndarray,
                                      person_id: int) -> Dict:
        """Create cluster for a single person"""
        pos = np.array(position[:2]) if len(position) > 2 else np.array(position)
        return {
            'center': pos,
            'radius': 0.5,
            'points': np.array([pos]),
            'ids': [person_id],
            'size': 1,
            'required_distance': 2.0,
            'formation': 'single_person',
            'density': 1.0,
            'spread': 0.0
        }

    def _determine_formation(self, positions: np.ndarray) -> str:
        """Determine cluster formation pattern"""
        n = len(positions)

        if n == 1:
            return 'single_person'
        elif n == 2:
            return 'pair'
        elif n <= 4:
            return 'small_group'
        elif n <= 8:
            # Check if roughly circular
            center = np.mean(positions, axis=0)
            distances = np.linalg.norm(positions - center, axis=1)
            std_dist = np.std(distances)
            mean_dist = np.mean(distances)

            if mean_dist > 0 and std_dist / mean_dist < 0.3:
                return 'circle'
            else:
                return 'scattered'
        else:
            return 'large_group'

    def _calculate_required_distance(self, size: int) -> float:
        """Calculate required viewing distance based on cluster size"""
        base_distance = 2.0
        size_factor = np.log(size + 1) * 0.5
        return base_distance + size_factor

    def get_performance_stats(self) -> Dict:
        """Get clustering performance statistics"""
        if not self.processing_times:
            return {'avg_ms': 0, 'max_ms': 0, 'min_ms': 0, 'count': 0}

        return {
            'avg_ms': np.mean(self.processing_times),
            'max_ms': np.max(self.processing_times),
            'min_ms': np.min(self.processing_times),
            'count': len(self.processing_times),
            'last_cluster_count': self.last_cluster_count,
            'optimization_enabled': self.optimized_ops is not None,
            'gpu_enabled': self.use_gpu,
            'jit_enabled': self.use_jit
        }

    def update_threshold(self, new_threshold: float):
        """
        Update BIRCH threshold dynamically

        Args:
            new_threshold: New threshold value
        """
        if new_threshold != self.clusterer.threshold:
            self.clusterer = Birch(
                threshold=new_threshold,
                branching_factor=self.branching_factor,
                n_clusters=self.n_clusters
            )
            self.threshold = new_threshold

            if self.node:
                self.node.get_logger().info(f"BIRCH threshold updated to {new_threshold}")
