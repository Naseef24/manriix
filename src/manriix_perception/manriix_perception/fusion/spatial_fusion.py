#!/usr/bin/env python3
"""
Spatial Fusion Module

Handles 3D spatial fusion of detections from multiple cameras.
Merges detections that represent the same physical object.
"""

import numpy as np
import time
from typing import List, Dict, Any, Optional, Tuple
from dataclasses import dataclass
from scipy.spatial.distance import cdist
from geometry_msgs.msg import Point

from manriix_perception.detection.zed_subscriber import ZedDetection
from manriix_perception.synchronizer import SynchronizedFrameSet


@dataclass
class TrackedObject:
    """Persistent tracked object across frames"""
    tracking_id: int
    position: np.ndarray  # [x, y, z]
    last_seen: float
    hit_count: int  # How many consecutive frames seen
    miss_count: int  # How many consecutive frames missed


@dataclass
class FusedDetection:
    """Result of spatial fusion from multiple camera detections"""
    tracking_id: int
    object_class: str
    object_subclass: str
    class_id: int
    confidence: float
    position: Point
    dimensions_3d: List[float]
    velocity: List[float]
    supporting_cameras: List[str]
    detection_count: int
    position_uncertainty: float
    timestamp: float


class SpatialFusion:
    """
    Performs spatial fusion of detections from multiple cameras
    
    Uses 3D position clustering to identify detections that represent
    the same physical object and fuses them into single detections.
    """
    
    def __init__(self, fusion_config: Dict[str, Any]):
        """
        Initialize spatial fusion
        
        Args:
            fusion_config: Fusion configuration dictionary
        """
        self.fusion_config = fusion_config

        # Spatial fusion parameters - aligned with zed_fusion_node config keys
        # Supports both old and new parameter names for backwards compatibility
        spatial_config = fusion_config.get('spatial_fusion',               # New: from zed_fusion_node config
                        fusion_config.get('spatial', {}))                   # Fallback: legacy name
        # INCREASED merge distance for better cross-camera fusion (was 0.8)
        self.max_distance_merge = spatial_config.get('duplicate_threshold', # New: from zed_fusion_node config
                                 spatial_config.get('max_distance_merge', 1.2))  # Fallback: legacy name
        self.position_weight_by_confidence = spatial_config.get('position_weight_by_confidence', True)

        # Confidence fusion parameters
        confidence_config = fusion_config.get('confidence', {})
        self.multi_camera_boost = confidence_config.get('multi_camera_boost', 0.1)
        self.min_cameras_for_boost = confidence_config.get('min_cameras_for_boost', 2)
        self.max_confidence = confidence_config.get('max_confidence', 1.0)

        # Cross-camera tracking (only needed when cameras overlap)
        # NOTE: ZED SDK DOES expose tracking IDs via label_id field!
        # For single-camera detections, we use ZED's native label_id directly.
        # Position-based tracking is only used for cross-camera fusion scenarios.
        self.tracked_objects: Dict[int, TrackedObject] = {}
        self.next_tracking_id = 10000  # Start high to avoid collision with ZED IDs
        self.tracking_match_distance = 1.0  # Max distance to match existing track
        self.track_timeout = 2.0  # Seconds before deleting lost track
        self.min_hits_to_confirm = 2  # Hits needed to confirm new track

        # Fusion statistics
        self.fusion_stats = {
            'total_input_detections': 0,
            'total_output_detections': 0,
            'fusion_efficiency': 0.0,
            'multi_camera_detections': 0,
            'single_camera_detections': 0,
            'average_position_uncertainty': 0.0,
            'active_tracks': 0
        }

        print(f"Spatial fusion initialized - merge distance: {self.max_distance_merge}m")
    
    def fuse_synchronized_detections(self, synchronized_set: SynchronizedFrameSet) -> List[FusedDetection]:
        """
        Fuse detections from a synchronized frame set
        
        Args:
            synchronized_set: Synchronized detections from multiple cameras
            
        Returns:
            List of fused detections
        """
        # Collect all detections from all cameras
        all_detections = []
        
        for camera_name, frame in synchronized_set.frames.items():
            for detection in frame.detections:
                # Ensure detection has the camera name set
                if detection.camera_name != camera_name:
                    detection.camera_name = camera_name
                all_detections.append(detection)
        
        if not all_detections:
            return []
        
        # Update statistics
        self.fusion_stats['total_input_detections'] += len(all_detections)
        
        # Perform spatial clustering
        detection_groups = self._cluster_detections_by_position(all_detections)
        
        # Create fused detections
        fused_detections = []
        for group in detection_groups:
            fused_detection = self._create_fused_detection(group, synchronized_set.reference_timestamp)
            if fused_detection:
                fused_detections.append(fused_detection)
        
        # Update statistics
        self.fusion_stats['total_output_detections'] += len(fused_detections)
        self._update_fusion_efficiency()
        
        # Count multi vs single camera detections
        for detection in fused_detections:
            if detection.detection_count > 1:
                self.fusion_stats['multi_camera_detections'] += 1
            else:
                self.fusion_stats['single_camera_detections'] += 1
        
        return fused_detections
    
    def _cluster_detections_by_position(self, detections: List[ZedDetection]) -> List[List[ZedDetection]]:
        """
        Cluster detections by 3D position proximity
        
        Args:
            detections: List of detections to cluster
            
        Returns:
            List of detection groups (clusters)
        """
        if len(detections) <= 1:
            return [detections] if detections else []
        
        # Extract 3D positions
        positions = np.array([
            [d.position.x, d.position.y, d.position.z] for d in detections
        ])
        
        # Calculate pairwise distances
        distances = cdist(positions, positions)
        
        # Use simple greedy clustering
        groups = []
        used_indices = set()
        
        for i, detection in enumerate(detections):
            if i in used_indices:
                continue
            
            # Start new group
            group = [detection]
            used_indices.add(i)
            
            # Find nearby detections
            for j in range(len(detections)):
                if j != i and j not in used_indices:
                    if distances[i, j] <= self.max_distance_merge:
                        # Additional checks for valid merging
                        if self._should_merge_detections(detection, detections[j]):
                            group.append(detections[j])
                            used_indices.add(j)
            
            groups.append(group)
        
        return groups
    
    def _should_merge_detections(self, detection1: ZedDetection, detection2: ZedDetection) -> bool:
        """
        Determine if two detections should be merged
        
        Args:
            detection1: First detection
            detection2: Second detection
            
        Returns:
            True if detections should be merged
        """
        # Must be the same object class
        if detection1.class_id != detection2.class_id:
            return False
        
        # Don't merge detections from the same camera
        if detection1.camera_name == detection2.camera_name:
            return False
        
        # Check confidence similarity (optional)
        confidence_diff = abs(detection1.confidence - detection2.confidence)
        if confidence_diff > 30.0:  # 30% confidence difference threshold
            return False
        
        # Check size similarity (optional)
        if (len(detection1.dimensions_3d) >= 3 and len(detection2.dimensions_3d) >= 3):
            size1 = max(detection1.dimensions_3d[:3])
            size2 = max(detection2.dimensions_3d[:3])
            
            if size1 > 0 and size2 > 0:
                size_ratio = max(size1, size2) / min(size1, size2)
                if size_ratio > 2.0:  # Size difference too large
                    return False
        
        return True
    
    def _create_fused_detection(self, detection_group: List[ZedDetection], 
                              timestamp: float) -> Optional[FusedDetection]:
        """
        Create fused detection from a group of similar detections
        
        Args:
            detection_group: Group of detections to fuse
            timestamp: Reference timestamp
            
        Returns:
            FusedDetection or None if fusion fails
        """
        if not detection_group:
            return None
        
        # Use highest confidence detection as primary
        primary_detection = max(detection_group, key=lambda d: d.confidence)

        # Calculate fused position
        fused_position, position_uncertainty = self._calculate_fused_position(detection_group)

        # Calculate fused confidence
        fused_confidence = self._calculate_fused_confidence(detection_group)

        # Calculate fused velocity
        fused_velocity = self._calculate_fused_velocity(detection_group)

        # Calculate fused dimensions
        fused_dimensions = self._calculate_fused_dimensions(detection_group)

        # Get supporting cameras
        supporting_cameras = list(set(d.camera_name for d in detection_group))

        # Update position uncertainty statistics
        self._update_position_uncertainty_stats(position_uncertainty)

        # Determine tracking ID strategy:
        # - Single camera: Use ZED's native label_id (which IS the tracking ID!)
        # - Multiple cameras: Use position-based matching (cross-camera fusion)
        if len(supporting_cameras) == 1:
            # Single camera detection - use ZED's native tracking ID directly
            # The label_id field contains unique per-object tracking IDs (52, 53, 54, etc.)
            tracking_id = primary_detection.label_id
        else:
            # Multi-camera fusion - need position-based tracking since
            # different cameras have independent ZED tracking systems
            tracking_id = self._get_or_create_tracking_id(
                fused_position, timestamp, primary_detection.object_class
            )

        return FusedDetection(
            tracking_id=tracking_id,
            object_class=primary_detection.object_class,
            object_subclass=primary_detection.object_subclass,
            class_id=primary_detection.class_id,
            confidence=fused_confidence,
            position=fused_position,
            dimensions_3d=fused_dimensions,
            velocity=fused_velocity,
            supporting_cameras=supporting_cameras,
            detection_count=len(detection_group),
            position_uncertainty=position_uncertainty,
            timestamp=timestamp
        )
    
    def _calculate_fused_position(self, detections: List[ZedDetection]) -> Tuple[Point, float]:
        """
        Calculate fused position using weighted average
        
        Args:
            detections: List of detections to fuse
            
        Returns:
            Tuple of (fused_position, position_uncertainty)
        """
        if len(detections) == 1:
            position_uncertainty = 0.1  # Default uncertainty for single detection
            return detections[0].position, position_uncertainty
        
        # Extract positions and weights
        positions = np.array([[d.position.x, d.position.y, d.position.z] for d in detections])
        
        if self.position_weight_by_confidence:
            # Weight by confidence
            confidences = np.array([d.confidence for d in detections])
            confidence_sum = np.sum(confidences)
            if confidence_sum > 0:
                weights = confidences / confidence_sum
            else:
                # Fall back to equal weights if all confidences are zero
                weights = np.ones(len(detections)) / len(detections)
        else:
            # Equal weights
            weights = np.ones(len(detections)) / len(detections)
        
        # Calculate weighted average position
        fused_pos_array = np.average(positions, axis=0, weights=weights)
        
        # Calculate position uncertainty as standard deviation
        if len(detections) > 1:
            # Weighted standard deviation
            variance = np.average((positions - fused_pos_array)**2, axis=0, weights=weights)
            position_uncertainty = float(np.mean(np.sqrt(variance)))
        else:
            position_uncertainty = 0.1
        
        # Create Point message
        fused_position = Point()
        fused_position.x = float(fused_pos_array[0])
        fused_position.y = float(fused_pos_array[1])
        fused_position.z = float(fused_pos_array[2])
        
        return fused_position, position_uncertainty
    
    def _calculate_fused_confidence(self, detections: List[ZedDetection]) -> float:
        """
        Calculate fused confidence with multi-camera boost
        
        Args:
            detections: List of detections to fuse
            
        Returns:
            Fused confidence value
        """
        if len(detections) == 1:
            return detections[0].confidence
        
        # Start with highest confidence
        base_confidence = max(d.confidence for d in detections)
        
        # Apply multi-camera boost
        num_cameras = len(set(d.camera_name for d in detections))
        
        if num_cameras >= self.min_cameras_for_boost:
            # Boost confidence based on number of cameras
            confidence_boost = self.multi_camera_boost * (num_cameras - 1)
            fused_confidence = min(self.max_confidence, base_confidence + confidence_boost)
        else:
            fused_confidence = base_confidence
        
        return fused_confidence
    
    def _calculate_fused_velocity(self, detections: List[ZedDetection]) -> List[float]:
        """
        Calculate fused velocity using average

        Args:
            detections: List of detections to fuse

        Returns:
            Fused velocity vector - always exactly 3 floats [vx, vy, vz]
        """
        default_vel = [0.0, 0.0, 0.0]

        if len(detections) == 1:
            vel = detections[0].velocity
            if vel and len(vel) >= 3:
                return [float(vel[0]), float(vel[1]), float(vel[2])]
            return default_vel

        # Filter out zero velocities
        valid_velocities = []
        for detection in detections:
            vel = detection.velocity
            if vel and len(vel) >= 3:
                velocity_magnitude = np.linalg.norm([vel[0], vel[1], vel[2]])
                if velocity_magnitude > 0.01:  # 1cm/s threshold
                    valid_velocities.append([float(vel[0]), float(vel[1]), float(vel[2])])

        if not valid_velocities:
            return default_vel

        # Calculate average velocity
        velocity_array = np.array(valid_velocities)
        avg_velocity = np.mean(velocity_array, axis=0)

        # Ensure exactly 3 Python floats are returned
        return [float(avg_velocity[0]), float(avg_velocity[1]), float(avg_velocity[2])]
    
    def _calculate_fused_dimensions(self, detections: List[ZedDetection]) -> List[float]:
        """
        Calculate fused dimensions using median

        Args:
            detections: List of detections to fuse

        Returns:
            Fused dimensions - always exactly 3 floats [width, height, depth]
        """
        default_dims = [0.5, 1.7, 0.5]  # Default human dimensions

        if len(detections) == 1:
            dims = detections[0].dimensions_3d
            if dims and len(dims) >= 3:
                return [float(dims[0]), float(dims[1]), float(dims[2])]
            return default_dims

        # Extract valid dimensions
        valid_dimensions = []
        for detection in detections:
            dims = detection.dimensions_3d
            if dims and len(dims) >= 3:
                dimensions = [float(dims[0]), float(dims[1]), float(dims[2])]
                if all(d > 0 for d in dimensions):  # All dimensions must be positive
                    valid_dimensions.append(dimensions)

        if not valid_dimensions:
            return default_dims

        # Use median dimensions (more robust than mean)
        dimensions_array = np.array(valid_dimensions)
        median_dimensions = np.median(dimensions_array, axis=0)

        # Ensure exactly 3 Python floats are returned
        return [float(median_dimensions[0]), float(median_dimensions[1]), float(median_dimensions[2])]
    
    def _update_fusion_efficiency(self):
        """Update fusion efficiency statistics"""
        total_input = self.fusion_stats['total_input_detections']
        total_output = self.fusion_stats['total_output_detections']
        
        if total_input > 0:
            self.fusion_stats['fusion_efficiency'] = 1.0 - (total_output / total_input)
        else:
            self.fusion_stats['fusion_efficiency'] = 0.0
    
    def _update_position_uncertainty_stats(self, uncertainty: float):
        """Update average position uncertainty statistics"""
        total_output = self.fusion_stats['total_output_detections']
        current_avg = self.fusion_stats['average_position_uncertainty']
        
        if total_output == 1:
            self.fusion_stats['average_position_uncertainty'] = uncertainty
        elif total_output > 1:
            new_avg = ((current_avg * (total_output - 1)) + uncertainty) / total_output
            self.fusion_stats['average_position_uncertainty'] = new_avg
    
    def get_fusion_statistics(self) -> Dict[str, Any]:
        """
        Get current fusion statistics
        
        Returns:
            Dictionary containing fusion statistics
        """
        return dict(self.fusion_stats)
    
    def reset_statistics(self):
        """Reset fusion statistics"""
        self.fusion_stats = {
            'total_input_detections': 0,
            'total_output_detections': 0,
            'fusion_efficiency': 0.0,
            'multi_camera_detections': 0,
            'single_camera_detections': 0,
            'average_position_uncertainty': 0.0,
            'active_tracks': len(self.tracked_objects)
        }
    
    def get_fusion_quality_metrics(self) -> Dict[str, Any]:
        """
        Get fusion quality metrics
        
        Returns:
            Dictionary containing quality metrics
        """
        total_output = self.fusion_stats['total_output_detections']
        
        metrics = {
            'fusion_efficiency': self.fusion_stats['fusion_efficiency'],
            'multi_camera_ratio': 0.0,
            'average_uncertainty': self.fusion_stats['average_position_uncertainty'],
            'quality_score': 0.0
        }
        
        if total_output > 0:
            multi_camera = self.fusion_stats['multi_camera_detections']
            metrics['multi_camera_ratio'] = multi_camera / total_output
            
            # Calculate overall quality score (0-1)
            efficiency_score = min(1.0, self.fusion_stats['fusion_efficiency'])
            multi_camera_score = metrics['multi_camera_ratio']
            uncertainty_score = max(0.0, 1.0 - self.fusion_stats['average_position_uncertainty'])
            
            metrics['quality_score'] = (efficiency_score + multi_camera_score + uncertainty_score) / 3.0

        return metrics

    def _get_or_create_tracking_id(self, position: Point, timestamp: float,
                                   object_class: str) -> int:
        """
        Get existing tracking ID or create new one based on position matching

        This implements persistent object tracking since ZED SDK's tracking ID
        is not exposed in the ROS2 message interface.

        Args:
            position: Fused position of the detection
            timestamp: Current timestamp
            object_class: Object class (e.g., "PERSON")

        Returns:
            Persistent tracking ID
        """
        current_time = time.time()
        pos_array = np.array([position.x, position.y, position.z])

        # First, clean up stale tracks
        self._cleanup_stale_tracks(current_time)

        # Find best matching existing track
        best_match_id = None
        best_distance = float('inf')

        for track_id, track in self.tracked_objects.items():
            dist = np.linalg.norm(pos_array - track.position)
            if dist < self.tracking_match_distance and dist < best_distance:
                best_distance = dist
                best_match_id = track_id

        if best_match_id is not None:
            # Update existing track
            track = self.tracked_objects[best_match_id]
            # Smooth position update (exponential moving average)
            alpha = 0.3  # Smoothing factor
            track.position = alpha * pos_array + (1 - alpha) * track.position
            track.last_seen = current_time
            track.hit_count += 1
            track.miss_count = 0
            return best_match_id
        else:
            # Create new track
            new_id = self.next_tracking_id
            self.next_tracking_id += 1

            self.tracked_objects[new_id] = TrackedObject(
                tracking_id=new_id,
                position=pos_array,
                last_seen=current_time,
                hit_count=1,
                miss_count=0
            )

            self.fusion_stats['active_tracks'] = len(self.tracked_objects)
            return new_id

    def _cleanup_stale_tracks(self, current_time: float):
        """Remove tracks that haven't been seen recently"""
        stale_ids = []

        for track_id, track in self.tracked_objects.items():
            age = current_time - track.last_seen
            if age > self.track_timeout:
                stale_ids.append(track_id)
            elif age > 0.1:  # If not seen this frame
                track.miss_count += 1
                # Remove if too many consecutive misses
                if track.miss_count > 10:
                    stale_ids.append(track_id)

        for track_id in stale_ids:
            del self.tracked_objects[track_id]

        self.fusion_stats['active_tracks'] = len(self.tracked_objects)