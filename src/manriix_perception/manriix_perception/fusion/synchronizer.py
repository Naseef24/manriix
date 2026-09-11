#!/usr/bin/env python3
"""
Multi-Camera Frame Synchronizer

Handles time-based synchronization of detection frames from multiple ZED cameras.
"""

import threading
import time
from collections import deque, defaultdict
from typing import Dict, List, Optional, Any, Callable
from dataclasses import dataclass

from manriix_perception.detection.zed_subscriber import ZedDetection


@dataclass
class CameraFrame:
    """Synchronized camera frame data"""
    camera_name: str
    detections: List[ZedDetection]
    timestamp: float
    frame_count: int


@dataclass 
class SynchronizedFrameSet:
    """Set of synchronized frames from multiple cameras"""
    frames: Dict[str, CameraFrame]
    reference_timestamp: float
    total_detections: int
    participating_cameras: List[str]


class FrameSynchronizer:
    """
    Synchronizes detection frames from multiple cameras based on timestamps
    
    Uses a time-based sliding window approach to find frames that arrived
    within a specified tolerance window.
    """
    
    def __init__(self, sync_config: Dict[str, Any], camera_names: List[str]):
        """
        Initialize frame synchronizer
        
        Args:
            sync_config: Synchronization configuration
            camera_names: List of camera names to synchronize
        """
        self.sync_config = sync_config
        self.camera_names = camera_names
        
        # Synchronization parameters - aligned with zed_fusion_node config keys
        # Supports both old and new parameter names for backwards compatibility
        self.time_tolerance = sync_config.get('max_time_diff',              # New: from zed_fusion_node
                             sync_config.get('time_tolerance', 0.1))         # Fallback: legacy name
        self.queue_size = sync_config.get('queue_size', 10)                  # Buffer size
        self.min_cameras_for_output = sync_config.get('min_cameras_required', # New: from zed_fusion_node
                                     sync_config.get('min_cameras_for_output', 1))  # Fallback: legacy name
        self.sync_timeout = sync_config.get('sync_timeout', 0.5)             # Sync timeout (seconds)
        
        # Frame buffers for each camera
        self.frame_buffers: Dict[str, deque] = {}
        self.frame_counters: Dict[str, int] = {}
        
        # Initialize buffers
        for camera_name in camera_names:
            self.frame_buffers[camera_name] = deque(maxlen=self.queue_size)
            self.frame_counters[camera_name] = 0
        
        # Synchronization statistics
        self.sync_stats = {
            'total_frames_received': 0,
            'synchronized_frame_sets': 0,
            'dropped_frames': 0,
            'average_sync_delay': 0.0,
            'camera_frame_counts': {name: 0 for name in camera_names},
            'last_sync_time': 0.0
        }
        
        # Callbacks for synchronized frames
        self.sync_callbacks: List[Callable] = []
        
        # Thread safety
        self.lock = threading.Lock()
        
        print(f"Frame synchronizer initialized for cameras: {camera_names}")
        print(f"Time tolerance: {self.time_tolerance}s, Queue size: {self.queue_size}")
    
    def add_camera_frame(self, camera_name: str, detections: List[ZedDetection], 
                        timestamp: float) -> Optional[SynchronizedFrameSet]:
        """
        Add a new frame from a camera and attempt synchronization
        
        Args:
            camera_name: Name of the camera
            detections: List of detections in this frame
            timestamp: Frame timestamp
            
        Returns:
            SynchronizedFrameSet if synchronization successful, None otherwise
        """
        if camera_name not in self.camera_names:
            return None
        
        with self.lock:
            # Update statistics
            self.sync_stats['total_frames_received'] += 1
            self.sync_stats['camera_frame_counts'][camera_name] += 1
            self.frame_counters[camera_name] += 1
            
            # Create camera frame
            camera_frame = CameraFrame(
                camera_name=camera_name,
                detections=detections,
                timestamp=timestamp,
                frame_count=self.frame_counters[camera_name]
            )
            
            # Add to buffer
            self.frame_buffers[camera_name].append(camera_frame)
            
            # Attempt synchronization
            synchronized_set = self._attempt_synchronization()
            
            if synchronized_set:
                self.sync_stats['synchronized_frame_sets'] += 1
                self.sync_stats['last_sync_time'] = time.time()
                
                # Calculate sync delay
                sync_delay = time.time() - synchronized_set.reference_timestamp
                self._update_average_sync_delay(sync_delay)
                
                # Notify callbacks
                self._notify_sync_callbacks(synchronized_set)
            
            return synchronized_set
    
    def _attempt_synchronization(self) -> Optional[SynchronizedFrameSet]:
        """
        Attempt to synchronize frames across cameras
        
        Returns:
            SynchronizedFrameSet if synchronization successful
        """
        # Find reference timestamp (latest timestamp across all cameras)
        reference_timestamp = self._find_reference_timestamp()
        
        if reference_timestamp is None:
            return None
        
        # Find frames within tolerance of reference timestamp
        synchronized_frames = {}
        total_detections = 0
        
        for camera_name in self.camera_names:
            frame = self._find_frame_near_timestamp(camera_name, reference_timestamp)
            
            if frame is not None:
                synchronized_frames[camera_name] = frame
                total_detections += len(frame.detections)
        
        # Check if we have minimum required cameras
        if len(synchronized_frames) < self.min_cameras_for_output:
            return None
        
        # Remove synchronized frames from buffers to avoid reuse
        self._remove_synchronized_frames(synchronized_frames, reference_timestamp)
        
        return SynchronizedFrameSet(
            frames=synchronized_frames,
            reference_timestamp=reference_timestamp,
            total_detections=total_detections,
            participating_cameras=list(synchronized_frames.keys())
        )
    
    def _find_reference_timestamp(self) -> Optional[float]:
        """
        Find reference timestamp for synchronization
        
        Returns:
            Reference timestamp or None if no suitable timestamp found
        """
        latest_timestamps = []
        
        # Get latest timestamp from each camera that has frames
        for camera_name in self.camera_names:
            buffer = self.frame_buffers[camera_name]
            if buffer:
                latest_timestamps.append(buffer[-1].timestamp)
        
        if not latest_timestamps:
            return None
        
        # Use the latest timestamp as reference
        return max(latest_timestamps)
    
    def _find_frame_near_timestamp(self, camera_name: str, 
                                 reference_timestamp: float) -> Optional[CameraFrame]:
        """
        Find frame closest to reference timestamp within tolerance
        
        Args:
            camera_name: Name of the camera
            reference_timestamp: Reference timestamp to match against
            
        Returns:
            CameraFrame if found within tolerance, None otherwise
        """
        buffer = self.frame_buffers[camera_name]
        
        if not buffer:
            return None
        
        best_frame = None
        min_time_diff = float('inf')
        
        # Search for frame within tolerance
        for frame in buffer:
            time_diff = abs(frame.timestamp - reference_timestamp)
            
            if time_diff <= self.time_tolerance and time_diff < min_time_diff:
                min_time_diff = time_diff
                best_frame = frame
        
        return best_frame
    
    def _remove_synchronized_frames(self, synchronized_frames: Dict[str, CameraFrame],
                                  reference_timestamp: float):
        """
        Remove synchronized frames from buffers to avoid reuse
        
        Args:
            synchronized_frames: Dictionary of synchronized frames
            reference_timestamp: Reference timestamp used for sync
        """
        for camera_name, frame in synchronized_frames.items():
            buffer = self.frame_buffers[camera_name]
            
            # Remove the synchronized frame and any older frames
            frames_to_remove = []
            for buffered_frame in buffer:
                if buffered_frame.timestamp <= frame.timestamp:
                    frames_to_remove.append(buffered_frame)
            
            for old_frame in frames_to_remove:
                try:
                    buffer.remove(old_frame)
                    if old_frame != frame:  # Don't count the synchronized frame as dropped
                        self.sync_stats['dropped_frames'] += 1
                except ValueError:
                    pass  # Frame already removed
    
    def _update_average_sync_delay(self, new_delay: float):
        """Update average synchronization delay"""
        total_sets = self.sync_stats['synchronized_frame_sets']
        current_avg = self.sync_stats['average_sync_delay']
        
        if total_sets == 1:
            self.sync_stats['average_sync_delay'] = new_delay
        else:
            new_avg = ((current_avg * (total_sets - 1)) + new_delay) / total_sets
            self.sync_stats['average_sync_delay'] = new_avg
    
    def _notify_sync_callbacks(self, synchronized_set: SynchronizedFrameSet):
        """
        Notify all registered callbacks of synchronized frame set
        
        Args:
            synchronized_set: Synchronized frame set
        """
        for callback in self.sync_callbacks:
            try:
                callback(synchronized_set)
            except Exception as e:
                print(f"Error in sync callback: {e}")
    
    def register_sync_callback(self, callback: Callable[[SynchronizedFrameSet], None]):
        """
        Register callback for synchronized frame sets
        
        Args:
            callback: Function to call with synchronized frame sets
        """
        with self.lock:
            self.sync_callbacks.append(callback)
        print(f"Registered sync callback: {callback.__name__}")
    
    def get_synchronization_statistics(self) -> Dict[str, Any]:
        """
        Get current synchronization statistics
        
        Returns:
            Dictionary containing synchronization statistics
        """
        with self.lock:
            stats = dict(self.sync_stats)
            
            # Add derived statistics
            total_received = stats['total_frames_received']
            if total_received > 0:
                stats['sync_success_rate'] = stats['synchronized_frame_sets'] / total_received
                stats['drop_rate'] = stats['dropped_frames'] / total_received
            else:
                stats['sync_success_rate'] = 0.0
                stats['drop_rate'] = 0.0
            
            # Add buffer status
            stats['buffer_status'] = {}
            for camera_name, buffer in self.frame_buffers.items():
                stats['buffer_status'][camera_name] = {
                    'queue_length': len(buffer),
                    'latest_timestamp': buffer[-1].timestamp if buffer else 0.0,
                    'oldest_timestamp': buffer[0].timestamp if buffer else 0.0
                }
            
            return stats
    
    def get_camera_sync_health(self) -> Dict[str, Any]:
        """
        Get synchronization health per camera
        
        Returns:
            Dictionary containing camera sync health information
        """
        current_time = time.time()
        health_info = {}
        
        with self.lock:
            for camera_name in self.camera_names:
                buffer = self.frame_buffers[camera_name]
                
                if not buffer:
                    health_info[camera_name] = {
                        'status': 'inactive',
                        'last_frame_age': float('inf'),
                        'buffer_utilization': 0.0
                    }
                else:
                    latest_frame = buffer[-1]
                    frame_age = current_time - latest_frame.timestamp
                    buffer_utilization = len(buffer) / self.queue_size
                    
                    # Determine status
                    if frame_age > 2.0:  # No frames in last 2 seconds
                        status = 'stale'
                    elif buffer_utilization > 0.8:  # Buffer getting full
                        status = 'overloaded'
                    else:
                        status = 'healthy'
                    
                    health_info[camera_name] = {
                        'status': status,
                        'last_frame_age': frame_age,
                        'buffer_utilization': buffer_utilization,
                        'frame_count': self.frame_counters[camera_name]
                    }
        
        return health_info
    
    def cleanup_old_frames(self, max_age: float = 5.0):
        """
        Clean up frames older than max_age to prevent memory buildup
        
        Args:
            max_age: Maximum age of frames to keep (seconds)
        """
        current_time = time.time()
        cutoff_time = current_time - max_age
        
        with self.lock:
            for camera_name, buffer in self.frame_buffers.items():
                frames_to_remove = []
                
                for frame in buffer:
                    if frame.timestamp < cutoff_time:
                        frames_to_remove.append(frame)
                
                for old_frame in frames_to_remove:
                    try:
                        buffer.remove(old_frame)
                        self.sync_stats['dropped_frames'] += 1
                    except ValueError:
                        pass
    
    def reset_statistics(self):
        """Reset synchronization statistics"""
        with self.lock:
            self.sync_stats = {
                'total_frames_received': 0,
                'synchronized_frame_sets': 0,
                'dropped_frames': 0,
                'average_sync_delay': 0.0,
                'camera_frame_counts': {name: 0 for name in self.camera_names},
                'last_sync_time': 0.0
            }
            
            # Reset frame counters
            for camera_name in self.camera_names:
                self.frame_counters[camera_name] = 0
    
    def shutdown(self):
        """Clean shutdown of synchronizer"""
        print("Shutting down frame synchronizer")
        
        with self.lock:
            # Clear all buffers
            for buffer in self.frame_buffers.values():
                buffer.clear()
            
            # Clear callbacks
            self.sync_callbacks.clear()