#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg import String, Bool

from .src.camera_manager import CameraManager, CameraConnectionError
from .src.video_manager import VideoManager, VideoManagerError
from .src.photo_capture import PhotoCapture, PhotoCaptureError
from .src.video_recorder import VideoRecorder, VideoRecordError
from .src.storage_manager import StorageManager, StorageError
from .src.frame_server import FrameServer
from .src.session_tracker import session_tracker

import time
import threading
import signal
import sys


class CameraNode(Node):
    def __init__(self):
        # Initialize ROS2 node
        super().__init__('camera_node')
        
        # Start session tracking
        session_tracker.start_session()

        self.last_capture_state = 'n'
        
        # Initialize class variables
        self.camera_manager = CameraManager()
        self.storage_manager = StorageManager()
        self.video_stream = None
        self.photo_capture = None
        self.video_recorder = None
        self.video_manager = None

        self.camera_lock = threading.Lock()
        
        # Declare ROS2 parameters with defaults
        self.declare_parameter('local_streaming', True)
        self.declare_parameter('internet_streaming', True)
        
        # Get parameter values
        self.enable_local_streaming = self.get_parameter('local_streaming').value
        self.enable_internet_streaming = self.get_parameter('internet_streaming').value
        
        # Busy state management
        self.busy_publisher = self.create_publisher(Bool, '/camera_node/busy', 10)
        self.current_busy_state = False
        self.busy_state_lock = threading.Lock()
        self.monitoring_thread = None
        
        # Create callback group for concurrent callbacks
        self.callback_group = ReentrantCallbackGroup()
        
        # Initialize subscriber for all camera commands
        self.command_sub = self.create_subscription(
            String,
            '/capture_image_topic',
            self.command_callback,
            10,
            callback_group=self.callback_group
        )
        
        # Setup everything
        self.setup()

    def setup(self):
        try:
            self.get_logger().info("Checking for camera connection...")
            self.camera_manager.initialize_camera()
            self.get_logger().info("✅ Camera connected successfully!")

            # Camera info
            camera_info = self.camera_manager.get_camera_info()
            
            self.get_logger().info('-' * 50)
            self.get_logger().info("\t\tCamera Information:")
            self.get_logger().info('-' * 50)
            for key, value in camera_info.items():
                self.get_logger().info(f"\n{key.capitalize().replace('_', ' ')}: {value}")
            self.get_logger().info('-' * 50)    
            
            # Storage initialization
            self.get_logger().info("\nInitializing storage...")
            save_dir = self.storage_manager.create_directory_structure()
            self.storage_manager.check_storage_space()
            self.get_logger().info(f"✅ Storage initialized. Content will be saved in: {save_dir}")

            # Initialize photo capture
            self.photo_capture = PhotoCapture(
                self.camera_manager.camera,
                self.camera_manager.context,
                self.storage_manager,
                self.camera_lock
            )
            self.photo_capture.busy_callback = self.on_component_busy_state
            
            # Initialize video recorder
            self.video_recorder = VideoRecorder(
                self.camera_manager.camera,
                self.camera_manager.context,
                self.storage_manager,
                self.camera_lock
            )
            self.video_recorder.busy_callback = self.on_component_busy_state
            
            # Initialize video manager
            self.get_logger().info("\nInitializing video manager...")
            self.video_manager = VideoManager(
                self.camera_manager.camera,
                self.camera_manager.context,
                self.photo_capture,
                self.video_recorder,
                enable_local_streaming=self.enable_local_streaming,
                enable_internet_streaming=self.enable_internet_streaming
            )
            self.video_manager.busy_callback = self.on_component_busy_state
            
            ### Frame server
            self.frame_server = FrameServer(host='127.0.0.1', port=8089)
            self.frame_server.start()
            
            # Register the frame callback
            self.video_manager.frame_callback = self.frame_server.frame_callback
            
            # Start busy state monitoring
            self.start_busy_monitoring()

        except CameraConnectionError as e:
            self.get_logger().error(f"Camera Connection Error: {str(e)}")
            self.shutdown()
        except VideoManagerError as e:
            self.get_logger().error(f"Video Manager Error: {str(e)}")
            self.shutdown()
        except StorageError as e:
            self.get_logger().error(f"Storage Error: {str(e)}")
            self.shutdown()
        except Exception as e:
            self.get_logger().error(f"Unexpected error during setup: {str(e)}")
            self.shutdown()
    
    def on_component_busy_state(self, is_busy):
        self.publish_busy_state(is_busy)
    
    def publish_busy_state(self, is_busy):
        with self.busy_state_lock:
            if is_busy != self.current_busy_state:
                self.current_busy_state = is_busy
                msg = Bool()
                msg.data = is_busy
                self.busy_publisher.publish(msg)
                self.get_logger().info(f"Camera busy state: {is_busy}")
    
    def start_busy_monitoring(self):
        self.monitoring_thread = threading.Thread(target=self.monitor_busy_state)
        self.monitoring_thread.daemon = True
        self.monitoring_thread.start()
    
    def monitor_busy_state(self):
        # Create a rate limiter (10 Hz)
        rate = self.create_rate(10)
        
        while rclpy.ok():
            try:
                is_busy = (
                    self.camera_lock.locked() or
                    (self.video_recorder and self.video_recorder.is_currently_recording()) or
                    (hasattr(self.photo_capture, 'is_busy_flag') and self.photo_capture.is_busy_flag) or
                    (hasattr(self.video_recorder, 'is_busy_flag') and self.video_recorder.is_busy_flag) or
                    (hasattr(self.video_manager, 'is_busy_flag') and self.video_manager.is_busy_flag)
                )
                self.publish_busy_state(is_busy)
                rate.sleep()
            except Exception as e:
                self.get_logger().error(f"Error in busy monitoring: {e}")
                rate.sleep()

    def command_callback(self, msg):
        """Callback function for the capture_image_topic subscriber handling all commands"""
        try:
            # Only proceed if the state has changed
            if msg.data != self.last_capture_state:
                self.last_capture_state = msg.data  # Update the state

                if msg.data == 'y':  # Capture photo when message changes to 'y'
                    self.get_logger().info("Photo capture command received")
                    if self.photo_capture:
                        try:
                            filepath = self.photo_capture.capture_photo()
                            if filepath:
                                self.get_logger().info(f"Photo captured successfully and saved to {filepath}")
                        except PhotoCaptureError as e:
                            # More detailed error logging
                            self.get_logger().error(f"Photo Capture Error: {str(e)}")
                            # Wait a bit before allowing new commands to prevent rapid error loops
                            time.sleep(1.0)
                    else:
                        self.get_logger().warn("Photo capture not initialized")
                
                elif msg.data == 'v':  # Start recording when message changes to 'v'
                    self.get_logger().info("Video recording command received")
                    if self.video_recorder:
                        if not self.video_recorder.is_currently_recording():
                            success = self.video_recorder.start_recording()
                            if success:
                                self.get_logger().info("Video recording started (will automatically stop after the configured duration)")
                            else:
                                self.get_logger().error("Failed to start video recording")
                        else:
                            self.get_logger().warn("Already recording a video")
                    else:
                        self.get_logger().warn("Video recorder not initialized")

        except PhotoCaptureError as e:
            self.get_logger().error(f"Photo Capture Error: {str(e)}")
        except VideoRecordError as e:
            self.get_logger().error(f"Video Record Error: {str(e)}")
        except Exception as e:
            self.get_logger().error(f"Unexpected error during command handling: {str(e)}")
            
    def toggle_streaming(self, enable=True):
        """Toggle the video streaming on or off"""
        try:
            if self.video_manager:
                if enable and not self.video_manager.is_running:
                    self.get_logger().info("Starting video stream...")
                    self.video_manager.start_stream()
        
                    self.get_logger().info("Video stream started")
                    return True
                elif not enable and self.video_manager.is_running:
                    self.get_logger().info("Stopping video stream...")
                    self.video_manager.stop_stream()
                
                    self.get_logger().info("Video stream stopped")
                    return True
            return False
        except Exception as e:
            self.get_logger().error(f"Error toggling streaming: {str(e)}")
            return False

    def shutdown(self):
        """Cleanup and shutdown"""
        self.get_logger().info("Shutting down camera node...")
        
        if hasattr(self, 'frame_server'):
            self.frame_server.stop()
        
        # Stop video streaming and recording
        if self.video_manager and self.video_manager.is_running:
            self.video_manager.stop_stream()
        if self.video_recorder and self.video_recorder.is_currently_recording():
            self.video_recorder.stop_recording()
        if self.camera_manager:
            self.camera_manager.close()
        
        self.get_logger().info("✅ Camera node shutdown complete")
        
        # Print session summary
        session_tracker.print_session_summary()

    def run(self):
        """Main run loop - starts streaming automatically"""
        try:
            # Always start streaming at initialization
            if self.video_manager:
                self.video_manager.start_stream()
                self.get_logger().info("Video streaming started automatically")
                
            self.get_logger().info("Camera node is running. Waiting for ROS messages...")
            self.get_logger().info("Command options:")
            self.get_logger().info("  'y' - Capture a photo")
            self.get_logger().info("  'v' - Start video recording (will stop automatically after the configured duration)")
            
        except Exception as e:
            self.get_logger().error(f"Error in run loop: {str(e)}")


def main(args=None):
    # Initialize rclpy
    rclpy.init(args=args)
    
    try:
        # Create the node
        camera_node = CameraNode()
        
        # Start automatic streaming
        camera_node.run()
        
        # Use MultiThreadedExecutor for concurrent callback handling
        executor = MultiThreadedExecutor()
        executor.add_node(camera_node)
        
        try:
            # Spin the executor
            executor.spin()
        except KeyboardInterrupt:
            camera_node.get_logger().info("Node interrupted by user")
        finally:
            # Cleanup
            camera_node.shutdown()
            camera_node.destroy_node()
            
    except Exception as e:
        print(f"Fatal error: {str(e)}")
    finally:
        # Shutdown rclpy
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()