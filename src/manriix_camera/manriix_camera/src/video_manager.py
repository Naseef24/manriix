import cv2
import numpy as np
import gphoto2 as gp
from threading import Thread, Lock
import time
import os
from manriix_camera.src.logger import logger as log
from manriix_camera.utils.config_loader import get_config

# Try to import Flask streamer
try:
    from manriix_camera.src.flask_streamer import FlaskStreamer
    FLASK_STREAMING_AVAILABLE = True
    log.info("Flask streaming module loaded successfully")
except Exception as e:
    log.warning(f"Flask streaming module not available: {e}")
    FLASK_STREAMING_AVAILABLE = False

# Try to import Internet streamer
try:
    from manriix_camera.src.internet_streamer import InternetStreamer
    INTERNET_STREAMING_AVAILABLE = True
    log.info("Internet streaming module loaded successfully")
except Exception as e:
    log.warning(f"Internet streaming module not available: {e}")
    INTERNET_STREAMING_AVAILABLE = False

class VideoManagerError(Exception):
    pass

class VideoManager:
    def __init__(self, camera, context, photo_capture, video_recorder=None, enable_local_streaming=True, enable_internet_streaming=True):
        self.camera = camera
        self.context = context
        self.photo_capture = photo_capture
        self.video_recorder = video_recorder
        self.is_running = False
        self.stream_thread = None
        self.lock = Lock()  # Shared with VideoRecorder
        self.start_time = None
        config = get_config()
        self.duration = config.get('video_recording', {}).get('duration', 30)
        
        # Frame callback and busy state management
        self.frame_callback = None
        self.is_busy_flag = False
        self.busy_callback = None
        
        # Share the lock with video_recorder 
        if self.video_recorder:
            self.video_recorder.camera_lock = self.lock
        
        # Local Flask streaming
        self.enable_local_streaming = enable_local_streaming and FLASK_STREAMING_AVAILABLE
        self.local_streamer = None
        
        # Internet streaming with authentication
        self.enable_internet_streaming = enable_internet_streaming and INTERNET_STREAMING_AVAILABLE
        self.internet_streamer = None
        
        # Initialize Flask streamer if local streaming is enabled
        if self.enable_local_streaming and FLASK_STREAMING_AVAILABLE:
            try:
                FRAME_W, FRAME_H, FPS, PORT = 1280, 720, 30, 8080
                self.local_streamer = FlaskStreamer(
                    width=FRAME_W,
                    height=FRAME_H,
                    framerate=FPS, 
                    port=PORT
                )
                log.info("Flask local streamer initialized")
            except Exception as e:
                log.error(f"Failed to initialize Flask local streamer: {e}")
                self.enable_local_streaming = False
        
        # Initialize Internet streamer if internet streaming is enabled
        if self.enable_internet_streaming and INTERNET_STREAMING_AVAILABLE:
            try:
                FRAME_W, FRAME_H, FPS, PORT = 1280, 720, 15, 5000
                self.internet_streamer = InternetStreamer(
                    width=FRAME_W,
                    height=FRAME_H,
                    framerate=FPS,
                    port=PORT
                )
                log.info("Internet streamer initialized")
            except Exception as e:
                log.error(f"Failed to initialize Internet streamer: {e}")
                self.enable_internet_streaming = False
    
    def _signal_busy(self, is_busy):
        """Signal busy state to callback"""
        self.is_busy_flag = is_busy
        if self.busy_callback:
            self.busy_callback(is_busy)

    def capture_preview(self):
        """Capture a preview frame from the camera"""
        try:
            # Try to acquire the lock, but don't block if it's held
            # This allows preview to continue during photo/video operations
            if self.lock.acquire(blocking=False):
                try:
                    camera_file = self.camera.capture_preview()
                    file_data = camera_file.get_data_and_size()
                    data = np.frombuffer(file_data, dtype=np.uint8)
                    frame = cv2.imdecode(data, cv2.IMREAD_COLOR)
                    
                    if frame is None:
                        raise VideoManagerError("Failed to decode the preview frame.")
                    return frame
                finally:
                    self.lock.release()
            else:
                # If lock is held by photo/video operation, return None
                # The calling code already handles None frames
                return None
        except gp.GPhoto2Error as e:
            if "I/O in progress" in str(e):
                return None
            raise VideoManagerError(f"Failed to capture preview: {str(e)}") from e
        except Exception as e:
            raise VideoManagerError(f"Unexpected error during frame capture: {str(e)}") from e

    def start_stream(self):
        """Start the video stream"""
        if self.is_running:
            return

        # Start Flask local streamer if enabled
        if self.enable_local_streaming and self.local_streamer:
            success = self.local_streamer.start()
            if not success:
                log.warning("Failed to start Flask local streamer. Continuing without local streaming.")
                self.enable_local_streaming = False
        
        # Start Internet streamer if enabled
        if self.enable_internet_streaming and self.internet_streamer:
            success = self.internet_streamer.start()
            if not success:
                log.warning("Failed to start Internet streamer. Continuing without internet streaming.")
                self.enable_internet_streaming = False

        self.is_running = True
        self.stream_thread = Thread(target=self.stream_video)
        self.stream_thread.daemon = True
        self.stream_thread.start()
        log.info("Video stream started")

    def stream_video(self):
        """Main video streaming loop"""
        try:
            self.start_time = time.time()

            frame_error_count = 0  # Count consecutive frame errors
            max_frame_errors = 10  # Maximum consecutive errors before warning

            while self.is_running:
                try:
                    # Check if recording is in progress - reduce frame rate during recording
                    is_recording = self.video_recorder and self.video_recorder.is_currently_recording()

                    # If recording, use a slower frame rate to reduce contention
                    if is_recording:
                        time.sleep(0.1)  # Slow down the preview capture rate during recording
                   
                    frame = self.capture_preview()
                    if frame is not None:
                        # Reset error counter on successful frame
                        frame_error_count = 0

                        # Stream the frame via Local HTTP if enabled
                        if self.enable_local_streaming and self.local_streamer:
                            self.local_streamer.push_frame(frame)

                        # Stream the frame via Internet HTTP if enabled
                        if self.enable_internet_streaming and self.internet_streamer:
                            self.internet_streamer.push_frame(frame)

                        # Create a display frame with timer for local display
                        elapsed_time = time.time() - self.start_time
                        minutes, seconds = divmod(int(elapsed_time), 60)
                        timer_text = f"{minutes:02}:{seconds:02}"

                        # Add text to the display copy of the frame
                        display_frame = frame.copy()
                        cv2.putText(display_frame, timer_text, (10, 30), 
                                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

                        # Add recording indicator if we're recording
                        if is_recording:
                            # Add blinking red circle and "REC" text to indicate recording
                            if int(elapsed_time) % 2 == 0:  # Toggle visibility every 1 second
                                # Adjust position to be on the right side of the window
                                circle_position = (frame.shape[1] - 120, 30)  # Right side of the frame
                                cv2.circle(display_frame, circle_position, 10, (0, 0, 255), -1)  # Red circle
                                cv2.putText(display_frame, "REC", (circle_position[0] + 20, 40), 
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
                                
                        # Frame callback for external consumers (e.g., FrameServer)
                        if self.frame_callback is not None:
                            try:
                                self.frame_callback(display_frame.copy())
                            except Exception as e:
                                log.error(f"Error in frame callback: {e}")

                    else:
                        # If frame is None but we're recording, this is expected
                        if is_recording:
                            # Create a blank frame with recording indicator
                            blank_frame = np.zeros((720, 1280, 3), dtype=np.uint8)
                            cv2.putText(blank_frame, "RECORDING IN PROGRESS", (400, 360), 
                                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                            if int(elapsed_time) % 2 == 0:  # Toggle visibility every 1 second
                                cv2.circle(blank_frame, (360, 360), 15, (0, 0, 255), -1)
                        else:
                            # Increment error counter if not recording
                            frame_error_count += 1
                            if frame_error_count >= max_frame_errors:
                                log.warning(f"Multiple consecutive frame capture failures: {frame_error_count}")
                                frame_error_count = 0  # Reset to avoid spam

                    key = cv2.waitKey(1) & 0xFF
                    if key == ord('q'):  
                        self.is_running = False
                        break
                    elif key == ord('c'):  
                        try:
                            with self.lock:
                                if not is_recording:  # Only capture photos when not recording
                                    save_path = self.photo_capture.capture_photo()
                                    if save_path:
                                        log.info(f"Photo saved: {save_path}.CR3")
                                else:
                                    log.warning("Cannot capture photo while recording video")
                        except Exception as e:
                            log.error(f"Photo capture error: {str(e)}")

                    elif key == ord('v'):
                        if self.video_recorder:
                            if self.video_recorder.is_currently_recording():
                                log.warning("Already recording a video.")
                            else:
                                log.info(f"\nStarting video recording ({self.duration}s)...")
                                success = self.video_recorder.start_recording()
                                if not success:
                                    log.error("Failed to start video recording.")
                        else:
                            log.error("Video recording not available.")

                except Exception as e:
                    # If recording, some errors are expected
                    if self.video_recorder and self.video_recorder.is_currently_recording():
                        if "I/O in progress" in str(e):
                            log.debug("Frame capture paused during recording.")
                            time.sleep(0.2)  # Sleep a bit longer on error during recording
                            continue

                    log.error(f"Frame capture error: {str(e)}")
                    time.sleep(0.1)  # Avoid spinning too fast on errors
                    continue
        except Exception as e:
            log.error(f"Streaming error: {str(e)}")
            print(f"Streaming error: {str(e)}")
        finally:
            cv2.destroyAllWindows()

    def stop_stream(self):
        """Stop the video stream"""
        log.info("Stopping video stream...")
        self.is_running = False

        # If we're recording video, stop it
        if self.video_recorder and self.video_recorder.is_currently_recording():
            self.video_recorder.stop_recording()
        
        # Stop Flask local streamer if it was started
        if self.enable_local_streaming and self.local_streamer:
            self.local_streamer.stop()
        
        # Stop Internet streamer if it was started
        if self.enable_internet_streaming and self.internet_streamer:
            self.internet_streamer.stop()
        
        # Close all OpenCV windows
        cv2.destroyAllWindows()
        log.info("Video stream stopped")