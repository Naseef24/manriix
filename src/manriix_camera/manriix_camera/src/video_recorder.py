import gphoto2 as gp 
import time 
import os 
import sys 
import threading
from datetime import datetime 
from manriix_camera.src.logger import logger as log
from manriix_camera.utils.config_loader import get_config

class VideoRecordError(Exception):
    pass 

class VideoRecorder:
    def __init__(self, camera, context, storage_manager, camera_lock=None):
        self.camera = camera 
        self.context = context 
        self.storage_manager = storage_manager 
        self.is_recording = False 
        self.recording_thread = None 
        # Fixed: removed duplicate assignment
        self.camera_lock = camera_lock if camera_lock else threading.Lock()
        
        config = get_config()
        self.duration = config.get('video_recording', {}).get('duration', 30)
        self.is_busy_flag = False
        self.busy_callback = None

    def _signal_busy(self, is_busy):
        """Signal busy state to callback"""
        self.is_busy_flag = is_busy
        if self.busy_callback:
            self.busy_callback(is_busy)

    def set_config_value(self, config_name, value, max_retries=5, retry_delay=0.5):
        """Set camera config values"""
        if not self.camera:
            log.error("Camera not initialized.")
            return False 
        
        for attempt in range(max_retries):
            try:
                with self.camera_lock:  # Use lock to avoid concurrent camera access
                    config = self.camera.get_config()
                    config_item = config.get_child_by_name(config_name)
                    current_value = config_item.get_value()
                    log.info(f"Current {config_name} setting: {current_value}")

                    # Set new value 
                    log.info(f"Setting {config_name} to {value} (attempt {attempt+1}/{max_retries})")
                    if isinstance(value, int):
                        config_item.set_value(value)
                    else:
                        config_item.set_value(str(value))

                    self.camera.set_config(config)
                    return True
            except gp.GPhoto2Error as e:
                if "I/O in progress" in str(e) and attempt < max_retries - 1:
                    log.warning(f"I/O in progress, retrying in {retry_delay}s...")
                    time.sleep(retry_delay)
                    continue
                else:
                    log.error(f"Failed to set {config_name} to {value}: {str(e)}")
                    return False 
            except Exception as e:
                log.error(f"Unexpected error setting {config_name}: {str(e)}")
                return False

    def wait_for_event(self, timeout_ms=1000, download_files=False, save_path=None):
        """Wait for a camera event -> to maintain the recording duration"""
        if not self.camera:
            log.error("Camera not initialized.")
            return None 

        try:
            with self.camera_lock:  # Use lock for camera access
                event_type, event_data = self.camera.wait_for_event(timeout_ms) 

                if event_type == gp.GP_EVENT_FILE_ADDED:
                    log.info(f"File added on camera: {event_data.folder}/{event_data.name}")

                    if download_files and save_path:

                        camera_file = self.camera.file_get(
                            event_data.folder,
                            event_data.name,
                            gp.GP_FILE_TYPE_NORMAL
                        )
                        camera_file.save(save_path)
                        return save_path
            return None 
        except gp.GPhoto2Error as e:
            log.error(f"Error waiting for event: {str(e)}")
            return None

    def progress_bar(self, duration_sec):
        """Display a progress bar for video recording"""
        BLUE = "\033[94m"  # Blue color
        RESET = "\033[0m"  # Reset color

        start_time = time.time()
        end_time = start_time + duration_sec
        last_percent = -1

        print(f"Recording for {duration_sec} seconds: ", end="", flush=True)

        while time.time() < end_time and self.is_recording:
            elapsed = time.time() - start_time
            percent = int((elapsed / duration_sec) * 20)  # 20 progress marks

            if percent > last_percent:
                print(f"{BLUE}{'█' * (percent - last_percent)}{RESET}", end="", flush=True)
                last_percent = percent

            time.sleep(0.1)

        if last_percent < 20 and self.is_recording:
            print(f"{BLUE}{'█' * (20 - last_percent)}{RESET}", end="", flush=True)
        
        print(" Done!" if self.is_recording else " Cancelled!", flush=True)


    def record_video(self):
        """Record Video"""
        # Import here to avoid circular imports
        from manriix_camera.src.session_tracker import session_tracker
        
        if self.is_recording:
            log.warning("Already recording a video.")
            return None 

        self.is_recording = True 
        self._signal_busy(True)
        session_tracker.increment_video_attempt(self.duration)
        
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        save_path = self.storage_manager.get_video_path(timestamp)     

        try:
            # Check storage 
            self.storage_manager.check_storage_space()
            # Make dirs if not exist
            os.makedirs(os.path.dirname(os.path.abspath(save_path)), exist_ok=True)

            # Step 1: Temporarily pause streaming to configure camera
            time.sleep(0.2)  # Short delay to let any pending operations complete

            # Step 2: Set EOS viewfinder to 1 (on)
            log.info("Setting up viewfinder...")
            if not self.set_config_value("viewfinder", 1, max_retries=5, retry_delay=0.5):
                error = "Failed to set viewfinder"
                log.error(error)
                raise VideoRecordError(error)

            # Step 3: Set movie record target to start recording
            log.info("Starting video recording...")
            if not self.set_config_value("movierecordtarget", "Card", max_retries=5, retry_delay=0.5):
                error = "Failed to start the recording."
                log.error(error)
                raise VideoRecordError(error)
            
            # Step 4: Display progress for the specified duration
            self.progress_bar(self.duration)

            # Step 5: Stop recording
            log.info("Stopping video recording...")
            if not self.set_config_value("movierecordtarget", "None", max_retries=5, retry_delay=0.5):
                error = "Failed to end the recording."
                log.error(error)
                raise VideoRecordError(error)
            
            # Step 6: Wait for file and download
            log.info("Waiting for video file to be ready...")

            # Try several times to wait for the file
            max_attempts = 10
            for attempt in range(max_attempts):
                log.info(f"Waiting for video file (attempt {attempt+1}/{max_attempts})...")
                
                # Wait for a event and try to download
                for _ in range(50): # Check events multiple times with short timeout
                    if not self.is_recording:
                        return None 
                    
                    result = self.wait_for_event(
                        timeout_ms=200,
                        download_files=True,
                        save_path=save_path
                    )
                    if result:
                        log.info(f"Video saved: {save_path}.mp4")
                        session_tracker.increment_successful_video()
                        return save_path 
                    
                # Try again if no file event
                time.sleep(0.5)
            
            log.warning("Video was recorded but automatic download failed.")
            log.warning("Please manually copy the video from your camera.")
            session_tracker.increment_failed_video()
            return None
        except Exception as e:
            log.error(f"Error during video recording: {str(e)}")
            session_tracker.increment_failed_video()
            return None 
        finally:
            self.is_recording = False 
            self._signal_busy(False) 

    def start_recording(self):
        """Start recording in separate thread"""
        if self.is_recording:
            log.warning("Already recording a video.")
            return False

        self.recording_thread = threading.Thread(
            target=self.record_video,
        )
        self.recording_thread.daemon = True 
        self.recording_thread.start()
        return True 
    
    def stop_recording(self):
        """Stop the recording"""
        if not self.is_recording:
            return False 
        
        log.info("Stopping video recording...")
        self.is_recording = False 

        try:
            with self.camera_lock:  # Use lock for camera access
                config = self.camera.get_config()
                config_item = config.get_child_by_name("movierecordtarget")
                config_item.set_value(str("None"))
                self.camera.set_config(config)
        except Exception as e:
            log.error(f"Error stopping recording: {str(e)}")
        
        # Wait for the thread to finish
        if self.recording_thread and self.recording_thread.is_alive():
            self.recording_thread.join(timeout=2.0)
        
        log.info("Video recording stopped.")
        return True 
    
    def is_currently_recording(self):
        """Check if currently recording a video"""
        return self.is_recording