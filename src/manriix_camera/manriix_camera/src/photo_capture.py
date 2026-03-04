import gphoto2 as gp
from datetime import datetime
import time
from manriix_camera.src.camera_manager import CameraManager
from manriix_camera.src.logger import logger as log 
import threading

class PhotoCaptureError(Exception):
    pass

class PhotoCapture:
    def __init__(self, camera, context, storage_manager, camera_lock=None):
        self.camera = camera
        self.context = context
        self.storage_manager = storage_manager
        self.camera_lock = camera_lock if camera_lock else threading.Lock()
        self.is_busy_flag = False
        self.busy_callback = None
    
    def _signal_busy(self, is_busy):
        """Signal busy state to callback"""
        self.is_busy_flag = is_busy
        if self.busy_callback:
            self.busy_callback(is_busy)
    
    def capture_photo(self, max_retries=10, retry_delay=0.2):
        """Capture a photo with retry logic"""
        # Import here to avoid circular imports
        from manriix_camera.src.session_tracker import session_tracker
        
        self._signal_busy(True)
        session_tracker.increment_photo_attempt()
        
        try:
            with self.camera_lock:
                for attempt in range(max_retries):
                    try:
                        self.storage_manager.check_storage_space()
                        
                        # Photo timestamp
                        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                        save_path = self.storage_manager.get_photo_path(timestamp)
                        
                        # Get camera summary for debugging
                        summary = self.camera.get_summary()
                        log.debug(f"Camera summary: {summary.text}")
                        
                        log.info(f"Capturing photo (attempt {attempt+1}/{max_retries})...")
                        camera_file_path = self.camera.capture(gp.GP_CAPTURE_IMAGE)
                        
                        camera_file = self.camera.file_get(
                            camera_file_path.folder,
                            camera_file_path.name,
                            gp.GP_FILE_TYPE_NORMAL
                        )
                        
                        camera_file.save(save_path)
                        log.info(f"Photo captured successfully and saved to {save_path}")
                        
                        # Track successful photo
                        session_tracker.increment_successful_photo()
                        return save_path
                
                    except gp.GPhoto2Error as e:
                        if "I/O in progress" in str(e) and attempt < max_retries - 1:
                            log.warning(f"I/O in progress, retrying in {retry_delay}s... (attempt {attempt+1}/{max_retries})")
                            time.sleep(retry_delay)
                            continue
                        else:
                            error = f"Failed to capture photo after {max_retries} attempts: {str(e)}"
                            log.error(error)
                            session_tracker.increment_failed_photo()
                            raise PhotoCaptureError(error) from e
                    
                    except FileNotFoundError as e:
                        error = f"File system error: {str(e)}"
                        log.error(error)
                        session_tracker.increment_failed_photo()
                        raise PhotoCaptureError(error) from e
                    
                    except Exception as e:
                        error = f"Unexpected error during photo capture: {str(e)}"
                        log.error(error)
                        session_tracker.increment_failed_photo()
                        raise PhotoCaptureError(error) from e
                        
        except PhotoCaptureError:
            # Already tracked as failed, just re-raise
            raise
        except Exception as e:
            # Track failed photo for any other exceptions
            session_tracker.increment_failed_photo()
            error = f"Unexpected error in capture_photo: {str(e)}"
            log.error(error)
            raise PhotoCaptureError(error) from e
        finally:
            self._signal_busy(False)