import gphoto2 as gp
from manriix_camera.src.logger import logger as log

class CameraConnectionError(Exception):
    pass

class CameraManager:
    def __init__(self):
        self.camera = None
        self.context = None

    def initialize_camera(self):

        try:
            self.context = gp.Context()
            self.camera = gp.Camera()
            self.camera.init(self.context)
            log.info("Camera initialized successfully.")
            return True
        except gp.GPhoto2Error as e:
            self.camera = None  
            self.context = None
            error = f"Failed to initialize camera: {str(e)}"
            log.error(error)
            raise CameraConnectionError(error) from e
        except Exception as e:
            error = f"Unexpected error during camera initialization: {str(e)}"
            log.error(error)
            raise CameraConnectionError(error) from e

    def get_camera_info(self):
        
        if not self.camera:
            error = "Camera not initialized. Please initialize the camera first."
            log.error(error)
            raise CameraConnectionError(error)
        
        try:
            config = self.camera.get_config()
            details = {
                "serialnumber": config.get_child_by_name("serialnumber").get_value(),
                "manufacturer": config.get_child_by_name("manufacturer").get_value(),
                "model": config.get_child_by_name("cameramodel").get_value(),
                "firmware_version": config.get_child_by_name("deviceversion").get_value(),
                "battery_level": config.get_child_by_name("batterylevel").get_value(),
                "lensname": config.get_child_by_name("lensname").get_value(),
                "focus_mode": config.get_child_by_name("focusmode").get_value(),  # focus mode
            }
            return details
        except gp.GPhoto2Error as e:
            error = f"Failed to get camera info: {str(e)}"
            log.error(error)
            raise CameraConnectionError(error) from e
        except Exception as e:
            error = f"Unexpected error while retrieving camera info: {str(e)}"
            log.error(error)
            raise CameraConnectionError(error) from e
        
    def set_focus_auto(self):
        """Set camera to auto-focus mode."""
        if not self.camera:
            raise CameraConnectionError("Camera not initialized. Please initialize the camera first.")
        
        try:
            config = self.camera.get_config()
            # Check if 'focusmode' exists in the camera config
            focus_mode = config.get_child_by_name("focusmode")
            
            if focus_mode:
                # Set the focus mode to auto
                focus_mode.set_value("auto")
                self.camera.set_config(config)
                log.info("Focus mode set to auto.")
            else:
                error = "Focus mode setting not available on this camera."
                log.error(error)
                raise CameraConnectionError(error)
        except gp.GPhoto2Error as e:
            error = f"Failed to set focus mode to auto: {str(e)}"
            log.error(error)
            raise CameraConnectionError(error) from e
        except Exception as e:
            error = f"Unexpected error while setting focus mode: {str(e)}"
            log.error(error)
            raise CameraConnectionError(error) from e
        
    def close(self):
        
        if self.camera:
            try:
                self.camera.exit(self.context)
                log.info(f"Camera connection closed successfully.")
            except gp.GPhoto2Error as e:
                log.error(f"Error while closing camera connection: {str(e)}")
            except Exception as e:
                log.error(f"Unexpected error while closing camera connection: {str(e)}")
            finally:
                self.camera = None
                self.context = None