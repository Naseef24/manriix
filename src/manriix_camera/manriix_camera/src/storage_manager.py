import os
import shutil
from datetime import datetime
from manriix_camera.src.logger import logger as log 
from manriix_camera.utils.config_loader import get_config 

class StorageError(Exception):
    pass

class StorageManager:
    def __init__(self, required_space=None, save_folder=None):
        config = get_config()
        storage_config = config.get('storage', {})
        
        # Get base path relative to current file location
        self.base_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..')
        
        if save_folder is None:
            # Use config or default to user's home directory
            save_folder = storage_config.get('save_folder', os.path.expanduser('~/camera_data'))
        
        # If save_folder is relative, make it absolute based on base_path
        # If it's already absolute, use it as-is
        if not os.path.isabs(save_folder):
            self.save_folder = os.path.join(self.base_path, save_folder)
        else:
            self.save_folder = save_folder
        
        self.current_date = datetime.now().strftime("%Y-%m-%d")
        
        if required_space is None:
            required_space_mb = storage_config.get('required_space_mb', 50)
            required_space = required_space_mb * 1024 * 1024
        self.required_space = required_space  
        
        self.current_dir = None
        self.photos_dir = None 
        self.videos_dir = None 
    
    def create_directory_structure(self):
        """Creates the directory structure."""
        try:
            if not os.path.exists(self.save_folder):
                os.makedirs(self.save_folder)
                log.info(f"Created save directory: {self.save_folder}")
            
            # Main directory for current date
            self.current_dir = os.path.join(self.save_folder, self.current_date)
            os.makedirs(self.current_dir, exist_ok=True)
            
            # Subdirectories for photos and videos
            self.photos_dir = os.path.join(self.current_dir, 'photos')
            self.videos_dir = os.path.join(self.current_dir, 'videos')
            os.makedirs(self.photos_dir, exist_ok=True)
            os.makedirs(self.videos_dir, exist_ok=True)
            
            log.info(f"Contents will be saved in: {self.current_dir}")
            return self.current_dir
        except Exception as e:
            error = f"Failed to create directory structure: {str(e)}"
            log.error(error)
            raise StorageError(error) from e
            
    def check_storage_space(self):
        """Check the storage space before saving"""
        try:
            if not os.path.exists(self.save_folder):
                error = f"Save folder does not exist: {self.save_folder}"
                log.error(error)
                raise StorageError(error) 
            
            total, used, free = shutil.disk_usage(self.save_folder)
            
            if free < self.required_space:
                error = (f"Insufficient storage space. "
                        f"Required: {self.required_space // (1024 * 1024)}MB, "
                        f"Available: {free // (1024 * 1024)}MB")
                log.error(error)
                raise StorageError(error)
            
            log.info(f"Storage check passed. Available: {free // (1024 * 1024)}MB")
            return True
        except StorageError:
            raise
        except Exception as e:
            error = f"Failed to check storage space: {str(e)}"
            log.error(error)
            raise StorageError(error) from e
    
    def get_photo_path(self, photo_timestamp=None):
        """Get the path for saving a photo"""
        if not self.current_dir or not self.photos_dir:
            error = "Directory structure not initialized. Call create_directory_structure() first."
            log.error(error)
            raise StorageError(error)
        
        if photo_timestamp is None:
            photo_timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        
        return os.path.join(self.photos_dir, f"IMG_{photo_timestamp}")
    
    def get_video_path(self, video_timestamp=None):
        """Get the path for saving a video"""
        if not self.current_dir or not self.videos_dir:
            error = "Directory structure not initialized. Call create_directory_structure() first."
            log.error(error)
            raise StorageError(error)
        
        if video_timestamp is None:
            video_timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        
        return os.path.join(self.videos_dir, f"VID_{video_timestamp}")