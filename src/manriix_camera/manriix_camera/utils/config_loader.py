import yaml
import os
from pathlib import Path

def find_config_path():
    """
    Find the config file using multiple strategies:
    1. Environment variable CONFIG_PATH
    2. ROS2 package share directory
    3. Relative to current file location
    4. Current working directory
    """
    # Strategy 1: Check environment variable
    if 'MANRIIX_CAMERA_CONFIG_PATH' in os.environ:
        config_path = os.environ['MANRIIX_CAMERA_CONFIG_PATH']
        if os.path.exists(config_path):
            return config_path
    
    # Strategy 2: Try to use ROS2 package share directory
    try:
        from ament_index_python.packages import get_package_share_directory
        pkg_share = get_package_share_directory('manriix_camera')
        config_path = os.path.join(pkg_share, 'config', 'config.yaml')
        if os.path.exists(config_path):
            return config_path
    except (ImportError, Exception):
        pass
    
    # Strategy 3: Relative to this file's location (development)
    # Go up from utils/ to package root, then to config/
    current_file = Path(__file__).resolve()
    # __file__ is in: manriix_camera/utils/config_loader.py
    # We want: manriix_camera/../config/config.yaml
    config_path = current_file.parent.parent.parent / 'config' / 'config.yaml'
    if config_path.exists():
        return str(config_path)
    
    # Strategy 4: Check current working directory
    config_path = os.path.join(os.getcwd(), 'config', 'config.yaml')
    if os.path.exists(config_path):
        return config_path
    
    # Strategy 5: Check workspace src directory pattern
    # Look for config in common workspace locations
    possible_paths = [
        os.path.expanduser('~/manriix2_ws/src/manriix_camera/config/config.yaml'),
        os.path.expanduser('~/colcon_ws/src/manriix_camera/config/config.yaml'),
        './config/config.yaml',
        '../config/config.yaml',
    ]
    
    for path in possible_paths:
        if os.path.exists(path):
            return path
    
    return None


def load_config(config_path=None):
    """Load configuration from YAML file"""
    try:
        # If no path provided, try to find it
        if config_path is None:
            config_path = find_config_path()
        
        if config_path is None or not os.path.exists(config_path):
            print(f"Warning: Config file not found. Using defaults.")
            print(f"Tried path: {config_path}")
            print("Set MANRIIX_CAMERA_CONFIG_PATH environment variable to specify config location.")
            return get_default_config()
        
        print(f"Loading config from: {config_path}")
        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)
        
        return config if config else get_default_config()
        
    except Exception as e:
        print(f"Error loading config: {e}")
        return get_default_config()


def get_default_config():
    """Return default configuration"""
    return {
        'logging': {
            'log_dir': 'logs',
            'retention_days': 60
        },
        'camera': {
            'auto_focus': True,
            'capture_timeout': 5
        },
        'storage': {
            'base_path': os.path.expanduser('~/camera_data'),
            'min_free_space_gb': 5
        },
        'video': {
            'recording_duration': 10,
            'format': 'mp4'
        }
    }


_config = None

def get_config(reload=False):
    """Get the global config instance"""
    global _config
    if _config is None or reload:
        _config = load_config()
    return _config