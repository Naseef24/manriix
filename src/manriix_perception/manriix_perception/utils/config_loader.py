#!/usr/bin/env python3
"""
Configuration Loader Utility

Handles loading and validation of YAML configuration files
for the ZED multi-camera detection system.
"""

import os
import yaml
from typing import Dict, Any, Optional
from pathlib import Path


class ConfigurationError(Exception):
    """Raised when configuration loading or validation fails"""
    pass


class ConfigLoader:
    """Utility class for loading and validating YAML configurations"""
    
    @staticmethod
    def load_config(config_path: str) -> Dict[str, Any]:
        """
        Load YAML configuration file
        
        Args:
            config_path: Path to YAML configuration file
            
        Returns:
            Dictionary containing configuration data
            
        Raises:
            ConfigurationError: If loading fails
        """
        try:
            config_file = Path(config_path)
            if not config_file.exists():
                raise ConfigurationError(f"Configuration file not found: {config_path}")
            
            with open(config_file, 'r') as f:
                config = yaml.safe_load(f)
                
            if config is None:
                raise ConfigurationError(f"Empty or invalid YAML file: {config_path}")
                
            return config
            
        except yaml.YAMLError as e:
            raise ConfigurationError(f"YAML parsing error in {config_path}: {e}")
        except Exception as e:
            raise ConfigurationError(f"Failed to load config {config_path}: {e}")
    
    @staticmethod
    def get_camera_config(config: Dict[str, Any]) -> Dict[str, Any]:
        """
        Extract and validate camera configuration
        
        Args:
            config: Main configuration dictionary
            
        Returns:
            Camera configuration dictionary
        """
        if 'cameras' not in config:
            raise ConfigurationError("Missing 'cameras' section in configuration")
        
        camera_config = config['cameras']
        
        # Validate each camera configuration
        for camera_name, camera_data in camera_config.items():
            required_fields = ['topic', 'frame_id', 'position', 'rotation', 'enabled']
            for field in required_fields:
                if field not in camera_data:
                    raise ConfigurationError(
                        f"Missing required field '{field}' for camera '{camera_name}'"
                    )
            
            # Validate position and rotation are lists of 3 numbers
            if len(camera_data['position']) != 3:
                raise ConfigurationError(
                    f"Camera '{camera_name}' position must be [x, y, z]"
                )
            if len(camera_data['rotation']) != 3:
                raise ConfigurationError(
                    f"Camera '{camera_name}' rotation must be [roll, pitch, yaw]"
                )
        
        return camera_config
    
    @staticmethod
    def get_detection_config(config: Dict[str, Any]) -> Dict[str, Any]:
        """
        Extract and validate detection configuration
        
        Args:
            config: Main configuration dictionary
            
        Returns:
            Detection configuration dictionary
        """
        if 'detection' not in config:
            raise ConfigurationError("Missing 'detection' section in configuration")
        
        detection_config = config['detection']
        
        # Set defaults for optional parameters
        defaults = {
            'min_confidence': 50.0,
            'min_detection_range': 0.3,
            'max_detection_range': 8.0,
            'enabled_labels': ['PERSON'],  # Only PERSON by default
        }
        
        for key, default_value in defaults.items():
            if key not in detection_config:
                detection_config[key] = default_value
        
        return detection_config
    
    @staticmethod
    def get_fusion_config(config: Dict[str, Any]) -> Dict[str, Any]:
        """
        Extract and validate fusion configuration
        
        Args:
            config: Main configuration dictionary
            
        Returns:
            Fusion configuration dictionary
        """
        if 'fusion' not in config:
            raise ConfigurationError("Missing 'fusion' section in configuration")
        
        fusion_config = config['fusion']
        
        # Validate synchronization config
        if 'synchronization' not in fusion_config:
            fusion_config['synchronization'] = {
                'time_tolerance': 0.05,
                'queue_size': 10,
                'min_cameras_for_output': 1
            }
        
        # Validate spatial config
        if 'spatial' not in fusion_config:
            fusion_config['spatial'] = {
                'max_distance_merge': 0.8,
                'position_weight_by_confidence': True
            }
        
        return fusion_config
    
    @staticmethod
    def get_output_config(config: Dict[str, Any]) -> Dict[str, Any]:
        """
        Extract and validate output configuration
        
        Args:
            config: Main configuration dictionary
            
        Returns:
            Output configuration dictionary
        """
        if 'output' not in config:
            raise ConfigurationError("Missing 'output' section in configuration")
        
        output_config = config['output']
        
        # Ensure required output topics are configured
        defaults = {
            'humans_list': {
                'topic': '/human_detection/human_position',
                'frame_id': 'base_link',
                'publish_rate': 30.0
            },
            'costmap_obstacles': {
                'topic': '/object_obstacles',
                'frame_id': 'base_link', 
                'publish_rate': 10.0,
                'enabled': True
            }
        }
        
        for key, default_value in defaults.items():
            if key not in output_config:
                output_config[key] = default_value
            else:
                # Merge with defaults
                for subkey, subvalue in default_value.items():
                    if subkey not in output_config[key]:
                        output_config[key][subkey] = subvalue
        
        return output_config
    
    @staticmethod
    def get_transform_config(config: Dict[str, Any]) -> Dict[str, Any]:
        """
        Extract transform configuration
        
        Args:
            config: Main configuration dictionary
            
        Returns:
            Transform configuration dictionary
        """
        defaults = {
            'target_frame': 'base_link',
            'use_tf2': True,
            'tf_timeout': 0.1,
            'transform_cache_time': 1.0
        }
        
        if 'transform' in config:
            transform_config = config['transform']
            # Merge with defaults
            for key, default_value in defaults.items():
                if key not in transform_config:
                    transform_config[key] = default_value
        else:
            transform_config = defaults
        
        return transform_config
    
    @staticmethod
    def validate_config(config: Dict[str, Any]) -> bool:
        """
        Validate the complete configuration
        
        Args:
            config: Configuration dictionary to validate
            
        Returns:
            True if configuration is valid
            
        Raises:
            ConfigurationError: If validation fails
        """
        # Check required top-level sections
        required_sections = ['cameras', 'detection', 'fusion', 'output']
        for section in required_sections:
            if section not in config:
                raise ConfigurationError(f"Missing required section: {section}")
        
        # Validate subsections
        ConfigLoader.get_camera_config(config)
        ConfigLoader.get_detection_config(config)
        ConfigLoader.get_fusion_config(config)
        ConfigLoader.get_output_config(config)
        ConfigLoader.get_transform_config(config)
        
        return True
    
    @staticmethod
    def load_and_validate(config_path: str) -> Dict[str, Any]:
        """
        Load and validate configuration file
        
        Args:
            config_path: Path to configuration file
            
        Returns:
            Validated configuration dictionary
            
        Raises:
            ConfigurationError: If loading or validation fails
        """
        config = ConfigLoader.load_config(config_path)
        ConfigLoader.validate_config(config)
        return config