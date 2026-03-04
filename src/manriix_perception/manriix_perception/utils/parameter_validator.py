#!/usr/bin/env python3
"""
Simple Parameter Validator for ROS2 Nodes
==========================================

Lightweight validation without external dependencies.
Provides type checking, range validation, and clear error reporting.
"""

from typing import Any, Optional, List, Union, Type
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter


class ParameterValidator:
    """Simple parameter validation for ROS2 nodes without external dependencies"""
    
    def __init__(self, node: Node):
        self.node = node
        
    def declare_and_validate(self, 
                           name: str, 
                           default: Any,
                           min_val: Optional[Union[int, float]] = None,
                           max_val: Optional[Union[int, float]] = None,
                           choices: Optional[List[Any]] = None,
                           description: Optional[str] = None) -> Any:
        """
        Declare parameter with validation
        
        Args:
            name: Parameter name
            default: Default value
            min_val: Minimum allowed value (for numeric types)
            max_val: Maximum allowed value (for numeric types)
            choices: List of allowed values
            description: Parameter description for logging
            
        Returns:
            Validated parameter value
            
        Raises:
            ValueError: If validation fails
        """
        try:
            # Declare parameter with default
            self.node.declare_parameter(name, default)
            value = self.node.get_parameter(name).value
            
            # Type validation
            if not isinstance(value, type(default)):
                self.node.get_logger().warn(
                    f"Parameter '{name}' type mismatch: expected {type(default).__name__}, "
                    f"got {type(value).__name__}. Using default: {default}"
                )
                return default
            
            # Range validation for numeric types
            if isinstance(value, (int, float)):
                if min_val is not None and value < min_val:
                    raise ValueError(f"Parameter '{name}' value {value} is below minimum {min_val}")
                if max_val is not None and value > max_val:
                    raise ValueError(f"Parameter '{name}' value {value} is above maximum {max_val}")
            
            # Choice validation
            if choices is not None and value not in choices:
                raise ValueError(f"Parameter '{name}' value '{value}' not in allowed choices: {choices}")
            
            # Log successful validation
            if description:
                self.node.get_logger().info(f"✅ {description}: {value}")
            else:
                self.node.get_logger().debug(f"✅ Parameter '{name}': {value}")
                
            return value
            
        except Exception as e:
            self.node.get_logger().error(f"❌ Parameter validation failed for '{name}': {e}")
            self.node.get_logger().warn(f"Using default value: {default}")
            return default
    
    def validate_duration_pair(self, min_name: str, max_name: str, 
                             min_default: float, max_default: float) -> tuple:
        """Validate min/max duration pair ensuring min < max"""
        min_val = self.declare_and_validate(min_name, min_default, min_val=1.0, max_val=1000.0)
        max_val = self.declare_and_validate(max_name, max_default, min_val=1.0, max_val=1000.0)
        
        if min_val >= max_val:
            self.node.get_logger().error(
                f"❌ {min_name} ({min_val}) must be < {max_name} ({max_val})"
            )
            self.node.get_logger().warn(f"Using defaults: {min_default}, {max_default}")
            return min_default, max_default
            
        return min_val, max_val


class ConfigValidator:
    """Validate entire configuration groups"""
    
    @staticmethod
    def validate_photo_intelligence_config(validator: ParameterValidator) -> dict:
        """Validate photo intelligence configuration with proper constraints"""
        
        # Basic timing parameters
        min_duration, max_duration = validator.validate_duration_pair(
            'min_duration', 'max_duration', 30.0, 120.0
        )
        
        base_duration = validator.declare_and_validate(
            'base_duration', 60.0, min_val=min_duration, max_val=max_duration,
            description="Base photo duration"
        )
        
        evaluation_time = validator.declare_and_validate(
            'evaluation_time', 3.0, min_val=0.5, max_val=10.0,
            description="Crowd evaluation time"
        )
        
        cooldown_time = validator.declare_and_validate(
            'cooldown_time', 10.0, min_val=1.0, max_val=60.0,
            description="Cooldown between sessions"
        )
        
        # Group size timing
        single_duration = validator.declare_and_validate(
            'single_person_duration', 35.0, min_val=15.0, max_val=120.0,
            description="Single person photo duration"
        )
        
        small_group_duration = validator.declare_and_validate(
            'small_group_duration', 50.0, min_val=20.0, max_val=120.0,
            description="Small group photo duration"
        )
        
        # Formation bonuses
        circle_bonus = validator.declare_and_validate(
            'circle_formation_bonus', 10.0, min_val=0.0, max_val=30.0,
            description="Circle formation time bonus"
        )
        
        # Multi-shot settings
        enable_multi_shot = validator.declare_and_validate(
            'enable_multi_shot', False,
            description="Enable multi-shot sequences"
        )
        
        multi_shot_threshold = validator.declare_and_validate(
            'multi_shot_threshold', 8, min_val=2, max_val=20,
            description="Group size threshold for multi-shot"
        )
        
        return {
            'min_duration': min_duration,
            'max_duration': max_duration, 
            'base_duration': base_duration,
            'evaluation_time': evaluation_time,
            'cooldown_time': cooldown_time,
            'single_person_duration': single_duration,
            'small_group_duration': small_group_duration,
            'circle_formation_bonus': circle_bonus,
            'enable_multi_shot': enable_multi_shot,
            'multi_shot_threshold': multi_shot_threshold
        }
    
    @staticmethod
    def validate_mission_config(validator: ParameterValidator) -> dict:
        """Validate mission controller configuration"""
        
        evaluation_time = validator.declare_and_validate(
            'evaluation_time', 2.0, min_val=0.5, max_val=10.0,
            description="Target evaluation time"
        )
        
        stability_wait_time = validator.declare_and_validate(
            'stability_wait_time', 3.0, min_val=1.0, max_val=10.0,
            description="Wait time for position stability"
        )
        
        arrival_threshold = validator.declare_and_validate(
            'arrival_threshold', 1.0, min_val=0.1, max_val=5.0,
            description="Distance threshold for arrival detection"
        )
        
        return {
            'evaluation_time': evaluation_time,
            'stability_wait_time': stability_wait_time,
            'arrival_threshold': arrival_threshold
        }