#!/usr/bin/env python3
"""
Enhanced Logger Setup Utility with Structured Colored Logging

Provides consistent, colorized logging configuration for the photographer robot system.
Supports structured logging with context fields and emoji indicators.
"""

import logging
import sys
import os
from typing import Optional, Dict, Any, Union
from rclpy.node import Node
import json
from datetime import datetime


class ColorCodes:
    """ANSI color codes for terminal output"""
    # Standard colors
    BLACK = '\033[30m'
    RED = '\033[31m'
    GREEN = '\033[32m'
    YELLOW = '\033[33m'
    BLUE = '\033[34m'
    MAGENTA = '\033[35m'
    CYAN = '\033[36m'
    WHITE = '\033[37m'
    
    # Bright colors
    BRIGHT_BLACK = '\033[90m'
    BRIGHT_RED = '\033[91m'
    BRIGHT_GREEN = '\033[92m'
    BRIGHT_YELLOW = '\033[93m'
    BRIGHT_BLUE = '\033[94m'
    BRIGHT_MAGENTA = '\033[95m'
    BRIGHT_CYAN = '\033[96m'
    BRIGHT_WHITE = '\033[97m'
    
    # Background colors
    BG_BLACK = '\033[40m'
    BG_RED = '\033[41m'
    BG_GREEN = '\033[42m'
    BG_YELLOW = '\033[43m'
    BG_BLUE = '\033[44m'
    BG_MAGENTA = '\033[45m'
    BG_CYAN = '\033[46m'
    BG_WHITE = '\033[47m'
    
    # Styles
    BOLD = '\033[1m'
    DIM = '\033[2m'
    ITALIC = '\033[3m'
    UNDERLINE = '\033[4m'
    BLINK = '\033[5m'
    REVERSE = '\033[7m'
    
    # Reset
    RESET = '\033[0m'
    
    @staticmethod
    def is_color_supported():
        """Check if the terminal supports color output"""
        # Check for explicit color forcing environment variable
        force_color = os.getenv('FORCE_COLOR', '').lower() in ('1', 'true', 'yes', 'on')
        if force_color:
            return True
            
        # Check for explicit color disabling
        no_color = os.getenv('NO_COLOR')
        if no_color:
            return False
            
        # Check if terminal supports color
        term = os.getenv('TERM', '')
        colorterm = os.getenv('COLORTERM', '')
        
        # Modern terminals with good color support
        if 'color' in term.lower() or colorterm:
            return True
            
        # Traditional isatty check (may fail in ROS2 launch environment)
        if hasattr(sys.stdout, 'isatty') and sys.stdout.isatty() and term != 'dumb':
            return True
            
        # Fallback for common terminal types even without isatty
        good_terminals = ['xterm', 'screen', 'tmux', 'rxvt', 'ansi']
        if any(good_term in term.lower() for good_term in good_terminals):
            return True
            
        return False


class StructuredColoredFormatter(logging.Formatter):
    """Custom formatter with colors, emojis, and structured fields"""
    
    # Level-specific configurations
    LEVEL_CONFIGS = {
        logging.DEBUG: {
            'color': ColorCodes.BRIGHT_BLACK,
            'emoji': '🔍',
            'prefix': 'DEBUG'
        },
        logging.INFO: {
            'color': ColorCodes.BRIGHT_BLUE,
            'emoji': 'ℹ️ ',
            'prefix': 'INFO '
        },
        logging.WARNING: {
            'color': ColorCodes.BRIGHT_YELLOW,
            'emoji': '⚠️ ',
            'prefix': 'WARN '
        },
        logging.ERROR: {
            'color': ColorCodes.BRIGHT_RED,
            'emoji': '❌',
            'prefix': 'ERROR'
        },
        logging.CRITICAL: {
            'color': ColorCodes.RED + ColorCodes.BOLD,
            'emoji': '💥',
            'prefix': 'FATAL'
        }
    }
    
    def __init__(self, use_colors=None, use_emojis=True, structured=True):
        """
        Initialize the colored formatter
        
        Args:
            use_colors: Enable color output (auto-detect if None)
            use_emojis: Enable emoji indicators
            structured: Enable structured logging format
        """
        super().__init__()
        
        self.use_colors = ColorCodes.is_color_supported() if use_colors is None else use_colors
        self.use_emojis = use_emojis
        self.structured = structured
    
    def format(self, record):
        """Format log record with colors and structure"""
        
        # Get level configuration
        level_config = self.LEVEL_CONFIGS.get(record.levelno, self.LEVEL_CONFIGS[logging.INFO])
        
        # Format timestamp
        timestamp = datetime.fromtimestamp(record.created).strftime('%H:%M:%S.%f')[:-3]
        
        # Format node name with color
        node_name = getattr(record, 'node_name', record.name)
        if self.use_colors:
            colored_node = f"{ColorCodes.CYAN}{node_name}{ColorCodes.RESET}"
        else:
            colored_node = node_name
        
        # Format level with color and emoji
        if self.use_colors:
            level_str = f"{level_config['color']}{level_config['prefix']}{ColorCodes.RESET}"
        else:
            level_str = level_config['prefix']
            
        if self.use_emojis:
            level_str = f"{level_config['emoji']} {level_str}"
        
        # Format message
        message = record.getMessage()
        
        # Add structured fields if available
        structured_fields = ""
        if self.structured and hasattr(record, 'extra_fields'):
            extra = record.extra_fields
            if extra:
                fields_str = " ".join([f"{k}={v}" for k, v in extra.items()])
                if self.use_colors:
                    structured_fields = f" {ColorCodes.DIM}[{fields_str}]{ColorCodes.RESET}"
                else:
                    structured_fields = f" [{fields_str}]"
        
        # Format final log line
        if self.structured:
            log_line = f"{ColorCodes.DIM if self.use_colors else ''}{timestamp}{ColorCodes.RESET if self.use_colors else ''} {level_str} {colored_node}: {message}{structured_fields}"
        else:
            log_line = f"[{timestamp}] {level_str} [{node_name}] {message}{structured_fields}"
        
        return log_line


class StructuredLogger:
    """Enhanced logger wrapper with structured logging capabilities"""
    
    def __init__(self, logger: logging.Logger):
        self.logger = logger
        
    def debug(self, message: str, **kwargs):
        """Log debug message with optional structured fields"""
        self._log(logging.DEBUG, message, kwargs)
        
    def info(self, message: str, **kwargs):
        """Log info message with optional structured fields"""
        self._log(logging.INFO, message, kwargs)
        
    def warning(self, message: str, **kwargs):
        """Log warning message with optional structured fields"""
        self._log(logging.WARNING, message, kwargs)
        
    def error(self, message: str, **kwargs):
        """Log error message with optional structured fields"""
        self._log(logging.ERROR, message, kwargs)
        
    def critical(self, message: str, **kwargs):
        """Log critical message with optional structured fields"""
        self._log(logging.CRITICAL, message, kwargs)
    
    def _log(self, level: int, message: str, extra_fields: Dict[str, Any]):
        """Internal logging method with structured field support"""
        record = self.logger.makeRecord(
            self.logger.name, level, "", 0, message, (), None
        )
        
        # Add extra fields for structured logging
        if extra_fields:
            record.extra_fields = extra_fields
        
        self.logger.handle(record)
        
    def performance(self, operation: str, duration_ms: float, **kwargs):
        """Log performance metrics"""
        self.info(f"Performance: {operation}", 
                 duration_ms=f"{duration_ms:.2f}ms", **kwargs)
                 
    def camera_status(self, camera_name: str, status: str, **kwargs):
        """Log camera status updates"""
        emoji = "🟢" if status == "ACTIVE" else "🔴"
        self.info(f"{emoji} Camera {camera_name}: {status}", 
                 camera=camera_name, **kwargs)
    
    def integration(self, source: str, target: str, topic: str, **kwargs):
        """Log integration status"""
        self.info(f"🔗 Integration: {source} → {target}", 
                 topic=topic, **kwargs)
                 
    def recovery(self, level: int, action: str, **kwargs):
        """Log recovery actions"""
        emoji = "🛡️" if level < 4 else "🚨"
        self.info(f"{emoji} Recovery L{level}: {action}", 
                 recovery_level=level, **kwargs)


class LoggerSetup:
    """Enhanced utility class for setting up colored, structured logging"""
    
    LOG_LEVELS = {
        'debug': logging.DEBUG,
        'info': logging.INFO,
        'warning': logging.WARNING,
        'error': logging.ERROR,
        'critical': logging.CRITICAL
    }
    
    @staticmethod
    def setup_logger(name: str, level: str = 'info', 
                    include_console: bool = True, 
                    use_colors: Optional[bool] = None,
                    use_emojis: bool = True,
                    structured: bool = True) -> logging.Logger:
        """
        Set up a logger with enhanced colored formatting
        
        Args:
            name: Logger name
            level: Logging level ('debug', 'info', 'warning', 'error', 'critical')
            include_console: Whether to include console output
            use_colors: Enable color output (auto-detect if None)
            use_emojis: Enable emoji indicators
            structured: Enable structured logging format
            
        Returns:
            Configured logger instance
        """
        logger = logging.getLogger(name)
        
        # Avoid adding handlers multiple times
        if logger.handlers:
            return logger
        
        # Set log level
        log_level = LoggerSetup.LOG_LEVELS.get(level.lower(), logging.INFO)
        logger.setLevel(log_level)
        
        # Create enhanced colored formatter
        formatter = StructuredColoredFormatter(
            use_colors=use_colors,
            use_emojis=use_emojis,
            structured=structured
        )
        
        # Console handler
        if include_console:
            console_handler = logging.StreamHandler(sys.stdout)
            console_handler.setLevel(log_level)
            console_handler.setFormatter(formatter)
            logger.addHandler(console_handler)
        
        return logger
    
    @staticmethod
    def setup_structured_logger(name: str, level: str = 'info', **kwargs) -> StructuredLogger:
        """
        Set up a structured logger with enhanced capabilities
        
        Args:
            name: Logger name
            level: Logging level
            **kwargs: Additional arguments for setup_logger
            
        Returns:
            StructuredLogger instance with enhanced methods
        """
        base_logger = LoggerSetup.setup_logger(name, level, **kwargs)
        return StructuredLogger(base_logger)
    
    @staticmethod
    def get_node_logger(node: Node, 
                       log_config: Optional[Dict[str, Any]] = None) -> StructuredLogger:
        """
        Get an enhanced structured logger for a ROS2 node
        
        Args:
            node: ROS2 node instance
            log_config: Optional logging configuration dictionary
            
        Returns:
            StructuredLogger instance with enhanced capabilities
        """
        node_name = node.get_name()
        
        if log_config:
            level = log_config.get('level', 'info')
            include_console = log_config.get('console_output', True)
            use_colors = log_config.get('colors', None)
            use_emojis = log_config.get('emojis', True)
            structured = log_config.get('structured', True)
            
            # Handle force_colors override
            if log_config.get('force_colors', False):
                use_colors = True
        else:
            level = 'info'
            include_console = True
            use_colors = None
            use_emojis = True
            structured = True
        
        return LoggerSetup.setup_structured_logger(
            node_name, level, 
            include_console=include_console,
            use_colors=use_colors,
            use_emojis=use_emojis,
            structured=structured
        )
    
    @staticmethod
    def get_node_logger_legacy(node: Node, 
                             log_config: Optional[Dict[str, Any]] = None) -> logging.Logger:
        """
        Get a legacy logger for backwards compatibility
        
        Args:
            node: ROS2 node instance
            log_config: Optional logging configuration dictionary
            
        Returns:
            Standard logging.Logger instance
        """
        node_name = node.get_name()
        
        if log_config:
            level = log_config.get('level', 'info')
            include_console = log_config.get('console_output', True)
        else:
            level = 'info'
            include_console = True
        
        return LoggerSetup.setup_logger(node_name, level, include_console)
    
    @staticmethod
    def log_system_info(logger: Union[logging.Logger, StructuredLogger], config: Dict[str, Any]):
        """
        Log system configuration information with enhanced formatting
        
        Args:
            logger: Logger instance (standard or structured)
            config: System configuration dictionary
        """
        # Check if it's a structured logger
        is_structured = isinstance(logger, StructuredLogger)
        
        if is_structured:
            logger.info("🚀 Photographer Robot System Starting", system="photographer_robot")
            logger.info("=" * 50)
        else:
            logger.info("Photographer Robot System Starting")
            logger.info("=" * 50)
        
        # Log camera configuration
        if 'cameras' in config:
            cameras = config['cameras']
            active_cameras = [name for name, cfg in cameras.items() if cfg.get('enabled', True)]
            
            if is_structured:
                logger.info(f"📷 Active cameras: {', '.join(active_cameras)}", 
                           camera_count=len(active_cameras))
                
                for camera_name, camera_config in cameras.items():
                    if camera_config.get('enabled', True):
                        topic = camera_config.get('topic', 'unknown')
                        confidence = camera_config.get('confidence_threshold', 60.0)
                        logger.camera_status(camera_name, "CONFIGURED", 
                                           topic=topic, confidence=confidence)
            else:
                logger.info(f"Active cameras: {', '.join(active_cameras)}")
                for camera_name, camera_config in cameras.items():
                    if camera_config.get('enabled', True):
                        topic = camera_config.get('topic', 'unknown')
                        logger.info(f"  {camera_name}: {topic}")
        
        # Log detection configuration
        if 'detection' in config:
            detection = config['detection']
            min_conf = detection.get('min_detection_confidence', 50.0)
            max_range = detection.get('max_detection_range', 8.0)
            tracking = detection.get('enable_tracking', True)
            
            if is_structured:
                logger.info(f"🎯 Detection configured", 
                           min_confidence=f"{min_conf}%", 
                           max_range=f"{max_range}m",
                           tracking_enabled=tracking)
            else:
                logger.info(f"Detection: conf>={min_conf}%, range<={max_range}m, tracking={tracking}")
        
        # Log output configuration
        if 'output' in config:
            output = config['output']
            humans_topic = output.get('humans_list', {}).get('topic', '/human_clustering/humans')
            debug_enabled = output.get('debug', {}).get('enabled', False)
            
            if is_structured:
                logger.integration("ZedFusion", "HumanClustering", humans_topic,
                                 debug_enabled=debug_enabled)
            else:
                logger.info(f"Output: {humans_topic}, debug={debug_enabled}")
        
        if is_structured:
            logger.info("=" * 50)
        else:
            logger.info("=" * 50)
    
    @staticmethod
    def log_performance_metrics(logger: Union[logging.Logger, StructuredLogger], metrics: Dict[str, Any]):
        """
        Log performance metrics with enhanced formatting
        
        Args:
            logger: Logger instance (standard or structured)
            metrics: Performance metrics dictionary
        """
        is_structured = isinstance(logger, StructuredLogger)
        
        if is_structured:
            # Use structured logging for performance metrics
            for metric_name, metric_value in metrics.items():
                if isinstance(metric_value, float):
                    if 'rate' in metric_name.lower() or 'hz' in metric_name.lower():
                        logger.performance(metric_name, metric_value, unit="Hz")
                    elif 'time' in metric_name.lower() or 'latency' in metric_name.lower():
                        logger.performance(metric_name, metric_value, unit="ms")
                    elif 'percentage' in metric_name.lower() or 'efficiency' in metric_name.lower():
                        logger.performance(metric_name, metric_value, unit="%")
                    else:
                        logger.performance(metric_name, metric_value)
                else:
                    logger.info(f"📊 {metric_name}: {metric_value}")
        else:
            # Legacy logging format
            logger.info("Performance Metrics:")
            for metric_name, metric_value in metrics.items():
                if isinstance(metric_value, float):
                    if 'rate' in metric_name.lower() or 'hz' in metric_name.lower():
                        logger.info(f"  {metric_name}: {metric_value:.1f} Hz")
                    elif 'time' in metric_name.lower() or 'latency' in metric_name.lower():
                        logger.info(f"  {metric_name}: {metric_value:.1f} ms")
                    elif 'percentage' in metric_name.lower():
                        logger.info(f"  {metric_name}: {metric_value:.1f}%")
                    else:
                        logger.info(f"  {metric_name}: {metric_value:.2f}")
                else:
                    logger.info(f"  {metric_name}: {metric_value}")
    
    @staticmethod
    def log_camera_status(logger: Union[logging.Logger, StructuredLogger], camera_status: Dict[str, Any]):
        """
        Log camera status information with enhanced formatting
        
        Args:
            logger: Logger instance (standard or structured)
            camera_status: Camera status dictionary
        """
        is_structured = isinstance(logger, StructuredLogger)
        
        if not is_structured:
            logger.info("Camera Status:")
        
        for camera_name, status in camera_status.items():
            if isinstance(status, dict):
                active = status.get('active', False)
                last_detection = status.get('last_detection_time', 'never')
                detection_count = status.get('detection_count', 0)
                avg_confidence = status.get('average_confidence', 0.0)
                
                status_str = "ACTIVE" if active else "INACTIVE"
                
                if is_structured:
                    logger.camera_status(camera_name, status_str,
                                       detections=detection_count,
                                       avg_confidence=f"{avg_confidence:.1f}%",
                                       last_detection=last_detection)
                else:
                    logger.info(f"  {camera_name}: {status_str} | "
                               f"detections={detection_count} | "
                               f"avg_conf={avg_confidence:.1f}% | "
                               f"last={last_detection}")
            else:
                if is_structured:
                    logger.camera_status(camera_name, str(status))
                else:
                    logger.info(f"  {camera_name}: {status}")
    
    @staticmethod
    def log_error_with_context(logger: Union[logging.Logger, StructuredLogger], error: Exception, 
                              context: str, additional_info: Optional[Dict[str, Any]] = None):
        """
        Log an error with contextual information and enhanced formatting
        
        Args:
            logger: Logger instance (standard or structured)
            error: Exception that occurred
            context: Context description (e.g., "camera processing", "fusion")
            additional_info: Optional additional information dictionary
        """
        is_structured = isinstance(logger, StructuredLogger)
        
        error_type = type(error).__name__
        error_msg = str(error)
        
        if is_structured:
            # Structured error logging with context fields
            error_context = {
                'error_type': error_type,
                'context': context,
            }
            if additional_info:
                error_context.update(additional_info)
            
            logger.error(f"💥 Error in {context}: {error_type}: {error_msg}", **error_context)
        else:
            # Legacy error logging
            logger.error(f"Error in {context}: {error_type}: {error_msg}")
            if additional_info:
                for key, value in additional_info.items():
                    logger.error(f"  {key}: {value}")
    
    @staticmethod
    def create_debug_logger(name: str, structured: bool = True) -> Union[logging.Logger, StructuredLogger]:
        """
        Create a debug-level logger for development with enhanced features
        
        Args:
            name: Logger name
            structured: Whether to return a StructuredLogger instance
            
        Returns:
            Debug logger instance (structured or standard)
        """
        if structured:
            return LoggerSetup.setup_structured_logger(f"{name}_debug", "debug", True)
        else:
            return LoggerSetup.setup_logger(f"{name}_debug", "debug", True)
    
    @staticmethod
    def configure_global_logging(level: str = "info", 
                               colors: Optional[bool] = None,
                               emojis: bool = True,
                               structured: bool = True):
        """
        Configure global logging settings for the entire system
        
        Args:
            level: Global logging level
            colors: Enable colors (auto-detect if None)
            emojis: Enable emoji indicators
            structured: Enable structured logging format
        """
        # Set root logger configuration
        root_logger = logging.getLogger()
        root_logger.setLevel(LoggerSetup.LOG_LEVELS.get(level.lower(), logging.INFO))
        
        # Clear existing handlers
        for handler in root_logger.handlers[:]:
            root_logger.removeHandler(handler)
        
        # Add new colored handler
        formatter = StructuredColoredFormatter(
            use_colors=colors,
            use_emojis=emojis, 
            structured=structured
        )
        
        console_handler = logging.StreamHandler(sys.stdout)
        console_handler.setFormatter(formatter)
        console_handler.setLevel(LoggerSetup.LOG_LEVELS.get(level.lower(), logging.INFO))
        
        root_logger.addHandler(console_handler)
        
        # Prevent duplicate messages
        root_logger.propagate = False
        
        print(f"🎨 Enhanced colored logging configured: level={level.upper()}, colors={ColorCodes.is_color_supported() if colors is None else colors}, emojis={emojis}")


# Quick access functions for convenience
def get_logger(name: str, **kwargs) -> StructuredLogger:
    """Convenience function to get a structured logger"""
    return LoggerSetup.setup_structured_logger(name, **kwargs)

def get_node_logger(node: Node, **kwargs) -> StructuredLogger:
    """Convenience function to get a structured logger for ROS2 nodes"""
    return LoggerSetup.get_node_logger(node, **kwargs)