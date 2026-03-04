import logging 
import os 
import sys 
from datetime import datetime, timedelta
import shutil 
from manriix_camera.utils.config_loader import get_config

class Logger:
    """Logging system"""

    #ANSI colors 
    COLORS = {
        'INFO': '\033[92m',  # Green
        'WARNING': '\033[93m',  # Yellow
        'ERROR': '\033[91m',  # Red
        'DEBUG': '\033[94m',  # Blue
        'RESET': '\033[0m'    # Reset color
    }

    #Icons
    ICONS = {
        'INFO': '✅',
        'WARNING': '⚠️',
        'ERROR': '❌',
        'DEBUG': '🔍'
    }

    _instance = None 

    def __new__(cls, *args, **kwargs):
        if cls._instance is None:
            cls._instance = super(Logger, cls).__new__(cls)
            cls._instance._initialized = False 
        return cls._instance 
    
    def __init__(self, name='CameraControlingSystem', level=logging.INFO, log_dir = 'logs',  retention_days=60):
        if self._initialized:
            return 
        
        config = get_config()
        if log_dir is None:
            log_dir = config.get('logging', {}).get('log_dir', 'logs')
        if retention_days is None:
            retention_days = config.get('logging', {}).get('retention_days', 60)
        
        self.logger = logging.getLogger(name)
        self.logger.setLevel(level)
        self.log_dir = log_dir
        self.retention_days = retention_days
        self.node_name = name


        if self.logger.handlers:
            self.logger.handlers.clear()

        console_handler = logging.StreamHandler(sys.stdout)
        console_handler.setLevel(level)
        console_formatter = logging.Formatter('%(message)s')  # Fixed: removed the % at the end
        console_handler.setFormatter(console_formatter)
        self.logger.addHandler(console_handler)

        self.setup_file_logging(name, level)

        self.cleanup_old_logs()

        self._initialized = True 

    def setup_file_logging(self, name, level):
        """Create file logs"""
        try:
            if not os.path.exists(self.log_dir):
                os.makedirs(self.log_dir)

            today = datetime.now().strftime('%Y-%m-%d')
            day_log_dir = os.path.join(self.log_dir, today)
            if not os.path.exists(day_log_dir):
                os.makedirs(day_log_dir)

            timestamp = datetime.now().strftime('%H%M%S')
            log_file = os.path.join(day_log_dir, f"{name}_{timestamp}.log")

            # Set up file handler
            file_handler = logging.FileHandler(log_file)
            file_handler.setLevel(level)

            #plain formatter(for log file)
            file_formatter = logging.Formatter('%(asctime)s - %(levelname)s - %(message)s')
            file_handler.setFormatter(file_formatter)

            self.logger.addHandler(file_handler)
            print(f"Log file created at: {log_file}")  # Print directly instead of logging

            self.log_file_path = log_file
            return log_file
        except Exception as e:
            print(f"Error setting up log file: {str(e)}")
            return None
        
    def cleanup_old_logs(self):
        """Cleanup old logs"""
        try:
            if not os.path.exists(self.log_dir):
                return 
            
            cutoff_date = datetime.now() - timedelta(days=self.retention_days)
            cutoff_str = cutoff_date.strftime('%Y-%m-%d')  # Use string comparison instead
            
            print(f"Checking for log files older than {self.retention_days} days...")
            
            deleted_dirs = 0
            for date_dir in os.listdir(self.log_dir):
                dir_path = os.path.join(self.log_dir, date_dir)
                
                if not os.path.isdir(dir_path):
                    continue
                    
                try:
                    # Only process directories with YYYY-MM-DD format
                    if len(date_dir) == 10 and date_dir.count('-') == 2:
                        # Simple string comparison works for YYYY-MM-DD format
                        if date_dir < cutoff_str:
                            shutil.rmtree(dir_path)
                            deleted_dirs += 1
                except Exception:
                    continue

            if deleted_dirs > 0:
                print(f"Cleaned up {deleted_dirs} directories with logs older than {self.retention_days} days.")
                
        except Exception as e:
            print(f"Error cleaning up old logs: {str(e)}")
        
    def format_message(self, level, message):
        icon = self.ICONS.get(level, '')
        color = self.COLORS.get(level, '')
        reset = self.COLORS['RESET']
        timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
        return f"{color}[{level}]-[{timestamp}] [{self.node_name}]: {icon} {message}{reset}"
    
    def info(self, message):
        formatted = self.format_message('INFO', message)
        self.logger.info(formatted)

    def warning(self, message):
        formatted = self.format_message('WARNING', message)
        self.logger.warning(formatted)
    
    def error(self, message):
        formatted = self.format_message('ERROR', message)
        self.logger.error(formatted)

    def debug(self, message):
        formatted = self.format_message('DEBUG', message)
        self.logger.debug(formatted)

    def progress(self, message, current, total):
        percent = int((current / total) * 20)  # 20 progress marks
        bar = '█' * percent + ' ' * (20 - percent)
        percent_display = int((current / total) * 100)
        print(f"\r{message}: [{bar}] {percent_display}%", end='', flush=True)
        
        if current >= total:
            print()

    def get_log_file_path(self):
        return getattr(self, 'log_file_path', None)

    def manual_cleanup(self, days=None):
        if days is not None:
            old_days = self.retention_days
            self.retention_days = days
            self.cleanup_old_logs()
            self.retention_days = old_days
        else:
            self.cleanup_old_logs()


logger = Logger()

def get_logger(name=None, level=None, log_dir=None, retention_days=None):
    """Get the global logger instance with optional reconfiguration."""
    global logger
    
    # Only update parameters that are provided
    if name is not None or level is not None or log_dir is not None or retention_days is not None:
        kwargs = {}
        if name is not None:
            kwargs['name'] = name
        if level is not None:
            kwargs['level'] = level
        if log_dir is not None:
            kwargs['log_dir'] = log_dir
        if retention_days is not None:
            kwargs['retention_days'] = retention_days
            
        logger = Logger(**kwargs)
        
    return logger

def get_log_file_path():
    """Convenience function to get the current log file path."""
    return logger.get_log_file_path()

def cleanup_old_logs(days=None):
    """Convenience function to manually trigger cleanup of old logs."""
    logger.manual_cleanup(days)