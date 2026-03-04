import cv2
import threading
import time
import logging
import socket
from flask import Flask, Response
import numpy as np
import requests
from manriix_camera.src.logger import logger as log
from manriix_camera.utils.config_loader import get_config

class FlaskStreamer:
    """Singleton Flask-based MJPEG video streamer"""
    _instance = None 
    _lock = threading.Lock()

    def __new__(cls, *args, **kwargs):
        with cls._lock:
            if cls._instance is None:
                cls._instance = super(FlaskStreamer, cls).__new__(cls)
        return cls._instance 
    
    def __init__(self, width=1920, height=1080, framerate=15, port=8080):
        if not hasattr(self, "initialized"): 
            config = get_config()
            streaming_config = config.get('streaming', {})
            local_config = streaming_config.get('local', {}) 
            self.width = width or local_config.get('frame_width', 1280)
            self.height = height or local_config.get('frame_height', 720)
            self.framerate = framerate or local_config.get('framerate', 30)
            self.port = port or local_config.get('port', 8080)
            self.is_running = False
            self.flask_thread = None
            self.app = Flask(__name__)
            self.setup_routes()
            self.frame_lock = threading.Lock()
            self.current_frame = None
            self.initialized = True

    def setup_routes(self):
        """Setup Flask routes for streaming"""
        @self.app.route('/')
        def index():
            doc = """
            <html>
              <head>
                <title>Camera Stream</title>
                <style>
                  body { font-family: Arial, sans-serif; margin: 0; padding: 20px; text-align: center; }
                  img { max-width: 100%; border: 1px solid #ddd; }
                  h1 { color: #333; }
                </style>
              </head>
              <body>
                <h1>Live Camera Stream - Local</h1>
                <img src="/video_feed" />
              </body>
            </html>
            """
            return doc

        @self.app.route('/video_feed')
        def video_feed():
            return Response(self.generate_frames(),
                        mimetype='multipart/x-mixed-replace; boundary=frame')
    
    def generate_frames(self):
        """Generate MJPEG frames for streaming"""
        while True:
            # Get current frame 
            with self.frame_lock:
                if self.current_frame is None:
                    # Blank frame if no frame available
                    frame = np.zeros((self.height, self.width, 3), dtype=np.uint8)
                    cv2.putText(frame, 'Waiting for camera...', (int(self.width/4), int(self.height/2)),
                               cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
                else:
                    frame = self.current_frame.copy()
            
            # Encode as JPEG
            ret, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
            if not ret:
                continue
                
            # Convert to bytes & yield
            frame_bytes = buffer.tobytes()
            yield (b'--frame\r\n'
                  b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
            
            # Frame rate control
            time.sleep(1.0 / self.framerate)

    def push_frame(self, frame):
        """Update the current frame to be streamed"""
        try:
            # Resize frame if needed
            if frame.shape[1] != self.width or frame.shape[0] != self.height:
                frame = cv2.resize(frame, (self.width, self.height))
            
            # Update the current frame
            with self.frame_lock:
                self.current_frame = frame.copy()
            
            return True
        except Exception as e:
            log.error(f"Error updating frame: {e}")
            return False
        
    def start(self):
        """Start the Flask server in a separate thread"""
        if self.is_running:
            return True
        
        try:
            self.is_running = True
            self.flask_thread = threading.Thread(target=self._run_server)
            self.flask_thread.daemon = True
            self.flask_thread.start()
            
            # Wait for server to start
            time.sleep(1)
            
            ip_address = self._get_ip_address()
            stream_url = f"http://{ip_address}:{self.port}/"
            log.info(f"Local HTTP stream started at {stream_url}")
            return True
        except Exception as e:
            log.error(f"Failed to start HTTP stream server: {e}")
            self.is_running = False
            return False
            
    def _run_server(self):
        """Run the Flask server"""
        try:
            # Suppress Flask's default logging
            flask_log = logging.getLogger('werkzeug')
            flask_log.setLevel(logging.ERROR)
            
            self.app.run(host='0.0.0.0', port=self.port, debug=False, threaded=True, use_reloader=False)
        except Exception as e:
            log.error(f"Flask server error: {e}")
            self.is_running = False

    def stop(self):
        """Stop the Flask server"""
        if not self.is_running:
            return
        
        try:
            log.info("Stopping HTTP stream server...")
            self.is_running = False
        
            try:
                requests.get(f"http://localhost:{self.port}/shutdown", timeout=0.1)
            except:
                pass
            
            if self.flask_thread:
                self.flask_thread.join(timeout=1.0)
            
            log.info("HTTP stream server stopped")
        except Exception as e:
            log.error(f"Error stopping HTTP stream server: {e}")
    
    def _get_ip_address(self):
        """Get the local IP address of the machine"""
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            s.connect(('10.255.255.255', 1))
            ip = s.getsockname()[0]
        except Exception:
            ip = '127.0.0.1'
        finally:
            s.close()
        return ip
