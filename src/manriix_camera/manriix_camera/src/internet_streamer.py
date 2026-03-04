import threading 
import time 
import os 
import json 
from flask import Flask, Response, request, session, redirect, url_for, render_template 
from flask_login import LoginManager, UserMixin, login_user, login_required, logout_user, current_user
from werkzeug.security import generate_password_hash, check_password_hash
import secrets
import cv2
import numpy as np
from manriix_camera.src.logger import logger as log
from manriix_camera.utils.config_loader import get_config

# User class for authentication
class User(UserMixin):
    def __init__(self, id, username, password_hash):
        self.id = id
        self.username = username
        self.password_hash = password_hash

class InternetStreamer:
    """Singleton Flask-based authenticated video streamer for internet access"""
    _instance = None
    _lock = threading.Lock()

    def __new__(cls, *args, **kwargs):
        with cls._lock:
            if cls._instance is None:
                cls._instance = super(InternetStreamer, cls).__new__(cls)
        return cls._instance
    
    def __init__(self, width=1280, height=720, framerate=15, port=5000, config_path=None):
        if not hasattr(self, "initialized"):
            config = get_config()
            streaming_config = config.get('streaming', {})
            internet_config = streaming_config.get('internet', {})

            self.width = width or internet_config.get('frame_width', 1280)
            self.height = height or internet_config.get('frame_height', 720)
            self.framerate = framerate or internet_config.get('framerate', 15)
            self.port = port or internet_config.get('port', 5000)
            self.jpeg_quality = config.get('streaming', {}).get('jpeg_quality', 70)
            self.is_running = False
            self.flask_thread = None
            self.frame_lock = threading.Lock()
            self.current_frame = None

            # Authentication setup
            self.config_path = config_path or os.path.join(
                os.path.dirname(os.path.abspath(__file__)), '..', 'config', 'users.json')
            self.users = {}
            self.secret_key = secrets.token_hex(16)
            
            # Initialize Flask app
            self.app = Flask(
                __name__, 
                template_folder=os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'templates'),
                static_folder=os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'static')
            )
            self.app.config['SECRET_KEY'] = self.secret_key
            
            # Initialize Flask-Login
            self.login_manager = LoginManager()
            self.login_manager.init_app(self.app)
            self.login_manager.login_view = 'login'
            
            # Load users from config
            self.load_users()
            
            # Setup Flask routes
            self.setup_routes()
            
            self.initialized = True
    
    def load_users(self):
        """Load users from the configuration file"""
        try:
            # Create config directory if it doesn't exist
            os.makedirs(os.path.dirname(self.config_path), exist_ok=True)
            
            # If config file doesn't exist, create it with a default admin user
            if not os.path.exists(self.config_path):
                default_password = secrets.token_urlsafe(8)  # Generate a random password
                self.users = {
                    "admin": {
                        "id": "1",
                        "username": "admin",
                        "password_hash": generate_password_hash("admin123")
                    }
                }
                
                # Save the default user configuration
                with open(self.config_path, 'w') as f:
                    json.dump({"users": self.users}, f, indent=4)
                
                log.info("Created default admin user with password: admin123")
            else:
                # Load existing user configuration
                with open(self.config_path, 'r') as f:
                    data = json.load(f)
                    self.users = data.get("users", {})
                
                log.info(f"Loaded {len(self.users)} users from configuration")
        except Exception as e:
            log.error(f"Error loading users: {e}")
            # Create a default admin user in memory if loading fails
            self.users = {
                "admin": {
                    "id": "1",
                    "username": "admin",
                    "password_hash": generate_password_hash("admin123")
                }
            }
    
    def save_users(self):
        """Save users to the configuration file"""
        try:
            with open(self.config_path, 'w') as f:
                json.dump({"users": self.users}, f, indent=4)
            log.info(f"Saved {len(self.users)} users to configuration")
        except Exception as e:
            log.error(f"Error saving users: {e}")
    
    def add_user(self, username, password):
        """Add a new user"""
        if username in self.users:
            return False, "Username already exists"
        
        user_id = str(len(self.users) + 1)
        self.users[username] = {
            "id": user_id,
            "username": username,
            "password_hash": generate_password_hash(password)
        }
        
        # Save the updated user configuration
        self.save_users()
        return True, f"User {username} added successfully"
    
    def setup_routes(self):
        """Setup Flask routes for authenticated streaming"""
        login_manager = self.login_manager
        
        @login_manager.user_loader
        def load_user(user_id):
            for username, user_data in self.users.items():
                if user_data.get("id") == user_id:
                    return User(
                        user_data.get("id"),
                        username,
                        user_data.get("password_hash")
                    )
            return None
        
        @self.app.route('/')
        @login_required
        def index():
            return render_template('stream.html', 
                                   title='Camera Stream',
                                   username=current_user.username)
        
        @self.app.route('/login', methods=['GET', 'POST'])
        def login():
            error = None
            if request.method == 'POST':
                username = request.form.get('username')
                password = request.form.get('password')
                
                user_data = self.users.get(username)
                if user_data and check_password_hash(user_data.get("password_hash"), password):
                    user = User(
                        user_data.get("id"),
                        username,
                        user_data.get("password_hash")
                    )
                    login_user(user)
                    return redirect(url_for('index'))
                else:
                    error = "Invalid username or password"
            
            return render_template('login.html', error=error)
        
        @self.app.route('/logout')
        @login_required
        def logout():
            logout_user()
            return redirect(url_for('login'))
        
        @self.app.route('/admin')
        @login_required
        def admin():
            # Only allow the admin user to access this page
            if current_user.username != 'admin':
                return redirect(url_for('index'))
            return render_template('admin.html', users=self.users)
        
        @self.app.route('/admin/add_user', methods=['POST'])
        @login_required
        def add_user_route():
            # Only allow the admin user to add users
            if current_user.username != 'admin':
                return redirect(url_for('index'))
            
            username = request.form.get('username')
            password = request.form.get('password')
            
            if not username or not password:
                return "Username and password are required", 400
            
            success, message = self.add_user(username, password)
            if success:
                return redirect(url_for('admin'))
            else:
                return message, 400
        
        @self.app.route('/video_feed')
        @login_required
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
            ret, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality])
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
            
            # Wait a bit for the server to start
            time.sleep(2)
            
            log.info(f"Internet stream started. Access at http://localhost:{self.port}/")
            log.info("Default login credentials - Username: admin, Password: admin123")
            
            return True
        except Exception as e:
            log.error(f"Failed to start Internet stream server: {e}")
            self.is_running = False
            return False
            
    def _run_server(self):
        """Run the Flask server"""
        try:
            # Suppress Flask's default logging
            import logging
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
            log.info("Stopping Internet stream server...")
            self.is_running = False
            
            if self.flask_thread:
                # Note: Stopping Flask cleanly is challenging
                # The thread will terminate when the main process exits
                self.flask_thread.join(timeout=1.0)
            
            log.info("Internet stream server stopped")
        except Exception as e:
            log.error(f"Error stopping Internet stream server: {e}")