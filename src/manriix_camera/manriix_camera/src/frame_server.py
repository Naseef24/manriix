import socket
import struct
import numpy as np
import cv2
import threading 
import time 
from manriix_camera.src.logger import logger as log

class FrameServer:
    """Socket-based frame server for streaming camera frames to clients"""
    
    def __init__(self, host='127.0.0.1', port=8089):
        self.host = host
        self.port = port
        self.server_socket = None
        self.connections = []
        self.running = False
        self.latest_frame = None
        self.frame_lock = threading.Lock()
        
    def start(self):
        """Start the frame server"""
        try:
            self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.server_socket.bind((self.host, self.port))
            self.server_socket.listen(5)
            self.running = True
            
            # Start connection acceptor thread
            acceptor_thread = threading.Thread(target=self._accept_connections)
            acceptor_thread.daemon = True
            acceptor_thread.start()
            
            # Start frame sender thread
            sender_thread = threading.Thread(target=self._send_frames)
            sender_thread.daemon = True
            sender_thread.start()
            
            log.info(f"Frame server started on {self.host}:{self.port}")
            return True
        except Exception as e:
            log.error(f"Error starting frame server: {e}")
            return False
    
    def _accept_connections(self):
        """Accept incoming client connections"""
        while self.running:
            try:
                client_socket, addr = self.server_socket.accept()
                log.info(f"New client connection from {addr}")
                client_socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                self.connections.append(client_socket)
            except Exception as e:
                if self.running:  # Only log if we're still supposed to be running
                    log.error(f"Error accepting connection: {e}")
                time.sleep(0.1)
    
    def _send_frames(self):
        """Send frames to all connected clients"""
        while self.running:
            try:
                # If no connections or no frame, wait
                if not self.connections or self.latest_frame is None:
                    time.sleep(0.01)
                    continue
                
                # Get the latest frame
                with self.frame_lock:
                    frame = self.latest_frame
                
                # Compress frame as JPEG
                encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 90]
                _, jpeg_frame = cv2.imencode('.jpg', frame, encode_param)
                data = jpeg_frame.tobytes()
                
                # Create message with size header
                message = struct.pack('<L', len(data)) + data
                
                # Send to all clients
                disconnected = []
                for client_socket in self.connections:
                    try:
                        client_socket.sendall(message)
                    except Exception as e:
                        log.warning(f"Error sending to client: {e}")
                        disconnected.append(client_socket)
                
                # Remove disconnected clients
                for client in disconnected:
                    try:
                        client.close()
                    except:
                        pass
                    if client in self.connections:
                        self.connections.remove(client)
                
                # Brief sleep to control rate
                time.sleep(0.01)
                
            except Exception as e:
                log.error(f"Error in frame sender: {e}")
                time.sleep(0.1)
    
    def frame_callback(self, frame):
        """Callback to receive frames from video manager"""
        with self.frame_lock:
            self.latest_frame = frame
    
    def stop(self):
        """Stop the frame server"""
        self.running = False
        
        # Close client connections
        for client in self.connections:
            try:
                client.close()
            except:
                pass
        self.connections = []
        
        # Close server socket
        if self.server_socket:
            try:
                self.server_socket.close()
            except:
                pass
        
        log.info("Frame server stopped")