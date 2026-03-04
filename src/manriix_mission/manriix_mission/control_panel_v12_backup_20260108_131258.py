#!/usr/bin/env python3
"""
Manriix Web Control Panel v12.1 - Multi-Camera Object Detection
- Object detection from all 3 cameras (front, left, right)
- Per-camera detection stats
- Navigation modes with 2D/3D perception
- Bounding box overlays on camera feeds
- Motor control panel
"""

from flask import Flask, render_template_string, jsonify, request, Response, send_from_directory
import subprocess
import threading
import time
import signal
import os
import glob
import numpy as np
import cv2

app = Flask(__name__)

MAPS_DIR = os.path.expanduser('~/manriix2_ws/src/manriix_navigation/maps')
ASSETS_DIR = os.path.join(os.path.dirname(__file__), 'web_assets')

class NodeManager:
    def __init__(self):
        self.processes = {}
        self.logs = {}
        self.max_log_lines = 500
        self.nav_mode = 'slam_nav'
        self.nav_perception = '2d'
        self.nav_map = ''
        
        self.node_configs = {
            'rosbridge': {
                'name': '🌐 ROSBridge',
                'description': 'WebSocket bridge',
                'command': ['ros2', 'launch', 'rosbridge_server', 'rosbridge_websocket_launch.xml'],
                'color': '#00bfff',
                'order': 0,
                'cleanup_patterns': ['rosbridge', 'rosapi']
            },
            'robot': {
                'name': '🤖 Robot + Sensors',
                'description': 'Hardware, ZED cameras, RPLidar, EKF',
                'command': ['ros2', 'launch', 'manriix_bringup', 'robot.launch.py',
                           'hardware_mode:=real', 'launch_sensors:=true', 
                           'launch_teleop:=true', 'launch_ekf:=true'],
                'color': '#00ff88',
                'order': 1,
                'cleanup_patterns': ['rplidar', 'zed', 'robot_state', 'controller', 'ekf', 'spawner']
            },
            'navigation': {
                'name': '🗺️ Navigation',
                'description': 'SLAM / Localization / Exploration',
                'command': None,
                'color': '#ffcc00',
                'order': 2,
                'cleanup_patterns': ['slam_toolbox', 'nav2', 'bt_navigator', 'controller_server', 
                                   'planner_server', 'behavior', 'smoother', 'waypoint', 
                                   'lifecycle', 'rviz', 'amcl', 'map_server']
            },
            'mission': {
                'name': '🎯 Mission System',
                'description': 'Human fusion, POI, Exploration',
                'command': ['ros2', 'launch', 'manriix_mission', 'mission.launch.py'],
                'color': '#e94560',
                'order': 3,
                'cleanup_patterns': ['human_fusion', 'poi_manager', 'exploration', 'mission_controller']
            },
            'object_detection': {
                'name': '👁️ Object Detection',
                'description': 'ZED AI (3 cameras) → Nav2 costmap',
                'command': ['ros2', 'launch', 'manriix_perception', 'object_bridge.launch.py'],
                'color': '#ff00ff',
                'order': 4,
                'cleanup_patterns': ['object_to_costmap_bridge']
            }
        }
        
        for node_id in self.node_configs:
            self.logs[node_id] = []
            self.processes[node_id] = None
    
    def get_nav_command(self, mode, perception, map_file=''):
        if mode == 'mapping':
            cmd = ['ros2', 'launch', 'manriix_navigation', 'mapping.launch.py', 'use_rviz:=false']
        elif mode == 'localization':
            cmd = ['ros2', 'launch', 'manriix_navigation', 'localization.launch.py', 'use_rviz:=false']
            if map_file:
                cmd.append(f'map:={map_file}')
            if perception == '3d':
                cmd.append('use_3d_perception:=true')
        else:
            cmd = ['ros2', 'launch', 'manriix_navigation', 'slam_nav.launch.py', 'use_rviz:=false']
            if perception == '3d':
                cmd.append('use_3d_perception:=true')
        return cmd
            
    def start_node(self, node_id, **kwargs):
        if self._is_process_running(node_id):
            return {'success': False, 'message': 'Already running'}
        config = self.node_configs.get(node_id)
        if not config:
            return {'success': False, 'message': 'Unknown node'}
        
        if node_id == 'navigation':
            mode = kwargs.get('mode', self.nav_mode)
            perception = kwargs.get('perception', self.nav_perception)
            map_file = kwargs.get('map', self.nav_map)
            command = self.get_nav_command(mode, perception, map_file)
            self.nav_mode = mode
            self.nav_perception = perception
            self.nav_map = map_file
        else:
            command = config['command']
        
        if not command:
            return {'success': False, 'message': 'No command configured'}
            
        self.logs[node_id] = []
        try:
            env = os.environ.copy()
            env['DISPLAY'] = ':0'
            process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, 
                                      stdin=subprocess.DEVNULL, env=env, bufsize=1, 
                                      universal_newlines=True, start_new_session=True)
            self.processes[node_id] = process
            threading.Thread(target=self._read_output, args=(node_id, process), daemon=True).start()
            self._add_log(node_id, f"▶ Started (PID: {process.pid})", 'success')
            self._add_log(node_id, f"  Command: {' '.join(command)}", 'info')
            return {'success': True, 'message': f'Started (PID: {process.pid})'}
        except Exception as e:
            self._add_log(node_id, f"✗ Failed: {str(e)}", 'error')
            return {'success': False, 'message': str(e)}
            
    def stop_node(self, node_id):
        config = self.node_configs.get(node_id)
        if not config:
            return {'success': False, 'message': 'Unknown node'}
        process = self.processes.get(node_id)
        if process is not None and process.poll() is None:
            try:
                pgid = os.getpgid(process.pid)
                os.killpg(pgid, signal.SIGINT)
                for _ in range(50):
                    if process.poll() is not None:
                        self.processes[node_id] = None
                        self._add_log(node_id, "✓ Stopped", 'success')
                        return {'success': True, 'message': 'Stopped'}
                    time.sleep(0.1)
                os.killpg(pgid, signal.SIGKILL)
                process.wait(timeout=2)
            except: pass
        self.processes[node_id] = None
        for pattern in config.get('cleanup_patterns', []):
            try: subprocess.run(['pkill', '-9', '-f', pattern], capture_output=True, timeout=1)
            except: pass
        self._add_log(node_id, "✓ Stopped", 'success')
        return {'success': True, 'message': 'Stopped'}
    
    def _is_process_running(self, node_id):
        process = self.processes.get(node_id)
        if process is None: return False
        if process.poll() is not None:
            self.processes[node_id] = None
            return False
        return True
        
    def stop_all(self):
        results = {}
        for node_id in reversed(['rosbridge', 'robot', 'navigation', 'mission', 'object_detection']):
            results[node_id] = self.stop_node(node_id)
            time.sleep(0.3)
        return results
            
    def get_status(self, node_id):
        if self._is_process_running(node_id): return 'running'
        if node_id == 'rosbridge':
            try:
                import socket
                sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                sock.settimeout(0.5)
                if sock.connect_ex(('localhost', 9090)) == 0:
                    sock.close()
                    return 'running'
                sock.close()
            except: pass
        return 'stopped'
        
    def _read_output(self, node_id, process):
        try:
            for line in iter(process.stdout.readline, ''):
                if not line: break
                level = 'info'
                ll = line.lower()
                if 'error' in ll or 'fatal' in ll: level = 'error'
                elif 'warn' in ll: level = 'warning'
                elif 'ready' in ll or 'active' in ll or 'started' in ll: level = 'success'
                self._add_log(node_id, line.rstrip(), level)
        except: pass
            
    def _add_log(self, node_id, message, level='info'):
        if node_id not in self.logs: self.logs[node_id] = []
        self.logs[node_id].append({'time': time.strftime('%H:%M:%S'), 'message': message, 'level': level})
        if len(self.logs[node_id]) > self.max_log_lines:
            self.logs[node_id] = self.logs[node_id][-self.max_log_lines:]
            
    def get_logs(self, node_id, since=0):
        return self.logs.get(node_id, [])[since:]
        
    def get_all_status(self):
        result = {}
        for node_id, config in self.node_configs.items():
            status_info = {
                'name': config['name'],
                'description': config['description'],
                'status': self.get_status(node_id),
                'color': config['color'],
                'order': config['order']
            }
            if node_id == 'navigation':
                status_info['nav_mode'] = self.nav_mode
                status_info['nav_perception'] = self.nav_perception
                status_info['nav_map'] = self.nav_map
            result[node_id] = status_info
        return result

node_manager = NodeManager()

status_data = {'mission_state': 'UNKNOWN', 'human_count': 0, 'poi_score': 0.0, 'poi_type': '-', 'should_capture': False}

def update_status_loop():
    global status_data
    while True:
        try:
            def get_topic(topic, field='data'):
                try:
                    result = subprocess.run(['ros2', 'topic', 'echo', '--once', topic], capture_output=True, text=True, timeout=0.5)
                    for line in result.stdout.split('\n'):
                        if f'{field}:' in line: return line.split(f'{field}:')[1].strip().strip('"').strip("'")
                except: pass
                return None
            val = get_topic('/mission/state')
            if val: status_data['mission_state'] = val
            val = get_topic('/human_clusters/count')
            if val:
                try: status_data['human_count'] = int(val)
                except: pass
            val = get_topic('/poi/score')
            if val:
                try: status_data['poi_score'] = float(val)
                except: pass
            val = get_topic('/poi/type')
            if val: status_data['poi_type'] = val
            val = get_topic('/poi/should_capture')
            if val: status_data['should_capture'] = val.lower() == 'true'
        except: pass
        time.sleep(0.5)

def get_available_maps():
    maps = []
    if os.path.exists(MAPS_DIR):
        for f in glob.glob(os.path.join(MAPS_DIR, '*.yaml')):
            name = os.path.basename(f).replace('.yaml', '')
            maps.append({'name': name, 'path': f})
    return maps

def save_map(map_name):
    if not map_name:
        return {'success': False, 'message': 'Map name required'}
    os.makedirs(MAPS_DIR, exist_ok=True)
    map_path = os.path.join(MAPS_DIR, map_name)
    try:
        result = subprocess.run(
            ['ros2', 'run', 'nav2_map_server', 'map_saver_cli', '-f', map_path],
            capture_output=True, text=True, timeout=30
        )
        if result.returncode == 0:
            return {'success': True, 'message': f'Map saved to {map_path}'}
        else:
            return {'success': False, 'message': result.stderr or 'Failed to save map'}
    except subprocess.TimeoutExpired:
        return {'success': False, 'message': 'Timeout saving map'}
    except Exception as e:
        return {'success': False, 'message': str(e)}

def call_trigger_service(service_name):
    try:
        result = subprocess.run(['ros2', 'service', 'call', service_name, 'std_srvs/srv/Trigger'], capture_output=True, text=True, timeout=5)
        return {'success': True, 'output': result.stdout + result.stderr}
    except subprocess.TimeoutExpired:
        return {'success': False, 'output': 'Timeout'}
    except Exception as e:
        return {'success': False, 'output': str(e)}

def call_setbool_service(service_name, value):
    try:
        result = subprocess.run(['ros2', 'service', 'call', service_name, 'std_srvs/srv/SetBool', f'{{data: {"true" if value else "false"}}}'], capture_output=True, text=True, timeout=5)
        return {'success': True, 'output': result.stdout + result.stderr}
    except subprocess.TimeoutExpired:
        return {'success': False, 'output': 'Timeout'}
    except Exception as e:
        return {'success': False, 'output': str(e)}

def generate_app_icon(size=180):
    img = np.zeros((size, size, 3), dtype=np.uint8)
    for y in range(size):
        img[y, :] = (int(30 + 20 * y/size), int(20 + 10 * y/size), int(40 + 60 * y/size))
    center, radius = size // 2, size // 3
    cv2.circle(img, (center, center), radius, (96, 69, 233), -1)
    cv2.circle(img, (center, center), radius, (255, 255, 255), 3)
    cv2.putText(img, 'M', (center - int(20 * size/180), center + int(25 * size/180)), cv2.FONT_HERSHEY_SIMPLEX, 1.5 * size/180, (255, 255, 255), int(3 * size/180))
    _, png = cv2.imencode('.png', img)
    return png.tobytes()

HTML_TEMPLATE = '''<!DOCTYPE html>
<html>
<head>
<title>MANRIIX Control</title>
<meta name="viewport" content="width=device-width,initial-scale=1,maximum-scale=1,user-scalable=no">
<meta name="mobile-web-app-capable" content="yes"><meta name="apple-mobile-web-app-capable" content="yes">
<meta name="theme-color" content="#0d0d1a"><link rel="manifest" href="/manifest.json">
<script src="https://cdn.jsdelivr.net/npm/roslib@1/build/roslib.min.js"></script>
<script src="https://cdnjs.cloudflare.com/ajax/libs/three.js/r128/three.min.js"></script>
<script src="https://cdn.jsdelivr.net/gh/mrdoob/three.js@r128/examples/js/controls/OrbitControls.js"></script>
<script src="https://cdn.jsdelivr.net/gh/mrdoob/three.js@r128/examples/js/loaders/STLLoader.js"></script>
<script src="https://cdn.jsdelivr.net/npm/urdf-loader@0.12.2/umd/URDFLoader.min.js"></script>
<style>
*{box-sizing:border-box;margin:0;padding:0;-webkit-user-select:none;user-select:none}
body{font-family:-apple-system,sans-serif;background:#0d0d1a;color:#fff}
.header{background:linear-gradient(135deg,#1a1a2e,#16213e);padding:10px 15px;display:flex;justify-content:space-between;align-items:center;border-bottom:2px solid #e94560;position:sticky;top:0;z-index:100}
.header h1{color:#e94560;font-size:1.3em}
.state-badge{padding:5px 12px;border-radius:12px;font-weight:bold;font-size:0.85em}
.tabs{display:flex;background:#12121f;border-bottom:1px solid #333;overflow-x:auto}
.tab{padding:10px 12px;cursor:pointer;border:none;background:transparent;color:#888;font-weight:bold;border-bottom:2px solid transparent;white-space:nowrap;font-size:0.8em}
.tab.active{color:#e94560;border-bottom-color:#e94560}
.tab-content{display:none;padding:12px}.tab-content.active{display:block}
.card{background:#16213e;border-radius:10px;padding:12px;margin-bottom:12px}
.card-title{font-size:0.95em;font-weight:bold;margin-bottom:10px;color:#e94560;border-bottom:1px solid #333;padding-bottom:6px}
.grid-2{display:grid;grid-template-columns:1fr 1fr;gap:12px}
@media(max-width:700px){.grid-2{grid-template-columns:1fr}}
.node-card{background:#1a1a2e;border-radius:8px;padding:10px;margin-bottom:8px;border-left:3px solid #444}
.node-card.running{border-left-color:#00ff88}
.node-header{display:flex;justify-content:space-between;align-items:center;margin-bottom:4px}
.node-name{font-weight:bold;font-size:0.85em}
.node-desc{color:#888;font-size:0.7em;margin-bottom:6px}
.node-status{font-size:0.6em;padding:2px 6px;border-radius:6px}
.node-status.running{background:#00ff88;color:#000}
.node-status.stopped{background:#444;color:#aaa}
.node-buttons{display:flex;gap:5px}
.btn{padding:5px 10px;border:none;border-radius:5px;cursor:pointer;font-weight:bold;font-size:0.75em}
.btn:disabled{opacity:0.4}
.btn-start{background:#00ff88;color:#000;flex:1}
.btn-stop{background:#ff4444;color:#fff;flex:1}
.btn-save{background:#00bfff;color:#000}
.mission-grid{display:grid;grid-template-columns:repeat(3,1fr);gap:6px}
.mission-btn{padding:12px;border:none;border-radius:8px;cursor:pointer;font-size:0.95em;font-weight:bold}
.mission-btn.start{background:#00ff88;color:#000}
.mission-btn.pause{background:#ffcc00;color:#000}
.mission-btn.stop{background:#ff4444;color:#fff}
.status-grid{display:grid;grid-template-columns:1fr 1fr;gap:6px}
.status-item{background:#1a1a2e;padding:8px;border-radius:6px;display:flex;justify-content:space-between}
.status-label{color:#888;font-size:0.8em}
.status-value{font-weight:bold}
.motor-grid{display:grid;grid-template-columns:repeat(2,1fr);gap:8px}
.motor-btn{padding:12px 8px;border:none;border-radius:8px;cursor:pointer;font-weight:bold;font-size:0.8em;display:flex;flex-direction:column;align-items:center;gap:4px}
.motor-btn .icon{font-size:1.4em}
.motor-btn.reset{background:linear-gradient(135deg,#0066aa,#00aaff);color:#fff}
.motor-btn.init{background:linear-gradient(135deg,#00aa55,#00ff88);color:#000}
.motor-btn.halt{background:linear-gradient(135deg,#aa0000,#ff4444);color:#fff}
.motor-btn.resume{background:linear-gradient(135deg,#00aa55,#00ff88);color:#000}
.motor-btn.park-on{background:linear-gradient(135deg,#aa6600,#ffaa00);color:#000}
.motor-btn.park-off{background:linear-gradient(135deg,#666,#999);color:#fff}
.motor-btn.info{background:linear-gradient(135deg,#6600aa,#aa00ff);color:#fff}
.motor-btn.temp{background:linear-gradient(135deg,#aa4400,#ff6600);color:#fff}
.motor-log{background:#0a0a15;border-radius:8px;padding:10px;margin-top:12px;max-height:150px;overflow-y:auto;font-family:monospace;font-size:0.75em}
.motor-log-entry{padding:3px 0;border-bottom:1px solid #1a1a2e}
.motor-log-entry.success{color:#00ff88}
.motor-log-entry.error{color:#ff4444}
.camera-container{position:relative;background:#0a0a15;border-radius:8px;overflow:hidden}
.camera-feed-wrapper{position:relative}
.camera-feed-wrapper img{width:100%;height:auto;display:block;min-height:150px;background:#1a1a2e}
.camera-feed-wrapper canvas{position:absolute;top:0;left:0;width:100%;height:100%;pointer-events:none}
.camera-label{position:absolute;top:5px;left:5px;background:rgba(0,0,0,0.7);padding:3px 8px;border-radius:4px;font-size:0.65em;z-index:5}
.camera-select{display:flex;gap:5px;margin-bottom:10px;flex-wrap:wrap}
.cam-btn{padding:6px 12px;border:1px solid #444;border-radius:5px;background:#1a1a2e;color:#888;cursor:pointer;font-size:0.8em}
.cam-btn.active{background:#e94560;color:#fff;border-color:#e94560}
.camera-grid{display:grid;grid-template-columns:repeat(3,1fr);gap:8px}
.camera-grid.single{grid-template-columns:1fr}
.camera-container.hidden{display:none}
.bbox-toggle{display:flex;align-items:center;gap:8px;margin:10px 0}
.bbox-toggle input{width:18px;height:18px}
@media(max-width:700px){.camera-grid{grid-template-columns:1fr}}
.obj-stats-grid{display:grid;grid-template-columns:repeat(4,1fr);gap:6px;margin-bottom:12px}
.obj-stat-card{background:#1a1a2e;padding:10px 6px;border-radius:8px;text-align:center;border-left:3px solid}
.obj-stat-card.person{border-color:#00ff88}.obj-stat-card.vehicle{border-color:#ff4444}
.obj-stat-card.animal{border-color:#ff8800}.obj-stat-card.total{border-color:#ff00ff}
.obj-stat-icon{font-size:1.2em}
.obj-stat-value{font-size:1.4em;font-weight:bold}
.obj-stat-card.person .obj-stat-value{color:#00ff88}
.obj-stat-card.vehicle .obj-stat-value{color:#ff4444}
.obj-stat-card.animal .obj-stat-value{color:#ff8800}
.obj-stat-card.total .obj-stat-value{color:#ff00ff}
.obj-stat-label{font-size:0.65em;color:#888}
@media(max-width:700px){.obj-stats-grid{grid-template-columns:repeat(2,1fr)}}
.cam-stats-grid{display:grid;grid-template-columns:repeat(3,1fr);gap:6px;margin-bottom:12px}
.cam-stat-card{background:#1a1a2e;padding:8px;border-radius:8px;text-align:center;border-top:3px solid}
.cam-stat-card.front{border-color:#00ff88}
.cam-stat-card.left{border-color:#ff4444}
.cam-stat-card.right{border-color:#00bfff}
.cam-stat-label{font-size:0.7em;color:#888}
.cam-stat-value{font-size:1.2em;font-weight:bold}
.cam-stat-card.front .cam-stat-value{color:#00ff88}
.cam-stat-card.left .cam-stat-value{color:#ff4444}
.cam-stat-card.right .cam-stat-value{color:#00bfff}
.obj-list{background:#0a0a15;border-radius:8px;max-height:200px;overflow-y:auto}
.obj-list-header{display:grid;grid-template-columns:40px 50px 1fr 50px 50px;padding:6px;background:#1a1a2e;font-size:0.65em;color:#888;font-weight:bold;position:sticky;top:0}
.obj-list-item{display:grid;grid-template-columns:40px 50px 1fr 50px 50px;padding:6px;border-bottom:1px solid #1a1a2e;font-size:0.75em}
.obj-list-empty{padding:20px;text-align:center;color:#666;font-size:0.8em}
.obj-class-badge{padding:2px 5px;border-radius:3px;font-size:0.65em;font-weight:bold}
.obj-class-badge.person{background:#00ff88;color:#000}
.obj-class-badge.vehicle{background:#ff4444;color:#fff}
.obj-class-badge.animal{background:#ff8800;color:#000}
.cam-badge{padding:2px 4px;border-radius:3px;font-size:0.6em;font-weight:bold}
.cam-badge.front{background:#00ff88;color:#000}
.cam-badge.left{background:#ff4444;color:#fff}
.cam-badge.right{background:#00bfff;color:#000}
.rviz-container{position:relative;background:#1a1a2e;border-radius:10px;overflow:hidden}
.rviz-canvas{width:100%;height:320px;display:block;touch-action:none}
.rviz-toolbar{display:flex;gap:4px;padding:6px;background:#12121f;flex-wrap:wrap}
.rviz-tool{padding:4px 8px;border:1px solid #444;border-radius:4px;background:#1a1a2e;color:#aaa;cursor:pointer;font-size:0.7em}
.rviz-tool.active{background:#e94560;color:#fff;border-color:#e94560}
.rviz-status{padding:5px 8px;background:#12121f;font-size:0.65em;color:#888;display:flex;justify-content:space-between;flex-wrap:wrap;gap:6px}
.view-toggle{display:flex;gap:4px;padding:6px;background:#0d0d1a}
.view-toggle-btn{padding:6px 14px;border:2px solid #444;border-radius:6px;background:#1a1a2e;color:#888;cursor:pointer;font-size:0.8em;font-weight:bold}
.view-toggle-btn.active{border-color:#e94560;color:#e94560;background:#2a1a2e}
.rviz-3d-container{width:100%;height:320px;display:none;background:#1a1a2e;position:relative}
.rviz-3d-container.active{display:block}
.rviz-3d-canvas{width:100%;height:100%}
.rviz-2d-container{display:block}
.rviz-2d-container.hidden{display:none}
.layer-toggle{display:flex;gap:4px;flex-wrap:wrap;padding:5px 8px;background:#0d0d1a}
.layer-btn{padding:3px 6px;border-radius:3px;font-size:0.6em;cursor:pointer;border:1px solid #444;background:#1a1a2e;color:#888}
.layer-btn.active{background:#333;color:#fff;border-color:#666}
.ros-status{position:absolute;top:40px;right:6px;padding:3px 6px;border-radius:4px;font-size:0.65em;z-index:10}
.ros-status.connected{background:#00ff88;color:#000}
.ros-status.disconnected{background:#ff4444;color:#fff}
.teleop-grid{display:grid;grid-template-columns:repeat(3,1fr);gap:4px;width:180px;margin:0 auto}
.teleop-btn{width:55px;height:55px;border:none;border-radius:8px;cursor:pointer;font-size:1.2em;font-weight:bold;display:flex;flex-direction:column;align-items:center;justify-content:center}
.teleop-btn .key{font-size:0.3em;opacity:0.7}
.teleop-btn.move{background:linear-gradient(135deg,#00aa55,#00ff88);color:#000}
.teleop-btn.turn{background:linear-gradient(135deg,#cc8800,#ffcc00);color:#000}
.teleop-btn.stop{background:linear-gradient(135deg,#cc0000,#ff4444);color:#fff}
.teleop-btn.back{background:linear-gradient(135deg,#0066aa,#0088ff);color:#fff}
.log-tabs{display:flex;gap:4px;margin-bottom:6px;flex-wrap:wrap}
.log-tab{padding:4px 8px;border-radius:4px;cursor:pointer;font-size:0.7em;background:#1a1a2e;border:1px solid transparent;color:#888}
.log-tab.active{border-color:#e94560;color:#fff}
.log-content{background:#0a0a15;border-radius:6px;padding:6px;height:280px;overflow-y:auto;font-family:monospace;font-size:0.7em}
.log-line{padding:2px 0;border-bottom:1px solid #1a1a2e;word-break:break-all}
.log-time{color:#666;margin-right:6px}
.log-info{color:#aaa}.log-warning{color:#ffcc00}.log-error{color:#ff4444}.log-success{color:#00ff88}
.emergency-btn{background:linear-gradient(135deg,#880000,#ff0000);color:#fff;border:none;padding:10px;border-radius:6px;font-weight:bold;cursor:pointer;width:100%;margin-top:10px}
.connection-status{position:fixed;bottom:10px;right:10px;padding:4px 8px;border-radius:12px;font-size:0.65em;z-index:1000}
.connection-status.online{background:#00ff88;color:#000}
.connection-status.offline{background:#ff4444;color:#fff}
.nav-mode-selector{display:flex;gap:6px;margin-bottom:12px;flex-wrap:wrap}
.nav-mode-btn{padding:10px 14px;border:2px solid #444;border-radius:8px;background:#1a1a2e;color:#888;cursor:pointer;font-size:0.85em;font-weight:bold;flex:1;text-align:center;min-width:100px}
.nav-mode-btn.active{border-color:#ffcc00;color:#ffcc00;background:#2a2a3e}
.perception-toggle{display:flex;gap:6px;margin-bottom:12px}
.perception-btn{padding:8px 16px;border:2px solid #444;border-radius:6px;background:#1a1a2e;color:#888;cursor:pointer;font-size:0.8em;font-weight:bold}
.perception-btn.active{border-color:#00bfff;color:#00bfff;background:#1a2a3e}
.map-selector{margin-bottom:12px}
.map-selector label{display:block;color:#888;font-size:0.8em;margin-bottom:4px}
.map-selector select{width:100%;padding:8px;border-radius:6px;background:#1a1a2e;color:#fff;border:1px solid #444;font-size:0.85em}
.map-save-row{display:flex;gap:8px;margin-bottom:12px}
.map-save-row input{flex:1;padding:8px;border-radius:6px;background:#1a1a2e;color:#fff;border:1px solid #444;font-size:0.85em}
.nav-status-row{display:flex;justify-content:space-between;align-items:center;padding:8px;background:#1a1a2e;border-radius:6px;margin-bottom:12px}
.nav-status-label{color:#888;font-size:0.8em}
.nav-status-value{font-weight:bold;font-size:0.85em}
</style>
</head>
<body>
<div class="header"><h1>🤖 MANRIIX</h1><span id="header-state" class="state-badge" style="background:#444">UNKNOWN</span></div>
<div class="tabs">
<button class="tab active" onclick="showTab('control')">🎮 Control</button>
<button class="tab" onclick="showTab('nav')">🗺️ Nav</button>
<button class="tab" onclick="showTab('rviz')">📍 Map</button>
<button class="tab" onclick="showTab('cameras')">📷 Cameras</button>
<button class="tab" onclick="showTab('objdet')">👁️ Objects</button>
<button class="tab" onclick="showTab('motor')">⚡ Motor</button>
<button class="tab" onclick="showTab('teleop')">🕹️ Teleop</button>
<button class="tab" onclick="showTab('system')">⚙️ System</button>
<button class="tab" onclick="showTab('logs')">📋 Logs</button>
</div>

<div id="tab-control" class="tab-content active">
<div class="grid-2">
<div>
<div class="card"><div class="card-title">🎯 Mission Control</div>
<div class="mission-grid">
<button class="mission-btn start" onclick="sendCommand('START')">▶ START</button>
<button class="mission-btn pause" onclick="sendCommand('PAUSE')">⏸ PAUSE</button>
<button class="mission-btn stop" onclick="sendCommand('STOP')">⏹ STOP</button>
</div></div>
<div class="card"><div class="card-title">📊 Status</div>
<div class="status-grid">
<div class="status-item"><span class="status-label">👥 Humans</span><span id="human-count" class="status-value">0</span></div>
<div class="status-item"><span class="status-label">📈 POI</span><span id="poi-score" class="status-value">0.00</span></div>
<div class="status-item"><span class="status-label">🎯 Capture</span><span id="should-capture" class="status-value">NO</span></div>
<div class="status-item"><span class="status-label">📷 Type</span><span id="poi-type" class="status-value">-</span></div>
</div></div>
</div>
<div><div class="card"><div class="card-title">🚀 Quick Launch</div><div id="quick-launch"></div></div></div>
</div></div>

<div id="tab-nav" class="tab-content">
<div class="card">
<div class="card-title">🗺️ Navigation Mode</div>
<div class="nav-mode-selector">
<button class="nav-mode-btn" id="nav-mode-mapping" onclick="setNavMode('mapping')">📝 Mapping</button>
<button class="nav-mode-btn" id="nav-mode-localization" onclick="setNavMode('localization')">📍 Localization</button>
<button class="nav-mode-btn active" id="nav-mode-slam_nav" onclick="setNavMode('slam_nav')">🚀 SLAM+Nav</button>
</div>
<div class="card-title" style="margin-top:12px">📡 Perception</div>
<div class="perception-toggle">
<button class="perception-btn active" id="perception-2d" onclick="setPerception('2d')">2D (RPLidar)</button>
<button class="perception-btn" id="perception-3d" onclick="setPerception('3d')">3D (+ ZED Depth)</button>
</div>
<div id="map-select-container" class="map-selector" style="display:none">
<label>Select Map:</label>
<select id="map-select" onchange="navState.map=this.value"></select>
</div>
<div id="map-save-container" class="map-save-row" style="display:none">
<input type="text" id="map-name-input" placeholder="Enter map name...">
<button class="btn btn-save" onclick="saveMap()">💾 Save Map</button>
</div>
<div class="nav-status-row">
<span class="nav-status-label">Status:</span>
<span id="nav-status-value" class="nav-status-value" style="color:#888">STOPPED</span>
</div>
<div style="display:flex;gap:10px">
<button class="btn btn-start" style="flex:1;padding:12px" onclick="startNavigation()">▶ Start Navigation</button>
<button class="btn btn-stop" style="flex:1;padding:12px" onclick="stopNavigation()">⏹ Stop</button>
</div>
</div>
<div class="card">
<div class="card-title">📋 Navigation Info</div>
<div id="nav-info" style="font-size:0.8em;color:#888">
<p><b>Mapping:</b> Build a new map by driving the robot manually. Save when done.</p>
<p><b>Localization:</b> Navigate using a previously saved map.</p>
<p><b>SLAM+Nav:</b> Explore and navigate while building a map.</p>
</div>
</div>
</div>

<div id="tab-rviz" class="tab-content">
<div class="card" style="padding:0;overflow:hidden">
<div class="view-toggle">
<button class="view-toggle-btn active" id="view-2d-btn" onclick="setMapView('2d')">📍 2D Map</button>
<button class="view-toggle-btn" id="view-3d-btn" onclick="setMapView('3d')">🎮 3D View</button>
</div>
<div class="rviz-toolbar">
<button class="rviz-tool active" id="tool-pan" onclick="setTool('pan')">🖐️ Pan</button>
<button class="rviz-tool" id="tool-goal" onclick="setTool('goal')">🎯 Goal</button>
<button class="rviz-tool" id="tool-pose" onclick="setTool('pose')">📍 Pose</button>
<button class="rviz-tool" onclick="resetView()">🔄</button>
<button class="rviz-tool" onclick="zoomIn()">➕</button>
<button class="rviz-tool" onclick="zoomOut()">➖</button>
</div>
<div class="rviz-container">
<div id="ros-status" class="ros-status disconnected">● Disconnected</div>
<div id="rviz-2d-container" class="rviz-2d-container"><canvas id="rviz-canvas" class="rviz-canvas"></canvas></div>
<div id="rviz-3d-container" class="rviz-3d-container"><canvas id="rviz-3d-canvas" class="rviz-3d-canvas"></canvas></div>
</div>
<div class="layer-toggle">
<button class="layer-btn active" id="layer-map" onclick="toggleLayer('map')">Map</button>
<button class="layer-btn active" id="layer-costmap" onclick="toggleLayer('costmap')">Cost</button>
<button class="layer-btn active" id="layer-laser" onclick="toggleLayer('laser')">Laser</button>
<button class="layer-btn active" id="layer-path" onclick="toggleLayer('path')">Path</button>
<button class="layer-btn active" id="layer-humans" onclick="toggleLayer('humans')">Human</button>
<button class="layer-btn active" id="layer-objects" onclick="toggleLayer('objects')">Obj</button>
<button class="layer-btn active" id="layer-robot" onclick="toggleLayer('robot')">Robot</button>
</div>
<div class="rviz-status">
<span>Pos: <span id="robot-pos">-</span></span>
<span>Obj: <span id="obj-det-count">0</span></span>
<span>FPS: <span id="fps-counter">0</span></span>
</div>
</div></div>

<div id="tab-cameras" class="tab-content">
<div class="card">
<div class="card-title">📷 ZED Cameras + Bounding Boxes</div>
<div class="camera-select">
<button class="cam-btn active" id="cam-btn-all" onclick="setCameraView('all')">All</button>
<button class="cam-btn" id="cam-btn-left" onclick="setCameraView('left')">Left</button>
<button class="cam-btn" id="cam-btn-front" onclick="setCameraView('front')">Front</button>
<button class="cam-btn" id="cam-btn-right" onclick="setCameraView('right')">Right</button>
</div>
<div class="bbox-toggle">
<input type="checkbox" id="bbox-enabled" checked onchange="toggleBoundingBoxes(this.checked)">
<label for="bbox-enabled">Show Bounding Boxes</label>
</div>
<div id="camera-grid" class="camera-grid">
<div class="camera-container" id="cam-left"><div class="camera-label">🔴 Left (<span id="det-count-left">0</span>)</div><div class="camera-feed-wrapper"><img id="img-left" src="" alt="Left"><canvas id="bbox-left"></canvas></div></div>
<div class="camera-container" id="cam-front"><div class="camera-label">🟢 Front (<span id="det-count-front">0</span>)</div><div class="camera-feed-wrapper"><img id="img-front" src="" alt="Front"><canvas id="bbox-front"></canvas></div></div>
<div class="camera-container" id="cam-right"><div class="camera-label">🔵 Right (<span id="det-count-right">0</span>)</div><div class="camera-feed-wrapper"><img id="img-right" src="" alt="Right"><canvas id="bbox-right"></canvas></div></div>
</div>
<div style="margin-top:8px;font-size:0.7em;color:#888;display:flex;justify-content:space-between">
<span>Stream: <span id="cam-stream-status">Waiting...</span></span>
<span>Total: <span id="cam-det-count">0</span></span>
</div>
</div></div>

<div id="tab-objdet" class="tab-content">
<div class="card">
<div class="card-title">👁️ Object Detection (All 3 Cameras)</div>
<div class="cam-stats-grid">
<div class="cam-stat-card front"><div class="cam-stat-label">🟢 Front</div><div id="cam-obj-front" class="cam-stat-value">0</div></div>
<div class="cam-stat-card left"><div class="cam-stat-label">🔴 Left</div><div id="cam-obj-left" class="cam-stat-value">0</div></div>
<div class="cam-stat-card right"><div class="cam-stat-label">🔵 Right</div><div id="cam-obj-right" class="cam-stat-value">0</div></div>
</div>
<div class="obj-stats-grid">
<div class="obj-stat-card person"><div class="obj-stat-icon">👤</div><div id="obj-stat-person" class="obj-stat-value">0</div><div class="obj-stat-label">Person</div></div>
<div class="obj-stat-card vehicle"><div class="obj-stat-icon">🚗</div><div id="obj-stat-vehicle" class="obj-stat-value">0</div><div class="obj-stat-label">Vehicle</div></div>
<div class="obj-stat-card animal"><div class="obj-stat-icon">🐕</div><div id="obj-stat-animal" class="obj-stat-value">0</div><div class="obj-stat-label">Animal</div></div>
<div class="obj-stat-card total"><div class="obj-stat-icon">📊</div><div id="obj-stat-total" class="obj-stat-value">0</div><div class="obj-stat-label">Total</div></div>
</div></div>
<div class="card">
<div class="card-title">📋 Live Detections</div>
<div class="obj-list">
<div class="obj-list-header"><span>ID</span><span>Cam</span><span>Class</span><span>Conf</span><span>Dist</span></div>
<div id="obj-list-items"><div class="obj-list-empty">No objects detected</div></div>
</div></div>
<div class="card">
<div class="card-title">⚙️ Bridge Control</div>
<div style="display:flex;gap:10px">
<button class="btn btn-start" style="flex:1" onclick="startNode('object_detection')">▶ Start Bridge</button>
<button class="btn btn-stop" style="flex:1" onclick="stopNode('object_detection')">⏹ Stop</button>
</div></div>
</div>

<div id="tab-motor" class="tab-content">
<div class="card">
<div class="card-title">⚡ Motor Control</div>
<div class="motor-grid">
<button class="motor-btn reset" onclick="motorCmd('reset')"><span class="icon">🔄</span><span>Reset</span></button>
<button class="motor-btn init" onclick="motorCmd('init')"><span class="icon">⚡</span><span>Init</span></button>
<button class="motor-btn halt" onclick="motorCmd('halt')"><span class="icon">🛑</span><span>HALT</span></button>
<button class="motor-btn resume" onclick="motorCmd('resume')"><span class="icon">▶️</span><span>Resume</span></button>
<button class="motor-btn park-on" onclick="motorCmd('park_on')"><span class="icon">🅿️</span><span>Park ON</span></button>
<button class="motor-btn park-off" onclick="motorCmd('park_off')"><span class="icon">🚗</span><span>Park OFF</span></button>
<button class="motor-btn info" onclick="motorCmd('fault')"><span class="icon">⚠️</span><span>Faults</span></button>
<button class="motor-btn temp" onclick="motorCmd('temp')"><span class="icon">🌡️</span><span>Temp</span></button>
</div>
<div class="motor-log" id="motor-log"><div class="motor-log-entry">Motor control ready</div></div>
</div></div>

<div id="tab-teleop" class="tab-content">
<div class="card">
<div class="card-title">🕹️ Manual Control</div>
<div class="teleop-grid">
<button class="teleop-btn move" onmousedown="startTeleop('u')" onmouseup="stopTeleop()" ontouchstart="startTeleop('u')" ontouchend="stopTeleop()">↖<span class="key">U</span></button>
<button class="teleop-btn move" onmousedown="startTeleop('i')" onmouseup="stopTeleop()" ontouchstart="startTeleop('i')" ontouchend="stopTeleop()">↑<span class="key">I</span></button>
<button class="teleop-btn move" onmousedown="startTeleop('o')" onmouseup="stopTeleop()" ontouchstart="startTeleop('o')" ontouchend="stopTeleop()">↗<span class="key">O</span></button>
<button class="teleop-btn turn" onmousedown="startTeleop('j')" onmouseup="stopTeleop()" ontouchstart="startTeleop('j')" ontouchend="stopTeleop()">↺<span class="key">J</span></button>
<button class="teleop-btn stop" onclick="stopTeleop()">⏹<span class="key">K</span></button>
<button class="teleop-btn turn" onmousedown="startTeleop('l')" onmouseup="stopTeleop()" ontouchstart="startTeleop('l')" ontouchend="stopTeleop()">↻<span class="key">L</span></button>
<button class="teleop-btn back" onmousedown="startTeleop('m')" onmouseup="stopTeleop()" ontouchstart="startTeleop('m')" ontouchend="stopTeleop()">↙<span class="key">M</span></button>
<button class="teleop-btn back" onmousedown="startTeleop(',')" onmouseup="stopTeleop()" ontouchstart="startTeleop(',')" ontouchend="stopTeleop()">↓<span class="key">,</span></button>
<button class="teleop-btn back" onmousedown="startTeleop('.')" onmouseup="stopTeleop()" ontouchstart="startTeleop('.')" ontouchend="stopTeleop()">↘<span class="key">.</span></button>
</div>
<p style="text-align:center;color:#888;font-size:0.7em;margin-top:10px">Hold or use keyboard</p>
</div></div>

<div id="tab-system" class="tab-content">
<div class="card">
<div class="card-title">⚙️ System Nodes</div>
<div id="node-list"></div>
<button class="emergency-btn" onclick="stopAll()">🛑 STOP ALL</button>
</div></div>

<div id="tab-logs" class="tab-content">
<div class="card">
<div class="card-title">📋 Logs</div>
<div class="log-tabs" id="log-tabs"></div>
<div class="log-content" id="log-content"></div>
</div></div>

<div id="connection-status" class="connection-status online">● Online</div>

<script>
let currentTab='control',currentLogTab='robot',logIndices={},teleopInterval=null,ros=null,rosConnected=false,bboxEnabled=true;
const navState={mode:'slam_nav',perception:'2d',map:''};
const cameraDetections={left:[],front:[],right:[]};
const cameraStats={front:{persons:0,vehicles:0,animals:0,total:0},left:{persons:0,vehicles:0,animals:0,total:0},right:{persons:0,vehicles:0,animals:0,total:0}};
const rviz={canvas:null,ctx:null,viewX:0,viewY:0,scale:50,map:null,mapInfo:null,costmap:null,costmapInfo:null,robotPose:{x:0,y:0,theta:0},goal:null,globalPath:[],localPath:[],laserScan:[],humans:[],detectedObjects:[],objectStats:{persons:0,vehicles:0,animals:0,total:0},layers:{map:true,costmap:true,laser:true,path:true,humans:true,objects:true,robot:true},tool:'pan',isDragging:false,dragStart:{x:0,y:0},lastFrameTime:0,fps:0};
const rviz3d={scene:null,camera:null,renderer:null,controls:null,robot:null,laserPoints:null,objectMeshes:[],pathLine:null,mapPlane:null,mapTexture:null,footprint:null,gridHelper:null,isInitialized:false,currentView:"2d"};
const teleop={'u':{l:0.3,a:0.5},'i':{l:0.5,a:0},'o':{l:0.3,a:-0.5},'j':{l:0,a:0.8},'k':{l:0,a:0},'l':{l:0,a:-0.8},'m':{l:-0.3,a:-0.5},',':{l:-0.5,a:0},'.':{l:-0.3,a:0.5}};
const OBJECT_CLASSES={0:{name:'Person',color:'#00ff88',bg:'rgba(0,255,136,0.3)'},1:{name:'Vehicle',color:'#ff4444',bg:'rgba(255,68,68,0.3)'},2:{name:'Bag',color:'#888',bg:'rgba(136,136,136,0.3)'},3:{name:'Animal',color:'#ff8800',bg:'rgba(255,136,0,0.3)'},4:{name:'Electronics',color:'#0088ff',bg:'rgba(0,136,255,0.3)'}};

function setNavMode(mode){navState.mode=mode;document.querySelectorAll('.nav-mode-btn').forEach(b=>b.classList.remove('active'));document.getElementById('nav-mode-'+mode).classList.add('active');document.getElementById('map-select-container').style.display=mode==='localization'?'block':'none';document.getElementById('map-save-container').style.display=mode==='mapping'?'flex':'none';if(mode==='localization')loadMaps();}
function setPerception(p){navState.perception=p;document.querySelectorAll('.perception-btn').forEach(b=>b.classList.remove('active'));document.getElementById('perception-'+p).classList.add('active');}
function loadMaps(){fetch('/maps/list').then(r=>r.json()).then(maps=>{const sel=document.getElementById('map-select');sel.innerHTML='<option value="">-- Select Map --</option>';maps.forEach(m=>{sel.innerHTML+=`<option value="${m.path}">${m.name}</option>`;});});}
function saveMap(){const name=document.getElementById('map-name-input').value.trim();if(!name){alert('Enter map name');return;}fetch('/maps/save',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({name})}).then(r=>r.json()).then(d=>{alert(d.message);if(d.success)loadMaps();});}
function startNavigation(){const body={mode:navState.mode,perception:navState.perception};if(navState.mode==='localization')body.map=navState.map;fetch('/navigation/start',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(body)}).then(r=>r.json()).then(d=>{if(d.success)updateNavStatus();else alert(d.message);});}
function stopNavigation(){fetch('/navigation/stop',{method:'POST'}).then(()=>updateNavStatus());}
function updateNavStatus(){fetch('/nodes/status').then(r=>r.json()).then(d=>{const nav=d.navigation;const el=document.getElementById('nav-status-value');if(nav.status==='running'){el.textContent='RUNNING';el.style.color='#00ff88';}else{el.textContent='STOPPED';el.style.color='#888';}});}

function connectROS(){const wsUrl='ws://'+window.location.hostname+':9090';ros=new ROSLIB.Ros({url:wsUrl});ros.on('connection',()=>{rosConnected=true;document.getElementById('ros-status').className='ros-status connected';document.getElementById('ros-status').textContent='● Connected';subscribeTopics();subscribeObjectDetection();if(currentTab==='cameras')subscribeCameras();});ros.on('error',e=>console.log('ROS error:',e));ros.on('close',()=>{rosConnected=false;document.getElementById('ros-status').className='ros-status disconnected';document.getElementById('ros-status').textContent='● Disconnected';setTimeout(connectROS,3000);});}

function subscribeTopics(){new ROSLIB.Topic({ros,name:'/map',messageType:'nav_msgs/OccupancyGrid'}).subscribe(m=>{rviz.mapInfo=m.info;rviz.map=m.data;});new ROSLIB.Topic({ros,name:'/global_costmap/costmap',messageType:'nav_msgs/OccupancyGrid'}).subscribe(m=>{rviz.costmapInfo=m.info;rviz.costmap=m.data;});new ROSLIB.Topic({ros,name:'/odom',messageType:'nav_msgs/Odometry'}).subscribe(m=>{rviz.robotPose.x=m.pose.pose.position.x;rviz.robotPose.y=m.pose.pose.position.y;const q=m.pose.pose.orientation;rviz.robotPose.theta=Math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z));document.getElementById('robot-pos').textContent=`(${rviz.robotPose.x.toFixed(1)},${rviz.robotPose.y.toFixed(1)})`;});new ROSLIB.Topic({ros,name:'/plan',messageType:'nav_msgs/Path'}).subscribe(m=>{rviz.globalPath=m.poses.map(p=>({x:p.pose.position.x,y:p.pose.position.y}));});new ROSLIB.Topic({ros,name:'/local_plan',messageType:'nav_msgs/Path'}).subscribe(m=>{rviz.localPath=m.poses.map(p=>({x:p.pose.position.x,y:p.pose.position.y}));});new ROSLIB.Topic({ros,name:'/scan',messageType:'sensor_msgs/LaserScan',throttle_rate:100}).subscribe(m=>{rviz.laserScan=[];for(let i=0;i<m.ranges.length;i++){const r=m.ranges[i];if(r>=m.range_min&&r<=m.range_max){const a=m.angle_min+i*m.angle_increment;rviz.laserScan.push({x:r*Math.cos(a),y:r*Math.sin(a)});}}});new ROSLIB.Topic({ros,name:'/human_clusters/markers',messageType:'visualization_msgs/MarkerArray'}).subscribe(m=>{rviz.humans=m.markers.filter(x=>x.action===0).map(x=>({x:x.pose.position.x,y:x.pose.position.y}));});new ROSLIB.Topic({ros,name:'/goal_pose',messageType:'geometry_msgs/PoseStamped'}).subscribe(m=>{rviz.goal={x:m.pose.position.x,y:m.pose.position.y};});}

function subscribeObjectDetection(){
// Subscribe to all 3 cameras
['front','left','right'].forEach(cam=>{
const topic=`/zedx_${cam}/obj_det_clean`;
new ROSLIB.Topic({ros,name:topic,messageType:'zed_msgs/ObjectsStamped',throttle_rate:200}).subscribe(msg=>{
if(msg.objects)processObjectDetections(msg.objects,cam);
});
});
// Subscribe to merged markers from bridge
new ROSLIB.Topic({ros,name:'/object_markers',messageType:'visualization_msgs/MarkerArray',throttle_rate:200}).subscribe(msg=>{
rviz.detectedObjects=msg.markers.filter(m=>m.action===0).map(m=>{
let classId=0;const r=m.color.r,g=m.color.g;
if(r>0.8&&g<0.3)classId=1;else if(r>0.8&&g>0.4)classId=3;else if(g>0.8)classId=0;else classId=2;
return{id:m.id,x:m.pose.position.x,y:m.pose.position.y,width:m.scale.x,height:m.scale.y,classId,camera:m.ns.replace('detected_objects_','')};
});
});
}

function processObjectDetections(objects,camera){
const stats={persons:0,vehicles:0,animals:0,total:0};
cameraDetections[camera]=objects.filter(o=>o.tracking_state===1).map(obj=>{
const classId=obj.label?.toUpperCase()==='PERSON'?0:obj.label?.toUpperCase()==='VEHICLE'?1:obj.label?.toUpperCase()==='ANIMAL'?3:2;
if(classId===0)stats.persons++;else if(classId===1)stats.vehicles++;else if(classId===3)stats.animals++;
stats.total++;
return{id:obj.label_id,classId,label:obj.label||OBJECT_CLASSES[classId]?.name||'Object',confidence:obj.confidence||0,bbox:obj.bounding_box_2d?.corners||[],distance:obj.position?Math.sqrt(obj.position[0]**2+obj.position[1]**2):0,camera};
});
cameraStats[camera]=stats;
updateObjectDetectionUI();
if(bboxEnabled)drawBoundingBoxes(camera);
document.getElementById('det-count-'+camera).textContent=stats.total;
}

function drawBoundingBoxes(camera){const canvas=document.getElementById('bbox-'+camera);const img=document.getElementById('img-'+camera);if(!canvas||!img)return;const ctx=canvas.getContext('2d');canvas.width=img.naturalWidth||1280;canvas.height=img.naturalHeight||720;ctx.clearRect(0,0,canvas.width,canvas.height);const detections=cameraDetections[camera]||[];detections.forEach(det=>{if(det.bbox.length<4)return;const classInfo=OBJECT_CLASSES[det.classId]||OBJECT_CLASSES[0];const x1=det.bbox[0]?.x||0,y1=det.bbox[0]?.y||0,x2=det.bbox[2]?.x||0,y2=det.bbox[2]?.y||0;const width=x2-x1,height=y2-y1;ctx.fillStyle=classInfo.bg;ctx.fillRect(x1,y1,width,height);ctx.strokeStyle=classInfo.color;ctx.lineWidth=3;ctx.strokeRect(x1,y1,width,height);const label=`${classInfo.name} ${det.confidence?Math.round(det.confidence)+'%':''} ${det.distance?det.distance.toFixed(1)+'m':''}`;ctx.font='bold 14px sans-serif';const textWidth=ctx.measureText(label).width;ctx.fillStyle=classInfo.color;ctx.fillRect(x1,y1-20,textWidth+8,20);ctx.fillStyle='#000';ctx.fillText(label,x1+4,y1-5);});const totalDets=Object.values(cameraDetections).reduce((sum,arr)=>sum+arr.length,0);document.getElementById('cam-det-count').textContent=totalDets;}

function toggleBoundingBoxes(enabled){bboxEnabled=enabled;if(!enabled){['left','front','right'].forEach(cam=>{const canvas=document.getElementById('bbox-'+cam);if(canvas){const ctx=canvas.getContext('2d');ctx.clearRect(0,0,canvas.width,canvas.height);}});}}

function updateObjectDetectionUI(){
// Per-camera counts
document.getElementById('cam-obj-front').textContent=cameraStats.front.total;
document.getElementById('cam-obj-left').textContent=cameraStats.left.total;
document.getElementById('cam-obj-right').textContent=cameraStats.right.total;
// Totals across all cameras
const totalStats={persons:cameraStats.front.persons+cameraStats.left.persons+cameraStats.right.persons,vehicles:cameraStats.front.vehicles+cameraStats.left.vehicles+cameraStats.right.vehicles,animals:cameraStats.front.animals+cameraStats.left.animals+cameraStats.right.animals,total:cameraStats.front.total+cameraStats.left.total+cameraStats.right.total};
document.getElementById('obj-stat-person').textContent=totalStats.persons;
document.getElementById('obj-stat-vehicle').textContent=totalStats.vehicles;
document.getElementById('obj-stat-animal').textContent=totalStats.animals;
document.getElementById('obj-stat-total').textContent=totalStats.total;
document.getElementById('obj-det-count').textContent=totalStats.total;
rviz.objectStats=totalStats;
updateObjectList();
}

function updateObjectList(){
const container=document.getElementById('obj-list-items');
const allDets=[...(cameraDetections.front||[]).map(d=>({...d,cam:'front'})),...(cameraDetections.left||[]).map(d=>({...d,cam:'left'})),...(cameraDetections.right||[]).map(d=>({...d,cam:'right'}))];
if(allDets.length===0){container.innerHTML='<div class="obj-list-empty">No objects detected</div>';return;}
let html='';
allDets.slice(0,20).forEach((det,i)=>{
const classInfo=OBJECT_CLASSES[det.classId]||OBJECT_CLASSES[0];
const className=classInfo.name.toLowerCase();
html+=`<div class="obj-list-item"><span>#${i+1}</span><span><span class="cam-badge ${det.cam}">${det.cam.charAt(0).toUpperCase()}</span></span><span><span class="obj-class-badge ${className}">${classInfo.name}</span></span><span>${det.confidence?Math.round(det.confidence)+'%':'-'}</span><span>${det.distance?det.distance.toFixed(1)+'m':'-'}</span></div>`;
});
container.innerHTML=html;
}

function motorCmd(cmd){const logEl=document.getElementById('motor-log');const ts=new Date().toLocaleTimeString();let endpoint='';switch(cmd){case'reset':endpoint='/motor/reset';break;case'init':endpoint='/motor/init';break;case'halt':endpoint='/motor/halt';break;case'resume':endpoint='/motor/resume';break;case'park_on':endpoint='/motor/park?enable=true';break;case'park_off':endpoint='/motor/park?enable=false';break;case'fault':endpoint='/motor/fault';break;case'temp':endpoint='/motor/temp';break;}logEl.innerHTML+=`<div class="motor-log-entry">[${ts}] ${cmd}...</div>`;logEl.scrollTop=logEl.scrollHeight;fetch(endpoint,{method:'POST'}).then(r=>r.json()).then(data=>{const cls=data.success?'success':'error';logEl.innerHTML+=`<div class="motor-log-entry ${cls}">[${ts}] ${data.output||'Done'}</div>`;logEl.scrollTop=logEl.scrollHeight;}).catch(err=>{logEl.innerHTML+=`<div class="motor-log-entry error">[${ts}] Error: ${err}</div>`;logEl.scrollTop=logEl.scrollHeight;});}

function setMapView(view){
rviz3d.currentView=view;
document.getElementById("view-2d-btn").classList.toggle("active",view==="2d");
document.getElementById("view-3d-btn").classList.toggle("active",view==="3d");
document.getElementById("rviz-2d-container").classList.toggle("hidden",view==="3d");
document.getElementById("rviz-3d-container").classList.toggle("active",view==="3d");
if(view==="3d"&&!rviz3d.isInitialized)init3D();
if(view==="3d")animate3D();
}
function init3D(){
const container=document.getElementById("rviz-3d-container");
const canvas=document.getElementById("rviz-3d-canvas");
rviz3d.scene=new THREE.Scene();
rviz3d.scene.background=new THREE.Color(0x1a1a2e);
rviz3d.camera=new THREE.PerspectiveCamera(60,container.clientWidth/container.clientHeight,0.1,1000);
rviz3d.camera.position.set(3,3,3);
rviz3d.camera.lookAt(0,0,0);
rviz3d.renderer=new THREE.WebGLRenderer({canvas,antialias:true});
rviz3d.renderer.setSize(container.clientWidth,container.clientHeight);
rviz3d.renderer.setPixelRatio(window.devicePixelRatio);
rviz3d.controls=new THREE.OrbitControls(rviz3d.camera,rviz3d.renderer.domElement);
rviz3d.controls.enableDamping=true;
rviz3d.controls.dampingFactor=0.05;
rviz3d.controls.maxPolarAngle=Math.PI/2;
const ambientLight=new THREE.AmbientLight(0xffffff,0.6);
rviz3d.scene.add(ambientLight);
const dirLight=new THREE.DirectionalLight(0xffffff,0.8);
dirLight.position.set(5,10,5);
rviz3d.scene.add(dirLight);
rviz3d.gridHelper=new THREE.GridHelper(20,20,0x444444,0x222222);
rviz3d.gridHelper.rotation.x=Math.PI/2;
rviz3d.scene.add(rviz3d.gridHelper);
const axesHelper=new THREE.AxesHelper(1);
rviz3d.scene.add(axesHelper);
loadRobotURDF();
createLaserPoints();
createFootprint();
rviz3d.isInitialized=true;
window.addEventListener("resize",onResize3D);
}
function onResize3D(){
if(!rviz3d.renderer)return;
const container=document.getElementById("rviz-3d-container");
rviz3d.camera.aspect=container.clientWidth/container.clientHeight;
rviz3d.camera.updateProjectionMatrix();
rviz3d.renderer.setSize(container.clientWidth,container.clientHeight);
}
function animate3D(){
if(rviz3d.currentView!=="3d")return;
requestAnimationFrame(animate3D);
rviz3d.controls.update();
updateRobot3D();
updateLaser3D();
updateObjects3D();
updatePath3D();
updateMap3D();
rviz3d.renderer.render(rviz3d.scene,rviz3d.camera);
}
function loadRobotURDF(){
const manager=new THREE.LoadingManager();
const loader=new URDFLoader(manager);
loader.loadMeshCb=function(path,manager,done){
const stlLoader=new THREE.STLLoader(manager);
const meshPath=path;
stlLoader.load(meshPath,function(geometry){
const material=new THREE.MeshPhongMaterial({color:0x00ff88,flatShading:false});
const mesh=new THREE.Mesh(geometry,material);
done(mesh);
},undefined,function(err){console.log("STL load error:",path);done(null);});
};
loader.load("/assets/robot.urdf",function(robot){
rviz3d.robot=robot;
robot.rotation.set(0, 0, 0);
robot.traverse(function(child){
if(child.isMesh){
child.material=new THREE.MeshPhongMaterial({color:0x00ff88,flatShading:false,transparent:true,opacity:0.9});
child.castShadow=true;
child.receiveShadow=true;
}
});
robot.visible=true;
rviz3d.scene.add(robot);
console.log("Robot URDF loaded");
},function(progress){},function(err){console.log("URDF load error:",err);createFallbackRobot();});
}
function createFallbackRobot(){
const bodyGeom=new THREE.BoxGeometry(0.5,0.3,0.2);
const bodyMat=new THREE.MeshPhongMaterial({color:0x00ff88});
const body=new THREE.Mesh(bodyGeom,bodyMat);
body.position.z=0.15;
const arrowGeom=new THREE.ConeGeometry(0.1,0.2,8);
const arrowMat=new THREE.MeshPhongMaterial({color:0xff0000});
const arrow=new THREE.Mesh(arrowGeom,arrowMat);
arrow.rotation.z=-Math.PI/2;
arrow.position.set(0.35,0,0.15);
rviz3d.robot=new THREE.Group();
rviz3d.robot.add(body);
rviz3d.robot.add(arrow);
rviz3d.scene.add(rviz3d.robot);
console.log("Fallback robot created");
}
function createLaserPoints(){
const geometry=new THREE.BufferGeometry();
const positions=new Float32Array(3600);
geometry.setAttribute("position",new THREE.BufferAttribute(positions,3));
const material=new THREE.PointsMaterial({color:0xff4444,size:0.05});
rviz3d.laserPoints=new THREE.Points(geometry,material);
rviz3d.scene.add(rviz3d.laserPoints);
}
function createFootprint(){
const shape=new THREE.Shape();
shape.moveTo(0.5,0.4);
shape.lineTo(0.5,-0.4);
shape.lineTo(-0.4,-0.4);
shape.lineTo(-0.4,0.4);
shape.lineTo(0.5,0.4);
const geometry=new THREE.ShapeGeometry(shape);
const material=new THREE.MeshBasicMaterial({color:0x00ff88,transparent:true,opacity:0.3,side:THREE.DoubleSide});
rviz3d.footprint=new THREE.Mesh(geometry,material);
rviz3d.footprint.rotation.set(0, 0, 0);
rviz3d.footprint.position.z=0.01;
rviz3d.scene.add(rviz3d.footprint);
const edges=new THREE.EdgesGeometry(geometry);
const line=new THREE.LineSegments(edges,new THREE.LineBasicMaterial({color:0x00ff88}));
line.rotation.set(0, 0, 0);
line.position.z=0.02;
rviz3d.footprint.outline=line;
rviz3d.scene.add(line);
}
function updateMap3D(){
if(!rviz.map||!rviz.mapInfo||!rviz3d.scene)return;
if(rviz3d.mapPlane)rviz3d.scene.remove(rviz3d.mapPlane);
const info=rviz.mapInfo;
const w=info.width*info.resolution;
const h=info.height*info.resolution;
const canvas=document.createElement("canvas");
canvas.width=info.width;
canvas.height=info.height;
const ctx=canvas.getContext("2d");
const imgData=ctx.createImageData(info.width,info.height);
for(let i=0;i<rviz.map.length;i++){
const v=rviz.map[i];
const c=v===-1?50:(v===0?240:30);
imgData.data[i*4]=c;imgData.data[i*4+1]=c;imgData.data[i*4+2]=c;imgData.data[i*4+3]=255;
}
ctx.putImageData(imgData,0,0);
const texture=new THREE.CanvasTexture(canvas);
texture.flipY=false;
const geometry=new THREE.PlaneGeometry(w,h);
const material=new THREE.MeshBasicMaterial({map:texture,transparent:true,opacity:0.9});
rviz3d.mapPlane=new THREE.Mesh(geometry,material);
rviz3d.mapPlane.position.x=info.origin.position.x+w/2;
rviz3d.mapPlane.position.y=info.origin.position.y+h/2;
rviz3d.mapPlane.position.z=0;
rviz3d.scene.add(rviz3d.mapPlane);
}
function updateRobot3D(){
if(rviz3d.robot){
if(!rviz3d.robot.visible&&(rviz.robotPose.x!==0||rviz.robotPose.y!==0))rviz3d.robot.visible=true;
rviz3d.robot.position.x=rviz.robotPose.x;
rviz3d.robot.position.y=rviz.robotPose.y;
rviz3d.robot.rotation.z=rviz.robotPose.theta;
}
if(rviz3d.footprint){
rviz3d.footprint.position.x=rviz.robotPose.x;
rviz3d.footprint.position.y=rviz.robotPose.y;
rviz3d.footprint.rotation.z=rviz.robotPose.theta;
if(rviz3d.footprint.outline){
rviz3d.footprint.outline.position.x=rviz.robotPose.x;
rviz3d.footprint.outline.position.y=rviz.robotPose.y;
rviz3d.footprint.outline.rotation.z=rviz.robotPose.theta;
}
}
}
function updateLaser3D(){
if(!rviz3d.laserPoints||!rviz.laserScan.length)return;
const positions=rviz3d.laserPoints.geometry.attributes.position.array;
const pose=rviz.robotPose;
const lt=pose.theta+Math.PI;
for(let i=0;i<rviz.laserScan.length&&i<1200;i++){
const p=rviz.laserScan[i];
const wx=pose.x+p.x*Math.cos(lt)-p.y*Math.sin(lt);
const wy=pose.y+p.x*Math.sin(lt)+p.y*Math.cos(lt);
positions[i*3]=wx;
positions[i*3+1]=wy;
positions[i*3+2]=0.1;
}
for(let i=rviz.laserScan.length;i<1200;i++){
positions[i*3]=0;positions[i*3+1]=0;positions[i*3+2]=-10;
}
rviz3d.laserPoints.geometry.attributes.position.needsUpdate=true;
}
function updateObjects3D(){
rviz3d.objectMeshes.forEach(m=>rviz3d.scene.remove(m));
rviz3d.objectMeshes=[];
rviz.detectedObjects.forEach(obj=>{
const colors={0:0x00ff88,1:0xff4444,2:0x888888,3:0xff8800};
const geometry=new THREE.BoxGeometry(obj.width||0.5,obj.height||0.5,1.7);
const material=new THREE.MeshPhongMaterial({color:colors[obj.classId]||0x888888,transparent:true,opacity:0.7});
const mesh=new THREE.Mesh(geometry,material);
mesh.position.set(obj.x,obj.y,0.85);
rviz3d.scene.add(mesh);
rviz3d.objectMeshes.push(mesh);
});
}
function updatePath3D(){
if(rviz3d.pathLine)rviz3d.scene.remove(rviz3d.pathLine);
if(rviz.globalPath.length<2)return;
const points=rviz.globalPath.map(p=>new THREE.Vector3(p.x,p.y,0.05));
const geometry=new THREE.BufferGeometry().setFromPoints(points);
const material=new THREE.LineBasicMaterial({color:0x00ff00,linewidth:2});
rviz3d.pathLine=new THREE.Line(geometry,material);
rviz3d.scene.add(rviz3d.pathLine);
}
function initRViz(){rviz.canvas=document.getElementById('rviz-canvas');rviz.ctx=rviz.canvas.getContext('2d');resizeCanvas();window.addEventListener('resize',resizeCanvas);rviz.canvas.addEventListener('mousedown',onCanvasMouseDown);rviz.canvas.addEventListener('mousemove',onCanvasMouseMove);rviz.canvas.addEventListener('mouseup',()=>rviz.isDragging=false);rviz.canvas.addEventListener('wheel',onCanvasWheel);rviz.canvas.addEventListener('touchstart',onCanvasTouchStart);rviz.canvas.addEventListener('touchmove',onCanvasTouchMove);rviz.canvas.addEventListener('touchend',()=>{rviz.isDragging=false;lastTouchDist=0;});requestAnimationFrame(renderLoop);}
function resizeCanvas(){const rect=rviz.canvas.getBoundingClientRect();rviz.canvas.width=rect.width*window.devicePixelRatio;rviz.canvas.height=rect.height*window.devicePixelRatio;rviz.ctx.scale(window.devicePixelRatio,window.devicePixelRatio);}
function worldToScreen(wx,wy){const cx=rviz.canvas.width/(2*window.devicePixelRatio);const cy=rviz.canvas.height/(2*window.devicePixelRatio);return{x:cx+(wx-rviz.viewX)*rviz.scale,y:cy-(wy-rviz.viewY)*rviz.scale};}
function screenToWorld(sx,sy){const cx=rviz.canvas.width/(2*window.devicePixelRatio);const cy=rviz.canvas.height/(2*window.devicePixelRatio);return{x:rviz.viewX+(sx-cx)/rviz.scale,y:rviz.viewY-(sy-cy)/rviz.scale};}
function renderLoop(ts){if(rviz.lastFrameTime){rviz.fps=Math.round(1000/(ts-rviz.lastFrameTime));document.getElementById('fps-counter').textContent=rviz.fps;}rviz.lastFrameTime=ts;render();requestAnimationFrame(renderLoop);}
function render(){const ctx=rviz.ctx;const w=rviz.canvas.width/window.devicePixelRatio;const h=rviz.canvas.height/window.devicePixelRatio;ctx.fillStyle='#1a1a2e';ctx.fillRect(0,0,w,h);drawGrid(ctx,w,h);if(rviz.layers.map&&rviz.map&&rviz.mapInfo)drawMap(ctx);if(rviz.layers.costmap&&rviz.costmap&&rviz.costmapInfo)drawCostmap(ctx,rviz.costmap,rviz.costmapInfo,'rgba(255,170,0,0.3)');if(rviz.layers.path){drawPath(ctx,rviz.globalPath,'#00ff00',2);drawPath(ctx,rviz.localPath,'#00aaff',2);}if(rviz.layers.laser&&rviz.laserScan.length)drawLaserScan(ctx);if(rviz.layers.objects&&rviz.detectedObjects.length)drawDetectedObjects(ctx);if(rviz.layers.humans)rviz.humans.forEach(h=>drawHuman(ctx,h.x,h.y));if(rviz.goal)drawGoal(ctx,rviz.goal.x,rviz.goal.y);if(rviz.layers.robot)drawRobot(ctx);}
function drawGrid(ctx,w,h){ctx.strokeStyle='#2a2a4e';ctx.lineWidth=1;const startWorld=screenToWorld(0,h),endWorld=screenToWorld(w,0);for(let wx=Math.floor(startWorld.x);wx<endWorld.x;wx++){const s=worldToScreen(wx,0);ctx.beginPath();ctx.moveTo(s.x,0);ctx.lineTo(s.x,h);ctx.stroke();}for(let wy=Math.floor(startWorld.y);wy<endWorld.y;wy++){const s=worldToScreen(0,wy);ctx.beginPath();ctx.moveTo(0,s.y);ctx.lineTo(w,s.y);ctx.stroke();}}
function drawMap(ctx){const{width,height,resolution,origin}=rviz.mapInfo;const ox=origin.position.x,oy=origin.position.y;const ps=Math.max(1,resolution*rviz.scale);for(let y=0;y<height;y++){for(let x=0;x<width;x++){const v=rviz.map[y*width+x];if(v===-1)continue;const s=worldToScreen(ox+x*resolution,oy+y*resolution);ctx.fillStyle=v===0?'#e0e0e0':'#303030';ctx.fillRect(s.x,s.y-ps,ps+1,ps+1);}}}
function drawCostmap(ctx,data,info,color){const{width,height,resolution,origin}=info;const ps=Math.max(1,resolution*rviz.scale);for(let y=0;y<height;y+=2){for(let x=0;x<width;x+=2){const v=data[y*width+x];if(v<50)continue;const s=worldToScreen(origin.position.x+x*resolution,origin.position.y+y*resolution);ctx.fillStyle=color.replace('0.3',(Math.min(v/100,1)*0.5).toFixed(2));ctx.fillRect(s.x,s.y-ps*2,ps*2+1,ps*2+1);}}}
function drawPath(ctx,path,color,width){if(path.length<2)return;ctx.strokeStyle=color;ctx.lineWidth=width;ctx.beginPath();const f=worldToScreen(path[0].x,path[0].y);ctx.moveTo(f.x,f.y);path.slice(1).forEach(p=>{const s=worldToScreen(p.x,p.y);ctx.lineTo(s.x,s.y);});ctx.stroke();}
function drawLaserScan(ctx){ctx.fillStyle='#ff4444';const{x,y,theta}=rviz.robotPose;const lt=theta+Math.PI;rviz.laserScan.forEach(p=>{const wx=x+p.x*Math.cos(lt)-p.y*Math.sin(lt);const wy=y+p.x*Math.sin(lt)+p.y*Math.cos(lt);const s=worldToScreen(wx,wy);ctx.fillRect(s.x-1,s.y-1,3,3);});}
function drawDetectedObjects(ctx){rviz.detectedObjects.forEach(obj=>{const ci=OBJECT_CLASSES[obj.classId]||OBJECT_CLASSES[0];const s=worldToScreen(obj.x,obj.y);const sw=Math.max(18,obj.width*rviz.scale);const sh=Math.max(18,obj.height*rviz.scale);ctx.strokeStyle=ci.color;ctx.lineWidth=2;ctx.strokeRect(s.x-sw/2,s.y-sh/2,sw,sh);ctx.fillStyle=ci.color;ctx.beginPath();ctx.arc(s.x,s.y,8,0,Math.PI*2);ctx.fill();ctx.fillStyle='#000';ctx.font='bold 8px sans-serif';ctx.textAlign='center';ctx.textBaseline='middle';ctx.fillText(ci.name.charAt(0),s.x,s.y);});}
function drawRobot(ctx){
const s=worldToScreen(rviz.robotPose.x,rviz.robotPose.y);
const scale=rviz.scale;
ctx.save();
ctx.translate(s.x,s.y);
ctx.rotate(-rviz.robotPose.theta);
const fw=0.5*scale,fb=0.4*scale,fh=0.4*scale;
ctx.fillStyle="rgba(0,255,136,0.2)";
ctx.fillRect(-fb,-fh,fw+fb,fh*2);
ctx.strokeStyle="#00ff88";
ctx.lineWidth=2;
ctx.strokeRect(-fb,-fh,fw+fb,fh*2);
const bw=0.45*scale,bb=0.35*scale,bh=0.35*scale;
ctx.fillStyle="#00aa55";
ctx.fillRect(-bb,-bh,bw+bb,bh*2);
ctx.strokeStyle="#008844";
ctx.strokeRect(-bb,-bh,bw+bb,bh*2);
const ww=0.08*scale,wh=0.06*scale,wo=0.22*scale,wd=0.28*scale;
ctx.fillStyle="#333";
ctx.fillRect(wd-ww,-wo-wh,ww*2,wh*2);
ctx.fillRect(wd-ww,wo-wh,ww*2,wh*2);
ctx.fillRect(-wd-ww,-wo-wh,ww*2,wh*2);
ctx.fillRect(-wd-ww,wo-wh,ww*2,wh*2);
ctx.fillStyle="#e94560";
ctx.beginPath();
ctx.moveTo(fw+0.05*scale,0);
ctx.lineTo(fw-0.05*scale,-0.08*scale);
ctx.lineTo(fw-0.05*scale,0.08*scale);
ctx.closePath();
ctx.fill();
ctx.restore();
}
function drawGoal(ctx,x,y){const s=worldToScreen(x,y);ctx.fillStyle='#ffcc00';ctx.beginPath();for(let i=0;i<5;i++){const a=(i*144-90)*Math.PI/180;const px=s.x+10*Math.cos(a),py=s.y+10*Math.sin(a);i===0?ctx.moveTo(px,py):ctx.lineTo(px,py);}ctx.closePath();ctx.fill();}
function drawHuman(ctx,x,y){const s=worldToScreen(x,y);ctx.fillStyle='#ff00ff';ctx.beginPath();ctx.arc(s.x,s.y,8,0,Math.PI*2);ctx.fill();ctx.fillStyle='#fff';ctx.font='bold 8px sans-serif';ctx.textAlign='center';ctx.textBaseline='middle';ctx.fillText('H',s.x,s.y);}
function onCanvasMouseDown(e){const rect=rviz.canvas.getBoundingClientRect();const x=e.clientX-rect.left,y=e.clientY-rect.top;if(rviz.tool==='pan'){rviz.isDragging=true;rviz.dragStart={x,y};}else if(rviz.tool==='goal'){const w=screenToWorld(x,y);publishGoal(w.x,w.y);}else if(rviz.tool==='pose'){const w=screenToWorld(x,y);publishInitialPose(w.x,w.y);}}
function onCanvasMouseMove(e){if(!rviz.isDragging)return;const rect=rviz.canvas.getBoundingClientRect();const x=e.clientX-rect.left,y=e.clientY-rect.top;rviz.viewX-=(x-rviz.dragStart.x)/rviz.scale;rviz.viewY+=(y-rviz.dragStart.y)/rviz.scale;rviz.dragStart={x,y};}
function onCanvasWheel(e){e.preventDefault();rviz.scale*=e.deltaY>0?0.9:1.1;rviz.scale=Math.max(10,Math.min(200,rviz.scale));}
let lastTouchDist=0;
function onCanvasTouchStart(e){e.preventDefault();if(e.touches.length===1){const rect=rviz.canvas.getBoundingClientRect();const x=e.touches[0].clientX-rect.left,y=e.touches[0].clientY-rect.top;if(rviz.tool==='pan'){rviz.isDragging=true;rviz.dragStart={x,y};}else if(rviz.tool==='goal'){const w=screenToWorld(x,y);publishGoal(w.x,w.y);}}else if(e.touches.length===2){lastTouchDist=Math.hypot(e.touches[0].clientX-e.touches[1].clientX,e.touches[0].clientY-e.touches[1].clientY);}}
function onCanvasTouchMove(e){e.preventDefault();if(e.touches.length===1&&rviz.isDragging){const rect=rviz.canvas.getBoundingClientRect();const x=e.touches[0].clientX-rect.left,y=e.touches[0].clientY-rect.top;rviz.viewX-=(x-rviz.dragStart.x)/rviz.scale;rviz.viewY+=(y-rviz.dragStart.y)/rviz.scale;rviz.dragStart={x,y};}else if(e.touches.length===2){const d=Math.hypot(e.touches[0].clientX-e.touches[1].clientX,e.touches[0].clientY-e.touches[1].clientY);if(lastTouchDist>0)rviz.scale=Math.max(10,Math.min(200,rviz.scale*d/lastTouchDist));lastTouchDist=d;}}
function publishGoal(x,y){if(!rosConnected)return;new ROSLIB.Topic({ros,name:'/goal_pose',messageType:'geometry_msgs/PoseStamped'}).publish(new ROSLIB.Message({header:{frame_id:'map'},pose:{position:{x,y,z:0},orientation:{x:0,y:0,z:0,w:1}}}));rviz.goal={x,y};}
function publishInitialPose(x,y){if(!rosConnected)return;new ROSLIB.Topic({ros,name:'/initialpose',messageType:'geometry_msgs/PoseWithCovarianceStamped'}).publish(new ROSLIB.Message({header:{frame_id:'map'},pose:{pose:{position:{x,y,z:0},orientation:{x:0,y:0,z:0,w:1}},covariance:new Array(36).fill(0)}}));}
function setTool(t){rviz.tool=t;document.querySelectorAll('.rviz-tool').forEach(b=>b.classList.remove('active'));document.getElementById('tool-'+t).classList.add('active');}
function toggleLayer(l){rviz.layers[l]=!rviz.layers[l];document.getElementById('layer-'+l).classList.toggle('active');}
function resetView(){rviz.viewX=rviz.robotPose.x;rviz.viewY=rviz.robotPose.y;rviz.scale=50;}
function zoomIn(){rviz.scale=Math.min(200,rviz.scale*1.2);}
function zoomOut(){rviz.scale=Math.max(10,rviz.scale*0.8);}

let cameraView='all';
const cameraTopics={left:'/zedx_left/zed_node/rgb/color/rect/image/compressed',front:'/zedx_front/zed_node/rgb/color/rect/image/compressed',right:'/zedx_right/zed_node/rgb/color/rect/image/compressed'};
let cameraSubscribers={};
function setCameraView(v){cameraView=v;document.querySelectorAll('.cam-btn').forEach(b=>b.classList.remove('active'));document.getElementById('cam-btn-'+v).classList.add('active');const grid=document.getElementById('camera-grid');if(v==='all'){grid.classList.remove('single');['left','front','right'].forEach(c=>document.getElementById('cam-'+c).classList.remove('hidden'));}else{grid.classList.add('single');['left','front','right'].forEach(c=>document.getElementById('cam-'+c).classList.toggle('hidden',c!==v));}}
function subscribeCameras(){if(!rosConnected)return;['left','front','right'].forEach(cam=>{if(cameraSubscribers[cam])return;cameraSubscribers[cam]=new ROSLIB.Topic({ros,name:cameraTopics[cam],messageType:'sensor_msgs/CompressedImage',throttle_rate:100});cameraSubscribers[cam].subscribe(msg=>{const img=document.getElementById('img-'+cam);if(img){img.src='data:image/jpeg;base64,'+msg.data;img.onload=()=>{if(bboxEnabled)drawBoundingBoxes(cam);};document.getElementById('cam-stream-status').textContent='Active';document.getElementById('cam-stream-status').style.color='#00ff88';}});});}
function unsubscribeCameras(){Object.values(cameraSubscribers).forEach(s=>s&&s.unsubscribe());cameraSubscribers={};}

function showTab(n){if(currentTab==='cameras'&&n!=='cameras')unsubscribeCameras();currentTab=n;document.querySelectorAll('.tab').forEach(t=>t.classList.remove('active'));document.querySelectorAll('.tab-content').forEach(t=>t.classList.remove('active'));event.target.classList.add('active');document.getElementById('tab-'+n).classList.add('active');if(n==='rviz')resizeCanvas();if(n==='cameras'&&rosConnected)subscribeCameras();if(n==='nav'){loadMaps();updateNavStatus();}}
function updateConnectionStatus(online){const el=document.getElementById('connection-status');el.className='connection-status '+(online?'online':'offline');el.textContent=online?'● Online':'● Offline';}
function startTeleop(k){const c=teleop[k];if(!c)return;stopTeleop();sendTeleop(c.l,c.a);teleopInterval=setInterval(()=>sendTeleop(c.l,c.a),100);}
function stopTeleop(){if(teleopInterval){clearInterval(teleopInterval);teleopInterval=null;}sendTeleop(0,0);}
function sendTeleop(l,a){if(rosConnected){new ROSLIB.Topic({ros,name:'/cmd_vel',messageType:'geometry_msgs/Twist'}).publish(new ROSLIB.Message({linear:{x:l,y:0,z:0},angular:{x:0,y:0,z:a}}));}else{fetch('/teleop',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({linear:l,angular:a})});}}
function updateNodeList(){fetch('/nodes/status').then(r=>r.json()).then(d=>{updateConnectionStatus(true);let h='',q='',t='';Object.entries(d).sort((a,b)=>a[1].order-b[1].order).forEach(([id,i])=>{const r=i.status==='running';h+=`<div class="node-card ${i.status}"><div class="node-header"><span class="node-name" style="color:${i.color}">${i.name}</span><span class="node-status ${i.status}">${i.status.toUpperCase()}</span></div><div class="node-desc">${i.description}</div><div class="node-buttons"><button class="btn btn-start" onclick="startNode('${id}')" ${r?'disabled':''}>▶</button><button class="btn btn-stop" onclick="stopNode('${id}')" ${!r?'disabled':''}>⏹</button></div></div>`;q+=`<div style="display:flex;gap:4px;margin-bottom:4px;align-items:center"><span style="flex:1;color:${i.color};font-size:0.75em">${i.name}</span><span class="node-status ${i.status}" style="font-size:0.55em">${i.status.toUpperCase()}</span><button class="btn btn-start" onclick="startNode('${id}')" ${r?'disabled':''} style="padding:2px 4px;font-size:0.6em">▶</button><button class="btn btn-stop" onclick="stopNode('${id}')" ${!r?'disabled':''} style="padding:2px 4px;font-size:0.6em">⏹</button></div>`;t+=`<button class="log-tab ${id===currentLogTab?'active':''}" onclick="showLogs('${id}')">${i.name.replace(/[^a-zA-Z0-9 ]/g,'')}</button>`;if(!(id in logIndices))logIndices[id]=0;});document.getElementById('node-list').innerHTML=h;document.getElementById('quick-launch').innerHTML=q;document.getElementById('log-tabs').innerHTML=t;}).catch(()=>updateConnectionStatus(false));}
function startNode(id){fetch(`/nodes/${id}/start`,{method:'POST'}).then(()=>updateNodeList());}
function stopNode(id){fetch(`/nodes/${id}/stop`,{method:'POST'}).then(()=>setTimeout(updateNodeList,1500));}
function stopAll(){if(confirm('Stop all nodes?'))fetch('/nodes/stop_all',{method:'POST'}).then(()=>setTimeout(updateNodeList,2000));}
function showLogs(id){currentLogTab=id;document.getElementById('log-content').innerHTML='';logIndices[id]=0;document.querySelectorAll('.log-tab').forEach(t=>t.classList.remove('active'));event.target.classList.add('active');updateLogs();}
function updateLogs(){fetch(`/nodes/${currentLogTab}/logs?since=${logIndices[currentLogTab]||0}`).then(r=>r.json()).then(logs=>{const c=document.getElementById('log-content');logs.forEach(l=>{const d=document.createElement('div');d.className='log-line';d.innerHTML=`<span class="log-time">${l.time}</span><span class="log-${l.level}">${l.message.replace(/</g,'&lt;')}</span>`;c.appendChild(d);});logIndices[currentLogTab]=(logIndices[currentLogTab]||0)+logs.length;c.scrollTop=c.scrollHeight;});}
function updateStatus(){fetch('/status').then(r=>r.json()).then(d=>{const s=document.getElementById('header-state');s.innerText=d.mission_state;const colors={'IDLE':'#444','UNKNOWN':'#444','EXPLORING':'#00ff88','NAVIGATING_TO_POI':'#ffcc00','WAITING_FOR_PHOTO':'#e94560','PAUSED':'#ff8800'};s.style.background=colors[d.mission_state]||'#444';s.style.color=['EXPLORING','NAVIGATING_TO_POI','PAUSED'].includes(d.mission_state)?'#000':'#fff';document.getElementById('human-count').innerText=d.human_count;document.getElementById('poi-score').innerText=d.poi_score.toFixed(2);document.getElementById('should-capture').innerText=d.should_capture?'YES':'NO';document.getElementById('should-capture').style.color=d.should_capture?'#00ff88':'#ff4444';document.getElementById('poi-type').innerText=d.poi_type||'-';});}
function sendCommand(c){if(rosConnected){new ROSLIB.Topic({ros,name:'/mission/command',messageType:'std_msgs/String'}).publish(new ROSLIB.Message({data:c}));}else{fetch('/command',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({command:c})});}}

document.addEventListener('DOMContentLoaded',()=>{initRViz();connectROS();updateNodeList();updateStatus();setInterval(updateStatus,500);setInterval(updateNodeList,2000);setInterval(updateLogs,1000);document.addEventListener('keydown',e=>{if(teleop[e.key.toLowerCase()]&&currentTab==='teleop'){e.preventDefault();startTeleop(e.key.toLowerCase());}});document.addEventListener('keyup',e=>{if(teleop[e.key.toLowerCase()])stopTeleop();});});
if('serviceWorker' in navigator)navigator.serviceWorker.register('/sw.js').catch(()=>{});
</script>
</body></html>'''

MANIFEST_JSON='{"name":"MANRIIX Control","short_name":"MANRIIX","start_url":"/","display":"standalone","background_color":"#0d0d1a","theme_color":"#0d0d1a","icons":[{"src":"/icon-180.png","sizes":"180x180","type":"image/png"}]}'
SERVICE_WORKER_JS='self.addEventListener("install",()=>self.skipWaiting());self.addEventListener("activate",e=>e.waitUntil(clients.claim()));self.addEventListener("fetch",e=>e.respondWith(fetch(e.request).catch(()=>caches.match(e.request))));'

@app.route('/')
def index(): return render_template_string(HTML_TEMPLATE)
@app.route('/status')
def get_status(): return jsonify(status_data)
@app.route('/command', methods=['POST'])
def send_command():
    cmd = request.json.get('command', '')
    if cmd: subprocess.Popen(['ros2', 'topic', 'pub', '--once', '/mission/command', 'std_msgs/String', f'data: {cmd}'], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    return jsonify({'success': True})
@app.route('/teleop', methods=['POST'])
def send_teleop():
    d = request.json
    subprocess.Popen(['ros2', 'topic', 'pub', '--once', '/cmd_vel', 'geometry_msgs/Twist', f'{{linear: {{x: {d.get("linear",0)}, y: 0.0, z: 0.0}}, angular: {{x: 0.0, y: 0.0, z: {d.get("angular",0)}}}}}'], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    return jsonify({'success': True})
@app.route('/nodes/status')
def get_nodes_status(): return jsonify(node_manager.get_all_status())
@app.route('/nodes/<node_id>/status')
def get_node_status(node_id): return jsonify({'status': node_manager.get_status(node_id)})
@app.route('/nodes/<node_id>/start', methods=['POST'])
def start_node_route(node_id): return jsonify(node_manager.start_node(node_id))
@app.route('/nodes/<node_id>/stop', methods=['POST'])
def stop_node_route(node_id): return jsonify(node_manager.stop_node(node_id))
@app.route('/nodes/stop_all', methods=['POST'])
def stop_all_nodes(): return jsonify(node_manager.stop_all())
@app.route('/nodes/<node_id>/logs')
def get_logs(node_id): return jsonify(node_manager.get_logs(node_id, request.args.get('since', 0, type=int)))
@app.route('/navigation/start', methods=['POST'])
def start_navigation():
    data = request.json or {}
    mode = data.get('mode', 'slam_nav')
    perception = data.get('perception', '2d')
    map_file = data.get('map', '')
    return jsonify(node_manager.start_node('navigation', mode=mode, perception=perception, map=map_file))
@app.route('/navigation/stop', methods=['POST'])
def stop_navigation():
    return jsonify(node_manager.stop_node('navigation'))
@app.route('/maps/list')
def list_maps():
    return jsonify(get_available_maps())
@app.route('/maps/save', methods=['POST'])
def save_map_route():
    data = request.json or {}
    name = data.get('name', '')
    return jsonify(save_map(name))
@app.route('/motor/reset', methods=['POST'])
def motor_reset(): return jsonify(call_trigger_service('/reset_driver'))
@app.route('/motor/init', methods=['POST'])
def motor_init(): return jsonify(call_trigger_service('/initiate_motor'))
@app.route('/motor/halt', methods=['POST'])
def motor_halt(): return jsonify(call_setbool_service('/halt_motor', True))
@app.route('/motor/resume', methods=['POST'])
def motor_resume(): return jsonify(call_setbool_service('/halt_motor', False))
@app.route('/motor/park', methods=['POST'])
def motor_park(): return jsonify(call_setbool_service('/set_park_mode', request.args.get('enable', 'true').lower() == 'true'))
@app.route('/motor/fault', methods=['POST'])
def motor_fault(): return jsonify(call_trigger_service('/get_fault_info'))
@app.route('/motor/temp', methods=['POST'])
def motor_temp(): return jsonify(call_trigger_service('/get_motor_temperature'))
@app.route('/manifest.json')
def manifest(): return Response(MANIFEST_JSON, mimetype='application/json')
@app.route('/sw.js')
def service_worker(): return Response(SERVICE_WORKER_JS, mimetype='application/javascript')
@app.route('/icon-180.png')
def icon_180(): return Response(generate_app_icon(180), mimetype='image/png')
@app.route('/icon-192.png')
def icon_192(): return Response(generate_app_icon(192), mimetype='image/png')

# ============ URDF/3D Model Assets ============
@app.route("/assets/robot.urdf")
def serve_urdf():
    return send_from_directory(ASSETS_DIR, "robot.urdf", mimetype="application/xml")

@app.route("/assets/meshes/<path:filename>")
def serve_mesh(filename):
    return send_from_directory(os.path.join(ASSETS_DIR, "meshes"), filename)

def main():


    print("\n" + "="*60)
    print("  MANRIIX Control Center v12.1")
    print("  Multi-Camera Object Detection + Navigation Modes")
    print("="*60)
    print("\n  🌐 http://localhost:5000")
    print("\n  📋 Tabs: Control | Nav | Map | Cameras | Objects | Motor | Teleop | System | Logs")
    print("\n  ✨ v12.1 Features:")
    print("     👁️ Object detection from ALL 3 cameras (front, left, right)")
    print("     📊 Per-camera detection stats")
    print("     🗺️ Navigation modes (Mapping/Localization/SLAM+Nav)")
    print("     📡 2D/3D perception toggle")
    print("\n  Press Ctrl+C to stop\n")
    threading.Thread(target=update_status_loop, daemon=True).start()
    app.run(host='0.0.0.0', port=5000, debug=False, threaded=True)

if __name__ == '__main__':
    main()
