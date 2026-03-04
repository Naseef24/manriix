#!/usr/bin/env python3
"""
Manriix Display Control Panel v3.1
Features:
- Click on map to set initial pose (drag for direction)
- Mode switcher: Set Initial Pose / Add Waypoint
- Visual pose indicator with arrow
- Waypoint markers on map
- Wait time, loop settings
- Camera views for waypoint/detection modes
"""

from flask import Flask, render_template_string, jsonify, request
from flask_cors import CORS
import subprocess
import threading
import time
import signal
import os
import json
import math

app = Flask(__name__)
CORS(app)

MAPS_DIR = os.path.expanduser('~/manriix2_ws/src/manriix_navigation/maps')


class NodeManager:
    def __init__(self):
        self.processes = {}
        self.logs = {}
        self.max_log_lines = 500
        
        self.node_configs = {
            'robot_cameras': {
                'name': 'Robot + Cameras',
                'command': ['ros2', 'launch', 'manriix_bringup', 'robot.launch.py',
                           'hardware_mode:=real', 'launch_cameras:=true'],
                'cleanup_patterns': ['rplidar', 'zed', 'robot_state', 'controller', 'ekf', 
                                   'spawner', 'manriix_bringup', 'twist_mux']
            },
            'robot_no_cameras': {
                'name': 'Robot (No Cameras)',
                'command': ['ros2', 'launch', 'manriix_bringup', 'robot.launch.py',
                           'hardware_mode:=real', 'launch_cameras:=false'],
                'cleanup_patterns': ['rplidar', 'robot_state', 'controller', 'ekf', 
                                   'spawner', 'manriix_bringup', 'twist_mux']
            },
            'slam_2d': {
                'name': 'SLAM 2D',
                'command': ['ros2', 'launch', 'manriix_navigation', 'slam_nav.launch.py', 
                           'use_rviz:=true', 'use_3d_perception:=false'],
                'cleanup_patterns': ['slam_toolbox', 'nav2', 'bt_navigator', 'controller_server',
                                   'planner_server', 'behavior', 'smoother', 'lifecycle', 'rviz']
            },
            'slam_3d': {
                'name': 'SLAM 3D',
                'command': ['ros2', 'launch', 'manriix_navigation', 'slam_nav.launch.py', 
                           'use_rviz:=true', 'use_3d_perception:=true'],
                'cleanup_patterns': ['slam_toolbox', 'nav2', 'bt_navigator', 'controller_server',
                                   'planner_server', 'behavior', 'smoother', 'lifecycle', 
                                   'collision_monitor', 'rviz', 'stvl']
            },
            'localization': {
                'name': 'Localization + Nav2',
                'command': None,
                'cleanup_patterns': ['amcl', 'map_server', 'nav2', 'bt_navigator', 'controller_server',
                                   'planner_server', 'behavior', 'smoother', 'lifecycle', 'rviz',
                                   'waypoint_navigator', 'map_manager']
            },
            'detection': {
                'name': 'Detection System',
                'command': None,
                'cleanup_patterns': ['direct_zed_detection', 'human_clustering', 'poi_manager',
                                   'mission_controller', 'photo_intelligence', 'recovery_manager',
                                   'nav2', 'slam_toolbox']
            }
        }
        
        for node_id in self.node_configs:
            self.logs[node_id] = []
            self.processes[node_id] = None
    
    def start_node(self, node_id, **kwargs):
        if self._is_running(node_id):
            self._log(node_id, "Already running", 'warning')
            return {'success': False, 'message': 'Already running'}
        
        config = self.node_configs.get(node_id)
        if not config:
            return {'success': False, 'message': 'Unknown node'}
        
        if node_id == 'localization':
            map_file = kwargs.get('map', '')
            if not map_file:
                return {'success': False, 'message': 'No map specified'}
            use_3d = kwargs.get('use_3d', False)
            command = ['ros2', 'launch', 'manriix_navigation', 'waypoint_nav.launch.py',
                      f'map:={map_file}', 'use_rviz:=true',
                      f'use_3d_perception:={"true" if use_3d else "false"}']
        elif node_id == 'detection':
            enable_auto = kwargs.get('enable_autonomous', False)
            command = ['ros2', 'launch', 'manriix_perception', 'complete_photography_robot.launch.py',
                      f'enable_autonomous_navigation:={str(enable_auto).lower()}']
        else:
            command = config['command']
        
        if not command:
            return {'success': False, 'message': 'No command configured'}
        
        self.logs[node_id] = []
        self._log(node_id, f"Command: {' '.join(command)}", 'info')
        
        try:
            env = os.environ.copy()
            # env['DISPLAY'] = ':0'  # Commented out to allow SSH X11 forwarding
            process = subprocess.Popen(
                command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                stdin=subprocess.DEVNULL, env=env, bufsize=1,
                universal_newlines=True, start_new_session=True
            )
            self.processes[node_id] = process
            threading.Thread(target=self._read_output, args=(node_id, process), daemon=True).start()
            self._log(node_id, f"Started PID:{process.pid}", 'success')
            return {'success': True, 'message': f'Started PID:{process.pid}'}
        except Exception as e:
            self._log(node_id, f"Failed: {e}", 'error')
            return {'success': False, 'message': str(e)}
    
    def stop_node(self, node_id):
        config = self.node_configs.get(node_id)
        if not config:
            return {'success': False, 'message': 'Unknown node'}
        
        self._log(node_id, "Stopping...", 'info')
        
        process = self.processes.get(node_id)
        if process and process.poll() is None:
            try:
                pgid = os.getpgid(process.pid)
                os.killpg(pgid, signal.SIGINT)
                for _ in range(30):
                    if process.poll() is not None:
                        break
                    time.sleep(0.1)
                else:
                    os.killpg(pgid, signal.SIGKILL)
                    process.wait(timeout=2)
            except Exception as e:
                self._log(node_id, f"Stop error: {e}", 'warning')
        
        self.processes[node_id] = None
        
        for pattern in config.get('cleanup_patterns', []):
            try:
                subprocess.run(['pkill', '-9', '-f', pattern], capture_output=True, timeout=1)
            except:
                pass
        
        self._log(node_id, "Stopped", 'success')
        return {'success': True, 'message': 'Stopped'}
    
    def stop_all(self):
        for node_id in self.node_configs:
            self.stop_node(node_id)
        return {'success': True, 'message': 'All stopped'}
    
    def emergency_stop(self):
        try:
            subprocess.Popen(['ros2', 'topic', 'pub', '--once', '/cmd_vel', 
                            'geometry_msgs/msg/Twist', '{linear: {x: 0.0}, angular: {z: 0.0}}'],
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        except:
            pass
        return {'success': True, 'message': 'Emergency stop'}
    
    def _is_running(self, node_id):
        proc = self.processes.get(node_id)
        if proc is None:
            return False
        if proc.poll() is not None:
            self.processes[node_id] = None
            return False
        return True
    
    def get_status(self):
        return {node_id: self._is_running(node_id) for node_id in self.node_configs}
    
    def _read_output(self, node_id, process):
        try:
            for line in iter(process.stdout.readline, ''):
                if not line:
                    break
                level = 'info'
                ll = line.lower()
                if 'error' in ll or 'fatal' in ll:
                    level = 'error'
                elif 'warn' in ll:
                    level = 'warning'
                elif 'ready' in ll or 'active' in ll or 'started' in ll:
                    level = 'success'
                self._log(node_id, line.rstrip(), level)
        except:
            pass
    
    def _log(self, node_id, msg, level='info'):
        if node_id not in self.logs:
            self.logs[node_id] = []
        self.logs[node_id].append({
            'time': time.strftime('%H:%M:%S'),
            'msg': msg,
            'level': level
        })
        if len(self.logs[node_id]) > self.max_log_lines:
            self.logs[node_id] = self.logs[node_id][-self.max_log_lines:]
    
    def get_logs(self, node_id):
        return self.logs.get(node_id, [])


node_manager = NodeManager()


def get_maps():
    maps = []
    if os.path.exists(MAPS_DIR):
        for f in os.listdir(MAPS_DIR):
            if f.endswith('.yaml'):
                name = f[:-5]
                pgm = os.path.join(MAPS_DIR, f"{name}.pgm")
                if os.path.exists(pgm):
                    maps.append({
                        'name': name,
                        'path': os.path.join(MAPS_DIR, f),
                        'modified': time.strftime('%Y-%m-%d %H:%M', 
                                    time.localtime(os.path.getmtime(os.path.join(MAPS_DIR, f))))
                    })
    return sorted(maps, key=lambda x: x['modified'], reverse=True)


def save_map(name):
    name = "".join(c for c in name if c.isalnum() or c in '_-')
    if not name:
        return {'success': False, 'message': 'Invalid name'}
    path = os.path.join(MAPS_DIR, name)
    try:
        result = subprocess.run(['ros2', 'run', 'nav2_map_server', 'map_saver_cli', '-f', path],
                               capture_output=True, text=True, timeout=30)
        return {'success': result.returncode == 0, 'message': 'Map saved!' if result.returncode == 0 else result.stderr}
    except Exception as e:
        return {'success': False, 'message': str(e)}


def send_teleop(linear_x, angular_z):
    try:
        subprocess.Popen(['ros2', 'topic', 'pub', '--once', '/cmd_vel', 'geometry_msgs/msg/Twist',
                         f'{{linear: {{x: {linear_x}}}, angular: {{z: {angular_z}}}}}'],
                        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    except:
        pass


def motor_cmd(cmd):
    try:
        if cmd == 'init':
            subprocess.run(['ros2', 'service', 'call', '/initiate_motor', 'std_srvs/srv/Trigger'],
                          capture_output=True, timeout=5)
        elif cmd == 'reset':
            subprocess.run(['ros2', 'service', 'call', '/reset_driver', 'std_srvs/srv/Trigger'],
                          capture_output=True, timeout=5)
        elif cmd == 'halt':
            subprocess.run(['ros2', 'service', 'call', '/halt_motor', 'std_srvs/srv/SetBool', '{data: true}'],
                          capture_output=True, timeout=5)
        elif cmd == 'resume':
            subprocess.run(['ros2', 'service', 'call', '/halt_motor', 'std_srvs/srv/SetBool', '{data: false}'],
                          capture_output=True, timeout=5)
        return {'success': True}
    except:
        return {'success': False}


def set_initial_pose(x, y, theta):
    try:
        qz = math.sin(theta / 2.0)
        qw = math.cos(theta / 2.0)
        
        pose_msg = f'''{{
            header: {{frame_id: "map"}},
            pose: {{
                pose: {{
                    position: {{x: {x}, y: {y}, z: 0.0}},
                    orientation: {{x: 0.0, y: 0.0, z: {qz}, w: {qw}}}
                }},
                covariance: [0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
                            0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
                            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                            0.0, 0.0, 0.0, 0.0, 0.0, 0.07]
            }}
        }}'''
        
        subprocess.Popen(['ros2', 'topic', 'pub', '--once', '/initialpose',
                         'geometry_msgs/msg/PoseWithCovarianceStamped', pose_msg],
                        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        return {'success': True, 'message': f'Pose set: ({x:.2f}, {y:.2f}, θ={theta:.2f})'}
    except Exception as e:
        return {'success': False, 'message': str(e)}


HTML = '''
<!DOCTYPE html>
<html>
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Manriix Control v3.1</title>
    <script src="https://cdn.jsdelivr.net/npm/eventemitter2@6.4.9/lib/eventemitter2.min.js"></script>
    <script src="https://cdn.jsdelivr.net/npm/roslib@1.3.0/build/roslib.min.js"></script>
    <style>
        * { box-sizing: border-box; margin: 0; padding: 0; }
        body { font-family: system-ui, sans-serif; background: #0a0f1a; color: #e2e8f0; min-height: 100vh; }
        
        .header { background: #1e293b; padding: 0.5rem 1rem; display: flex; justify-content: space-between; align-items: center; border-bottom: 1px solid #334155; position: sticky; top: 0; z-index: 100; }
        .header h1 { font-size: 1rem; color: #38bdf8; }
        .header-btns { display: flex; gap: 0.3rem; align-items: center; }
        .ros-status { padding: 0.2rem 0.4rem; border-radius: 1rem; font-size: 0.65rem; font-weight: 600; }
        .ros-status.connected { background: #22c55e; color: white; }
        .ros-status.disconnected { background: #ef4444; color: white; }
        
        .btn { padding: 0.4rem 0.8rem; border: none; border-radius: 0.4rem; cursor: pointer; font-weight: 600; font-size: 0.75rem; transition: all 0.15s; }
        .btn-primary { background: #3b82f6; color: white; }
        .btn-success { background: #22c55e; color: white; }
        .btn-warning { background: #f59e0b; color: white; }
        .btn-danger { background: #ef4444; color: white; }
        .btn-secondary { background: #475569; color: white; }
        .btn:disabled { opacity: 0.4; cursor: not-allowed; }
        .btn-sm { padding: 0.25rem 0.5rem; font-size: 0.7rem; }
        .btn-block { width: 100%; }
        .btn.active { box-shadow: 0 0 0 2px #fff; }
        
        .screen { display: none; padding: 0.75rem; max-width: 1400px; margin: 0 auto; }
        .screen.active { display: block; }
        
        .mode-grid { display: grid; grid-template-columns: repeat(3, 1fr); gap: 1rem; margin-top: 0.5rem; }
        @media (max-width: 800px) { .mode-grid { grid-template-columns: 1fr; } }
        
        .mode-card { background: #1e293b; border: 2px solid #334155; border-radius: 0.75rem; padding: 1.25rem; text-align: center; cursor: pointer; transition: all 0.2s; }
        .mode-card:hover { border-color: #3b82f6; transform: translateY(-2px); }
        .mode-card .icon { font-size: 2rem; margin-bottom: 0.4rem; }
        .mode-card h2 { color: #f8fafc; font-size: 0.95rem; margin-bottom: 0.2rem; }
        .mode-card p { color: #94a3b8; font-size: 0.7rem; }
        
        .panel { background: #1e293b; border-radius: 0.5rem; padding: 0.6rem; margin-bottom: 0.5rem; border: 1px solid #334155; }
        .panel h3 { color: #38bdf8; margin-bottom: 0.4rem; font-size: 0.8rem; }
        
        .grid-2 { display: grid; grid-template-columns: 1fr 1fr; gap: 0.5rem; }
        .grid-3 { display: grid; grid-template-columns: repeat(3, 1fr); gap: 0.3rem; }
        @media (max-width: 900px) { .grid-2 { grid-template-columns: 1fr; } }
        
        .camera-view { background: #000; border-radius: 0.4rem; aspect-ratio: 16/9; display: flex; align-items: center; justify-content: center; color: #475569; border: 1px solid #334155; overflow: hidden; position: relative; font-size: 0.65rem; }
        .camera-view canvas { width: 100%; height: 100%; object-fit: contain; }
        .camera-label { position: absolute; top: 0.2rem; left: 0.2rem; background: rgba(0,0,0,0.7); padding: 0.1rem 0.25rem; border-radius: 0.2rem; font-size: 0.6rem; }
        
        .map-view { background: #000; border-radius: 0.4rem; height: 250px; display: flex; align-items: center; justify-content: center; color: #475569; border: 2px solid #334155; overflow: hidden; position: relative; font-size: 0.7rem; }
        .map-view.pose-mode { border-color: #f59e0b; cursor: crosshair; }
        .map-view.waypoint-mode { border-color: #22c55e; cursor: crosshair; }
        .map-view canvas { max-width: 100%; max-height: 100%; }
        
        .map-mode-label { position: absolute; top: 0.3rem; left: 0.3rem; padding: 0.15rem 0.4rem; border-radius: 0.25rem; font-size: 0.65rem; font-weight: 600; z-index: 10; }
        .map-mode-label.pose { background: #f59e0b; color: white; }
        .map-mode-label.waypoint { background: #22c55e; color: white; }
        
        .pose-indicator { position: absolute; pointer-events: none; z-index: 5; }
        .pose-dot { width: 12px; height: 12px; background: #f59e0b; border: 2px solid white; border-radius: 50%; position: absolute; transform: translate(-50%, -50%); }
        .pose-arrow { width: 0; height: 0; border-left: 5px solid transparent; border-right: 5px solid transparent; border-bottom: 18px solid #f59e0b; position: absolute; transform-origin: center bottom; }
        
        .waypoint-marker { position: absolute; width: 20px; height: 20px; background: #22c55e; border: 2px solid white; border-radius: 50%; transform: translate(-50%, -50%); display: flex; align-items: center; justify-content: center; font-size: 0.6rem; font-weight: bold; color: white; cursor: pointer; z-index: 5; }
        
        .mode-switcher { display: flex; gap: 0.25rem; margin-bottom: 0.4rem; }
        .mode-switcher .btn { flex: 1; }
        
        .teleop-pad { display: grid; grid-template-columns: repeat(3, 1fr); gap: 0.2rem; max-width: 110px; margin: 0 auto; }
        .teleop-btn { aspect-ratio: 1; background: #334155; border: 1px solid #475569; border-radius: 0.3rem; color: white; font-size: 0.9rem; cursor: pointer; }
        .teleop-btn:hover { background: #3b82f6; }
        .teleop-btn:active { transform: scale(0.95); }
        .teleop-btn.stop { background: #ef4444; }
        
        .motor-grid { display: grid; grid-template-columns: repeat(2, 1fr); gap: 0.2rem; }
        
        .map-list { max-height: 140px; overflow-y: auto; }
        .map-item { padding: 0.4rem; background: #0f172a; border-radius: 0.3rem; margin-bottom: 0.3rem; cursor: pointer; border: 2px solid transparent; }
        .map-item:hover { border-color: #3b82f6; }
        .map-item.selected { border-color: #22c55e; background: rgba(34,197,94,0.1); }
        .map-item-name { font-weight: 600; font-size: 0.75rem; }
        .map-item-meta { font-size: 0.6rem; color: #64748b; }
        
        .waypoint-list { max-height: 100px; overflow-y: auto; }
        .waypoint-item { display: flex; justify-content: space-between; align-items: center; padding: 0.3rem 0.4rem; background: #0f172a; border-radius: 0.25rem; margin-bottom: 0.25rem; font-size: 0.7rem; }
        .wp-num { background: #22c55e; color: white; min-width: 1.1rem; height: 1.1rem; border-radius: 50%; display: flex; align-items: center; justify-content: center; font-weight: 600; font-size: 0.6rem; }
        .wp-remove { color: #ef4444; cursor: pointer; }
        
        .log-panel { background: #0a0f1a; border-radius: 0.3rem; padding: 0.3rem; height: 100px; overflow-y: auto; font-family: monospace; font-size: 0.6rem; border: 1px solid #1e293b; }
        .log-line { margin-bottom: 0.1rem; line-height: 1.2; }
        .log-line.error { color: #f87171; }
        .log-line.warning { color: #fbbf24; }
        .log-line.success { color: #4ade80; }
        .log-line.info { color: #64748b; }
        
        .status-dot { display: inline-block; width: 7px; height: 7px; border-radius: 50%; margin-right: 0.3rem; background: #475569; }
        .status-dot.running { background: #22c55e; animation: pulse 2s infinite; }
        @keyframes pulse { 0%, 100% { opacity: 1; } 50% { opacity: 0.5; } }
        
        .node-control { display: flex; justify-content: space-between; align-items: center; padding: 0.35rem; background: #0f172a; border-radius: 0.3rem; margin-bottom: 0.25rem; }
        .node-name { display: flex; align-items: center; font-size: 0.7rem; }
        .node-btns { display: flex; gap: 0.2rem; }
        
        .form-group { margin-bottom: 0.4rem; }
        .form-group label { display: block; margin-bottom: 0.2rem; color: #94a3b8; font-size: 0.7rem; }
        .form-group input, .form-group select { width: 100%; padding: 0.35rem; background: #0f172a; border: 1px solid #334155; border-radius: 0.3rem; color: #e2e8f0; font-size: 0.75rem; }
        .form-row { display: flex; gap: 0.2rem; }
        .form-row input { flex: 1; min-width: 0; }
        
        .checkbox-group { display: flex; align-items: center; gap: 0.3rem; padding: 0.35rem; background: #0f172a; border-radius: 0.3rem; }
        .checkbox-group input { width: 14px; height: 14px; }
        .checkbox-group label { font-size: 0.75rem; }
        
        .progress-bar { height: 5px; background: #1e293b; border-radius: 3px; overflow: hidden; margin: 0.3rem 0; }
        .progress-fill { height: 100%; background: linear-gradient(90deg, #3b82f6, #22c55e); }
        
        .emergency-btn { position: fixed; bottom: 0.5rem; right: 0.5rem; width: 50px; height: 50px; border-radius: 50%; background: #ef4444; border: 2px solid #fca5a5; color: white; font-weight: bold; font-size: 0.55rem; cursor: pointer; z-index: 1000; display: flex; flex-direction: column; align-items: center; justify-content: center; }
        
        .breadcrumb { display: flex; gap: 0.3rem; margin-bottom: 0.4rem; color: #64748b; font-size: 0.7rem; }
        .breadcrumb a { color: #3b82f6; cursor: pointer; }
        
        .tabs { display: flex; gap: 0.1rem; margin-bottom: 0.25rem; background: #0f172a; border-radius: 0.3rem; padding: 0.1rem; }
        .tab { padding: 0.25rem 0.4rem; border-radius: 0.2rem; cursor: pointer; color: #64748b; font-size: 0.65rem; }
        .tab.active { background: #3b82f6; color: white; }
        
        .step-indicator { display: flex; justify-content: center; gap: 0.25rem; margin-bottom: 0.4rem; }
        .step { width: 22px; height: 22px; border-radius: 50%; background: #1e293b; border: 2px solid #334155; display: flex; align-items: center; justify-content: center; font-size: 0.65rem; font-weight: 600; }
        .step.active { background: #3b82f6; border-color: #3b82f6; }
        .step.completed { background: #22c55e; border-color: #22c55e; }
        .step-line { width: 25px; height: 2px; background: #334155; align-self: center; }
        .step-line.completed { background: #22c55e; }
        
        .info-box { background: rgba(59,130,246,0.1); border: 1px solid rgba(59,130,246,0.3); border-radius: 0.3rem; padding: 0.4rem; margin-bottom: 0.4rem; }
        .info-box p { color: #94a3b8; font-size: 0.7rem; margin: 0; }
        .info-box strong { color: #3b82f6; }
        .info-box.warning { background: rgba(245,158,11,0.1); border-color: rgba(245,158,11,0.3); }
        .info-box.warning strong { color: #f59e0b; }
        
        .pose-display { background: #0f172a; border-radius: 0.3rem; padding: 0.4rem; margin-top: 0.3rem; display: flex; align-items: center; gap: 0.5rem; }
        .pose-display .value { color: #f59e0b; font-family: monospace; font-size: 0.75rem; }
        
        .nav-status { text-align: center; padding: 0.5rem; }
        .nav-status .wp-current { font-size: 1.8rem; font-weight: bold; color: #3b82f6; }
        .nav-status .wp-total { font-size: 1rem; color: #64748b; }
    </style>
</head>
<body>
    <header class="header">
        <h1>🤖 Manriix v3.1</h1>
        <div class="header-btns">
            <span id="ros-status" class="ros-status disconnected">ROS ○</span>
            <button class="btn btn-secondary btn-sm" onclick="showScreen('main')">🏠</button>
            <button class="btn btn-danger btn-sm" onclick="stopAll()">⏹ Stop</button>
        </div>
    </header>
    
    <!-- MAIN MENU -->
    <div id="screen-main" class="screen active">
        <div class="mode-grid">
            <div class="mode-card" onclick="selectMode('waypoint')">
                <div class="icon">📍</div>
                <h2>Waypoint Navigation</h2>
                <p>Build map, then navigate waypoints</p>
            </div>
            <div class="mode-card" onclick="selectMode('detection')">
                <div class="icon">👁️</div>
                <h2>Detection Mode</h2>
                <p>AI human detection & photography</p>
            </div>
            <div class="mode-card" onclick="selectMode('manual')">
                <div class="icon">🎮</div>
                <h2>Manual Teleop</h2>
                <p>Direct manual control</p>
            </div>
        </div>
    </div>
    
    <!-- WAYPOINT: SELECT MAP -->
    <div id="screen-waypoint-select" class="screen">
        <div class="breadcrumb"><a onclick="showScreen('main')">Home</a> › Waypoint</div>
        <div class="step-indicator">
            <div class="step active">1</div><div class="step-line"></div>
            <div class="step">2</div><div class="step-line"></div>
            <div class="step">3</div>
        </div>
        <div class="grid-2">
            <div>
                <div class="panel">
                    <h3>📂 Select Map</h3>
                    <div class="map-list" id="map-list"></div>
                    <button class="btn btn-secondary btn-sm btn-block" onclick="loadMaps()" style="margin-top:0.3rem;">🔄 Refresh</button>
                </div>
                <div class="panel">
                    <h3>🗺️ Or Create New</h3>
                    <button class="btn btn-success btn-block" onclick="showScreen('mapping')">▶ Start SLAM</button>
                </div>
            </div>
            <div>
                <div class="panel">
                    <h3>✅ Selected</h3>
                    <div id="selected-map-info" style="padding:0.3rem;background:#0f172a;border-radius:0.3rem;min-height:40px;">
                        <p style="color:#64748b;font-size:0.75rem;">No map selected</p>
                    </div>
                </div>
                <div class="panel">
                    <h3>⚙️ Options</h3>
                    <div class="checkbox-group">
                        <input type="checkbox" id="use-3d-perception">
                        <label for="use-3d-perception">3D Perception (STVL)</label>
                    </div>
                </div>
                <button class="btn btn-primary btn-block" onclick="goToNavSetup()" id="btn-goto-nav" disabled style="margin-top:0.4rem;">▶ Continue</button>
            </div>
        </div>
    </div>
    
    <!-- WAYPOINT: MAPPING -->
    <div id="screen-mapping" class="screen">
        <div class="breadcrumb"><a onclick="showScreen('main')">Home</a> › <a onclick="showScreen('waypoint-select')">Waypoint</a> › SLAM</div>
        <div class="info-box"><p><strong>Steps:</strong> 1) Start Robot+Cam 2) Start SLAM 3) Drive around 4) Save map</p></div>
        <div class="grid-2">
            <div>
                <div class="panel">
                    <h3>🗺️ Map</h3>
                    <div class="map-view"><canvas id="map-canvas-mapping"></canvas><span>Building...</span></div>
                </div>
                <div class="panel">
                    <h3>💾 Save</h3>
                    <div class="form-row">
                        <input type="text" id="map-name-input" placeholder="Map name">
                        <button class="btn btn-success" onclick="saveCurrentMap()">💾</button>
                    </div>
                </div>
            </div>
            <div>
                <div class="panel">
                    <h3>📷 Cameras</h3>
                    <div class="grid-3">
                        <div class="camera-view"><canvas id="cam-front-mapping"></canvas><span class="camera-label">F</span></div>
                        <div class="camera-view"><canvas id="cam-left-mapping"></canvas><span class="camera-label">L</span></div>
                        <div class="camera-view"><canvas id="cam-right-mapping"></canvas><span class="camera-label">R</span></div>
                    </div>
                </div>
                <div class="panel">
                    <h3>🚀 Launch</h3>
                    <div class="checkbox-group" style="margin-bottom:0.3rem;">
                        <input type="checkbox" id="slam-3d-mode"><label for="slam-3d-mode">3D Mode</label>
                    </div>
                    <div class="node-control">
                        <span class="node-name"><span class="status-dot" id="dot-robot_cameras"></span>Robot+Cam</span>
                        <div class="node-btns">
                            <button class="btn btn-success btn-sm" id="btn-start-robot_cameras" onclick="startNode('robot_cameras')">▶</button>
                            <button class="btn btn-danger btn-sm" id="btn-stop-robot_cameras" onclick="stopNode('robot_cameras')" disabled>⏹</button>
                        </div>
                    </div>
                    <div class="node-control">
                        <span class="node-name"><span class="status-dot" id="dot-slam"></span>SLAM</span>
                        <div class="node-btns">
                            <button class="btn btn-success btn-sm" id="btn-start-slam" onclick="startSlam()">▶</button>
                            <button class="btn btn-danger btn-sm" id="btn-stop-slam" onclick="stopSlam()" disabled>⏹</button>
                        </div>
                    </div>
                </div>
                <div class="grid-2">
                    <div class="panel"><h3>🎮</h3><div class="teleop-pad" id="teleop-mapping"></div></div>
                    <div class="panel"><h3>⚡</h3><div class="motor-grid" id="motor-mapping"></div></div>
                </div>
            </div>
        </div>
        <div class="panel"><h3>📜 Logs</h3><div class="log-panel" id="log-mapping"></div></div>
        <div style="display:flex;gap:0.3rem;margin-top:0.3rem;">
            <button class="btn btn-secondary" onclick="cancelMapping()">❌ Cancel</button>
            <button class="btn btn-primary" onclick="finishMapping()">✅ Done</button>
        </div>
    </div>
    
    <!-- WAYPOINT: NAV SETUP -->
    <div id="screen-nav-setup" class="screen">
        <div class="breadcrumb"><a onclick="showScreen('main')">Home</a> › <a onclick="showScreen('waypoint-select')">Waypoint</a> › Setup</div>
        <div class="step-indicator">
            <div class="step completed">1</div><div class="step-line completed"></div>
            <div class="step active">2</div><div class="step-line"></div>
            <div class="step">3</div>
        </div>
        <div class="info-box warning"><p><strong>Steps:</strong> 1) Start Robot+Cam 2) Start Localization 3) <strong>Set Initial Pose</strong> 4) Add Waypoints 5) Start</p></div>
        <div class="grid-2">
            <div>
                <div class="panel">
                    <h3>🗺️ Map</h3>
                    <div class="mode-switcher">
                        <button class="btn btn-warning btn-sm active" id="btn-mode-pose" onclick="setMapMode('pose')">📍 Initial Pose</button>
                        <button class="btn btn-success btn-sm" id="btn-mode-waypoint" onclick="setMapMode('waypoint')">🚩 Waypoint</button>
                    </div>
                    <div class="map-view pose-mode" id="nav-map-view" onmousedown="mapMouseDown(event)" onmousemove="mapMouseMove(event)" onmouseup="mapMouseUp(event)" onmouseleave="mapMouseUp(event)">
                        <span class="map-mode-label pose" id="map-mode-label">INITIAL POSE</span>
                        <canvas id="map-canvas-nav"></canvas>
                        <div id="pose-indicator" class="pose-indicator" style="display:none;">
                            <div class="pose-dot"></div>
                            <div class="pose-arrow" id="pose-arrow"></div>
                        </div>
                    </div>
                    <div class="pose-display" id="pose-display" style="display:none;">
                        <span style="color:#64748b;font-size:0.7rem;">Pose:</span>
                        <span class="value" id="pose-value">x=0, y=0, θ=0</span>
                        <button class="btn btn-warning btn-sm" onclick="sendPose()">📤 Send</button>
                    </div>
                </div>
                <div class="panel">
                    <h3>📍 Manual Pose</h3>
                    <div class="form-row">
                        <input type="number" id="init-x" placeholder="X" step="0.1">
                        <input type="number" id="init-y" placeholder="Y" step="0.1">
                        <input type="number" id="init-theta" placeholder="θ" step="0.1" value="0">
                        <button class="btn btn-warning btn-sm" onclick="setManualPose()">Set</button>
                    </div>
                </div>
                <div class="panel">
                    <h3>🚩 Manual Waypoint</h3>
                    <div class="form-row">
                        <input type="number" id="wp-x" placeholder="X" step="0.1">
                        <input type="number" id="wp-y" placeholder="Y" step="0.1">
                        <input type="number" id="wp-theta" placeholder="θ" step="0.1" value="0">
                        <button class="btn btn-success btn-sm" onclick="addManualWp()">+</button>
                    </div>
                </div>
            </div>
            <div>
                <div class="panel">
                    <h3>📷 Cameras</h3>
                    <div class="grid-3">
                        <div class="camera-view"><canvas id="cam-front-nav"></canvas><span class="camera-label">F</span></div>
                        <div class="camera-view"><canvas id="cam-left-nav"></canvas><span class="camera-label">L</span></div>
                        <div class="camera-view"><canvas id="cam-right-nav"></canvas><span class="camera-label">R</span></div>
                    </div>
                </div>
                <div class="panel">
                    <h3>🚩 Waypoints (<span id="wp-count">0</span>)</h3>
                    <div class="waypoint-list" id="waypoint-list"></div>
                    <div style="display:flex;gap:0.2rem;margin-top:0.25rem;">
                        <button class="btn btn-secondary btn-sm" onclick="clearWps()">🗑 Clear</button>
                        <button class="btn btn-secondary btn-sm" onclick="reverseWps()">🔄 Reverse</button>
                    </div>
                </div>
                <div class="panel">
                    <h3>⏱ Settings</h3>
                    <div class="grid-2" style="gap:0.25rem;">
                        <div class="form-group" style="margin:0;"><label>Wait Mode</label>
                            <select id="wait-mode"><option value="time_based">Time-based</option><option value="photo_command">Photo Command</option></select>
                        </div>
                        <div class="form-group" style="margin:0;"><label>Wait (sec)</label>
                            <input type="number" id="wait-time" value="30" min="5">
                        </div>
                    </div>
                    <div class="checkbox-group" style="margin-top:0.25rem;">
                        <input type="checkbox" id="loop-continuous" checked><label for="loop-continuous">Loop</label>
                    </div>
                </div>
                <div class="panel">
                    <h3>🚀 Launch</h3>
                    <div class="node-control">
                        <span class="node-name"><span class="status-dot" id="dot2-robot_cameras"></span>Robot+Cam</span>
                        <div class="node-btns">
                            <button class="btn btn-success btn-sm" id="btn-start2-robot_cameras" onclick="startNode('robot_cameras')">▶</button>
                            <button class="btn btn-danger btn-sm" id="btn-stop2-robot_cameras" onclick="stopNode('robot_cameras')" disabled>⏹</button>
                        </div>
                    </div>
                    <div class="node-control">
                        <span class="node-name"><span class="status-dot" id="dot2-localization"></span>Localization</span>
                        <div class="node-btns">
                            <button class="btn btn-success btn-sm" id="btn-start2-localization" onclick="startLoc()">▶</button>
                            <button class="btn btn-danger btn-sm" id="btn-stop2-localization" onclick="stopNode('localization')" disabled>⏹</button>
                        </div>
                    </div>
                </div>
                <div class="grid-2">
                    <div class="panel"><h3>🎮</h3><div class="teleop-pad" id="teleop-nav"></div></div>
                    <div class="panel"><h3>⚡</h3><div class="motor-grid" id="motor-nav"></div></div>
                </div>
            </div>
        </div>
        <div class="panel"><h3>📜 Logs</h3><div class="log-panel" id="log-nav"></div></div>
        <div style="display:flex;gap:0.3rem;margin-top:0.3rem;">
            <button class="btn btn-secondary" onclick="showScreen('waypoint-select')">← Back</button>
            <button class="btn btn-success" onclick="startNav()" id="btn-start-nav">🚀 Start Navigation</button>
        </div>
    </div>
    
    <!-- WAYPOINT: RUNNING -->
    <div id="screen-nav-running" class="screen">
        <div class="breadcrumb"><a onclick="showScreen('main')">Home</a> › Navigation</div>
        <div class="step-indicator">
            <div class="step completed">1</div><div class="step-line completed"></div>
            <div class="step completed">2</div><div class="step-line completed"></div>
            <div class="step active">3</div>
        </div>
        <div class="panel nav-status">
            <div class="wp-current">WP <span id="current-wp">1</span></div>
            <div class="wp-total">of <span id="total-wp">0</span></div>
            <div class="progress-bar"><div class="progress-fill" id="nav-progress" style="width:0%;"></div></div>
            <div id="nav-status-text" style="color:#94a3b8;font-size:0.8rem;">...</div>
            <div style="display:flex;gap:0.3rem;justify-content:center;margin-top:0.5rem;">
                <button class="btn btn-success" onclick="wpCtrl('resume')">▶</button>
                <button class="btn btn-warning" onclick="wpCtrl('pause')">⏸</button>
                <button class="btn btn-secondary" onclick="wpCtrl('skip')">⏭</button>
                <button class="btn btn-danger" onclick="stopNav()">⏹</button>
            </div>
        </div>
        <div class="grid-2">
            <div class="panel"><h3>🗺️ Map</h3><div class="map-view"><canvas id="map-canvas-running"></canvas></div></div>
            <div>
                <div class="panel"><h3>📷 Cameras</h3>
                    <div class="grid-3">
                        <div class="camera-view"><canvas id="cam-front-running"></canvas><span class="camera-label">F</span></div>
                        <div class="camera-view"><canvas id="cam-left-running"></canvas><span class="camera-label">L</span></div>
                        <div class="camera-view"><canvas id="cam-right-running"></canvas><span class="camera-label">R</span></div>
                    </div>
                </div>
                <div class="grid-2">
                    <div class="panel"><h3>🎮</h3><div class="teleop-pad" id="teleop-running"></div></div>
                    <div class="panel"><h3>⚡</h3><div class="motor-grid" id="motor-running"></div></div>
                </div>
            </div>
        </div>
        <div class="panel"><h3>📜 Logs</h3><div class="log-panel" id="log-running"></div></div>
    </div>
    
    <!-- DETECTION -->
    <div id="screen-detection" class="screen">
        <div class="breadcrumb"><a onclick="showScreen('main')">Home</a> › Detection</div>
        <div class="panel"><h3>📷 Cameras</h3>
            <div class="grid-3">
                <div class="camera-view"><canvas id="cam-front-det"></canvas><span class="camera-label">Front</span></div>
                <div class="camera-view"><canvas id="cam-left-det"></canvas><span class="camera-label">Left</span></div>
                <div class="camera-view"><canvas id="cam-right-det"></canvas><span class="camera-label">Right</span></div>
            </div>
        </div>
        <div class="grid-2">
            <div class="panel">
                <h3>🚀 Launch</h3>
                <div class="node-control">
                    <span class="node-name"><span class="status-dot" id="dot-robot_no_cameras"></span>Robot</span>
                    <div class="node-btns">
                        <button class="btn btn-success btn-sm" id="btn-start-robot_no_cameras" onclick="startNode('robot_no_cameras')">▶</button>
                        <button class="btn btn-danger btn-sm" id="btn-stop-robot_no_cameras" onclick="stopNode('robot_no_cameras')" disabled>⏹</button>
                    </div>
                </div>
                <div class="node-control">
                    <span class="node-name"><span class="status-dot" id="dot-detection"></span>Detection</span>
                    <div class="node-btns">
                        <button class="btn btn-success btn-sm" id="btn-start-detection" onclick="startDet()">▶</button>
                        <button class="btn btn-danger btn-sm" id="btn-stop-detection" onclick="stopNode('detection')" disabled>⏹</button>
                    </div>
                </div>
                <div class="checkbox-group" style="margin-top:0.3rem;">
                    <input type="checkbox" id="enable-autonomous"><label for="enable-autonomous">Autonomous Nav</label>
                </div>
            </div>
            <div class="grid-2">
                <div class="panel"><h3>🎮</h3><div class="teleop-pad" id="teleop-det"></div></div>
                <div class="panel"><h3>⚡</h3><div class="motor-grid" id="motor-det"></div></div>
            </div>
        </div>
        <div class="panel"><h3>📜 Logs</h3><div class="log-panel" id="log-detection"></div></div>
    </div>
    
    <!-- MANUAL -->
    <div id="screen-manual" class="screen">
        <div class="breadcrumb"><a onclick="showScreen('main')">Home</a> › Manual</div>
        <div class="grid-2">
            <div class="panel" style="text-align:center;">
                <h3>🎮 Teleop</h3>
                <div class="teleop-pad" style="max-width:140px;margin:0.5rem auto;" id="teleop-manual"></div>
                <p style="color:#64748b;font-size:0.7rem;">W/A/S/D | Space=Stop</p>
            </div>
            <div>
                <div class="panel"><h3>⚡ Motor</h3><div class="motor-grid" id="motor-manual"></div></div>
                <div class="panel">
                    <h3>🚀 Launch</h3>
                    <div class="node-control">
                        <span class="node-name"><span class="status-dot" id="dot-robot_no_cameras-man"></span>Robot</span>
                        <div class="node-btns">
                            <button class="btn btn-success btn-sm" id="btn-start-robot_no_cameras-man" onclick="startNode('robot_no_cameras')">▶</button>
                            <button class="btn btn-danger btn-sm" id="btn-stop-robot_no_cameras-man" onclick="stopNode('robot_no_cameras')" disabled>⏹</button>
                        </div>
                    </div>
                </div>
            </div>
        </div>
        <div class="panel"><h3>📜 Logs</h3><div class="log-panel" id="log-manual"></div></div>
    </div>
    
    <button class="emergency-btn" onclick="emergencyStop()">🛑<br>STOP</button>
    
    <script>
        let selectedMap='', selectedMapName='', waypoints=[], ros=null, rosConnected=false;
        let mapMode='pose', initialPose={x:0,y:0,theta:0,set:false}, isDragging=false, dragStart={x:0,y:0};
        let mapInfo={width:0,height:0,resolution:0.05,origin:{x:-10,y:-10}};
        let slamRunning=false, locRunning=false, currentLogNode={};
        
        function init(){
            initControls(); connectROS();
            setInterval(updateStatus,2000); setInterval(updateLogs,1500);
            loadMaps();
        }
        
        function initControls(){
            const tp=`<button class="teleop-btn" onmousedown="teleop(0.3,0.5)" onmouseup="teleopStop()" ontouchstart="teleop(0.3,0.5)" ontouchend="teleopStop()">↖</button>
                <button class="teleop-btn" onmousedown="teleop(0.3,0)" onmouseup="teleopStop()" ontouchstart="teleop(0.3,0)" ontouchend="teleopStop()">↑</button>
                <button class="teleop-btn" onmousedown="teleop(0.3,-0.5)" onmouseup="teleopStop()" ontouchstart="teleop(0.3,-0.5)" ontouchend="teleopStop()">↗</button>
                <button class="teleop-btn" onmousedown="teleop(0,0.5)" onmouseup="teleopStop()" ontouchstart="teleop(0,0.5)" ontouchend="teleopStop()">←</button>
                <button class="teleop-btn stop" onclick="teleopStop()">⏹</button>
                <button class="teleop-btn" onmousedown="teleop(0,-0.5)" onmouseup="teleopStop()" ontouchstart="teleop(0,-0.5)" ontouchend="teleopStop()">→</button>
                <button class="teleop-btn" onmousedown="teleop(-0.3,-0.5)" onmouseup="teleopStop()" ontouchstart="teleop(-0.3,-0.5)" ontouchend="teleopStop()">↙</button>
                <button class="teleop-btn" onmousedown="teleop(-0.3,0)" onmouseup="teleopStop()" ontouchstart="teleop(-0.3,0)" ontouchend="teleopStop()">↓</button>
                <button class="teleop-btn" onmousedown="teleop(-0.3,0.5)" onmouseup="teleopStop()" ontouchstart="teleop(-0.3,0.5)" ontouchend="teleopStop()">↘</button>`;
            const mt=`<button class="btn btn-primary btn-sm" onclick="motor('init')">⚡Init</button>
                <button class="btn btn-secondary btn-sm" onclick="motor('reset')">🔄Reset</button>
                <button class="btn btn-warning btn-sm" onclick="motor('halt')">⏸Halt</button>
                <button class="btn btn-success btn-sm" onclick="motor('resume')">▶Resume</button>`;
            document.querySelectorAll('.teleop-pad').forEach(e=>e.innerHTML=tp);
            document.querySelectorAll('.motor-grid').forEach(e=>e.innerHTML=mt);
        }
        
        function connectROS(){
            try{
                ros=new ROSLIB.Ros({url:'ws://'+window.location.hostname+':9090'});
                ros.on('connection',()=>{rosConnected=true;document.getElementById('ros-status').className='ros-status connected';document.getElementById('ros-status').textContent='ROS ●';subscribeTopics();});
                ros.on('close',()=>{rosConnected=false;document.getElementById('ros-status').className='ros-status disconnected';document.getElementById('ros-status').textContent='ROS ○';setTimeout(connectROS,3000);});
                ros.on('error',()=>{});
            }catch(e){setTimeout(connectROS,3000);}
        }
        
        function subscribeTopics(){
            if(!ros||!rosConnected)return;
            new ROSLIB.Topic({ros,name:'/map',messageType:'nav_msgs/OccupancyGrid'}).subscribe(msg=>{
                mapInfo.width=msg.info.width;mapInfo.height=msg.info.height;mapInfo.resolution=msg.info.resolution;
                mapInfo.origin.x=msg.info.origin.position.x;mapInfo.origin.y=msg.info.origin.position.y;
                drawMap(msg);
            });
            ['front','left','right'].forEach(cam=>{
                new ROSLIB.Topic({ros,name:'/viz/camera/zedx_'+cam+'/image_rect_color/compressed',messageType:'sensor_msgs/CompressedImage'}).subscribe(msg=>drawCam(cam,msg,'detection'));
                new ROSLIB.Topic({ros,name:'/zedx_'+cam+'/zed_node/rgb/color/rect/image/compressed',messageType:'sensor_msgs/CompressedImage'}).subscribe(msg=>drawCam(cam,msg,'waypoint'));
            });
            new ROSLIB.Topic({ros,name:'/waypoint/status',messageType:'std_msgs/String'}).subscribe(msg=>updateNavStatus(msg.data));
        }
        
        function drawMap(msg){
            const as=document.querySelector('.screen.active');if(!as)return;
            const cs=as.querySelectorAll('[id^="map-canvas-"]');if(cs.length===0)return;
            cs.forEach(c=>{const ctx=c.getContext('2d');c.width=msg.info.width;c.height=msg.info.height;
                const id=ctx.createImageData(msg.info.width,msg.info.height);
                for(let i=0;i<msg.data.length;i++){let v=msg.data[i],col=128;if(v===0)col=255;else if(v===100)col=0;
                    id.data[i*4]=col;id.data[i*4+1]=col;id.data[i*4+2]=col;id.data[i*4+3]=255;}
                ctx.putImageData(id,0,0);});
            updateMapOverlays();
        }
        
        function drawCam(cam,msg,src){
            const as=document.querySelector('.screen.active');if(!as)return;
            const sid=as.id,isDet=sid==='screen-detection',isWp=sid.includes('mapping')||sid.includes('nav')||sid.includes('running');
            if(src==='detection'&&!isDet)return;if(src==='waypoint'&&!isWp)return;
            const cs=as.querySelectorAll('[id^="cam-'+cam+'-"]');if(cs.length===0)return;
            const img=new Image();img.onload=()=>{cs.forEach(c=>{const ctx=c.getContext('2d');c.width=img.width;c.height=img.height;ctx.drawImage(img,0,0);});};
            img.src='data:image/jpeg;base64,'+msg.data;
        }
        
        function updateNavStatus(data){try{const s=JSON.parse(data);document.getElementById('current-wp').textContent=s.current_waypoint||1;document.getElementById('nav-status-text').textContent=s.state||'';const t=waypoints.length||1,c=parseInt(s.current_waypoint)||1;document.getElementById('nav-progress').style.width=((c/t)*100)+'%';}catch(e){}}
        
        function setMapMode(m){
            mapMode=m;const mv=document.getElementById('nav-map-view'),ml=document.getElementById('map-mode-label'),bp=document.getElementById('btn-mode-pose'),bw=document.getElementById('btn-mode-waypoint');
            if(m==='pose'){mv.className='map-view pose-mode';ml.className='map-mode-label pose';ml.textContent='INITIAL POSE';bp.classList.add('active');bw.classList.remove('active');}
            else{mv.className='map-view waypoint-mode';ml.className='map-mode-label waypoint';ml.textContent='ADD WAYPOINT';bp.classList.remove('active');bw.classList.add('active');}
        }
        
        function mapMouseDown(e){
            const r=e.currentTarget.getBoundingClientRect(),c=document.getElementById('map-canvas-nav');if(!c||c.width===0)return;
            const cx=e.clientX-r.left,cy=e.clientY-r.top,sx=c.width/r.width,sy=c.height/r.height;
            const px=cx*sx,py=cy*sy,wx=px*mapInfo.resolution+mapInfo.origin.x,wy=(c.height-py)*mapInfo.resolution+mapInfo.origin.y;
            if(mapMode==='pose'){isDragging=true;dragStart={x:cx,y:cy};initialPose.x=wx;initialPose.y=wy;initialPose.theta=0;initialPose.set=true;updatePoseInd(cx,cy,0);updatePoseDisp();notify('Drag for direction','info');}
            else{waypoints.push({x:parseFloat(wx.toFixed(2)),y:parseFloat(wy.toFixed(2)),theta:0});updateWpList();updateMapOverlays();notify('WP '+waypoints.length+' added','success');}
        }
        
        function mapMouseMove(e){if(!isDragging||mapMode!=='pose')return;const r=e.currentTarget.getBoundingClientRect(),cx=e.clientX-r.left,cy=e.clientY-r.top,dx=cx-dragStart.x,dy=cy-dragStart.y;if(Math.abs(dx)>5||Math.abs(dy)>5){initialPose.theta=Math.atan2(-dy,dx);updatePoseInd(dragStart.x,dragStart.y,initialPose.theta);updatePoseDisp();}}
        
        function mapMouseUp(e){if(isDragging&&mapMode==='pose'){isDragging=false;notify('Click "Send" to apply','info');}}
        
        function updatePoseInd(x,y,th){const ind=document.getElementById('pose-indicator'),arr=document.getElementById('pose-arrow');ind.style.display='block';ind.style.left=x+'px';ind.style.top=y+'px';arr.style.transform='rotate('+(-th*180/Math.PI-90)+'deg)';document.getElementById('pose-display').style.display='flex';}
        
        function updatePoseDisp(){document.getElementById('pose-value').textContent='x='+initialPose.x.toFixed(2)+', y='+initialPose.y.toFixed(2)+', θ='+initialPose.theta.toFixed(2);}
        
        function updateMapOverlays(){document.querySelectorAll('.waypoint-marker').forEach(e=>e.remove());const mv=document.getElementById('nav-map-view'),c=document.getElementById('map-canvas-nav');if(!c||c.width===0||!mv)return;const r=mv.getBoundingClientRect(),sx=r.width/c.width,sy=r.height/c.height;
            waypoints.forEach((wp,i)=>{const px=(wp.x-mapInfo.origin.x)/mapInfo.resolution,py=c.height-(wp.y-mapInfo.origin.y)/mapInfo.resolution;const m=document.createElement('div');m.className='waypoint-marker';m.textContent=(i+1);m.style.left=(px*sx)+'px';m.style.top=(py*sy)+'px';m.onclick=(e)=>{e.stopPropagation();removeWp(i);};mv.appendChild(m);});}
        
        async function sendPose(){if(!initialPose.set){notify('Set pose first','warning');return;}const r=await apiPost('/set_initial_pose',{x:initialPose.x,y:initialPose.y,theta:initialPose.theta});notify(r.message,r.success?'success':'error');}
        
        function setManualPose(){const x=parseFloat(document.getElementById('init-x').value)||0,y=parseFloat(document.getElementById('init-y').value)||0,th=parseFloat(document.getElementById('init-theta').value)||0;initialPose={x,y,theta:th,set:true};updatePoseDisp();document.getElementById('pose-display').style.display='flex';sendPose();}
        
        function addManualWp(){const x=parseFloat(document.getElementById('wp-x').value)||0,y=parseFloat(document.getElementById('wp-y').value)||0,th=parseFloat(document.getElementById('wp-theta').value)||0;waypoints.push({x,y,theta:th});updateWpList();updateMapOverlays();document.getElementById('wp-x').value='';document.getElementById('wp-y').value='';notify('WP '+waypoints.length+' added','success');}
        
        function showScreen(n){document.querySelectorAll('.screen').forEach(s=>s.classList.remove('active'));document.getElementById('screen-'+n).classList.add('active');if(n==='waypoint-select')loadMaps();if(n==='nav-setup')updateMapOverlays();}
        function selectMode(m){if(m==='waypoint')showScreen('waypoint-select');else if(m==='detection')showScreen('detection');else if(m==='manual')showScreen('manual');}
        
        async function apiPost(url,data){try{const r=await fetch(url,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(data||{})});return await r.json();}catch(e){return{success:false,message:e.toString()};}}
        async function apiGet(url){try{return await(await fetch(url)).json();}catch{return null;}}
        
        async function startNode(id,ex={}){notify('Starting '+id+'...','info');const r=await apiPost('/nodes/'+id+'/start',ex);notify(r.message||'Started',r.success?'success':'error');updateStatus();}
        async function stopNode(id){notify('Stopping...','info');const r=await apiPost('/nodes/'+id+'/stop',{});notify(r.message||'Stopped',r.success?'success':'error');updateStatus();}
        async function stopAll(){if(confirm('Stop all?')){await apiPost('/nodes/stop_all',{});notify('Stopped','success');updateStatus();}}
        async function emergencyStop(){await apiPost('/emergency_stop',{});notify('🛑 EMERGENCY!','warning');}
        
        async function startSlam(){const use3D=document.getElementById('slam-3d-mode').checked;slamRunning=true;await startNode(use3D?'slam_3d':'slam_2d');}
        async function stopSlam(){await stopNode('slam_2d');await stopNode('slam_3d');slamRunning=false;}
        async function startLoc(){if(!selectedMap){alert('Select map first');return;}const use3D=document.getElementById('use-3d-perception').checked;locRunning=true;await startNode('localization',{map:selectedMap,use_3d:use3D});}
        async function startDet(){const auto=document.getElementById('enable-autonomous').checked;await startNode('detection',{enable_autonomous:auto});}
        
        async function loadMaps(){const d=await apiGet('/maps/list'),c=document.getElementById('map-list');if(d&&d.maps){c.innerHTML=d.maps.map(m=>'<div class="map-item'+(selectedMap===m.path?' selected':'')+'" onclick="selectMap(\\''+m.path+'\\',\\''+m.name+'\\')"><div class="map-item-name">'+m.name+'</div><div class="map-item-meta">'+m.modified+'</div></div>').join('')||'<p style="color:#64748b;font-size:0.7rem;">No maps</p>';}}
        function selectMap(p,n){selectedMap=p;selectedMapName=n;document.querySelectorAll('.map-item').forEach(e=>e.classList.remove('selected'));if(event&&event.target){const i=event.target.closest('.map-item');if(i)i.classList.add('selected');}document.getElementById('selected-map-info').innerHTML='<p style="color:#22c55e;font-weight:600;">✓ '+n+'</p>';document.getElementById('btn-goto-nav').disabled=false;}
        async function saveCurrentMap(){const n=document.getElementById('map-name-input').value.trim();if(!n){alert('Enter name');return;}notify('Saving...','info');const r=await apiPost('/maps/save',{name:n});notify(r.message,r.success?'success':'error');if(r.success){selectedMap='/home/hype/manriix2_ws/src/manriix_navigation/maps/'+n+'.yaml';selectedMapName=n;}}
        function goToNavSetup(){if(!selectedMap){alert('Select map');return;}showScreen('nav-setup');}
        function cancelMapping(){stopSlam();stopNode('robot_cameras');showScreen('waypoint-select');}
        function finishMapping(){const n=document.getElementById('map-name-input').value.trim();if(!n){alert('Save map first');return;}saveCurrentMap();setTimeout(()=>{stopSlam();showScreen('nav-setup');},1500);}
        
        function removeWp(i){waypoints.splice(i,1);updateWpList();updateMapOverlays();}
        function clearWps(){waypoints=[];updateWpList();updateMapOverlays();}
        function reverseWps(){waypoints.reverse();updateWpList();updateMapOverlays();}
        function updateWpList(){document.getElementById('wp-count').textContent=waypoints.length;document.getElementById('total-wp').textContent=waypoints.length;document.getElementById('waypoint-list').innerHTML=waypoints.map((w,i)=>'<div class="waypoint-item"><span class="wp-num">'+(i+1)+'</span><span>('+w.x+','+w.y+')</span><span class="wp-remove" onclick="removeWp('+i+')">✕</span></div>').join('')||'<p style="color:#64748b;font-size:0.7rem;">No waypoints</p>';}
        
        async function startNav(){if(waypoints.length===0){alert('Add waypoints');return;}if(!initialPose.set&&!confirm('Pose not set. Continue?'))return;await apiPost('/waypoints/set',{waypoints,wait_mode:document.getElementById('wait-mode').value,wait_time:parseFloat(document.getElementById('wait-time').value)||30,loop:document.getElementById('loop-continuous').checked});await apiPost('/waypoints/control',{command:'start'});showScreen('nav-running');}
        async function wpCtrl(cmd){await apiPost('/waypoints/control',{command:cmd});}
        function stopNav(){wpCtrl('stop');showScreen('nav-setup');}
        
        function teleop(l,a){apiPost('/teleop',{linear_x:l,angular_z:a});}
        function teleopStop(){teleop(0,0);}
        document.addEventListener('keydown',e=>{if(e.target.tagName==='INPUT')return;const k=e.key.toLowerCase();if(k==='w'||k==='arrowup')teleop(0.3,0);else if(k==='s'||k==='arrowdown')teleop(-0.3,0);else if(k==='a'||k==='arrowleft')teleop(0,0.5);else if(k==='d'||k==='arrowright')teleop(0,-0.5);else if(k===' '){teleopStop();e.preventDefault();}});
        document.addEventListener('keyup',e=>{if(e.target.tagName==='INPUT')return;if(['w','s','a','d','arrowup','arrowdown','arrowleft','arrowright'].includes(e.key.toLowerCase()))teleopStop();});
        
        function motor(cmd){apiPost('/motor/'+cmd,{});notify('Motor: '+cmd,'success');}
        
        async function updateStatus(){const d=await apiGet('/status');if(!d||!d.nodes)return;slamRunning=d.nodes.slam_2d||d.nodes.slam_3d;locRunning=d.nodes.localization;
            Object.entries(d.nodes).forEach(([id,r])=>{document.querySelectorAll('[id*="dot-'+id+'"], [id*="dot2-'+id+'"]').forEach(e=>{e.className='status-dot'+(r?' running':'');});document.querySelectorAll('[id*="btn-start-'+id+'"], [id*="btn-start2-'+id+'"]').forEach(e=>{e.disabled=r;});document.querySelectorAll('[id*="btn-stop-'+id+'"], [id*="btn-stop2-'+id+'"]').forEach(e=>{e.disabled=!r;});});
            document.querySelectorAll('[id="dot-slam"]').forEach(e=>{e.className='status-dot'+(slamRunning?' running':'');});document.querySelectorAll('[id="btn-start-slam"]').forEach(e=>{e.disabled=slamRunning;});document.querySelectorAll('[id="btn-stop-slam"]').forEach(e=>{e.disabled=!slamRunning;});}
        
        async function updateLogs(){const d=await apiGet('/logs/all');if(!d)return;const m={'log-mapping':'robot_cameras','log-nav':'localization','log-detection':'detection','log-manual':'robot_no_cameras','log-running':'localization'};for(const[p,n]of Object.entries(m)){const el=document.getElementById(p);if(!el)continue;const logs=d[n]||[];el.innerHTML=logs.slice(-40).map(l=>'<div class="log-line '+l.level+'">['+l.time+'] '+l.msg+'</div>').join('');el.scrollTop=el.scrollHeight;}}
        
        function notify(msg,type='info'){const colors={success:'#22c55e',error:'#ef4444',warning:'#f59e0b',info:'#3b82f6'};const n=document.createElement('div');n.style.cssText='position:fixed;top:45px;right:8px;padding:0.4rem 0.7rem;background:'+colors[type]+';color:white;border-radius:0.3rem;z-index:1001;font-weight:600;font-size:0.75rem;max-width:220px;';n.textContent=msg;document.body.appendChild(n);setTimeout(()=>n.remove(),2500);}
        
        init();
    </script>
</body>
</html>
'''


@app.route('/')
def index():
    return render_template_string(HTML)

@app.route('/status')
def status():
    return jsonify({'nodes': node_manager.get_status()})

@app.route('/logs/all')
def logs_all():
    return jsonify({node_id: node_manager.get_logs(node_id) for node_id in node_manager.node_configs})

@app.route('/nodes/<node_id>/start', methods=['POST'])
def start_node_route(node_id):
    data = request.get_json(force=True, silent=True) or {}
    return jsonify(node_manager.start_node(node_id, **data))

@app.route('/nodes/<node_id>/stop', methods=['POST'])
def stop_node_route(node_id):
    return jsonify(node_manager.stop_node(node_id))

@app.route('/nodes/stop_all', methods=['POST'])
def stop_all_route():
    return jsonify(node_manager.stop_all())

@app.route('/emergency_stop', methods=['POST'])
def emergency_stop_route():
    return jsonify(node_manager.emergency_stop())

@app.route('/maps/list')
def list_maps():
    return jsonify({'maps': get_maps()})

@app.route('/maps/save', methods=['POST'])
def save_map_route():
    data = request.get_json(force=True, silent=True) or {}
    return jsonify(save_map(data.get('name', '')))

@app.route('/set_initial_pose', methods=['POST'])
def set_initial_pose_route():
    data = request.get_json(force=True, silent=True) or {}
    return jsonify(set_initial_pose(data.get('x', 0), data.get('y', 0), data.get('theta', 0)))

@app.route('/teleop', methods=['POST'])
def teleop_route():
    data = request.get_json(force=True, silent=True) or {}
    send_teleop(data.get('linear_x', 0), data.get('angular_z', 0))
    return jsonify({'success': True})

@app.route('/motor/<cmd>', methods=['POST'])
def motor_route(cmd):
    return jsonify(motor_cmd(cmd))

@app.route('/waypoints/set', methods=['POST'])
def set_waypoints():
    data = request.get_json(force=True, silent=True) or {}
    wps = data.get('waypoints', [])
    try:
        wp_json = json.dumps(wps).replace('"', '\\"')
        subprocess.Popen(['ros2', 'topic', 'pub', '--once', '/waypoint/list', 'std_msgs/msg/String', '{"data": "' + wp_json + '"}'], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        config = {'wait_mode': data.get('wait_mode', 'time_based'), 'wait_time': data.get('wait_time', 30), 'loop_continuous': data.get('loop', True)}
        config_json = json.dumps(config).replace('"', '\\"')
        subprocess.Popen(['ros2', 'topic', 'pub', '--once', '/waypoint/config', 'std_msgs/msg/String', '{"data": "' + config_json + '"}'], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    except: pass
    return jsonify({'success': True})

@app.route('/waypoints/control', methods=['POST'])
def waypoint_control():
    data = request.get_json(force=True, silent=True) or {}
    cmd = data.get('command', '')
    try:
        subprocess.Popen(['ros2', 'topic', 'pub', '--once', '/waypoint/control', 'std_msgs/msg/String', '{"data": "' + cmd + '"}'], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    except: pass
    return jsonify({'success': True})


def cleanup():
    print("\nCleaning up...")
    node_manager.stop_all()
    print("Done.")


def main():
    import atexit
    atexit.register(cleanup)
    def sig_handler(sig, frame):
        cleanup()
        exit(0)
    signal.signal(signal.SIGINT, sig_handler)
    signal.signal(signal.SIGTERM, sig_handler)
    print("="*50)
    print("Manriix Control Panel v3.1")
    print("="*50)
    print("Web: http://0.0.0.0:5000")
    print("ROSBridge: ros2 launch rosbridge_server rosbridge_websocket_launch.xml")
    print("="*50)
    app.run(host='0.0.0.0', port=5000, debug=False, threaded=True)


if __name__ == '__main__':
    main()
