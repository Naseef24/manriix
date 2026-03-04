#!/usr/bin/env python3
"""
ZLLG80ASM250 V1.0 - Hub Motor Monitor Node (ROS2)
Real-time visualization of current draw, torque (Nm), temperature, power, speed and voltage
for all 4 hub motors using ZLAC8015D controllers.

Motor Specifications (ZLLG80ASM250-L-24V):
    - Torque Constant (Kt): 0.8 Nm/A
    - Max Torque: 19.15 Nm @ 23.08A
    - Max Speed: 262 RPM (no load)
    - Max Efficiency: 80.7% @ 1.77 Nm

Author: Hype
Date: December 2024
"""

import rclpy
from rclpy.node import Node
import can
import struct
import threading
import time
from collections import deque
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.gridspec import GridSpec
import numpy as np


class ZLLG80ASM250Monitor(Node):
    """
    ZLLG80ASM250 V1.0 Motor Monitor
    Monitors 4 hub motors via 2 ZLAC8015D controllers
    """
    
    # Motor Constants (from ZLLG80ASM250-L-24V datasheet)
    KT_CONSTANT = 0.8       # Torque constant in Nm/A
    MAX_TORQUE = 19.15      # Maximum torque in Nm
    MAX_CURRENT = 23.082    # Maximum current in A
    RATED_VOLTAGE = 24.0    # Rated voltage in V
    MAX_SPEED = 262.0       # Max speed (no load) in RPM
    
    def __init__(self):
        super().__init__('zllg80asm250_monitor')
        
        # Declare parameters
        self.declare_parameter('can_channel', 'can2')
        self.declare_parameter('bitrate', 1000000)
        self.declare_parameter('update_rate', 10.0)
        self.declare_parameter('kt_constant', self.KT_CONSTANT)
        
        # Get parameters
        can_channel = self.get_parameter('can_channel').get_parameter_value().string_value
        bitrate = self.get_parameter('bitrate').get_parameter_value().integer_value
        self.update_rate = self.get_parameter('update_rate').get_parameter_value().double_value
        self.kt = self.get_parameter('kt_constant').get_parameter_value().double_value
        
        # CAN Bus setup
        try:
            self.bus = can.Bus(interface='socketcan', channel=can_channel, bitrate=bitrate)
            self.get_logger().info(f'CAN bus initialized on {can_channel} at {bitrate} bps')
        except Exception as e:
            self.get_logger().error(f'Failed to initialize CAN bus: {e}')
            raise
        
        # Driver IDs
        self.driver1_id = 0x601  # Left wheels
        self.driver2_id = 0x602  # Right wheels
        
        # Data storage (last 100 samples for plotting)
        self.history_length = 100
        self.time_data = deque(maxlen=self.history_length)
        
        # Current data (A)
        self.left_front_current = deque(maxlen=self.history_length)
        self.left_back_current = deque(maxlen=self.history_length)
        self.right_front_current = deque(maxlen=self.history_length)
        self.right_back_current = deque(maxlen=self.history_length)
        
        # Torque data (Nm) - calculated from current using Kt
        self.left_front_torque = deque(maxlen=self.history_length)
        self.left_back_torque = deque(maxlen=self.history_length)
        self.right_front_torque = deque(maxlen=self.history_length)
        self.right_back_torque = deque(maxlen=self.history_length)
        
        # Voltage data (V)
        self.bus_voltage = deque(maxlen=self.history_length)
        
        # Speed data (RPM)
        self.left_front_speed = deque(maxlen=self.history_length)
        self.left_back_speed = deque(maxlen=self.history_length)
        self.right_front_speed = deque(maxlen=self.history_length)
        self.right_back_speed = deque(maxlen=self.history_length)
        
        # Temperature data (°C)
        self.left_motor_temp = deque(maxlen=self.history_length)
        self.right_motor_temp = deque(maxlen=self.history_length)
        self.driver1_temp = deque(maxlen=self.history_length)
        self.driver2_temp = deque(maxlen=self.history_length)
        
        # Power data (W)
        self.total_power = deque(maxlen=self.history_length)
        
        # Initialize with zeros
        for _ in range(self.history_length):
            self.time_data.append(0)
            self.left_front_current.append(0)
            self.left_back_current.append(0)
            self.right_front_current.append(0)
            self.right_back_current.append(0)
            self.left_front_torque.append(0)
            self.left_back_torque.append(0)
            self.right_front_torque.append(0)
            self.right_back_torque.append(0)
            self.bus_voltage.append(0)
            self.left_front_speed.append(0)
            self.left_back_speed.append(0)
            self.right_front_speed.append(0)
            self.right_back_speed.append(0)
            self.left_motor_temp.append(0)
            self.right_motor_temp.append(0)
            self.driver1_temp.append(0)
            self.driver2_temp.append(0)
            self.total_power.append(0)
        
        # Start time
        self.start_time = time.time()
        
        # Lock for thread safety
        self.lock = threading.Lock()
        
        # Latest values for display
        self.latest_values = {
            'lf_current': 0, 'lb_current': 0, 'rf_current': 0, 'rb_current': 0,
            'lf_torque': 0, 'lb_torque': 0, 'rf_torque': 0, 'rb_torque': 0,
            'voltage': 0, 'power': 0,
            'lf_speed': 0, 'lb_speed': 0, 'rf_speed': 0, 'rb_speed': 0,
            'left_temp': 0, 'right_temp': 0, 'driver1_temp': 0, 'driver2_temp': 0
        }
        
        # Running flag
        self.running = True
        
        self.get_logger().info(f'ZLLG80ASM250 V1.0 Monitor initialized (Kt = {self.kt} Nm/A)')
    
    def send_sdo_read(self, driver_id, index, subindex):
        """Send SDO read request and get response"""
        data = [0x40, index & 0xFF, (index >> 8) & 0xFF, subindex, 0, 0, 0, 0]
        msg = can.Message(arbitration_id=driver_id, data=data, is_extended_id=False)
        
        try:
            self.bus.send(msg)
            
            response_id = 0x580 + (driver_id - 0x600)
            start = time.time()
            while time.time() - start < 0.1:
                recv_msg = self.bus.recv(timeout=0.05)
                if recv_msg and recv_msg.arbitration_id == response_id:
                    if recv_msg.data[1] == (index & 0xFF) and recv_msg.data[2] == ((index >> 8) & 0xFF):
                        return recv_msg.data
            return None
        except Exception as e:
            self.get_logger().warn(f'SDO read error: {e}')
            return None
    
    def read_current(self, driver_id):
        """
        Read current for both motors on a driver (Index 6077h - Torque actual value)
        Returns current in Amps (raw value is in 0.1A units)
        """
        left_data = self.send_sdo_read(driver_id, 0x6077, 0x01)
        right_data = self.send_sdo_read(driver_id, 0x6077, 0x02)
        
        left_current = 0
        right_current = 0
        
        if left_data and left_data[0] in [0x4B, 0x43, 0x4F]:
            # Signed 16-bit value in 0.1A units
            left_current = struct.unpack('<h', bytes(left_data[4:6]))[0] / 10.0
        
        if right_data and right_data[0] in [0x4B, 0x43, 0x4F]:
            right_current = struct.unpack('<h', bytes(right_data[4:6]))[0] / 10.0
        
        return left_current, right_current
    
    def current_to_torque(self, current_amps):
        """
        Convert current (A) to torque (Nm) using motor torque constant
        Torque (Nm) = Current (A) × Kt (Nm/A)
        """
        return current_amps * self.kt
    
    def read_voltage(self, driver_id):
        """Read bus voltage (Index 2035h) - returns voltage in V"""
        data = self.send_sdo_read(driver_id, 0x2035, 0x00)
        
        if data and data[0] in [0x4B, 0x43, 0x4F]:
            # Unsigned 16-bit value in 0.01V units
            voltage = struct.unpack('<H', bytes(data[4:6]))[0] / 100.0
            return voltage
        return 0
    
    def read_speed(self, driver_id):
        """Read actual speed for both motors (Index 606Ch) - returns speed in RPM"""
        left_data = self.send_sdo_read(driver_id, 0x606C, 0x01)
        right_data = self.send_sdo_read(driver_id, 0x606C, 0x02)
        
        left_speed = 0
        right_speed = 0
        
        if left_data and left_data[0] in [0x4B, 0x43, 0x4F]:
            # Signed 32-bit value in 0.1 RPM units
            left_speed = struct.unpack('<i', bytes(left_data[4:8]))[0] / 10.0
        
        if right_data and right_data[0] in [0x4B, 0x43, 0x4F]:
            right_speed = struct.unpack('<i', bytes(right_data[4:8]))[0] / 10.0
        
        return left_speed, right_speed
    
    def read_temperature(self, driver_id):
        """Read motor and driver temperatures (Index 2032h) - returns temps in °C"""
        left_data = self.send_sdo_read(driver_id, 0x2032, 0x01)
        right_data = self.send_sdo_read(driver_id, 0x2032, 0x02)
        driver_data = self.send_sdo_read(driver_id, 0x2032, 0x03)
        
        left_temp = 0
        right_temp = 0
        drv_temp = 0
        
        if left_data and left_data[0] in [0x4B, 0x43, 0x4F]:
            left_temp = struct.unpack('<h', bytes(left_data[4:6]))[0] / 10.0
        
        if right_data and right_data[0] in [0x4B, 0x43, 0x4F]:
            right_temp = struct.unpack('<h', bytes(right_data[4:6]))[0] / 10.0
        
        if driver_data and driver_data[0] in [0x4B, 0x43, 0x4F]:
            drv_temp = struct.unpack('<h', bytes(driver_data[4:6]))[0] / 10.0
        
        return left_temp, right_temp, drv_temp
    
    def update_data(self):
        """Read all sensor data and update storage"""
        current_time = time.time() - self.start_time
        
        # Read from Driver 1 (Left wheels)
        lf_current, lb_current = self.read_current(self.driver1_id)
        lf_speed, lb_speed = self.read_speed(self.driver1_id)
        voltage = self.read_voltage(self.driver1_id)
        l_temp, _, d1_temp = self.read_temperature(self.driver1_id)
        
        # Read from Driver 2 (Right wheels)
        rf_current, rb_current = self.read_current(self.driver2_id)
        rf_speed, rb_speed = self.read_speed(self.driver2_id)
        r_temp, _, d2_temp = self.read_temperature(self.driver2_id)
        
        # Convert current to torque (Nm)
        lf_torque = self.current_to_torque(lf_current)
        lb_torque = self.current_to_torque(lb_current)
        rf_torque = self.current_to_torque(rf_current)
        rb_torque = self.current_to_torque(rb_current)
        
        # Calculate power (W) = Voltage × Total Current
        total_current = abs(lf_current) + abs(lb_current) + abs(rf_current) + abs(rb_current)
        power = voltage * total_current
        
        with self.lock:
            self.time_data.append(current_time)
            
            # Store current (A)
            self.left_front_current.append(lf_current)
            self.left_back_current.append(lb_current)
            self.right_front_current.append(rf_current)
            self.right_back_current.append(rb_current)
            
            # Store torque (Nm)
            self.left_front_torque.append(lf_torque)
            self.left_back_torque.append(lb_torque)
            self.right_front_torque.append(rf_torque)
            self.right_back_torque.append(rb_torque)
            
            # Store other data
            self.bus_voltage.append(voltage)
            self.total_power.append(power)
            
            self.left_front_speed.append(lf_speed)
            self.left_back_speed.append(lb_speed)
            self.right_front_speed.append(rf_speed)
            self.right_back_speed.append(rb_speed)
            
            self.left_motor_temp.append(l_temp)
            self.right_motor_temp.append(r_temp)
            self.driver1_temp.append(d1_temp)
            self.driver2_temp.append(d2_temp)
            
            # Update latest values
            self.latest_values = {
                'lf_current': lf_current, 'lb_current': lb_current,
                'rf_current': rf_current, 'rb_current': rb_current,
                'lf_torque': lf_torque, 'lb_torque': lb_torque,
                'rf_torque': rf_torque, 'rb_torque': rb_torque,
                'voltage': voltage, 'power': power,
                'lf_speed': lf_speed, 'lb_speed': lb_speed,
                'rf_speed': rf_speed, 'rb_speed': rb_speed,
                'left_temp': l_temp, 'right_temp': r_temp,
                'driver1_temp': d1_temp, 'driver2_temp': d2_temp
            }
    
    def data_collection_thread(self):
        """Background thread for continuous data collection"""
        period = 1.0 / self.update_rate
        while self.running and rclpy.ok():
            self.update_data()
            time.sleep(period)
    
    def setup_plot(self):
        """Setup matplotlib figure with subplots"""
        plt.style.use('dark_background')
        self.fig = plt.figure(figsize=(18, 14))
        self.fig.suptitle('ZLLG80ASM250 V1.0 - Hub Motor Monitor (Kt = 0.8 Nm/A)', 
                          fontsize=18, fontweight='bold', color='cyan')
        
        # 5 rows, 2 columns grid
        gs = GridSpec(5, 2, figure=self.fig, hspace=0.4, wspace=0.25)
        
        # Current plot (row 0, left)
        self.ax_current = self.fig.add_subplot(gs[0, 0])
        self.ax_current.set_title('Motor Current', fontsize=12, color='cyan')
        self.ax_current.set_xlabel('Time (s)')
        self.ax_current.set_ylabel('Current (A)')
        self.ax_current.grid(True, alpha=0.3)
        
        # Torque plot (row 0, right) - NOW IN Nm!
        self.ax_torque = self.fig.add_subplot(gs[0, 1])
        self.ax_torque.set_title('Motor Torque', fontsize=12, color='magenta')
        self.ax_torque.set_xlabel('Time (s)')
        self.ax_torque.set_ylabel('Torque (Nm)')
        self.ax_torque.grid(True, alpha=0.3)
        
        # Speed plot (row 1, left)
        self.ax_speed = self.fig.add_subplot(gs[1, 0])
        self.ax_speed.set_title('Motor Speed', fontsize=12, color='lime')
        self.ax_speed.set_xlabel('Time (s)')
        self.ax_speed.set_ylabel('Speed (RPM)')
        self.ax_speed.grid(True, alpha=0.3)
        
        # Voltage & Power plot (row 1, right)
        self.ax_voltage = self.fig.add_subplot(gs[1, 1])
        self.ax_voltage.set_title('Bus Voltage & Power', fontsize=12, color='yellow')
        self.ax_voltage.set_xlabel('Time (s)')
        self.ax_voltage.set_ylabel('Voltage (V)', color='yellow')
        self.ax_power = self.ax_voltage.twinx()
        self.ax_power.set_ylabel('Power (W)', color='red')
        self.ax_voltage.grid(True, alpha=0.3)
        
        # Motor Temperature plot (row 2, left)
        self.ax_temp = self.fig.add_subplot(gs[2, 0])
        self.ax_temp.set_title('Motor Temperature', fontsize=12, color='orange')
        self.ax_temp.set_xlabel('Time (s)')
        self.ax_temp.set_ylabel('Temperature (°C)')
        self.ax_temp.grid(True, alpha=0.3)
        
        # Driver Temperature plot (row 2, right)
        self.ax_drv_temp = self.fig.add_subplot(gs[2, 1])
        self.ax_drv_temp.set_title('Driver Temperature', fontsize=12, color='red')
        self.ax_drv_temp.set_xlabel('Time (s)')
        self.ax_drv_temp.set_ylabel('Temperature (°C)')
        self.ax_drv_temp.grid(True, alpha=0.3)
        
        # Status display (rows 3-4, spanning both columns)
        self.ax_status = self.fig.add_subplot(gs[3:, :])
        self.ax_status.set_xlim(0, 10)
        self.ax_status.set_ylim(0, 10)
        self.ax_status.axis('off')
        self.ax_status.set_title('Real-Time Status', fontsize=12, color='white')
        
        # Initialize plot lines - Current
        self.line_lf_current, = self.ax_current.plot([], [], 'c-', label='LF', linewidth=1.5)
        self.line_lb_current, = self.ax_current.plot([], [], 'b-', label='LB', linewidth=1.5)
        self.line_rf_current, = self.ax_current.plot([], [], 'm-', label='RF', linewidth=1.5)
        self.line_rb_current, = self.ax_current.plot([], [], 'r-', label='RB', linewidth=1.5)
        self.ax_current.legend(loc='upper right', fontsize=8)
        
        # Initialize plot lines - Torque (Nm)
        self.line_lf_torque, = self.ax_torque.plot([], [], 'c-', label='LF', linewidth=1.5)
        self.line_lb_torque, = self.ax_torque.plot([], [], 'b-', label='LB', linewidth=1.5)
        self.line_rf_torque, = self.ax_torque.plot([], [], 'm-', label='RF', linewidth=1.5)
        self.line_rb_torque, = self.ax_torque.plot([], [], 'r-', label='RB', linewidth=1.5)
        self.ax_torque.legend(loc='upper right', fontsize=8)
        # Add max torque reference line
        self.ax_torque.axhline(y=self.MAX_TORQUE, color='red', linestyle='--', alpha=0.5, label=f'Max {self.MAX_TORQUE} Nm')
        self.ax_torque.axhline(y=-self.MAX_TORQUE, color='red', linestyle='--', alpha=0.5)
        
        # Initialize plot lines - Voltage & Power
        self.line_voltage, = self.ax_voltage.plot([], [], 'y-', label='Voltage', linewidth=2)
        self.line_power, = self.ax_power.plot([], [], 'r-', label='Power', linewidth=2)
        self.ax_voltage.legend(loc='upper left', fontsize=8)
        self.ax_power.legend(loc='upper right', fontsize=8)
        
        # Initialize plot lines - Speed
        self.line_lf_speed, = self.ax_speed.plot([], [], 'c-', label='LF', linewidth=1.5)
        self.line_lb_speed, = self.ax_speed.plot([], [], 'b-', label='LB', linewidth=1.5)
        self.line_rf_speed, = self.ax_speed.plot([], [], 'm-', label='RF', linewidth=1.5)
        self.line_rb_speed, = self.ax_speed.plot([], [], 'r-', label='RB', linewidth=1.5)
        self.ax_speed.legend(loc='upper right', fontsize=8)
        
        # Initialize plot lines - Motor Temperature
        self.line_left_temp, = self.ax_temp.plot([], [], 'c-', label='Left Motor', linewidth=1.5)
        self.line_right_temp, = self.ax_temp.plot([], [], 'm-', label='Right Motor', linewidth=1.5)
        self.ax_temp.legend(loc='upper right', fontsize=8)
        
        # Initialize plot lines - Driver Temperature
        self.line_drv1_temp, = self.ax_drv_temp.plot([], [], 'orange', label='Driver 1', linewidth=1.5)
        self.line_drv2_temp, = self.ax_drv_temp.plot([], [], 'red', label='Driver 2', linewidth=1.5)
        self.ax_drv_temp.legend(loc='upper right', fontsize=8)
        
        # Status text
        self.status_text = self.ax_status.text(0.5, 0.5, '', transform=self.ax_status.transAxes,
                                                fontsize=11, verticalalignment='center',
                                                horizontalalignment='center',
                                                fontfamily='monospace', color='white')
        
        plt.tight_layout(rect=[0, 0, 1, 0.96])
    
    def animate(self, frame):
        """Animation update function"""
        with self.lock:
            time_list = list(self.time_data)
            
            # Update current plot
            self.line_lf_current.set_data(time_list, list(self.left_front_current))
            self.line_lb_current.set_data(time_list, list(self.left_back_current))
            self.line_rf_current.set_data(time_list, list(self.right_front_current))
            self.line_rb_current.set_data(time_list, list(self.right_back_current))
            
            # Update torque plot (Nm)
            self.line_lf_torque.set_data(time_list, list(self.left_front_torque))
            self.line_lb_torque.set_data(time_list, list(self.left_back_torque))
            self.line_rf_torque.set_data(time_list, list(self.right_front_torque))
            self.line_rb_torque.set_data(time_list, list(self.right_back_torque))
            
            # Update voltage & power plot
            self.line_voltage.set_data(time_list, list(self.bus_voltage))
            self.line_power.set_data(time_list, list(self.total_power))
            
            # Update speed plot
            self.line_lf_speed.set_data(time_list, list(self.left_front_speed))
            self.line_lb_speed.set_data(time_list, list(self.left_back_speed))
            self.line_rf_speed.set_data(time_list, list(self.right_front_speed))
            self.line_rb_speed.set_data(time_list, list(self.right_back_speed))
            
            # Update temperature plots
            self.line_left_temp.set_data(time_list, list(self.left_motor_temp))
            self.line_right_temp.set_data(time_list, list(self.right_motor_temp))
            self.line_drv1_temp.set_data(time_list, list(self.driver1_temp))
            self.line_drv2_temp.set_data(time_list, list(self.driver2_temp))
            
            # Update axis limits
            if len(time_list) > 0 and time_list[-1] > 0:
                x_min = max(0, time_list[-1] - 10)
                x_max = time_list[-1] + 0.5
                for ax in [self.ax_current, self.ax_torque, self.ax_voltage, 
                           self.ax_speed, self.ax_temp, self.ax_drv_temp]:
                    ax.set_xlim(x_min, x_max)
            
            # Auto-scale Y axes
            def safe_max(data, default=1):
                try:
                    return max(max(abs(x) for x in data), default)
                except:
                    return default
            
            # Current axis
            current_max = max(
                safe_max(self.left_front_current),
                safe_max(self.left_back_current),
                safe_max(self.right_front_current),
                safe_max(self.right_back_current),
                1
            )
            self.ax_current.set_ylim(-current_max * 0.1, current_max * 1.2)
            
            # Torque axis (Nm)
            torque_max = max(
                safe_max(self.left_front_torque),
                safe_max(self.left_back_torque),
                safe_max(self.right_front_torque),
                safe_max(self.right_back_torque),
                1
            )
            # Ensure we can see the max torque reference line
            torque_max = max(torque_max, self.MAX_TORQUE * 0.5)
            self.ax_torque.set_ylim(-torque_max * 1.2, torque_max * 1.2)
            
            # Speed axis
            speed_max = max(
                safe_max(self.left_front_speed),
                safe_max(self.left_back_speed),
                safe_max(self.right_front_speed),
                safe_max(self.right_back_speed),
                10
            )
            self.ax_speed.set_ylim(-speed_max * 1.2, speed_max * 1.2)
            
            # Voltage and power axes
            self.ax_voltage.set_ylim(20, 30)
            power_max = safe_max(self.total_power, 10)
            self.ax_power.set_ylim(0, power_max * 1.2)
            
            # Temperature axes
            temp_max = max(
                safe_max(self.left_motor_temp, 20),
                safe_max(self.right_motor_temp, 20),
                40
            )
            self.ax_temp.set_ylim(0, temp_max * 1.2)
            
            drv_temp_max = max(
                safe_max(self.driver1_temp, 20),
                safe_max(self.driver2_temp, 20),
                40
            )
            self.ax_drv_temp.set_ylim(0, drv_temp_max * 1.2)
            
            # Calculate totals
            v = self.latest_values
            total_torque = abs(v['lf_torque']) + abs(v['lb_torque']) + abs(v['rf_torque']) + abs(v['rb_torque'])
            total_current = abs(v['lf_current']) + abs(v['lb_current']) + abs(v['rf_current']) + abs(v['rb_current'])
            
            # Update status text with both Current (A) and Torque (Nm)
            status_str = (
                f"╔════════════════════════════════════════════════════════════════════════════════════════════════════════════════╗\n"
                f"║                                ZLLG80ASM250 V1.0 - Real-Time Status                                            ║\n"
                f"║                                Motor: ZLLG80ASM250-L-24V | Kt = 0.8 Nm/A                                        ║\n"
                f"╠════════════════════════════════════════════════════════════════════════════════════════════════════════════════╣\n"
                f"║  CURRENT (A)     │  LF: {v['lf_current']:7.2f}  │  LB: {v['lb_current']:7.2f}  │  RF: {v['rf_current']:7.2f}  │  RB: {v['rb_current']:7.2f}  │  Total: {total_current:7.2f}  ║\n"
                f"║  TORQUE (Nm)     │  LF: {v['lf_torque']:7.2f}  │  LB: {v['lb_torque']:7.2f}  │  RF: {v['rf_torque']:7.2f}  │  RB: {v['rb_torque']:7.2f}  │  Total: {total_torque:7.2f}  ║\n"
                f"║  SPEED (RPM)     │  LF: {v['lf_speed']:7.1f}  │  LB: {v['lb_speed']:7.1f}  │  RF: {v['rf_speed']:7.1f}  │  RB: {v['rb_speed']:7.1f}  │                  ║\n"
                f"╠════════════════════════════════════════════════════════════════════════════════════════════════════════════════╣\n"
                f"║  VOLTAGE: {v['voltage']:6.2f} V   │   POWER: {v['power']:7.1f} W   │   MOTOR TEMP: L={v['left_temp']:5.1f}°C  R={v['right_temp']:5.1f}°C                     ║\n"
                f"║  Max Torque: {self.MAX_TORQUE:5.2f} Nm  │   Max Current: {self.MAX_CURRENT:5.2f} A   │   DRIVER TEMP: D1={v['driver1_temp']:5.1f}°C  D2={v['driver2_temp']:5.1f}°C               ║\n"
                f"╚════════════════════════════════════════════════════════════════════════════════════════════════════════════════╝"
            )
            self.status_text.set_text(status_str)
        
        return (self.line_lf_current, self.line_lb_current, self.line_rf_current, self.line_rb_current,
                self.line_lf_torque, self.line_lb_torque, self.line_rf_torque, self.line_rb_torque,
                self.line_voltage, self.line_power,
                self.line_lf_speed, self.line_lb_speed, self.line_rf_speed, self.line_rb_speed,
                self.line_left_temp, self.line_right_temp,
                self.line_drv1_temp, self.line_drv2_temp,
                self.status_text)
    
    def run(self):
        """Main run function"""
        # Start data collection thread
        data_thread = threading.Thread(target=self.data_collection_thread)
        data_thread.daemon = True
        data_thread.start()
        
        # Setup and start plot
        self.setup_plot()
        
        # Create animation
        ani = animation.FuncAnimation(
            self.fig, self.animate, interval=100, blit=False, cache_frame_data=False
        )
        
        self.get_logger().info('Starting ZLLG80ASM250 V1.0 Monitor visualization...')
        self.get_logger().info('Close the plot window to exit.')
        
        plt.show()
        
        # Cleanup
        self.running = False
        self.bus.shutdown()
        self.get_logger().info('Monitor shutdown complete.')


def main(args=None):
    rclpy.init(args=args)
    
    try:
        monitor = ZLLG80ASM250Monitor()
        monitor.run()
    except KeyboardInterrupt:
        print("\nShutting down...")
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
