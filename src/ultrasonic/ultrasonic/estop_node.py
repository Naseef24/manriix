#!/usr/bin/env python3
"""
E-Stop Safety Node

Monitors ultrasonic sensors and triggers emergency stop when obstacles
are detected within threshold distance.

Features:
- Configurable threshold distance (default 15cm)
- Debounce logic to prevent false triggers
- Watchdog timer to trigger E-Stop on sensor data loss
- Per-sensor enable/disable configuration
- Safety-critical QoS settings

Topics:
    Subscribed:
        /ultrasonic (std_msgs/Float32MultiArray): Raw sensor readings
    Published:
        /estop (std_msgs/Bool): Emergency stop command

Parameters:
    threshold_cm (float): Distance threshold in cm (default: 15.0)
    debounce_count (int): Consecutive readings to trigger (default: 2)
    sensor_names (list): Names for each sensor position
    enabled_sensors (list): Enable/disable individual sensors
    watchdog_timeout_sec (float): Timeout before watchdog triggers (default: 1.0)

Author: Hype
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import Float32MultiArray, Bool
from rcl_interfaces.msg import ParameterDescriptor, ParameterType


class EStopNode(Node):
    """
    Emergency Stop Node
    
    Monitors ultrasonic sensor readings and publishes E-Stop commands
    when obstacles are detected within the threshold distance.
    """
    
    def __init__(self):
        super().__init__('estop_node')
        
        # ============== PARAMETERS ==============
        
        self.declare_parameter(
            'threshold_cm', 
            15.0,
            ParameterDescriptor(
                description='Distance threshold in cm to trigger e-stop',
                type=ParameterType.PARAMETER_DOUBLE
            )
        )
        
        self.declare_parameter(
            'sensor_names',
            ['front_left', 'front_center', 'front_right', 
             'rear_right', 'rear_center', 'rear_left'],
            ParameterDescriptor(
                description='Names of ultrasonic sensors in array order',
                type=ParameterType.PARAMETER_STRING_ARRAY
            )
        )
        
        self.declare_parameter(
            'enabled_sensors',
            [True, True, True, True, True, True],
            ParameterDescriptor(
                description='Which sensors to monitor for e-stop',
                type=ParameterType.PARAMETER_BOOL_ARRAY
            )
        )
        
        self.declare_parameter(
            'debounce_count',
            2,
            ParameterDescriptor(
                description='Number of consecutive readings below threshold to trigger',
                type=ParameterType.PARAMETER_INTEGER
            )
        )
        
        self.declare_parameter(
            'watchdog_timeout_sec',
            1.0,
            ParameterDescriptor(
                description='Seconds without data before watchdog triggers e-stop',
                type=ParameterType.PARAMETER_DOUBLE
            )
        )
        
        self.declare_parameter(
            'invalid_reading_value',
            999.0,
            ParameterDescriptor(
                description='Value indicating no echo received (ignore this reading)',
                type=ParameterType.PARAMETER_DOUBLE
            )
        )
        
        # Get parameter values
        self.threshold = self.get_parameter('threshold_cm').value
        self.sensor_names = self.get_parameter('sensor_names').value
        self.enabled_sensors = self.get_parameter('enabled_sensors').value
        self.debounce_count = self.get_parameter('debounce_count').value
        self.watchdog_timeout = self.get_parameter('watchdog_timeout_sec').value
        self.invalid_value = self.get_parameter('invalid_reading_value').value
        
        # ============== STATE ==============
        
        self.num_sensors = len(self.sensor_names)
        self.consecutive_triggers = [0] * self.num_sensors
        self.estop_active = False
        self.last_msg_time = self.get_clock().now()
        self.sensor_data_received = False
        
        # ============== QOS PROFILES ==============
        
        # Safety-critical QoS for e-stop
        safety_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Sensor data QoS
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )
        
        # ============== SUBSCRIBERS ==============
        
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'ultrasonic',
            self.ultrasonic_callback,
            sensor_qos
        )
        
        # ============== PUBLISHERS ==============
        
        self.estop_pub = self.create_publisher(
            Bool, 
            'estop', 
            safety_qos
        )
        
        # Also publish detailed status for debugging
        self.status_pub = self.create_publisher(
            Float32MultiArray,
            'ultrasonic_status',
            10
        )
        
        # ============== TIMERS ==============
        
        # Watchdog timer
        self.watchdog_timer = self.create_timer(
            0.5,  # Check every 500ms
            self.watchdog_callback
        )
        
        # Parameter update callback
        self.add_on_set_parameters_callback(self.parameter_callback)
        
        # ============== STARTUP ==============
        
        self.get_logger().info(
            f'E-Stop node started\n'
            f'  Threshold: {self.threshold}cm\n'
            f'  Debounce: {self.debounce_count} readings\n'
            f'  Watchdog timeout: {self.watchdog_timeout}s\n'
            f'  Sensors: {", ".join(self.sensor_names)}'
        )
        
        # Publish initial e-stop state (safe = stopped until sensors active)
        initial_msg = Bool()
        initial_msg.data = True  # Start with e-stop active
        self.estop_pub.publish(initial_msg)
        self.estop_active = True
        self.get_logger().warn('E-Stop ACTIVE until sensor data received')

    def parameter_callback(self, params):
        """Handle parameter updates at runtime"""
        from rcl_interfaces.msg import SetParametersResult
        
        for param in params:
            if param.name == 'threshold_cm':
                self.threshold = param.value
                self.get_logger().info(f'Threshold updated to {self.threshold}cm')
            elif param.name == 'debounce_count':
                self.debounce_count = param.value
                self.get_logger().info(f'Debounce count updated to {self.debounce_count}')
            elif param.name == 'enabled_sensors':
                self.enabled_sensors = param.value
                self.get_logger().info(f'Enabled sensors updated')
        
        return SetParametersResult(successful=True)

    def ultrasonic_callback(self, msg: Float32MultiArray):
        """
        Process incoming ultrasonic sensor data
        
        Checks each sensor reading against threshold and triggers
        e-stop if any enabled sensor detects an obstacle too close.
        """
        self.last_msg_time = self.get_clock().now()
        self.sensor_data_received = True
        
        # Validate message length
        if len(msg.data) != self.num_sensors:
            self.get_logger().warn(
                f'Expected {self.num_sensors} sensors, got {len(msg.data)}'
            )
            return
        
        trigger_estop = False
        triggered_sensors = []
        
        # Check each sensor
        for i, distance in enumerate(msg.data):
            # Skip disabled sensors
            if i >= len(self.enabled_sensors) or not self.enabled_sensors[i]:
                self.consecutive_triggers[i] = 0
                continue
            
            # Skip invalid readings (no echo)
            if distance >= self.invalid_value:
                self.consecutive_triggers[i] = 0
                continue
            
            # Check threshold
            if distance < self.threshold and distance > 0:
                self.consecutive_triggers[i] += 1
                
                # Trigger if debounce count reached
                if self.consecutive_triggers[i] >= self.debounce_count:
                    trigger_estop = True
                    triggered_sensors.append(
                        f'{self.sensor_names[i]}: {distance:.1f}cm'
                    )
            else:
                # Reset counter when reading is safe
                self.consecutive_triggers[i] = 0
        
        # Publish e-stop state
        estop_msg = Bool()
        estop_msg.data = trigger_estop
        self.estop_pub.publish(estop_msg)
        
        # Log state changes
        if trigger_estop and not self.estop_active:
            self.get_logger().warn(
                f'🛑 E-STOP TRIGGERED! Sensors: {", ".join(triggered_sensors)}'
            )
            self.estop_active = True
        elif not trigger_estop and self.estop_active:
            self.get_logger().info('✅ E-STOP CLEARED - Path clear')
            self.estop_active = False

    def watchdog_callback(self):
        """
        Watchdog timer callback
        
        Triggers e-stop if no sensor data received within timeout period.
        This is a critical safety feature to handle sensor/communication failures.
        """
        if not self.sensor_data_received:
            # Still waiting for first message
            return
        
        elapsed = (self.get_clock().now() - self.last_msg_time).nanoseconds / 1e9
        
        if elapsed > self.watchdog_timeout:
            if not self.estop_active:
                self.get_logger().error(
                    f'⚠️ WATCHDOG: No ultrasonic data for {elapsed:.2f}s! '
                    f'Triggering E-STOP'
                )
            
            # Always publish e-stop in watchdog condition
            estop_msg = Bool()
            estop_msg.data = True
            self.estop_pub.publish(estop_msg)
            self.estop_active = True


def main(args=None):
    rclpy.init(args=args)
    
    node = EStopNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down E-Stop node...')
    finally:
        # CRITICAL: Ensure e-stop is active on shutdown
        estop_msg = Bool()
        estop_msg.data = True
        node.estop_pub.publish(estop_msg)
        node.get_logger().warn('E-Stop activated on shutdown')
        
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
