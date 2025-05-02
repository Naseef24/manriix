#!/usr/bin/env python3

import rospy
import numpy as np
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import tf.transformations

class SwerveCommandFilter:
    """
    Filter for swerve controller commands that:
    1. Smooths velocity transitions
    2. Optimizes commands for limited steering angles
    3. Transforms motion to use preferred directions
    """
    def __init__(self):
        # Initialize node
        rospy.init_node('swerve_command_filter')
        
        # Parameters
        self.max_steering_angle = rospy.get_param('~max_steering_angle', 1.0)  # Maximum steering angle allowed
        self.preferred_angle_threshold = rospy.get_param('~preferred_angle_threshold', 0.6)  # When to start biasing movement
        self.direction_bias = rospy.get_param('~direction_bias', 0.7)  # Strength of direction bias (0-1)
        self.track = rospy.get_param('~track', 0.75)  # Distance between wheels
        self.wheel_base = rospy.get_param('~wheel_base', 0.365)  # Wheel base
        self.smoothing_factor = rospy.get_param('~smoothing_factor', 0.3)  # Command smoothing (0-1)
        
        # Velocities
        self.max_x_vel = rospy.get_param('~max_x_vel', 1.667)
        self.max_y_vel = rospy.get_param('~max_y_vel', 1.0)
        self.max_angular_vel = rospy.get_param('~max_angular_vel', 1.0)
        
        # Acceleration
        self.max_linear_accel = rospy.get_param('~max_linear_accel', 1.0) 
        self.max_angular_accel = rospy.get_param('~max_angular_accel', 1.0)
        
        # State variables
        self.last_cmd = Twist()
        self.last_cmd_time = rospy.Time.now()
        self.robot_heading = 0.0
        
        # Subscribe to input cmd_vel (from move_base/TEB)
        self.sub_cmd_vel = rospy.Subscriber('cmd_vel_in', Twist, self.cmd_vel_callback)
        
        # Subscribe to odometry for robot orientation
        self.sub_odom = rospy.Subscriber('odom', Odometry, self.odom_callback)
        
        # Publisher for filtered commands
        self.pub_cmd_vel = rospy.Publisher('cmd_vel_out', Twist, queue_size=1)
        
        rospy.loginfo("Swerve Command Filter initialized")
        
    def odom_callback(self, msg):
        # Extract robot heading from odometry quaternion
        quaternion = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        )
        euler = tf.transformations.euler_from_quaternion(quaternion)
        self.robot_heading = euler[2]  # yaw in radians
        
    def cmd_vel_callback(self, msg):
        # Get time difference for acceleration limiting
        current_time = rospy.Time.now()
        dt = (current_time - self.last_cmd_time).to_sec()
        self.last_cmd_time = current_time
        
        # Ensure dt is valid
        if dt <= 0 or dt > 1.0:
            dt = 0.1
            
        # Create modified command
        filtered_cmd = Twist()
        
        # Apply acceleration limits
        filtered_cmd.linear.x = self.limit_acceleration(
            msg.linear.x, self.last_cmd.linear.x, self.max_linear_accel, dt)
        filtered_cmd.linear.y = self.limit_acceleration(
            msg.linear.y, self.last_cmd.linear.y, self.max_linear_accel, dt)
        filtered_cmd.angular.z = self.limit_acceleration(
            msg.angular.z, self.last_cmd.angular.z, self.max_angular_accel, dt)
        
        # Apply velocity smoothing (with previous command)
        filtered_cmd.linear.x = self.last_cmd.linear.x + self.smoothing_factor * (filtered_cmd.linear.x - self.last_cmd.linear.x)
        filtered_cmd.linear.y = self.last_cmd.linear.y + self.smoothing_factor * (filtered_cmd.linear.y - self.last_cmd.linear.y)
        filtered_cmd.angular.z = self.last_cmd.angular.z + self.smoothing_factor * (filtered_cmd.angular.z - self.last_cmd.angular.z)
        
        # Apply direction bias (prefer forward/backward over diagonal movement)
        filtered_cmd = self.apply_direction_bias(filtered_cmd)
        
        # Apply steering angle optimization
        filtered_cmd = self.optimize_for_steering_limits(filtered_cmd)
        
        # Apply maximum velocity limits
        filtered_cmd.linear.x = self.clamp(filtered_cmd.linear.x, -self.max_x_vel, self.max_x_vel)
        filtered_cmd.linear.y = self.clamp(filtered_cmd.linear.y, -self.max_y_vel, self.max_y_vel)
        filtered_cmd.angular.z = self.clamp(filtered_cmd.angular.z, -self.max_angular_vel, self.max_angular_vel)
        
        # Update last command
        self.last_cmd = filtered_cmd
        
        # Publish filtered command
        self.pub_cmd_vel.publish(filtered_cmd)
        
    def limit_acceleration(self, target_vel, current_vel, max_accel, dt):
        """Apply acceleration limits"""
        max_change = max_accel * dt
        return current_vel + self.clamp(target_vel - current_vel, -max_change, max_change)
        
    def apply_direction_bias(self, cmd):
        """Bias movement toward cardinal directions based on current velocity magnitude"""
        # Calculate velocity magnitude
        vel_magnitude = math.sqrt(cmd.linear.x**2 + cmd.linear.y**2)
        
        # If velocity is too small, don't modify
        if vel_magnitude < 0.05:
            return cmd
            
        # Calculate movement angle in robot frame
        movement_angle = math.atan2(cmd.linear.y, cmd.linear.x)
        
        # Find nearest cardinal direction (0, 90, 180, 270 degrees)
        cardinal_angle = round(movement_angle / (math.pi/2)) * (math.pi/2)
        
        # Calculate angle difference
        angle_diff = abs(movement_angle - cardinal_angle)
        
        # Apply bias if within threshold
        if angle_diff < self.preferred_angle_threshold:
            # Calculate bias strength based on angle difference
            bias_strength = (1.0 - angle_diff/self.preferred_angle_threshold) * self.direction_bias
            
            # Interpolate between original angle and cardinal angle
            new_angle = movement_angle * (1 - bias_strength) + cardinal_angle * bias_strength
            
            # Convert back to x/y components while preserving magnitude
            cmd.linear.x = vel_magnitude * math.cos(new_angle)
            cmd.linear.y = vel_magnitude * math.sin(new_angle)
            
        return cmd
        
    def optimize_for_steering_limits(self, cmd):
        """Adjust motion commands to work within steering angle limitations"""
        # Calculate wheel steering angles to see if we need to adjust
        # Using swerve kinematics, calculate what the steering angles would be
        a = cmd.linear.y - cmd.angular.z * self.wheel_base / 2
        b = cmd.linear.y + cmd.angular.z * self.wheel_base / 2
        c = cmd.linear.x - cmd.angular.z * self.track / 2
        d = cmd.linear.x + cmd.angular.z * self.track / 2
        
        # Calculate the steering angles that would be commanded
        lf_angle = math.atan2(b, c) if (abs(b) > 0.001 or abs(c) > 0.001) else 0
        rf_angle = math.atan2(b, d) if (abs(b) > 0.001 or abs(d) > 0.001) else 0
        lh_angle = math.atan2(a, c) if (abs(a) > 0.001 or abs(c) > 0.001) else 0
        rh_angle = math.atan2(a, d) if (abs(a) > 0.001 or abs(d) > 0.001) else 0
        
        # Get the maximum angle
        max_angle = max(abs(lf_angle), abs(rf_angle), abs(lh_angle), abs(rh_angle))
        
        # If any angle exceeds our limit, scale down the y component and angular velocity
        if max_angle > self.max_steering_angle:
            scale_factor = self.max_steering_angle / max_angle
            
            # Scale the components that affect steering angle the most
            cmd.linear.y *= scale_factor
            cmd.angular.z *= scale_factor
            
            # Don't scale x as much (prefer forward motion)
            x_scale = scale_factor * 0.5 + 0.5  # Less aggressive scaling for x
            cmd.linear.x *= x_scale
            
        return cmd
    
    def clamp(self, value, min_value, max_value):
        """Clamp value between min and max"""
        return max(min(value, max_value), min_value)

if __name__ == '__main__':
    try:
        filter_node = SwerveCommandFilter()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass