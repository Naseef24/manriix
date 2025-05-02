#!/usr/bin/env python3

import rospy
import math
from geometry_msgs.msg import Twist

class MoveInCircle:
    def __init__(self):
        rospy.init_node('circle_movement', anonymous=True)
        self.cmd_vel_ = rospy.Publisher('/four_wheel_steering_controller/cmd_vel', Twist, queue_size=10)
        
        rospy.loginfo("Receiving the user's input:")
        self.linear_vel_ = float(input("Input linear speed: "))
        self.radius_ = float(input("Input circle radius: "))
        self.clockwise_ = input("Clockwise? (y/n): ").lower() == 'y'
        
        # Calculate angular velocity
        self.angular_vel_ = self.linear_vel_ / self.radius_
        
        rospy.loginfo("Circular movement has been initiated")
        self.move_in_circle()

    def move_in_circle(self):
        msg = Twist()
        
        # Set linear and angular velocities
        msg.linear.x = self.linear_vel_
        if self.clockwise_:
            msg.angular.z = -self.angular_vel_
        else:
            msg.angular.z = self.angular_vel_
        
        # Calculate time to complete one full circle
        circumference = 2 * math.pi * self.radius_
        duration = circumference / self.linear_vel_
        
        rate = rospy.Rate(10)  # 10Hz
        start_time = rospy.Time.now()
        
        while not rospy.is_shutdown() and (rospy.Time.now() - start_time).to_sec() < duration:
            self.cmd_vel_.publish(msg)
            rospy.loginfo("Moving in circle. Time left: %.2f seconds" % (duration - (rospy.Time.now() - start_time).to_sec()))
            rate.sleep()
        
        # Stop the robot
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.cmd_vel_.publish(msg)
        rospy.loginfo("Circular movement completed")

if __name__ == '__main__':
    try:
        MoveInCircle()
    except rospy.ROSInterruptException:
        pass