#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist

class MoveForward:
    def __init__(self):
        rospy.init_node('x_movement', anonymous=True)
        self.cmd_vel_ = rospy.Publisher('/four_wheel_steering_controller/cmd_vel', Twist, queue_size=10)
        
        rospy.loginfo("Receiving the user's input:")
        self.vel_ = float(input("Input speed: "))
        self.distance_ = float(input("Type your distance: "))
        self.isForward_ = input("Forward? (y/n): ").lower() == 'y'
        
        rospy.loginfo("Movement in x-direction has been initiated")
        self.send_velocity_command()

    def send_velocity_command(self):
        msg = Twist()
        
        if self.isForward_:
            msg.linear.x = self.vel_
        else:
            msg.linear.x = -self.vel_
        
        current_distance = 0.0
        t0 = rospy.Time.now().to_sec()
        
        rate = rospy.Rate(10)  # 10Hz
        while not rospy.is_shutdown() and current_distance < self.distance_:
            self.cmd_vel_.publish(msg)
            
            t1 = rospy.Time.now().to_sec()
            current_distance = self.vel_ * (t1 - t0)
            rospy.loginfo("%.4f" % current_distance)
            
            rate.sleep()
        
        # Stop the robot
        msg.linear.x = 0.0
        self.cmd_vel_.publish(msg)
        rospy.loginfo("Movement completed")

if __name__ == '__main__':
    try:
        MoveForward()
    except rospy.ROSInterruptException:
        pass