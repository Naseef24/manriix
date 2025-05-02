#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist

class MoveForward:
    def __init__(self):
        rospy.init_node('x_movement', anonymous=True)
        self.ang_cmd_vel_ = rospy.Publisher('/four_wheel_steering_controller/cmd_vel', Twist, queue_size=10)
        
        rospy.loginfo("Receiving the user's input:")
        self.ang_vel_ = float(input("Input rotational speed: "))
        self.distance_ = float(input("Type steering angle: "))
        self.isForward_ = input("Clockwise? (y/n): ").lower() == 'y'
        
        rospy.loginfo("Steering in z-direction has been initiated")
        self.send_velocity_command()

    def send_velocity_command(self):
        msg = Twist()
        
        if self.isForward_:
            msg.angular.z = self.ang_vel_
        else:
            msg.angular.z = -self.ang_vel_
        
        current_distance = 0.0
        t0 = rospy.Time.now().to_sec()
        
        rate = rospy.Rate(10)  # 10Hz
        while not rospy.is_shutdown() and current_distance < self.distance_:
            self.ang_cmd_vel_.publish(msg)
            
            t1 = rospy.Time.now().to_sec()
            current_distance = self.ang_vel_ * (t1 - t0)
            rospy.loginfo("%.4f" % current_distance)
            
            rate.sleep()
        
        # Stop the robot
        msg.angular.x = 0.0
        self.ang_cmd_vel_.publish(msg)
        rospy.loginfo("Movement completed")

if __name__ == '__main__':
    try:
        MoveForward()
    except rospy.ROSInterruptException:
        pass