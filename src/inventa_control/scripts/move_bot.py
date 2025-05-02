#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import math

class MoveRobot:
    def __init__(self):
        rospy.init_node('move_robot', anonymous=True)
        
        # Publisher for cmd_vel topic (same as teleop_twist_keyboard)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        # Parameters
        self.linear_speed = 0.5  # m/s
        self.angular_speed = 1.0  # rad/s

    def move_to_position(self, distance, bearing_angle):
        twist = Twist()
        
        # Rotate
        twist.angular.z = math.radians(bearing_angle)
        start_time = rospy.Time.now()
        while (rospy.Time.now() - start_time).to_sec() < 2.0:
            self.cmd_vel_pub.publish(twist)
            rospy.sleep(0.1)
        
        # Stop rotation
        twist.angular.z = 0
        self.cmd_vel_pub.publish(twist)
        rospy.sleep(0.5)
        
        # Move forward
        twist.linear.x = distance
        start_time = rospy.Time.now()
        move_duration = distance / self.linear_speed
        while (rospy.Time.now() - start_time).to_sec() < move_duration:
            self.cmd_vel_pub.publish(twist)
            rospy.sleep(0.1)
        
        # Stop
        twist.linear.x = 0
        self.cmd_vel_pub.publish(twist)

    def run(self):
        while not rospy.is_shutdown():
            try:
                print("Enter distance (m) and bearing angle (degrees) separated by space (or 'q' to quit):")
                user_input = input()
                
                if user_input.lower() == 'q':
                    break
                
                distance, bearing_angle = map(float, user_input.split())
                self.move_to_position(distance, bearing_angle)
                
            except ValueError:
                print("Invalid input. Please enter distance and angle.")
            except Exception as e:
                print(f"An error occurred: {e}")

def main():
    try:
        robot_controller = MoveRobot()
        robot_controller.run()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()