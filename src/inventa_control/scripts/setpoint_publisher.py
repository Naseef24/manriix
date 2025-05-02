#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64

def publish_setpoints():
    rospy.init_node('setpoint_publisher', anonymous=True)
    
    # Steering joint setpoint publishers
    steering_pubs = {
        'front_left': rospy.Publisher('/front_left_steering_joint/setpoint', Float64, queue_size=10),
        'front_right': rospy.Publisher('/front_right_steering_joint/setpoint', Float64, queue_size=10),
        'rear_left': rospy.Publisher('/rear_left_steering_joint/setpoint', Float64, queue_size=10),
        'rear_right': rospy.Publisher('/rear_right_steering_joint/setpoint', Float64, queue_size=10),
    }
    
    # Wheel joint setpoint publishers
    wheel_pubs = {
        'ffl': rospy.Publisher('/ffl_wheel_joint/setpoint', Float64, queue_size=10),
        'ffr': rospy.Publisher('/ffr_wheel_joint/setpoint', Float64, queue_size=10),
        'rfl': rospy.Publisher('/rfl_wheel_joint/setpoint', Float64, queue_size=10),
        'rfr': rospy.Publisher('/rfr_wheel_joint/setpoint', Float64, queue_size=10),
    }
    
    # Lidar head setpoint publisher
    # lidar_pub = rospy.Publisher('/lidar_head_joint/setpoint', Float64, queue_size=10)
    
    rate = rospy.Rate(10)  # 10 Hz
    
    while not rospy.is_shutdown():
        setpoint = Float64()
        
        # Publish steering setpoints
        setpoint.data = 0.5  # Adjust as needed
        for pub in steering_pubs.values():
            pub.publish(setpoint)
        
        # Publish wheel setpoints
        setpoint.data = 1.0  # Adjust as needed
        for pub in wheel_pubs.values():
            pub.publish(setpoint)
        
        # Publish lidar head setpoint
        # setpoint.data = 0.5  # Adjust as needed
        # lidar_pub.publish(setpoint)
        
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_setpoints()
    except rospy.ROSInterruptException:
        pass