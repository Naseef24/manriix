#!/usr/bin/env python3

# import rclpy #ROS2
import rospy   #ROS1

from geometry_msgs.msg import PointStamped #common

x = 0.0 #common
y = 0.0 #common 
z = 0.0 #common 

def callback(msg):    #common
    global x, y, z    #common
        
    point = PointStamped()  #common
    point.header.stamp = msg.header.stamp #common
    point.header.frame_id = "/map" #common
    point.point.x = msg.point.x    #common
    point.point.y = msg.point.y    #common
    point.point.z = msg.point.z    #common
    x = point.point.x  #common
    y = point.point.y  #common
    z = point.point.z  #common
    print("coordinates: x=%f y=%f z=%f" % (x, y, z))  #common
    
def listener():    #common
    # rclpy.init()   #ROS2 
    # node = rclpy.create_node("robot_listener") #ROS2 
    rospy.init_node("robot_listener")  #ROS1
    
    # subscriber = node.create_subscription(PointStamped, '/clicked_point', callback, 10)  #ROS2 
    rospy.Subscriber('/clicked_point', PointStamped, callback)  #ROS1
    
    # rclpy.spin(node)  #ROS2 
    rospy.spin()  #ROS1
    
if __name__ == '__main__': #common
    # listener()           #ROS2 
    try:                                   #ROS1
        listener()                         #ROS1
    except rospy.ROSInterruptException:    #ROS1
        pass                               #ROS1
    
            
    