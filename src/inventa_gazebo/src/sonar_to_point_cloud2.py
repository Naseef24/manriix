#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Range, PointCloud2, PointField
from sensor_msgs import point_cloud2
from geometry_msgs.msg import Point32
import std_msgs.msg

class SonarToPointCloudConverter:
    def __init__(self):
        rospy.init_node('sonar_to_pointcloud_converter')
        
        # Subscribe to sonar topics
        self.sonar_topics = [f'sonar_{i}' for i in range(1, 7)]  # Assuming 6 sonars
        self.sonar_subs = [rospy.Subscriber(topic, Range, self.sonar_callback, callback_args=i) 
                           for i, topic in enumerate(self.sonar_topics)]
        
        # Publisher for combined point cloud
        self.cloud_pub = rospy.Publisher('sonar_point_cloud', PointCloud2, queue_size=10)
        
        self.sonar_data = [None] * len(self.sonar_topics)
    
    def sonar_callback(self, msg, sonar_index):
        self.sonar_data[sonar_index] = msg
        self.publish_point_cloud()
    
    def publish_point_cloud(self):
        if all(self.sonar_data):
            points = []
            for i, sonar in enumerate(self.sonar_data):
                # Convert sonar reading to a 3D point
                # This is a simplification; you may need to adjust based on your sonar arrangement
                angle = i * (2 * 3.14159 / len(self.sonar_data))
                x = sonar.range * cos(angle)
                y = sonar.range * sin(angle)
                z = 0  # Assuming all sonars are on the same plane
                points.append([x, y, z])
            
            # Create PointCloud2 message
            header = std_msgs.msg.Header()
            header.stamp = rospy.Time.now()
            header.frame_id = 'base_link'  # Adjust as needed
            
            # Create point cloud
            pc2 = point_cloud2.create_cloud_xyz32(header, points)
            
            # Publish point cloud
            self.cloud_pub.publish(pc2)

if __name__ == '__main__':
    converter = SonarToPointCloudConverter()
    rospy.spin()