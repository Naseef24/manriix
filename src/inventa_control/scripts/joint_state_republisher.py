#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

# List the joints you want to republish
joints_to_republish = [
    'rfl_wheel_joint',
    'ffl_wheel_joint',
    'ffr_wheel_joint',
    'rfr_wheel_joint',
    'front_left_steering_joint',
    'front_right_steering_joint',
    'rear_left_steering_joint',
    'rear_right_steering_joint'
]

# Dictionary to hold publishers for each joint
pubs = {}

def callback(data):
    for joint_name in joints_to_republish:
        if joint_name in data.name:
            index = data.name.index(joint_name)
            msg = Float64()
            msg.data = data.position[index]
            pubs[joint_name].publish(msg)

if __name__ == '__main__':
    rospy.init_node('joint_states_republisher', anonymous=True)

    # Initialize publishers for each joint
    for joint in joints_to_republish:
        topic_name = f"/{joint}/state"
        pubs[joint] = rospy.Publisher(topic_name, Float64, queue_size=10)

    rospy.Subscriber('/joint_states', JointState, callback)
    rospy.loginfo("Joint state republisher started.")
    rospy.spin()


# #!/usr/bin/env python3

# import rospy
# from sensor_msgs.msg import JointState
# from std_msgs.msg import Float64

# def callback(data):
#     if 'rfl_wheel_joint' in data.name:
#         index = data.name.index('rfl_wheel_joint')
#         msg = Float64()
#         msg.data = data.position[index]
#         pub.publish(msg)

# rospy.init_node('joint_state_republisher', anonymous=True)
# pub = rospy.Publisher('/rfl_wheel_joint/state', Float64, queue_size=10)
# rospy.Subscriber('/joint_states', JointState, callback)
# rospy.spin()