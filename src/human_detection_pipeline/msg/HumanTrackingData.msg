# Define a message for storing the position (x, y, z)
geometry_msgs/Point position   # A standard ROS message type for a 3D point (x, y, z)

# Tracking information for one human
int32 tracking_id               # Unique identifier for the human
float32 distance                # Distance to the human
float32 confidence              # Confidence of the tracking data
