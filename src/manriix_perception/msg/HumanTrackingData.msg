# Define a message for storing the position (x, y, z)
std_msgs/Header header
geometry_msgs/Point position   # A standard ROS message type for a 3D point (x, y, z)

# Tracking information for one human
int32 tracking_id               # Unique identifier for the human
float32 distance                # Distance to the human
float32 confidence              # Confidence of the tracking data

# Velocity information (for cluster stability analysis)
# High velocity humans can be filtered for more stable optimal positions
geometry_msgs/Vector3 velocity  # 3D velocity vector (m/s) in base_footprint frame
float32 velocity_magnitude      # Speed magnitude (m/s) - sqrt(vx^2 + vy^2 + vz^2)

# Camera source information
string camera_name              # Source camera: 'front', 'left', 'right'