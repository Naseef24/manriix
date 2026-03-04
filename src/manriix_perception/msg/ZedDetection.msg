# Single ZED camera detection
std_msgs/Header header

# Detection metadata
int32 tracking_id
string object_class
string object_subclass
int16 class_id
int16 subclass_id
float32 confidence

# 3D position and size
geometry_msgs/Point position
float32[3] dimensions_3d        # width, height, length
float32[3] velocity             # vx, vy, vz

# Source camera
string camera_name
string camera_frame_id

# Detection timestamp
builtin_interfaces/Time detection_time