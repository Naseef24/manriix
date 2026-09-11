# Multi-class object tracking data (similar to HumanTrackingData)
int32 tracking_id
int32 class_id              # NEW: COCO class ID (0=person, 2=car, etc.)
string class_name           # NEW: Human-readable class name
geometry_msgs/Point position
float32 confidence
float32 distance
geometry_msgs/Vector3 velocity
float32 velocity_magnitude
string camera_name