# Cluster of humans
std_msgs/Header header
int32 id
geometry_msgs/Point center
float32 radius
int32 size
string formation
float64 stability
float64 density
float32 average_confidence      # Average confidence of humans in cluster
float32[2] average_velocity     # Average velocity [vx, vy] of cluster movement

