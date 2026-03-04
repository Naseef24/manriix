# Multi-camera system status
std_msgs/Header header

# Camera status
string[] camera_names
bool[] camera_active
builtin_interfaces/Time[] last_detection_time
int32[] detection_counts
float32[] average_confidence

# System performance
float32 fusion_rate_hz
float32 output_rate_hz
float32 total_cpu_usage
float32 total_memory_usage_mb

# Error tracking
int32 total_errors
string[] recent_errors