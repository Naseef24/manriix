# List of human tracking data
std_msgs/Header header
HumanTrackingData[] humans

# Summary statistics
int32 human_count               # Total number of humans detected
int32 stationary_count          # Humans with velocity < 0.5 m/s (for cluster stability)