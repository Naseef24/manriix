# Fusion algorithm diagnostics
std_msgs/Header header

# Synchronization metrics
float32 time_sync_tolerance
int32 frames_synchronized
int32 frames_dropped
float32 sync_rate_percentage

# Spatial fusion metrics
int32 objects_before_fusion
int32 objects_after_fusion
float32 fusion_efficiency_percentage
float32 average_position_accuracy

# Performance metrics
float32 fusion_latency_ms
float32 processing_frequency_hz
float32 memory_usage_mb