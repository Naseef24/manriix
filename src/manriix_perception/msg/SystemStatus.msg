# Comprehensive System Status
# Aggregated status from all modular photographer robot components

std_msgs/Header header

# Overall system state
string system_state           # Overall system state (operational, degraded, recovery, emergency)
float32 system_uptime        # Total system uptime (seconds)
float32 mission_progress     # Current mission progress (0.0-1.0)
bool all_components_healthy  # Whether all components are healthy

# Component status summary
bool zed_fusion_active       # ZED fusion node status
bool poi_manager_active      # POI manager status
bool mission_controller_active # Mission controller status
bool photo_intelligence_active # Photo intelligence system status
bool recovery_manager_active # Recovery manager status

# Mission statistics
int32 total_photos_taken     # Total photos taken this session
int32 targets_processed     # Total targets processed
float32 average_photo_quality # Average photo quality score
float32 mission_efficiency  # Overall mission efficiency
float32 distance_traveled   # Total distance traveled (meters)

# Current activity
string current_activity      # What the robot is currently doing
geometry_msgs/Point current_position # Current robot position
geometry_msgs/Point target_position  # Current target position (if any)
float32 progress_to_target   # Progress towards current target (0.0-1.0)

# Performance metrics
float32 cpu_usage_overall    # Overall CPU usage (%)
float32 memory_usage_overall # Overall memory usage (%)
int32 active_processes      # Number of active ROS processes
float32 network_latency     # Network latency (milliseconds)

# Error and warning counts
int32 total_errors          # Total errors since startup
int32 total_warnings        # Total warnings since startup
int32 recovery_activations  # Number of times recovery was activated
string[] recent_issues      # Recent issues encountered

# Photo session summary
bool photo_session_active   # Whether a photo session is currently active
float32 current_photo_duration # Current photo session duration
int32 photos_in_current_session # Photos taken in current session
string photo_session_state  # Photo session state

# Environmental data
int32 humans_detected       # Currently detected humans
int32 clusters_detected     # Currently detected clusters
string[] formation_types    # Types of formations currently detected
float32 crowd_density       # Current crowd density (people/m²)

# Diagnostics
string[] component_versions  # Version information for components
builtin_interfaces/Time last_full_diagnostic # Last comprehensive diagnostic
string diagnostic_summary   # Summary of last diagnostic