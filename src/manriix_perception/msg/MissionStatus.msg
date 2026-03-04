# Advanced Mission Controller Status
# Comprehensive status information for mission execution

std_msgs/Header header

# Current mission state
string current_state              # Current mission state (idle, evaluating, positioning, etc.)
string previous_state            # Previous state for transition tracking
float32 state_duration          # Time spent in current state (seconds)

# Target information
bool has_target                  # Whether there is an active target
geometry_msgs/Point target_position    # Current target position
float32 target_orientation       # Target orientation (radians)
float32 target_priority_score    # Priority score of current target
int32 target_cluster_id         # Associated cluster ID
float32 distance_to_target      # Distance to current target (meters)

# Mission progress
int32 targets_evaluated         # Total targets evaluated this session
int32 photos_completed         # Total photos completed
int32 transitions_made         # Number of state transitions
float32 total_distance_traveled # Total distance traveled (meters)
float32 mission_efficiency     # Overall mission efficiency (0.0-1.0)

# Navigation status
bool navigation_active          # Whether robot is actively navigating
float32 nav_progress           # Navigation progress (0.0-1.0)
string nav_status              # Navigation status string
float32 eta_to_target          # Estimated time to arrival (seconds)

# System integration
bool poi_manager_connected     # POI manager connection status
bool photo_system_connected    # Photo intelligence system connection
bool recovery_system_active   # Recovery system status
string last_recovery_action    # Last recovery action taken

# Performance metrics
float32 average_target_evaluation_time  # Average time to evaluate targets
float32 average_positioning_time       # Average time to reach positions
float32 success_rate                   # Success rate of completed missions