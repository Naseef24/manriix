# Recovery Manager Status
# Comprehensive status of the 8-level recovery escalation system

std_msgs/Header header

# Current recovery state
string current_level            # Current recovery level (normal, wait_and_monitor, sensor_reset, etc.)
string current_issue           # Type of issue being addressed
float32 level_duration        # Time spent in current level (seconds)
float32 total_recovery_time   # Total time in recovery (seconds)

# Escalation information
int32 recovery_attempts       # Number of recovery attempts at current level
int32 max_attempts           # Maximum attempts allowed at current level
int32 total_recovery_sessions # Total recovery sessions since startup
string next_escalation_level # Next level if current fails

# Issue detection
bool data_flow_healthy       # Whether data flow is healthy
float32 time_since_last_data # Time since last valid data (seconds)
bool system_health_ok        # Overall system health status
bool navigation_healthy      # Navigation system health

# System health metrics
float32 cpu_usage           # Current CPU usage (%)
float32 memory_usage        # Current memory usage (%)
float32 disk_usage          # Current disk usage (%)
bool network_connectivity   # Network connectivity status

# Node health status
string[] monitored_nodes     # List of monitored nodes
bool[] node_health_status   # Health status for each monitored node
string[] node_issues        # Issues detected for each node

# Recovery actions taken
string[] recent_actions     # Recent recovery actions taken
builtin_interfaces/Time[] action_timestamps    # Timestamps of recent actions
string[] action_results     # Results of recent actions

# Emergency status
bool emergency_stop_active  # Whether emergency stop is active
bool operator_notification_sent # Whether operators have been notified
string emergency_reason     # Reason for emergency state
builtin_interfaces/Time emergency_start_time   # When emergency started

# Performance tracking
int32 successful_recoveries # Number of successful recovery operations
int32 failed_recoveries    # Number of failed recovery operations
float32 average_recovery_time # Average time to complete recovery
float32 system_uptime      # Total system uptime (seconds)