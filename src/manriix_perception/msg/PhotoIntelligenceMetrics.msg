# Photo Intelligence System Metrics
# Provides detailed analysis of crowd intelligence and photo timing decisions

std_msgs/Header header             # Standard message header

# Group analysis
int32 group_size                    # Number of people in target group
float32 crowd_density              # People per square meter
string formation_type              # Type of formation (line, circle, cluster, etc.)
float32 formation_stability        # How stable the formation is (0.0-1.0)
float32 attention_score           # How engaged/posed the group appears (0.0-1.0)
float32 movement_variance         # Amount of movement in group (0.0-1.0)

# Photo timing decisions
float32 optimal_duration          # Calculated optimal photo duration (seconds)
int32 shot_sequence               # Number of shots planned
float32 base_duration             # Base duration before adjustments
float32 formation_adjustment      # Adjustment for formation type
float32 stability_adjustment      # Adjustment for stability
float32 movement_adjustment       # Adjustment for movement

# Aesthetic analysis
float32 composition_score         # Overall composition quality (0.0-1.0)
float32 distance_score           # Distance optimization score (0.0-1.0)
float32 symmetry_score           # Formation symmetry score (0.0-1.0)
float32 rule_of_thirds_score     # Rule of thirds compliance (0.0-1.0)

# System state
string current_state             # Current photo system state
float32 time_remaining           # Time remaining in current state (seconds)
bool multi_shot_active          # Whether multi-shot sequence is active
int32 shots_completed           # Number of shots completed in sequence

int32 total_people                # Total number of people detected
float32 average_stability         # Average stability across all clusters
float32 average_activity          # Average activity level across clusters
int32 formation_count            # Number of detected formations
float32 overall_photo_readiness  # Overall readiness score for photo
float32 recommended_duration     # Recommended photo duration
