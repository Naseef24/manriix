# Individual Point of Interest Data
# Detailed information about a single POI

std_msgs/Header header

# Position and orientation
geometry_msgs/Point position     # POI position in map frame
float32 orientation             # Optimal robot orientation (radians)

# Scoring breakdown
float32 priority_score          # Overall priority score (0.0-1.0)
float32 aesthetic_score         # Aesthetic composition score (0.0-1.0)
float32 accessibility_score     # Position accessibility score (0.0-1.0)
float32 stability_score         # Position stability over time (0.0-1.0)
float32 social_distance_score   # Social navigation compliance (0.0-1.0)

# Associated cluster information
int32 cluster_id               # ID of associated cluster
string formation_type          # Formation type (single_person, line, circle, etc.)
int32 group_size              # Number of people in associated group
float32 social_distance       # Distance from humans (meters)

# Composition analysis
float32 rule_of_thirds_score   # Rule of thirds compliance (0.0-1.0)
float32 symmetry_score        # Symmetry in composition (0.0-1.0)
float32 depth_score           # Depth perception in shot (0.0-1.0)
float32 framing_score         # Overall framing quality (0.0-1.0)

# Accessibility details
bool path_clear               # Whether path to position is clear
float32 navigation_difficulty # Navigation difficulty (0.0-1.0)
float32 distance_from_robot   # Distance from current robot position (meters)
string obstacles_detected     # Description of detected obstacles

# Temporal information
float32 stability_duration    # How long position has been stable (seconds)
float32 estimated_photo_duration  # Estimated duration for photo session (seconds) - SET BY FUSION NODE
int32 observation_count      # Number of times this POI has been calculated
builtin_interfaces/Time first_observed    # When this POI was first calculated
builtin_interfaces/Time last_updated      # Last update time