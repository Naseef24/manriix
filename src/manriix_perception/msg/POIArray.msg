# Array of Points of Interest from POI Manager
# Contains multiple POI options with detailed scoring

std_msgs/Header header

# POI entries
POIData[] pois

# Summary statistics
int32 total_pois_calculated     # Total number of POIs calculated
float32 average_priority_score  # Average priority across all POIs
float32 average_aesthetic_score # Average aesthetic score
float32 average_accessibility_score # Average accessibility
int32 clusters_analyzed         # Number of clusters analyzed
string formation_types_detected # Comma-separated list of formations detected