# A single tracked 3D cluster/object.
# Populated incrementally by the pipeline:
#   Step 2 (clustering/tracking):      track_id, centroid, num_points, age, points
#   Step 3 (static/dynamic voting):    classification, dynamic_vote_ratio
#   Step 5 (2D-3D fusion):             classification may be overridden to "person"
#   Step 6 (motion estimation / KF):   velocity, velocity_covariance

uint32 track_id

geometry_msgs/Point centroid
geometry_msgs/Vector3 velocity

# Diagonal of the 2x2 (x,y) velocity covariance from the Kalman filter,
# used later for gated re-association after short occlusions (Sec III-G).
float32 velocity_cov_xx
float32 velocity_cov_yy

uint32 num_points
float32 age               # seconds since this track was first created
uint32 missed_frames      # consecutive frames this track had no matched detection

# "unknown" | "static" | "dynamic" | "uncertain" | "person"
string classification
float32 dynamic_vote_ratio   # fraction of points that voted "dynamic" (Sec III-D)

sensor_msgs/PointCloud2 points