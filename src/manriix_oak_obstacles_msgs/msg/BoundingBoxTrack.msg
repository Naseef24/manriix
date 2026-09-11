# A tracked 2D person bounding box in the image plane (Sec III-E).
# Coordinates are pixels in the NN input image resolution.

uint32 box_id            # stable across frames (IoU tracking-by-detection)
float32 xmin
float32 ymin
float32 xmax
float32 ymax
float32 confidence       # detector confidence of the latest matched detection
uint32 hit_count         # consecutive matched frames
uint32 missed_frames     # consecutive frames without a matched detection

# Optional spatial (3D) position from the OAK's on-device depth fusion,
# in the camera frame. (0,0,0) if unavailable.
geometry_msgs/Point position