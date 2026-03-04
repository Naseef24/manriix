# Manriix Perception - Object Detection Integration

Efficient integration of ZED Object Detection with Nav2 costmaps.

## Overview

This package provides a lightweight bridge that converts ZED camera object detections into costmap obstacles for Nav2 navigation. The approach is resource-efficient and requires no custom C++ plugins.

### Resource Usage

| Component | CPU | GPU | Memory |
|-----------|-----|-----|--------|
| ZED Object Detection | ~5% | ~25% | ~500MB |
| Bridge Node | ~2% | 0% | ~30MB |
| **Total Additional** | **~7%** | **~25%** | **~530MB** |

## Architecture

```
┌─────────────────────────────────────────────────────────────────────────┐
│                        PERCEPTION PIPELINE                              │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                         │
│  ZED Camera ───► Object Detection AI ───► ObjectsStamped               │
│  (Hardware)       (On-camera GPU)          (ROS2 Topic)                │
│                                                  │                      │
│                                                  ▼                      │
│                              ┌───────────────────────────────────────┐  │
│                              │   Object to Costmap Bridge           │  │
│                              │   • Filters by confidence/range      │  │
│                              │   • Generates obstacle points        │  │
│                              │   • Class-specific inflation         │  │
│                              └───────────────────────────────────────┘  │
│                                                  │                      │
│                                                  ▼                      │
│                                           PointCloud2                   │
│                                        /object_obstacles                │
│                                                  │                      │
│  RPLidar ────────────────────────────────────────┼──────────────────►  │
│  /scan                                           │                      │
│                                                  ▼                      │
│                              ┌───────────────────────────────────────┐  │
│                              │      Nav2 Costmap Voxel Layer        │  │
│                              │   observation_sources: scan,          │  │
│                              │                        object_obstacles│  │
│                              └───────────────────────────────────────┘  │
│                                                  │                      │
│                                                  ▼                      │
│                                        Navigation Costmap               │
│                                                                         │
└─────────────────────────────────────────────────────────────────────────┘
```

## Installation

### 1. Install ZED ROS2 Wrapper (if not already installed)

```bash
cd ~/manriix2_ws/src
git clone https://github.com/stereolabs/zed-ros2-wrapper.git
cd ..
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select zed_interfaces zed_wrapper
```

### 2. Install Manriix Perception Package

```bash
# Copy package to workspace
cp -r manriix_perception ~/manriix2_ws/src/

# Build
cd ~/manriix2_ws
colcon build --packages-select manriix_perception
source install/setup.bash
```

### 3. Enable ZED Object Detection

Add to your ZED camera config (`common_stereo.yaml`):

```yaml
object_detection:
    od_enabled: true
    model: 'MULTI_CLASS_BOX_MEDIUM'
    confidence_threshold: 50.0
    mc_people: true
    mc_vehicle: true
    mc_animal: true
    object_tracking_enabled: true
```

Or use the provided config file as reference:
```bash
# View the recommended settings
cat ~/manriix2_ws/src/manriix_perception/config/zed_object_detection.yaml
```

### 4. Update Nav2 Parameters

Replace or update your Nav2 params to include `object_obstacles` observation source:

```yaml
# In local_costmap and global_costmap voxel_layer/obstacle_layer:
observation_sources: scan object_obstacles

object_obstacles:
  topic: /object_obstacles
  data_type: "PointCloud2"
  marking: true
  clearing: false
  max_obstacle_height: 2.0
  min_obstacle_height: 0.1
  obstacle_max_range: 8.0
  obstacle_min_range: 0.3
```

A complete Nav2 config is provided:
```bash
# Use the provided config
ros2 launch manriix_navigation slam_nav.launch.py \
    params_file:=$(ros2 pkg prefix manriix_perception)/share/manriix_perception/config/nav2_params_object_detection.yaml
```

## Usage

### Launch Bridge Node

```bash
# Default (front camera)
ros2 launch manriix_perception object_bridge.launch.py

# Custom camera
ros2 launch manriix_perception object_bridge.launch.py camera_name:=zedx_left

# With custom output topic
ros2 launch manriix_perception object_bridge.launch.py output_topic:=/my_obstacles
```

### Verify Object Detection

```bash
# Check ZED is publishing objects
ros2 topic echo /zedx_front/zed_node/obj_det/objects --once

# Check bridge is publishing obstacles
ros2 topic echo /object_obstacles --once

# Check obstacle count
ros2 topic hz /object_obstacles
```

### Visualize in RViz

Add these displays:
- **PointCloud2**: Topic `/object_obstacles` (obstacle points)
- **MarkerArray**: Topic `/object_markers` (visualization boxes)
- **Map**: Topic `/local_costmap/costmap` (see obstacles in costmap)

## Configuration

### Bridge Parameters (`config/object_bridge.yaml`)

| Parameter | Default | Description |
|-----------|---------|-------------|
| `objects_topic` | `/zedx_front/.../objects` | Input detection topic |
| `output_topic` | `/object_obstacles` | Output PointCloud2 topic |
| `min_confidence` | 50.0 | Minimum detection confidence [0-100] |
| `max_range` | 8.0 | Maximum detection range (meters) |
| `min_range` | 0.3 | Minimum detection range (meters) |
| `points_per_meter` | 10.0 | Point density for obstacles |
| `obstacle_height` | 0.5 | Height of generated points |
| `detect_persons` | true | Detect people |
| `detect_vehicles` | true | Detect vehicles |
| `detect_animals` | true | Detect animals |
| `person_radius` | 0.4 | Inflation radius for people |
| `vehicle_radius` | 0.5 | Inflation radius for vehicles |
| `publish_markers` | true | Publish RViz markers |

### Class-Specific Behavior

| Class | Default Radius | Color (RViz) | Notes |
|-------|---------------|--------------|-------|
| Person | 0.4m | Green | Social navigation buffer |
| Vehicle | 0.5m | Red | Safety-critical |
| Animal | 0.3m | Orange | Unpredictable movement |
| Bag | 0.3m | Gray | Disabled by default |
| Electronics | 0.3m | Blue | Disabled by default |
| Sport | 0.3m | Yellow | Disabled by default |

## Multi-Camera Setup

For multiple ZED cameras, run multiple bridge instances:

```bash
# Terminal 1: Front camera
ros2 launch manriix_perception object_bridge.launch.py \
    camera_name:=zedx_front \
    output_topic:=/object_obstacles_front

# Terminal 2: Left camera  
ros2 launch manriix_perception object_bridge.launch.py \
    camera_name:=zedx_left \
    output_topic:=/object_obstacles_left

# Terminal 3: Right camera
ros2 launch manriix_perception object_bridge.launch.py \
    camera_name:=zedx_right \
    output_topic:=/object_obstacles_right
```

Then add all topics to the costmap:
```yaml
observation_sources: scan object_front object_left object_right

object_front:
  topic: /object_obstacles_front
  # ...

object_left:
  topic: /object_obstacles_left
  # ...
```

## Troubleshooting

### No Objects Detected

1. **Check ZED object detection is enabled:**
   ```bash
   ros2 param get /zedx_front/zed_node object_detection.od_enabled
   ```

2. **Verify topic is publishing:**
   ```bash
   ros2 topic echo /zedx_front/zed_node/obj_det/objects --once
   ```

3. **Check confidence threshold** - lower if missing objects:
   ```bash
   ros2 param set /object_to_costmap_bridge min_confidence 30.0
   ```

### Objects Not in Costmap

1. **Check bridge is publishing:**
   ```bash
   ros2 topic hz /object_obstacles
   ```

2. **Verify costmap observation source:**
   ```bash
   ros2 param get /local_costmap/local_costmap voxel_layer.observation_sources
   ```

3. **Check frame transforms:**
   ```bash
   ros2 run tf2_ros tf2_echo base_footprint zedx_front_left_camera_frame
   ```

### High CPU/GPU Usage

1. **Use FAST detection model** in ZED config
2. **Disable unused object classes** (bags, electronics, sport)
3. **Reduce detection range** (max_range: 5.0)
4. **Disable visualization markers** in production

## Performance Optimization

| Setting | Impact | Recommendation |
|---------|--------|----------------|
| Detection model | High | Use FAST for real-time |
| Object classes | Medium | Disable unused classes |
| max_range | Medium | Reduce if possible |
| points_per_meter | Low | Reduce to 5.0 if needed |
| publish_markers | Low | Disable in production |

## Files

```
manriix_perception/
├── CMakeLists.txt
├── package.xml
├── README.md
├── config/
│   ├── object_bridge.yaml          # Bridge node config
│   ├── nav2_params_object_detection.yaml  # Complete Nav2 config
│   └── zed_object_detection.yaml   # ZED camera config reference
├── launch/
│   └── object_bridge.launch.py     # Launch file
└── manriix_perception/
    ├── __init__.py
    └── object_to_costmap_bridge.py # Main bridge node
```

## License

MIT License

## Author

Manriix Development Team
