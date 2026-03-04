cmake_minimum_required(VERSION 3.16)
project(manriix_perception)

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

# Generate messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Cluster.msg"
  "msg/ClusterArray.msg"
  "msg/ClusterTracking.msg"
  "msg/FusionDiagnostics.msg"
  "msg/HumansList.msg"
  "msg/HumanTrackingData.msg"
  "msg/MissionStatus.msg"
  "msg/MultiCameraStatus.msg"
  "msg/OptimalPosition.msg"
  "msg/PerformanceMetrics.msg"
  "msg/PhotoIntelligenceMetrics.msg"
  "msg/POIArray.msg"
  "msg/POIData.msg"
  "msg/RecoveryStatus.msg"
  "msg/RobotStatus.msg"
  "msg/SystemStatus.msg"
  "msg/TransformedHuman.msg"
  "msg/TransformedHumansArray.msg"
  "msg/ZedDetection.msg"
  "msg/ZedDetectionArray.msg"
  DEPENDENCIES std_msgs geometry_msgs sensor_msgs builtin_interfaces
)

# Manual Python module installation (AVOIDING ament_python_install_package)
install(
  DIRECTORY manriix_perception/
  DESTINATION local/lib/python3.10/dist-packages/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
)

# Install Python executables
install(PROGRAMS
  manriix_perception/nodes/direct_zed_detection_node.py
  manriix_perception/nodes/zed_fusion_node.py
  manriix_perception/nodes/diagnostics_node.py
  manriix_perception/nodes/zed_zmq_bridge_node.py
  manriix_perception/clustering/human_clustering_node.py
  manriix_perception/mission/mission_controller.py
  manriix_perception/mission/poi_manager.py
  manriix_perception/mission/mission_telemetry_recorder.py
  manriix_perception/intelligence/photo_intelligence_system.py
  manriix_perception/recovery/recovery_manager.py
  manriix_perception/navigation/exploration_node.py
  manriix_perception/navigation/human_fusion_node.py
  manriix_perception/navigation/object_costmap_bridge.py
  manriix_perception/web/remote_viz.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch/config
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)

ament_package()
