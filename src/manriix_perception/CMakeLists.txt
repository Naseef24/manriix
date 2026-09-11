cmake_minimum_required(VERSION 3.16)
project(manriix_perception)

# ── Build tools ───────────────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# ── C++ dependencies for BT.CPP nodes ────────────────────────────────────────
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)

# ── Message dependencies ──────────────────────────────────────────────────────
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

# ── Generate messages ─────────────────────────────────────────────────────────
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Cluster.msg"
  "msg/ClusterArray.msg"
  "msg/ClusterTracking.msg"
  "msg/FusionDiagnostics.msg"
  "msg/HumansList.msg"
  "msg/HumanTrackingData.msg"
  "msg/MissionStatus.msg"
  "msg/MultiCameraStatus.msg"
  "msg/ObjectsList.msg"
  "msg/ObjectTrackingData.msg"
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
  "action/TakePhoto.action"
  DEPENDENCIES std_msgs geometry_msgs sensor_msgs builtin_interfaces
)

# ── C++ BT node library ───────────────────────────────────────────────────────
# Placeholder target — .cpp files added as C++ migration progresses.
# Uncomment each source file as it is implemented.
# ── C++ BT node library ──────────────────────────────────────────
add_library(manriix_bt_nodes SHARED
  src/bt/evaluate_target.cpp
  src/bt/navigate_multi_poi.cpp
  src/bt/take_photo_action.cpp
  src/bt/updater_nodes.cpp
  src/bt/wait_for_stability.cpp
  src/bt/wait_for_humans.cpp
  src/bt/spin_and_scan.cpp
  src/bt/return_to_last_position.cpp
  src/bt/recovery_idle.cpp
)
target_include_directories(manriix_bt_nodes PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(manriix_bt_nodes
  rclcpp rclcpp_action behaviortree_cpp nav2_msgs
  tf2 tf2_geometry_msgs std_msgs geometry_msgs nav_msgs
)
target_link_libraries(manriix_bt_nodes
  nlohmann_json::nlohmann_json
)
rosidl_target_interfaces(manriix_bt_nodes
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)
install(TARGETS manriix_bt_nodes
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY bt_trees/ DESTINATION share/${PROJECT_NAME}/bt_trees)

# ── C++ mission controller node ─────────────────────────────────
add_executable(mission_controller_cpp
  src/mission_controller_cpp.cpp
)
target_include_directories(mission_controller_cpp PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(mission_controller_cpp
  rclcpp behaviortree_cpp ament_index_cpp
)
target_link_libraries(mission_controller_cpp
  manriix_bt_nodes
  nlohmann_json::nlohmann_json
)
install(TARGETS mission_controller_cpp
  DESTINATION lib/${PROJECT_NAME}
)


# ── Python module installation ────────────────────────────────────────────────
# Uses local/lib/python3.10/dist-packages — correct for Ubuntu 22.04 + Humble.
# colcon's setup.bash adds this path to PYTHONPATH automatically.
#
# DO NOT use ament_python_install_package() here — this package also runs
# rosidl_generate_interfaces which installs msg Python bindings into the same
# manriix_perception/ directory. Both must merge into one location.
install(
  DIRECTORY manriix_perception/
  DESTINATION local/lib/python3.10/dist-packages/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.backup" EXCLUDE
  PATTERN "*.bak" EXCLUDE
  PATTERN "*.back" EXCLUDE
)

# ── Python executables ────────────────────────────────────────────────────────
install(PROGRAMS
  manriix_perception/nodes/direct_zed_detection_node.py
  manriix_perception/nodes/direct_zed_detection_node_modified.py
  manriix_perception/nodes/remote_camera_compressor_node.py
  manriix_perception/nodes/zed_camera_supervisor.py
  manriix_perception/nodes/zed_side_worker_supervisor.py
  manriix_perception/nodes/zed_side_capture_supervisor.py
  manriix_perception/nodes/shared_side_yolo_detector.py
  manriix_perception/nodes/zed_minimal_capture_node.py
  manriix_perception/nodes/zed_side_minimal_manager.py
  manriix_perception/nodes/shared_three_camera_yolo_detector.py
  manriix_perception/nodes/multi_zed_capture_node.py
  manriix_perception/nodes/multi_zed_capture_node_v2.py
  manriix_perception/nodes/zed_fusion_node.py
  manriix_perception/nodes/diagnostics_node.py
  manriix_perception/nodes/zed_zmq_bridge_node.py
  manriix_perception/nodes/photo_capture_bridge_node.py
  manriix_perception/clustering/human_clustering_node.py
  manriix_perception/mission/mission_controller.py
  manriix_perception/mission/poi_manager.py
  manriix_perception/mission/mission_telemetry_recorder.py
  manriix_perception/mission/teleop_toggle.py
  manriix_perception/intelligence/photo_intelligence_system.py
  manriix_perception/recovery/recovery_manager.py
  manriix_perception/navigation/exploration_node.py
  manriix_perception/navigation/human_fusion_node.py
  manriix_perception/navigation/object_costmap_bridge.py
  manriix_perception/web/remote_viz.py
  DESTINATION lib/${PROJECT_NAME}
)

# ── Launch and config ─────────────────────────────────────────────────────────
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)

ament_package()