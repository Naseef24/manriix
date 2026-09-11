cmake_minimum_required(VERSION 3.8)
project(manriix_description)

find_package(ament_cmake REQUIRED)

# ============================================================
# BEGIN ADDITION: C++ node — oak_ground_remover
# ============================================================
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters segmentation sample_consensus)

add_executable(oak_ground_remover_node
  src/oak_ground_remover_node.cpp
)

ament_target_dependencies(oak_ground_remover_node
  rclcpp
  sensor_msgs
  pcl_conversions
)

target_include_directories(oak_ground_remover_node PRIVATE
  ${PCL_INCLUDE_DIRS}
)

target_link_libraries(oak_ground_remover_node
  ${PCL_LIBRARIES}
)

install(TARGETS oak_ground_remover_node
  DESTINATION lib/${PROJECT_NAME}
)
# ============================================================
# END ADDITION
# ============================================================

# Install directories
install(DIRECTORY
  urdf
  meshes
  launch
  rviz2
  config
  DESTINATION share/${PROJECT_NAME}
)

# Install Python executables (qos_bridge.py)
install(PROGRAMS
  scripts/qos_bridge.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()