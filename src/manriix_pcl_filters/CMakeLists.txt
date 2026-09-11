cmake_minimum_required(VERSION 3.8)
project(manriix_pcl_filters)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)

add_executable(passthrough_voxel_node src/passthrough_voxel_node.cpp)
add_executable(passthrough_node src/passthrough_node.cpp)
add_executable(passthrough_voxel_ransac_node
  src/passthrough_voxel_ransac_node.cpp)
add_executable(clustering_node src/clustering_node.cpp)
add_executable(static_tf_broadcaster src/static_tf_broadcaster.cpp)
add_executable(temporal_consistency_filter_node
  src/temporal_consistency_filter_node.cpp
)
add_executable(dbscan_obstacle_node
  src/dbscan_obstacle_node.cpp
)


ament_target_dependencies(dbscan_obstacle_node
  rclcpp
  sensor_msgs
  pcl_ros
  pcl_conversions
)

ament_target_dependencies(temporal_consistency_filter_node
  rclcpp
  sensor_msgs
  pcl_ros
  pcl_conversions
)

ament_target_dependencies(passthrough_voxel_node
  rclcpp
  sensor_msgs
  pcl_ros
  pcl_conversions
)

ament_target_dependencies(passthrough_node
  rclcpp
  sensor_msgs
  pcl_ros
  pcl_conversions
)

ament_target_dependencies(passthrough_voxel_ransac_node
  rclcpp
  sensor_msgs
  pcl_ros
  pcl_conversions
)

ament_target_dependencies(clustering_node
  rclcpp
  sensor_msgs
  pcl_ros
  pcl_conversions
)

ament_target_dependencies(static_tf_broadcaster
  rclcpp
  tf2_ros
  geometry_msgs
)

target_link_libraries(passthrough_voxel_node ${PCL_LIBRARIES})
target_link_libraries(passthrough_node ${PCL_LIBRARIES})
target_link_libraries(passthrough_voxel_ransac_node ${PCL_LIBRARIES})
target_link_libraries(clustering_node ${PCL_LIBRARIES})
target_link_libraries(temporal_consistency_filter_node
  ${PCL_LIBRARIES}
)
target_link_libraries(dbscan_obstacle_node
  ${PCL_LIBRARIES}
)


install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS static_tf_broadcaster passthrough_node passthrough_voxel_node passthrough_voxel_ransac_node clustering_node temporal_consistency_filter_node
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS dbscan_obstacle_node
  DESTINATION lib/${PROJECT_NAME})
  
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
