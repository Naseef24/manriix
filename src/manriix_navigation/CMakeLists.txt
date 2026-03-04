cmake_minimum_required(VERSION 3.8)
project(manriix_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(nav2_bringup REQUIRED)
find_package(nav2_common REQUIRED)
find_package(slam_toolbox REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  manriix_navigation/move_bot_xy.py
  manriix_navigation/cliff_detector_node.py
  manriix_navigation/waypoint_navigator.py
  manriix_navigation/map_manager.py
  manriix_navigation/set_initial_pose.py
  manriix_navigation/auto_localize.py
  manriix_navigation/home_manager.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

# Install rviz files
install(DIRECTORY
  rviz
  DESTINATION share/${PROJECT_NAME}
)

# Install maps directory
install(DIRECTORY
  maps
  DESTINATION share/${PROJECT_NAME}
)

# Install behavior trees
install(DIRECTORY
  behavior_trees
  DESTINATION share/${PROJECT_NAME}
)

# Install route graphs (GeoJSON files)
install(DIRECTORY
  graphs
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()