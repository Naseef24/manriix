cmake_minimum_required(VERSION 3.8)
project(manriix_bringup)

# find dependencies
find_package(ament_cmake REQUIRED)

# Install launch files (when we add them later)
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/jk_bms_node.py
  scripts/depth_timestamp_fix_node.py
  scripts/docking_stage_manager.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
