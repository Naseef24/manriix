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

ament_package()
