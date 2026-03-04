cmake_minimum_required(VERSION 3.8)
project(manriix_gazebo)

find_package(ament_cmake REQUIRED)

# Install directories
install(DIRECTORY
  launch
  worlds
  config
  urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()