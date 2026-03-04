cmake_minimum_required(VERSION 3.8)
project(manriix_description)

find_package(ament_cmake REQUIRED)

# Install directories
install(DIRECTORY
  urdf
  meshes
  launch
  rviz2
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
