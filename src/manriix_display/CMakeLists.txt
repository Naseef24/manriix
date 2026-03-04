cmake_minimum_required(VERSION 3.10)
project(manriix_display)

find_package(ament_cmake REQUIRED)

# Install Python executables
install(PROGRAMS
  manriix_display/display_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)

# Install config files  
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)

# Install web assets
install(DIRECTORY manriix_display/web_assets/ DESTINATION share/${PROJECT_NAME}/web_assets)

ament_package()
