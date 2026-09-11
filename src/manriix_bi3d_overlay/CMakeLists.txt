cmake_minimum_required(VERSION 3.8)
project(manriix_bi3d_overlay)

find_package(ament_cmake REQUIRED)

# Install our modified bi3d_node.yaml into share/isaac_ros_bi3d/config/
# This intentionally uses the OTHER package's name in the install path,
# so that ament_index_cpp::get_package_share_directory("isaac_ros_bi3d")
# returns OUR path (via AMENT_PREFIX_PATH precedence).
install(FILES config/bi3d_node.yaml
        DESTINATION share/isaac_ros_bi3d/config)

# Register isaac_ros_bi3d in our ament index so get_package_share_directory
# can find it via our prefix. The marker file is empty by convention.
install(CODE "
  file(WRITE \${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/packages/isaac_ros_bi3d \"\")
")

ament_package()
