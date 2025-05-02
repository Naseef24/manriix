# Install script for directory: /home/nsf24/inventa_sim/src

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/nsf24/inventa_sim/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  
      if (NOT EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}")
        file(MAKE_DIRECTORY "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}")
      endif()
      if (NOT EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/.catkin")
        file(WRITE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/.catkin" "")
      endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/nsf24/inventa_sim/install/_setup_util.py")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/nsf24/inventa_sim/install" TYPE PROGRAM FILES "/home/nsf24/inventa_sim/build/catkin_generated/installspace/_setup_util.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/nsf24/inventa_sim/install/env.sh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/nsf24/inventa_sim/install" TYPE PROGRAM FILES "/home/nsf24/inventa_sim/build/catkin_generated/installspace/env.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/nsf24/inventa_sim/install/setup.bash;/home/nsf24/inventa_sim/install/local_setup.bash")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/nsf24/inventa_sim/install" TYPE FILE FILES
    "/home/nsf24/inventa_sim/build/catkin_generated/installspace/setup.bash"
    "/home/nsf24/inventa_sim/build/catkin_generated/installspace/local_setup.bash"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/nsf24/inventa_sim/install/setup.sh;/home/nsf24/inventa_sim/install/local_setup.sh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/nsf24/inventa_sim/install" TYPE FILE FILES
    "/home/nsf24/inventa_sim/build/catkin_generated/installspace/setup.sh"
    "/home/nsf24/inventa_sim/build/catkin_generated/installspace/local_setup.sh"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/nsf24/inventa_sim/install/setup.zsh;/home/nsf24/inventa_sim/install/local_setup.zsh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/nsf24/inventa_sim/install" TYPE FILE FILES
    "/home/nsf24/inventa_sim/build/catkin_generated/installspace/setup.zsh"
    "/home/nsf24/inventa_sim/build/catkin_generated/installspace/local_setup.zsh"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/nsf24/inventa_sim/install/setup.fish;/home/nsf24/inventa_sim/install/local_setup.fish")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/nsf24/inventa_sim/install" TYPE FILE FILES
    "/home/nsf24/inventa_sim/build/catkin_generated/installspace/setup.fish"
    "/home/nsf24/inventa_sim/build/catkin_generated/installspace/local_setup.fish"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/nsf24/inventa_sim/install/.rosinstall")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/nsf24/inventa_sim/install" TYPE FILE FILES "/home/nsf24/inventa_sim/build/catkin_generated/installspace/.rosinstall")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/nsf24/inventa_sim/build/gtest/cmake_install.cmake")
  include("/home/nsf24/inventa_sim/build/aws-robomaker-hospital-world/cmake_install.cmake")
  include("/home/nsf24/inventa_sim/build/pedsim_ros_with_gazebo/pedsim_ros/cmake_install.cmake")
  include("/home/nsf24/inventa_sim/build/rtabmap_ros/rtabmap_launch/cmake_install.cmake")
  include("/home/nsf24/inventa_sim/build/rtabmap_ros/rtabmap_ros/cmake_install.cmake")
  include("/home/nsf24/inventa_sim/build/ros-sensor_msgs_ext/cmake_install.cmake")
  include("/home/nsf24/inventa_sim/build/Controllers/four_wheel_steering_msgs/cmake_install.cmake")
  include("/home/nsf24/inventa_sim/build/ar_track_alvar/ar_track_alvar_msgs/cmake_install.cmake")
  include("/home/nsf24/inventa_sim/build/ros-driver_mt3339/cmake_install.cmake")
  include("/home/nsf24/inventa_sim/build/Controllers/lns_controller/cmake_install.cmake")
  include("/home/nsf24/inventa_sim/build/pedsim_ros_with_gazebo/3rdparty/libpedsim/cmake_install.cmake")
  include("/home/nsf24/inventa_sim/build/rtabmap_ros/rtabmap_python/cmake_install.cmake")
  include("/home/nsf24/inventa_sim/build/pedsim_ros_with_gazebo/2ndparty/spencer_messages/spencer_human_attribute_msgs/cmake_install.cmake")
  include("/home/nsf24/inventa_sim/build/pedsim_ros_with_gazebo/2ndparty/spencer_messages/spencer_tracking_msgs/cmake_install.cmake")
  include("/home/nsf24/inventa_sim/build/pid/cmake_install.cmake")
  include("/home/nsf24/inventa_sim/build/human_detection_pipeline/cmake_install.cmake")
  include("/home/nsf24/inventa_sim/build/pedsim_ros_with_gazebo/pedsim_msgs/cmake_install.cmake")
  include("/home/nsf24/inventa_sim/build/pedsim_ros_with_gazebo/pedsim_srvs/cmake_install.cmake")
  include("/home/nsf24/inventa_sim/build/pedsim_ros_with_gazebo/pedsim_utils/cmake_install.cmake")
  include("/home/nsf24/inventa_sim/build/pedsim_ros_with_gazebo/pedsim_visualizer/cmake_install.cmake")
  include("/home/nsf24/inventa_sim/build/ros_imu_bno055/cmake_install.cmake")
  include("/home/nsf24/inventa_sim/build/rtabmap_ros/rtabmap_msgs/cmake_install.cmake")
  include("/home/nsf24/inventa_sim/build/sonar_to_laserscan/cmake_install.cmake")
  include("/home/nsf24/inventa_sim/build/pedsim_ros_with_gazebo/2ndparty/spencer_messages/spencer_social_relation_msgs/cmake_install.cmake")
  include("/home/nsf24/inventa_sim/build/pedsim_ros_with_gazebo/2ndparty/spencer_messages/spencer_vision_msgs/cmake_install.cmake")
  include("/home/nsf24/inventa_sim/build/teleop_twist_joy/cmake_install.cmake")
  include("/home/nsf24/inventa_sim/build/opencv_tools/cmake_install.cmake")
  include("/home/nsf24/inventa_sim/build/ar_track_alvar/ar_track_alvar/cmake_install.cmake")
  include("/home/nsf24/inventa_sim/build/pedsim_ros_with_gazebo/pedsim_gazebo_plugin/cmake_install.cmake")
  include("/home/nsf24/inventa_sim/build/pedsim_ros_with_gazebo/pedsim_sensors/cmake_install.cmake")
  include("/home/nsf24/inventa_sim/build/pedsim_ros_with_gazebo/pedsim_simulator/cmake_install.cmake")
  include("/home/nsf24/inventa_sim/build/realsense_gazebo_plugin/cmake_install.cmake")
  include("/home/nsf24/inventa_sim/build/human_clustering_pipeline/cmake_install.cmake")
  include("/home/nsf24/inventa_sim/build/ira_laser_tools/cmake_install.cmake")
  include("/home/nsf24/inventa_sim/build/rtabmap_ros/rtabmap_conversions/cmake_install.cmake")
  include("/home/nsf24/inventa_sim/build/rtabmap_ros/rtabmap_demos/cmake_install.cmake")
  include("/home/nsf24/inventa_sim/build/rtabmap_ros/rtabmap_examples/cmake_install.cmake")
  include("/home/nsf24/inventa_sim/build/rtabmap_ros/rtabmap_sync/cmake_install.cmake")
  include("/home/nsf24/inventa_sim/build/rtabmap_ros/rtabmap_util/cmake_install.cmake")
  include("/home/nsf24/inventa_sim/build/rtabmap_ros/rtabmap_legacy/cmake_install.cmake")
  include("/home/nsf24/inventa_sim/build/rtabmap_ros/rtabmap_odom/cmake_install.cmake")
  include("/home/nsf24/inventa_sim/build/rtabmap_ros/rtabmap_slam/cmake_install.cmake")
  include("/home/nsf24/inventa_sim/build/rtabmap_ros/rtabmap_viz/cmake_install.cmake")
  include("/home/nsf24/inventa_sim/build/unilidar_sdk/unitree_lidar_ros/src/unitree_lidar_ros/cmake_install.cmake")
  include("/home/nsf24/inventa_sim/build/rtabmap_ros/rtabmap_rviz_plugins/cmake_install.cmake")
  include("/home/nsf24/inventa_sim/build/pedsim_ros_with_gazebo/2ndparty/spencer_tracking_rviz_plugin/cmake_install.cmake")
  include("/home/nsf24/inventa_sim/build/velodyne_gazebo_plugins/cmake_install.cmake")
  include("/home/nsf24/inventa_sim/build/rtabmap_ros/rtabmap_costmap_plugins/cmake_install.cmake")
  include("/home/nsf24/inventa_sim/build/inventa_control/cmake_install.cmake")
  include("/home/nsf24/inventa_sim/build/inventa_description/cmake_install.cmake")
  include("/home/nsf24/inventa_sim/build/inventa_dock/cmake_install.cmake")
  include("/home/nsf24/inventa_sim/build/inventa_gazebo/cmake_install.cmake")
  include("/home/nsf24/inventa_sim/build/inventa_msgs/cmake_install.cmake")
  include("/home/nsf24/inventa_sim/build/inventa_navigation/cmake_install.cmake")
  include("/home/nsf24/inventa_sim/build/inventa_viz/cmake_install.cmake")
  include("/home/nsf24/inventa_sim/build/Controllers/four_wheel_steering_controller/cmake_install.cmake")
  include("/home/nsf24/inventa_sim/build/swerve_controller/cmake_install.cmake")

endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/nsf24/inventa_sim/build/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
