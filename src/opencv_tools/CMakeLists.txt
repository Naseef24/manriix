cmake_minimum_required(VERSION 3.0.2)
project(opencv_tools)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  geometry_msgs
  cv_bridge
  tf
)

# Find OpenCV
find_package(OpenCV REQUIRED)

# Declare the catkin package
catkin_package(
  CATKIN_DEPENDS
    rospy
    sensor_msgs
    geometry_msgs
    cv_bridge
    tf
  DEPENDS OpenCV
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# Install Python scripts
catkin_install_python(PROGRAMS
  opencv_tools/basic_image_publisher.py
  opencv_tools/basic_image_subscriber.py
  opencv_tools/aruco_marker_pose_estimation_tf.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install configuration files
install(
  FILES
  opencv_tools/calibration_chessboard.yaml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/opencv_tools
)

# Install test scripts if they exist
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/test)
  catkin_install_python(PROGRAMS
    test/test_copyright.py
    test/test_flake8.py
    test/test_pep257.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/test
  )
endif()