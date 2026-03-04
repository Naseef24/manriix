cmake_minimum_required(VERSION 3.0.2)
project(inventa_sonar)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rosserial_arduino
  rosserial_client
  std_msgs
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    rosserial_arduino
    rosserial_client
    std_msgs
    sensor_msgs
)

# Generate ros_lib for Arduino
rosserial_generate_ros_lib(
  PACKAGE rosserial_arduino
  SCRIPT make_libraries.py
)

# Generate the firmware
rosserial_configure_client(
  DIRECTORY firmware
  TOOLCHAIN_FILE ${ROSSERIAL_ARDUINO_TOOLCHAIN}
)

rosserial_add_client_target(
  DIRECTORY firmware
  NAME sonar_controller
  BOARD mega2560
  PORT /dev/ttyACM0
)

rosserial_add_client_target(
  DIRECTORY firmware
  NAME sonar_controller-upload
  BOARD mega2560
  PORT /dev/ttyACM0
  DEPENDS sonar_controller
)