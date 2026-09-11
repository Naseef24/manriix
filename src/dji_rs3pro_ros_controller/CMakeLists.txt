cmake_minimum_required(VERSION 3.8)
project(dji_rs3pro_ros_controller)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

# ── Custom messages and services ──────────────────────────────────────
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/EularAngle.msg"
  "msg/GimbalCmd.msg"
  "msg/GimbalTelemetry.msg"
  "srv/AddTwolnts.srv"
  "srv/AutoCalibrate.srv"
  "srv/CalibrateFocusMotor.srv"
  "srv/CameraControl.srv"
  "srv/SendJointPos.srv"
  "srv/SendJointSpeed.srv"
  "srv/SetFocusPosition.srv"
  "srv/SetGimbalSpeed.srv"
  "srv/SetInt.srv"
  DEPENDENCIES std_msgs sensor_msgs builtin_interfaces
)

# ── Python module — note: requires rebuild after source edits
# Cannot use ament_python_install_package here — rosidl_generate_interfaces
# already registers this package internally, causing duplicate target error.
# Split into _msgs package to resolve this properly in future.
install(
  DIRECTORY ${PROJECT_NAME}/
  DESTINATION local/lib/python3.10/dist-packages/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.py"
  PATTERN "__pycache__" EXCLUDE
  PATTERN "APR17-*" EXCLUDE
)

# ── Executable scripts ────────────────────────────────────────────────
install(PROGRAMS
  scripts/can_bus_controller_node.py
  scripts/control_gimbal_node.py
  scripts/control_focus_position_node.py
  scripts/check_angle_detection_delay_node.py
  scripts/rosbag2dataset_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install scripts as executables
install(
  PROGRAMS
    scripts/can_bus_controller_node.py
    scripts/control_gimbal_node.py
    scripts/control_focus_position_node.py
    scripts/check_angle_detection_delay_node.py
    scripts/rosbag2dataset_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# ── Launch and config files ───────────────────────────────────────────
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
