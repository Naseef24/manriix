cmake_minimum_required(VERSION 3.16)
project(manriix_learn)

# ── Build tools ──────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# ── Message dependencies ─────────────────────────────────────
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

# ── Generate messages ────────────────────────────────────────
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RobotStatus.msg"
  "srv/ResetCounter.srv"
  "action/CountSession.action"
  DEPENDENCIES std_msgs builtin_interfaces
)

# ── Install Python module ────────────────────────────────────
install(
  DIRECTORY manriix_learn/
  DESTINATION local/lib/python3.10/dist-packages/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
)

# ── Install Python executables ───────────────────────────────
install(PROGRAMS
  manriix_learn/publisher_node.py
  manriix_learn/subscriber_node.py
  manriix_learn/lifecycle_node.py
  manriix_learn/status_node.py
  manriix_learn/action_server_node.py
  manriix_learn/action_client_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# ── Register package ─────────────────────────────────────────
install(FILES package.xml
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY resource/
  DESTINATION share/ament_index/resource_index/packages
  FILES_MATCHING PATTERN "manriix_learn"
)

ament_package()