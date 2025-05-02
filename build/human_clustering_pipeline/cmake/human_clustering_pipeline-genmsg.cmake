# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "human_clustering_pipeline: 1 messages, 0 services")

set(MSG_I_FLAGS "-Ihuman_clustering_pipeline:/home/nsf24/inventa_sim/src/human_clustering_pipeline/msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(human_clustering_pipeline_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/nsf24/inventa_sim/src/human_clustering_pipeline/msg/OptimalPosition.msg" NAME_WE)
add_custom_target(_human_clustering_pipeline_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "human_clustering_pipeline" "/home/nsf24/inventa_sim/src/human_clustering_pipeline/msg/OptimalPosition.msg" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(human_clustering_pipeline
  "/home/nsf24/inventa_sim/src/human_clustering_pipeline/msg/OptimalPosition.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/human_clustering_pipeline
)

### Generating Services

### Generating Module File
_generate_module_cpp(human_clustering_pipeline
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/human_clustering_pipeline
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(human_clustering_pipeline_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(human_clustering_pipeline_generate_messages human_clustering_pipeline_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/nsf24/inventa_sim/src/human_clustering_pipeline/msg/OptimalPosition.msg" NAME_WE)
add_dependencies(human_clustering_pipeline_generate_messages_cpp _human_clustering_pipeline_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(human_clustering_pipeline_gencpp)
add_dependencies(human_clustering_pipeline_gencpp human_clustering_pipeline_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS human_clustering_pipeline_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(human_clustering_pipeline
  "/home/nsf24/inventa_sim/src/human_clustering_pipeline/msg/OptimalPosition.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/human_clustering_pipeline
)

### Generating Services

### Generating Module File
_generate_module_eus(human_clustering_pipeline
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/human_clustering_pipeline
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(human_clustering_pipeline_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(human_clustering_pipeline_generate_messages human_clustering_pipeline_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/nsf24/inventa_sim/src/human_clustering_pipeline/msg/OptimalPosition.msg" NAME_WE)
add_dependencies(human_clustering_pipeline_generate_messages_eus _human_clustering_pipeline_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(human_clustering_pipeline_geneus)
add_dependencies(human_clustering_pipeline_geneus human_clustering_pipeline_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS human_clustering_pipeline_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(human_clustering_pipeline
  "/home/nsf24/inventa_sim/src/human_clustering_pipeline/msg/OptimalPosition.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/human_clustering_pipeline
)

### Generating Services

### Generating Module File
_generate_module_lisp(human_clustering_pipeline
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/human_clustering_pipeline
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(human_clustering_pipeline_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(human_clustering_pipeline_generate_messages human_clustering_pipeline_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/nsf24/inventa_sim/src/human_clustering_pipeline/msg/OptimalPosition.msg" NAME_WE)
add_dependencies(human_clustering_pipeline_generate_messages_lisp _human_clustering_pipeline_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(human_clustering_pipeline_genlisp)
add_dependencies(human_clustering_pipeline_genlisp human_clustering_pipeline_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS human_clustering_pipeline_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(human_clustering_pipeline
  "/home/nsf24/inventa_sim/src/human_clustering_pipeline/msg/OptimalPosition.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/human_clustering_pipeline
)

### Generating Services

### Generating Module File
_generate_module_nodejs(human_clustering_pipeline
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/human_clustering_pipeline
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(human_clustering_pipeline_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(human_clustering_pipeline_generate_messages human_clustering_pipeline_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/nsf24/inventa_sim/src/human_clustering_pipeline/msg/OptimalPosition.msg" NAME_WE)
add_dependencies(human_clustering_pipeline_generate_messages_nodejs _human_clustering_pipeline_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(human_clustering_pipeline_gennodejs)
add_dependencies(human_clustering_pipeline_gennodejs human_clustering_pipeline_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS human_clustering_pipeline_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(human_clustering_pipeline
  "/home/nsf24/inventa_sim/src/human_clustering_pipeline/msg/OptimalPosition.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/human_clustering_pipeline
)

### Generating Services

### Generating Module File
_generate_module_py(human_clustering_pipeline
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/human_clustering_pipeline
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(human_clustering_pipeline_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(human_clustering_pipeline_generate_messages human_clustering_pipeline_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/nsf24/inventa_sim/src/human_clustering_pipeline/msg/OptimalPosition.msg" NAME_WE)
add_dependencies(human_clustering_pipeline_generate_messages_py _human_clustering_pipeline_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(human_clustering_pipeline_genpy)
add_dependencies(human_clustering_pipeline_genpy human_clustering_pipeline_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS human_clustering_pipeline_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/human_clustering_pipeline)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/human_clustering_pipeline
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(human_clustering_pipeline_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/human_clustering_pipeline)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/human_clustering_pipeline
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(human_clustering_pipeline_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/human_clustering_pipeline)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/human_clustering_pipeline
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(human_clustering_pipeline_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/human_clustering_pipeline)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/human_clustering_pipeline
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(human_clustering_pipeline_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/human_clustering_pipeline)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/human_clustering_pipeline\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/human_clustering_pipeline
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(human_clustering_pipeline_generate_messages_py std_msgs_generate_messages_py)
endif()
