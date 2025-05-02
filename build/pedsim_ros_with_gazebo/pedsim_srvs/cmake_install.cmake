# Install script for directory: /home/nsf24/inventa_sim/src/pedsim_ros_with_gazebo/pedsim_srvs

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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/pedsim_srvs/srv" TYPE FILE FILES
    "/home/nsf24/inventa_sim/src/pedsim_ros_with_gazebo/pedsim_srvs/srv/SetAgentState.srv"
    "/home/nsf24/inventa_sim/src/pedsim_ros_with_gazebo/pedsim_srvs/srv/GetAgentState.srv"
    "/home/nsf24/inventa_sim/src/pedsim_ros_with_gazebo/pedsim_srvs/srv/SetAllAgentsState.srv"
    "/home/nsf24/inventa_sim/src/pedsim_ros_with_gazebo/pedsim_srvs/srv/GetAllAgentsState.srv"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/pedsim_srvs/cmake" TYPE FILE FILES "/home/nsf24/inventa_sim/build/pedsim_ros_with_gazebo/pedsim_srvs/catkin_generated/installspace/pedsim_srvs-msg-paths.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/nsf24/inventa_sim/devel/include/pedsim_srvs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/nsf24/inventa_sim/devel/share/roseus/ros/pedsim_srvs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/nsf24/inventa_sim/devel/share/common-lisp/ros/pedsim_srvs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/nsf24/inventa_sim/devel/share/gennodejs/ros/pedsim_srvs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python3" -m compileall "/home/nsf24/inventa_sim/devel/lib/python3/dist-packages/pedsim_srvs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3/dist-packages" TYPE DIRECTORY FILES "/home/nsf24/inventa_sim/devel/lib/python3/dist-packages/pedsim_srvs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/nsf24/inventa_sim/build/pedsim_ros_with_gazebo/pedsim_srvs/catkin_generated/installspace/pedsim_srvs.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/pedsim_srvs/cmake" TYPE FILE FILES "/home/nsf24/inventa_sim/build/pedsim_ros_with_gazebo/pedsim_srvs/catkin_generated/installspace/pedsim_srvs-msg-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/pedsim_srvs/cmake" TYPE FILE FILES
    "/home/nsf24/inventa_sim/build/pedsim_ros_with_gazebo/pedsim_srvs/catkin_generated/installspace/pedsim_srvsConfig.cmake"
    "/home/nsf24/inventa_sim/build/pedsim_ros_with_gazebo/pedsim_srvs/catkin_generated/installspace/pedsim_srvsConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/pedsim_srvs" TYPE FILE FILES "/home/nsf24/inventa_sim/src/pedsim_ros_with_gazebo/pedsim_srvs/package.xml")
endif()

