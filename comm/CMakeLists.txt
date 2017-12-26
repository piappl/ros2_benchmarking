cmake_minimum_required(VERSION 2.8.3)

project(benchmarking)

find_package(Qt5Core REQUIRED)

set(CMAKE_AUTOMOC TRUE)
set(CMAKE_COLOR_MAKEFILE ON)
set(CMAKE_SHARED_LINKER_FLAGS "-Wl,--no-undefined")
include_directories(.)
add_definitions(-std=c++11 -Wall -Wextra)

add_subdirectory(common)
if(COMM_ROS2_FASTRTPS OR COMM_ROS2_OPENSPLICE OR COMM_ROS2_CONNEXT)
  link_directories("/ros2_ws/install/lib")
  add_subdirectory(ros2node)
endif()
if(COMM_ROS)
  include_directories(/opt/ros/lunar/include)
  link_directories(/opt/ros/lunar/lib) #TODO
  add_subdirectory(ros1node/messages)
  add_subdirectory(ros1node)
endif()
if(COMM_OPENSPLICE)
  add_subdirectory(ddsnode)
  include_directories(${OpenSplice_INCLUDE_DIRS})
endif()
add_subdirectory(scenarios)
