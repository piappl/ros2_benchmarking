include(DFMacros.cmake)
cmake_minimum_required(VERSION 2.8.3)
project(benchmarking)
set(CMAKE_COLOR_MAKEFILE ON)
set(CMAKE_SHARED_LINKER_FLAGS "-Wl,--no-undefined")

find_package(Qt5Core REQUIRED)
set(CMAKE_AUTOMOC TRUE)

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}
)

use_cxx11()

add_subdirectory(common)
if(COMM_ROS2)
  set(ROS2_PREFIX /ros2_ws/install) #TODO
  include_directories(${ROS2_PREFIX}/include)
  include_directories(${ROS2_PREFIX}/include/rosidl_typesupport_opensplice_cpp/impl)
  link_directories(${ROS2_PREFIX}/lib)
  add_subdirectory(ros2node)
elseif(COMM_ROS)
  include_directories(/opt/ros/jade/include)
  link_directories(/opt/ros/jade/lib) #TODO
  add_subdirectory(ros1node)
endif()
if(COMM_DDS)
  add_subdirectory(ddsnode)
  include_directories(${OpenSplice_INCLUDE_DIRS})
endif()
add_subdirectory(scenarios)
