cmake_minimum_required(VERSION 3.5.1)
project(ros2node)

set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(SOURCES
    ros2node.cpp
    ros2qosprofile.cpp)

set(HEADERS
    ros2node.h
    ros2messages.h
    ros2messagetovariant.h
    ros2subscriptionlistenerinterface.h
    ros2qosprofile.h
    ros2nodefactory.h)

set(INSTALL_HEADERS
    ros2node.h
    ros2nodefactory.h)

add_library(ros2node SHARED ${SOURCES} ${HEADERS})

if(COMM_ROS2_FASTRTPS)
  set(DDS rmw_fastrtps_cpp)
  add_definitions(-DFASTRTPS)
  target_link_libraries(ros2node fastrtps)
endif()
if(COMM_ROS2_OPENSPLICE)
  set(DDS rmw_opensplice_cpp)
  add_definitions(-DOPENSPLICE)
endif()
if(COMM_ROS2_CONNEXT)
  set(DDS rmw_connext_cpp)
  add_definitions(-DCONNEXT)
endif()

find_package(${DDS} REQUIRED)
find_package(ros2eval_msgs REQUIRED)

target_link_libraries(ros2node common rcl rclcpp  ${${DDS}_LIBRARIES} ${ros2eval_msgs_LIBRARIES})
include_directories(${${DDS}_INCLUDE_DIRS} ${rcl_INCLUDE_DIRS})
install(FILES ${INSTALL_HEADERS} DESTINATION /include/ros2node)

