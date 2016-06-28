cmake_minimum_required(VERSION 2.8.3)
project(ros2node)

set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

find_package(${DDS} REQUIRED)
find_package(messages REQUIRED)

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
target_link_libraries(ros2node common rcl rclcpp rclcpp__${DDS} ${${DDS}_LIBRARIES} ${messages_LIBRARIES} fastrtps)
include_directories(${${DDS}_INCLUDE_DIRS} ${rcl_INCLUDE_DIRS})
INSTALL(FILES ${INSTALL_HEADERS} DESTINATION /include/ros2node)

