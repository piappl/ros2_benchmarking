cmake_minimum_required(VERSION 2.8.3)
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
target_link_libraries(ros2node common rcl rclcpp__rmw_opensplice_cpp rmw rmw_opensplice_cpp rosidl_typesupport_opensplice_cpp
                               std_msgs__rosidl_typesupport_opensplice_cpp
                               geometry_msgs__rosidl_typesupport_opensplice_cpp
                               robot_information_msgs__rosidl_typesupport_opensplice_cpp)
INSTALL(FILES ${INSTALL_HEADERS} DESTINATION /include/ros2node)

