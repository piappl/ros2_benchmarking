cmake_minimum_required(VERSION 2.8.3)
project(ros1node)
set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(SOURCES
    ros1node.cpp
    ros1callbacks.cpp
    ros1initializer.cpp
    ros1masterchecker.cpp
    ros1participant.cpp
    ros1publisher.cpp
    ros1subscriber.cpp)

set(HEADERS
    ros1node.h
    ros1nodefactory.h)

add_library(ros1node SHARED ${SOURCES} ${HEADERS})
target_link_libraries(ros1node Qt5::Core common roscpp rosconsole roscpp_serialization)
INSTALL(FILES ${HEADERS} DESTINATION ${CMAKE_BINARY_DIR}/include/ros1node)
