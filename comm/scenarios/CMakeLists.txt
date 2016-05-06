cmake_minimum_required(VERSION 2.8.3)
project(scenarios)

if(COMM_ROS)
  add_executable(ros1Console ros1Console.cpp)
  add_executable(ros1Robot ros1Robot.cpp)
  target_link_libraries(ros1Console ros1node)
  target_link_libraries(ros1Robot ros1node)
endif()

if(COMM_ROS2)
  add_executable(ros2Console ros2Console.cpp)
  add_executable(ros2Robot ros2Robot.cpp)
  target_link_libraries(ros2Console ros2node)
  target_link_libraries(ros2Robot ros2node)
endif()


