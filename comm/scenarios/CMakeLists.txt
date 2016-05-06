cmake_minimum_required(VERSION 2.8.3)
project(scenarios)

if(COMM_ROS)
  add_executable(ros1runner ros1runner.cpp testrunner.cpp configparser.cpp)
  target_link_libraries(ros1runner ros1node)
endif()

if(COMM_ROS2)
  add_executable(ros2runner ros2runner.cpp testrunner.cpp configparser.cpp)
  target_link_libraries(ros2runner ros2node)
endif()


configure_file(rosconsole.ini rosconsole.ini COPYONLY)
configure_file(rosrobot.ini rosrobot.ini COPYONLY)

