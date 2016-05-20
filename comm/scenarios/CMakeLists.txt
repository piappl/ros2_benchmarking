cmake_minimum_required(VERSION 2.8.3)
project(scenarios)

set(COMMON_SRC testrunner.cpp configparser.cpp nodestarter.cpp timersenum.h)

if(COMM_ROS)
  add_executable(ros1runner ros1runner.cpp ${COMMON_SRC})
  target_link_libraries(ros1runner ros1node)
endif()

if(COMM_ROS2)
  add_executable(ros2runner ros2runner.cpp ${COMMON_SRC})
  target_link_libraries(ros2runner ros2node)
endif()

if(COMM_DDS)
  include_directories(${CMAKE_BINARY_DIR}/ddsnode)   #TODO
  add_executable(ddsrunner ddsrunner.cpp ${COMMON_SRC})
  target_link_libraries(ddsrunner ddsnode)
endif()

configure_file(rosconsole.ini rosconsole.ini COPYONLY)
configure_file(rosrobot.ini rosrobot.ini COPYONLY)

