cmake_minimum_required(VERSION 3.5.1)

project(ros2eval_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

set(msg_files
  "msg/RobotAlarm.msg"
  "msg/RobotControl.msg"
  "msg/RobotSensor.msg"
)
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES std_msgs
)

ament_package()
