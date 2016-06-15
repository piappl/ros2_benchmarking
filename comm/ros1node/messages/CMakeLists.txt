cmake_minimum_required(VERSION 2.6)
project(messages)

find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs message_generation)

add_message_files(
    FILES
    RobotSensor.msg
    RobotControl.msg
    RobotAlarm.msg
)
generate_messages(
    DEPENDENCIES
    std_msgs
)
catkin_package(
   CATKIN_DEPENDS message_runtime
)
