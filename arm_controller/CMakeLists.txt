cmake_minimum_required(VERSION 3.0.2)
project(arm_controller)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  roscpp
  std_msgs
)

add_service_files(
  FILES
  move.srv
  PickPlace.srv
)

add_message_files(
  FILES
  control.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs message_runtime roscpp std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)