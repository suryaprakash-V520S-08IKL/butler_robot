cmake_minimum_required(VERSION 3.0.2)
project(butler_robot)

find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
  geometry_msgs
  nav_msgs
  rospy
  std_msgs
  message_generation  # Added message generation
)

## Declare messages
add_message_files(
  FILES
  Order.msg  # Add your message file here
)

## Generate added messages and services
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS actionlib_msgs geometry_msgs nav_msgs rospy std_msgs message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

