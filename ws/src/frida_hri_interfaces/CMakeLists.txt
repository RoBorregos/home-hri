cmake_minimum_required(VERSION 3.0.2)
project(frida_hri_interfaces)

find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
  message_generation
  rospy
  actionlib
  std_msgs
  std_srvs
)

add_message_files(
  FILES
  Command.msg
  CommandList.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  Speak.srv
)

## Generate actions in the 'action' folder
add_action_files(
  FILES
  Conversate.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs actionlib_msgs
)

catkin_package(
 CATKIN_DEPENDS actionlib_msgs message_runtime rospy std_msgs actionlib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
