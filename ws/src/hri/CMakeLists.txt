## For further reference on `CmakeLists` check the documentation: https://wiki.ros.org/catkin/CMakeLists.txt

cmake_minimum_required(VERSION 3.0.2)
project(hri)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
