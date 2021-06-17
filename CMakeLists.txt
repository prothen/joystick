cmake_minimum_required(VERSION 2.8.3)
project(joystick)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  rospy
  std_msgs
)


catkin_python_setup()

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES joystick
#  CATKIN_DEPENDS geometry_msgs rospy std_msgs
#  DEPENDS system_lib
)
