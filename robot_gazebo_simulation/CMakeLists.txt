cmake_minimum_required(VERSION 3.0.2)
project(catabot_gazebo_simulation)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  std_msgs
  eigen_conversions
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES robot_gazebo_simulation
#  CATKIN_DEPENDS r^Cpy roscpp std_msg uri_soft_base uri_soft_gazebo_differential_drive
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  # ${utility_functions_DIRS}
  # ${eigen_INCLUDE_DIRS}
)