cmake_minimum_required(VERSION 3.0.2)
project(oculus_reader)

find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
  tf2_ros
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs
)
