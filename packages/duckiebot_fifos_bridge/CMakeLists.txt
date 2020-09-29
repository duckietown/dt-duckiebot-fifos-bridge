cmake_minimum_required(VERSION 2.8.3)
project(duckiebot_fifos_bridge)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
)

catkin_python_setup()

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)
