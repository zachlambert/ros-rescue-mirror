cmake_minimum_required(VERSION 3.0.2)
project(arbie_hardware)

find_package(catkin REQUIRED COMPONENTS
    rospy
    std_msgs
    geometry_msgs
)

catkin_python_setup()

catkin_package(
    CATKIN_DEPENDS rospy std_msgs geometry_msgs
)
include_directories(
    ${catkin_INCLUDE_DIRECTORIES}
)

install(PROGRAMS
  nodes/arm
  nodes/wrist
  nodes/odrives
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
