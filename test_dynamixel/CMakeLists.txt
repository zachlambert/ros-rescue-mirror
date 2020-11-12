cmake_minimum_required(VERSION 3.0.2)
project(test_dynamixel)

find_package(catkin REQUIRED COMPONENTS
    dynamixel_workbench_controllers
    roscpp
    rospy
    std_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)
