cmake_minimum_required(VERSION 3.0.2)
project(test_dynamixel)

find_package(catkin REQUIRED COMPONENTS
    dynamixel_workbench_controllers
    dynamixel_workbench_msgs
    roscpp
    rospy
    std_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)
add_executable(wrist_tilt
    src/wrist_tilt.cpp
)
target_link_libraries(wrist_tilt
    ${catkin_LIBRARIES}
)
