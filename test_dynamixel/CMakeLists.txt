cmake_minimum_required(VERSION 3.0.2)
project(test_dynamixel)

find_package(catkin REQUIRED COMPONENTS
    dynamixel_sdk
    roscpp
    std_msgs
    sensor_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)
add_executable(dynamixel_controller
    src/main.cpp
    src/dynamixel_controller.cpp
)
target_link_libraries(dynamixel_controller
    ${catkin_LIBRARIES}
)
