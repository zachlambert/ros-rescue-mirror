cmake_minimum_required(VERSION 3.0.2)
project(test_dynamixel)

find_package(catkin REQUIRED COMPONENTS
    dynamixel_sdk
    roscpp
    std_msgs
    sensor_msgs
    controller_manager
)

catkin_package(CATKIN_DEPENDS
    dynamixel_sdk
    roscpp
    std_msgs
    sensor_msgs
    controller_manager
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

add_executable(dxl_test
    src/dxl_test/main.cpp
)
target_link_libraries(dxl_test
    ${catkin_LIBRARIES}
)

add_executable(hardware
    src/hardware/main.cpp
)
target_link_libraries(hardware
    ${catkin_LIBRARIES}
)
