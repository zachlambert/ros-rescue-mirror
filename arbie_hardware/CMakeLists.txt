cmake_minimum_required(VERSION 3.0.2)
project(arbie_hardware)

find_package(catkin REQUIRED COMPONENTS
    rospy
    roscpp
    dynamixel_sdk
    std_msgs
    geometry_msgs
    arbie_msgs
    controller_manager
)

catkin_python_setup()

catkin_package(CATKIN_DEPENDS
    rospy
    roscpp
    dynamixel_sdk
    std_msgs
    geometry_msgs
    arbie_msgs
    controller_manager
)

include_directories(
    ${catkin_INCLUDE_DIRS}
)

add_executable(hardware
    src/hardware/main.cpp
)
target_link_libraries(hardware
    ${catkin_LIBRARIES}
)
target_include_directories(hardware PRIVATE
    include
)

add_dependencies(hardware
    arbie_msgs_generate_messages_cpp
    arbie_msgs_generate_messages
)

install(PROGRAMS
  nodes/odrives
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
