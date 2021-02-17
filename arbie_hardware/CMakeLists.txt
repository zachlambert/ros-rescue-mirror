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
    include
)

add_executable(hardware src/hardware/main.cpp)
target_link_libraries(hardware ${catkin_LIBRARIES})

add_executable(test_ax12a src/test/ax12a.cpp)
target_link_libraries(test_ax12a ${catkin_LIBRARIES})

add_executable(test_ax12a_pair src/test/ax12a_pair.cpp)
target_link_libraries(test_ax12a_pair ${catkin_LIBRARIES})

add_dependencies(hardware
    arbie_msgs_generate_messages_cpp
    arbie_msgs_generate_messages
)

install(PROGRAMS
  nodes/odrives
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
