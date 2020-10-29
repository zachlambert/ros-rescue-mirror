cmake_minimum_required(VERSION 3.0.2)
project(arbie_hardware)

find_package(catkin REQUIRED COMPONENTS
    rospy
    roscpp
    std_msgs
    geometry_msgs
    arbie_msgs
    controller_manager
)

catkin_python_setup()

catkin_package(CATKIN_DEPENDS
    rospy
    roscpp
    std_msgs
    geometry_msgs
    arbie_msgs
    controller_manager
)

include_directories(
    ${catkin_INCLUDE_DIRS}
)
add_executable(interface
    src/interface/main.cpp
    src/interface/joint_handle_group.cpp
)
target_link_libraries(interface
    ${catkin_LIBRARIES}
)
target_include_directories(interface PRIVATE
    include/interface
)

install(PROGRAMS
  nodes/arm
  nodes/wrist
  nodes/odrives
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
