cmake_minimum_required(VERSION 3.0.2)
project(control_demo)

find_package(catkin REQUIRED COMPONENTS
    geometry_msgs
    roscpp
    rospy
    std_msgs
    tf2_ros
    rescue_control
)
catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS geometry_msgs roscpp rospy std_msgs rescue_control
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)
add_executable(control_demo
    src/main.cpp
    src/command_publisher.cpp
    src/joystick_listener.cpp
)
target_link_libraries(control_demo
    ${catkin_LIBRARIES}
)
add_dependencies(control_demo
    ${catkin_EXPORTED_TARGETS}
)
