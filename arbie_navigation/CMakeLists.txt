cmake_minimum_required(VERSION 3.0.2)
project(arbie_navigation)

find_package(catkin REQUIRED COMPONENTS
    geometry_msgs
    roscpp
    rospy
    std_msgs
    tf2_ros
    arbie_msgs
)
catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS geometry_msgs roscpp rospy std_msgs arbie_msgs
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)
add_executable(navigation
    src/setgoal.cpp
    src/command_publisher.cpp
    src/joystick_listener.cpp
)
target_link_libraries(navigation
    ${catkin_LIBRARIES}
)
add_dependencies(navigation
    ${catkin_EXPORTED_TARGETS}
    arbie_msgs_generate_messages_cpp
    arbie_msgs_generate_messages
)
