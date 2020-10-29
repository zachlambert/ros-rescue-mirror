cmake_minimum_required(VERSION 3.0.2)
project(arbie_debug)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    geometry_msgs
    nav_msgs
    tf2_ros
)

catkin_package(CATKIN_DEPENDS
    roscpp
    std_msgs
    geometry_msgs
    nav_msgs
    tf2_ros
)

include_directories(
    ${catkin_INCLUDE_DIRS}
)

add_executable(localisation_placeholder
    src/localisation_placeholder/main.cpp
)
target_link_libraries(localisation_placeholder
    ${catkin_LIBRARIES}
)
target_include_directories(localisation_placeholder PRIVATE
    include/localisation_placeholder
)
