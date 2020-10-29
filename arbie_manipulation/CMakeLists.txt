cmake_minimum_required(VERSION 3.0.2)
project(arbie_manipulation)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    geometry_msgs
    arbie_msgs
    controller_manager
    controller_manager_msgs
    moveit_core
    moveit_ros_planning
    moveit_ros_planning_interface
    tf2_eigen
    eigen_conversions
)

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED system filesystem date_time thread)

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS
        roscpp
        std_msgs
        geometry_msgs
        arbie_msgs
        controller_manager
        controller_manager_msgs
        moveit_core
        moveit_ros_planning
        moveit_ros_planning_interface
        tf2_eigen
        eigen_conversions
    DEPENDS
        EIGEN3
)

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIR}
    ${EIGEN3_INCLUDE_DIRS}
    include
)
add_executable(manipulation
    src/main.cpp
    src/kinematics_handler.cpp
)
target_link_libraries(manipulation
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
)
