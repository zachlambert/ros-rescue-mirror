cmake_minimum_required(VERSION 3.0.2)
project(arbie_control)

find_package(catkin REQUIRED COMPONENTS
    controller_manager
    geometry_msgs
    joint_state_controller
    robot_state_publisher
    roscpp
    rospy
    std_msgs
    geometry_msgs
    sensor_msgs
    nav_msgs
    message_generation
    moveit_core
    moveit_ros_planning
    moveit_ros_planning_interface
    tf2_ros
    tf2_eigen
    tf2_geometry_msgs
    arbie_hardware
)

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED system filesystem date_time thread)

add_service_files(
    FILES
        GripperCommand.srv
)
generate_messages(
    DEPENDENCIES
        std_msgs
)

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS
        message_runtime
        controller_manager
        geometry_msgs
        joint_state_controller
        robot_state_publisher
        roscpp
        rospy
        std_msgs
        tf2_geometry_msgs
        tf2_eigen
        arbie_hardware
    DEPENDS
        EIGEN3
)

add_executable(control
    src/control/main.cpp
    src/control/joint_handle_group.cpp
)
target_link_libraries(control
    ${catkin_LIBRARIES}
)
target_include_directories(control PRIVATE
    include/control
    ${catkin_INCLUDE_DIRS}
)

add_executable(inverse_kinematics
    src/inverse_kinematics/main.cpp
    src/inverse_kinematics/kinematics_handler.cpp
)
target_link_libraries(inverse_kinematics
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
)
target_include_directories(inverse_kinematics PRIVATE
    include/inverse_kinematics
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIR}
    ${EIGEN3_INCLUDE_DIRS}
)

add_executable(localisation_placeholder
    src/localisation_placeholder/main.cpp
)
target_link_libraries(localisation_placeholder
    ${catkin_LIBRARIES}
)
target_include_directories(localisation_placeholder PRIVATE
    ${catkin_INCLUDE_DIRS}
)
