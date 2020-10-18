cmake_minimum_required(VERSION 3.0.2)
project(arbie_hardware)

find_package(catkin REQUIRED COMPONENTS
    rospy
    std_msgs
    message_generation
)

catkin_python_setup()

add_service_files(
    FILES
        ReadHardware.srv
        WriteHardware.srv
)

generate_messages(
    DEPENDENCIES
        std_msgs
)

catkin_package(
    CATKIN_DEPENDS message_runtime geometry_msgs rospy std_msgs
)
include_directories(
    ${catkin_INCLUDE_DIRECTORIES}
)

install(PROGRAMS
  nodes/arm
  nodes/wrist
  nodes/odrives
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
