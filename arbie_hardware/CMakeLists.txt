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

# 1. Compile the dynamixel drivers into a static library to link to the
# ros_control handles.
add_library(dxl STATIC
    src/dxl/dxl.cpp
    src/dxl/xl430.cpp
    src/dxl/ax12a.cpp
)
target_link_libraries(dxl ${catkin_LIBRARIES})

# 2. Compile the ros_control handles into a static library to link to the nodes
add_library(handles STATIC
    src/handle/handle.cpp
    src/handle/dxl.cpp
)
target_link_libraries(handles dxl ${catkin_LIBRARIES})

# Requires some custom messages in arbie_msgs. Add this to make sure these
# are generated before trying to compile hardware.
add_dependencies(handles
    arbie_msgs_generate_messages_cpp
    arbie_msgs_generate_messages
)

# 3. Compile the main hardware nodes
add_executable(hardware src/hardware/main.cpp)
target_link_libraries(hardware handles ${catkin_LIBRARIES})

# 4. Compile some test nodes. These either test the dynamixel drivers individually
# or test a handle.

add_executable(test_ax12a src/test/ax12a.cpp)
target_link_libraries(test_ax12a dxl ${catkin_LIBRARIES})

add_executable(test_ax12a_pair src/test/ax12a_pair.cpp)
target_link_libraries(test_ax12a_pair handles ${catkin_LIBRARIES})

install(PROGRAMS
  nodes/odrives
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
