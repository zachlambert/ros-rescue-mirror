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
add_executable(hardware src/hardware/hardware.cpp)
target_link_libraries(hardware handles ${catkin_LIBRARIES})

add_executable(hardware_fake src/hardware/hardware_fake.cpp)
target_link_libraries(hardware_fake handles ${catkin_LIBRARIES})

# 4. Compile some test nodes. These either test the dynamixel drivers individually
# or test a handle.

add_executable(test_simple_xl430 src/test/simple_xl430.cpp)
target_link_libraries(test_simple_xl430 ${catkin_LIBRARIES})

add_executable(benchmark_ax12a src/test/benchmark_ax12a.cpp)
target_link_libraries(benchmark_ax12a dxl ${catkin_LIBRARIES})

add_executable(benchmark_xl430 src/test/benchmark_xl430.cpp)
target_link_libraries(benchmark_xl430 dxl ${catkin_LIBRARIES})

add_executable(test_xl430 src/test/xl430.cpp)
target_link_libraries(test_xl430 dxl ${catkin_LIBRARIES})

add_executable(test_ax12a src/test/ax12a.cpp)
target_link_libraries(test_ax12a dxl ${catkin_LIBRARIES})

add_executable(test_arm_3 src/test/arm_3.cpp)
target_link_libraries(test_arm_3 handles ${catkin_LIBRARIES})

add_executable(test_pitch src/test/pitch.cpp)
target_link_libraries(test_pitch handles ${catkin_LIBRARIES})

# 5. Install python nodes

install(PROGRAMS
  nodes/odrives
  nodes/spawner
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
