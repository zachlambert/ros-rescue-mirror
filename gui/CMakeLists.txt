cmake_minimum_required(VERSION 2.8.3)

project(gui)
# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
  rqt_gui
  rqt_gui_cpp

)

catkin_python_setup()

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(PROGRAMS
  scripts/gui
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
