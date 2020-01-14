cmake_minimum_required(VERSION 2.8.3)
project(tf_urdf)

find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs tf urdf)

include_directories(include ${catkin_INCLUDE_DIRS})