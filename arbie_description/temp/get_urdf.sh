#!/bin/sh
# Used to generate an urdf file for generating the moveit_config package
# The actual robot uses the xacro file
xacro ../model/arbie.urdf.xacro > arbie_temp.urdf
