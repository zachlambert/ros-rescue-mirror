#!/bin/sh
xacro ../arbie_sim.urdf.xacro > temp.urdf
gz sdf -p temp.urdf > arbie_sim.sdf
rm temp.urdf
