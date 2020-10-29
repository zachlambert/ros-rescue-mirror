# ARBIE ros packages

ROS packages for the actual robot.  
Packages are named `arbie_<package>`. 

## Running with hardware connected

`roslaunch arbie_main real.launch`

## Running in simulation

First run the gazebo server with: `roslaunch arbie_main gzserver.launch`

Then run the robot code: `roslaunch arbie_main simulation.launch`

To view the simulation, launch the gazebo client with: `rosrun gazebo_ros gzclient`

## Viewing the web gui

Visit `localhost:8000/arbie_webgui`

ROS starts a webserver on localhost at port 8000.  
Visiting `localhost:8000/<ros-package>` accesses the `www` folder for the given ros package.  

# EXP ros packages

TODO

A minimal set of packages for testing the perception and navigation tasks in a basic simulation.  
Packages are named `exp_<package>`.

Eventually these can be integrated into the main code.