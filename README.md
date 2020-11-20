# ARBIE ros packages

ROS packages for the actual robot.  
Packages are named `arbie_<package>`. 

## Running with hardware connected

`roslaunch arbie_main real.launch`

## Running in simulation

Start the simulation server: `roslaunch arbie_main gzserver.launch`

Then run the robot code: `roslaunch arbie_main simulation.launch`

To view the simulation, launch the gazebo client with: `rosrun gazebo_ros gzclient`

## Viewing the web gui

Visit `localhost:8000/arbie_webgui`

ROS starts a webserver on localhost at port 8000.  
Visiting `localhost:8000/<ros-package>` accesses the `www` folder for the given ros package.  

# EXP ros packages

A minimal set of packages for testing the perception and navigation tasks in a basic simulation.  
Packages are named `exp_<package>`.

## Start the simulation

Start the simulation server: `roslaunch exp_main gzserver.launch`

(Optionally) Start the simulation client: `rosrun gazebo_ros gzclient`  
This allows you to view the simulation environment, add objects, move things around.

Spawn the robot model: `roslaunch exp_main main.launch`

To control the robot: `roslaunch exp_main control.launch`  
This starts a teleoperation node in the current terminal with the controls specified [here](https://github.com/ros-teleop/teleop_twist_keyboard)

The robot simulation outputs imu and camera data. The camera information can be viewed with:  
`roslaunch exp_main cameras.launch`

## Run SLAM (work in progress)

After the simulation is started:  
`roslaunch exp_main slam.launch`