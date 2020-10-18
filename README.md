# Rescue-Major-ROS-Packages
This is a ROS repository for the ROS packages we are using on the robot.

Keep it clean and in a good state, the only thing that should be in the root is
folders that each contain their own ROS package. this can then be cloned into
the src of the catkin workspace and catkin make can then be done to make it
accessible via rosrun, roslaunch, etc.

Below is a brief description of the current packages.

## Robot management

**arbie_main:** Contains the main launch files which start the robot, either in real mode, or simulation mode, along with any relevant configuration.

**arbie_msgs:** All custom messages and services go here.

**arbie_debug:** Any other nodes or config which won't run on the finished robot, but are useful during development. eg: Placeholder code, rviz.

## Robot description

**arbie_description:** Provides urdf of robot, which defines the robot model.

**arbie_moveit_config:** Package generated from the moveit_setup_assistant, from the arbie.urdf file. Required by MoveIt for controlling the arm.

## Robot control

**arbie_gamepad:** Receives information from xbox controller and commands the robot accordingly.

**arbie_manipulation:** Moves the robot arm with commanded velocity, or plans and executes a trajectory to a pose goal.

## Robot hardware

**arbie_hardware:** Actual hardware drivers, for the various motors.

**arbie_gazebo:** Simulation of robot hardware.

Only one of the above are run, and have the same interface.

## Robot interface

**arbie_webgui:** Starts a webserver for visualising, monitoring and controlling the robot.

## Robot PCBs / microcontrollers - rosserial nodes

Not included in this repository, as they are not ros nodes, but interact with ROS using rosserial nodes.

**Main PCB:** Outputs information from various sensors such as LiPo cell voltages, and flipper encoders.
