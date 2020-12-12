# Starting the robot

`roslaunch arbie_bringup bringup.launch`

`arbie_bringup` contains the main launch file and launch files for subsystems.  
For testing individual systems in isolation, the `arbie_test` package has various launch files.

# Testing the robot with hardware

Test basic arm control:  
`roslaunch arbie_test arm.launch`

Todo: Tests for MoveIt, base control, slam.

# Testing the robot in simulation

Must first start `gzserver` with an appropriate world:  
`roslaunch arbie_gazebo navigation.launch` which has checkered obstacles.  
`roslaunch arbie_gazebo manipulation.launch` (currently just empty, but may add objects to pick up).  

To view the simulation:  
`rosrun gazebo_ros gzclient`

Test basic arm control:  
`roslaunch arbie_test sim_arm.launch`

Test base control on its own, or with slam:  
`roslaunch arbie_test sim_base.launch`  
`roslaunch arbie_test sim_base_slam.launch`

Todo: Test for MoveIt.

# Viewing the web gui

The final robot and some tests will start the webgui for monitoring and user input.  
Hosts the `www` folders within ros packages.  

The web gui content is in `arbie_webgui/www`, view at the url:  
`localhost:8000/arbie_webgui`