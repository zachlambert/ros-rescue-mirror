# Starting the robot

The `arbie_bringup` package contains launch files for running the robot:  
`roslaunch arbie_bringup arbie.launch` - Starts everything  
`roslaunch arbie_bringup arm.launch` - Starts the arm hardware and manipulation  
`roslaunch arbie_bringup sim_arm.launch` - Starts Gazebo with the arm simulation  
`roslaunch arbie_bringup sim_base.launch` - Starts Gazebo with the base simulation  

If using the simulated robots, before running the above, must start the appropriate simulation:  
`roslaunch arbie_gazebo sim_arm.launch`  
`roslaunch arbie_gazebo sim_base.launch`  

This uses a webserver for teleoperation, which can be viewed at:  
`localhost:8000/arbie_webgui`

A more detailed description of these are put on the main software wiki [here](https://gitlab.developers.cam.ac.uk/curobotics/rescue-major/rescue-major-main/-/wikis/MK.III/Software-interface-and-ROS).
