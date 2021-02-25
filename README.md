# Starting the robot

The `arbie_bringup` package contains launch files for running the robot:  
`roslaunch arbie_bringup arbie.launch` - Starts everything  
`roslaunch arbie_arm` - Starts the arm hardware and manipulation  
`roslaunch sim_arm` - Starts Gazebo with the arm simulation  
`roslaunch sim_base` - Starts Gazebo with the base simulation  

This uses a webserver for teleoperation, which can be viewed at:  
`localhost:8000/arbie_webgui`

A more detailed description of these are put on the main software wiki [here](https://gitlab.developers.cam.ac.uk/curobotics/rescue-major/rescue-major-main/-/wikis/MK.III/Software-interface-and-ROS).
