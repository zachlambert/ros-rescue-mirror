To connect to realsense cameras, see:  
https://github.com/IntelRealSense/realsense-ros/issues/1408

Need to copy the following file to /etc/udev/rules.d/  
https://github.com/IntelRealSense/librealsense/blob/master/config/99-realsense-libusb.rules

I think the ROS wrapper for the realsense sdk doesn't do this.
