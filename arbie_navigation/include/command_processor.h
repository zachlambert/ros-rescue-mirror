#ifndef RESCUE_CONTROL_CONTROL_ARM_PUBLISHER_H
#define RESCUE_CONTROL_CONTROL_ARM_PUBLISHER_H

#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/Joy.h"
#include "std_msgs/Bool.h"



class CommandPublisher {
public:
    CommandPublisher(ros::NodeHandle &n);
    void set_cursor_pose(double x, double y, double orientation);
    void cursor_publish();
    void goal_publish();

private:

    //Cursor
    ros::Publisher cursor_pose_pub;
    ros::Publisher goal_pose_pub;
    geometry_msgs::PoseStamped cursor_msg;
};


#endif
