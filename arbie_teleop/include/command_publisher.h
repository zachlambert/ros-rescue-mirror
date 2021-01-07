#ifndef RESCUE_CONTROL_CONTROL_ARM_PUBLISHER_H
#define RESCUE_CONTROL_CONTROL_ARM_PUBLISHER_H

#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float64MultiArray.h"
#include "geometry_msgs/Twist.h"
#include "arbie_msgs/GripperCommand.h"

class CommandPublisher {
public:
    CommandPublisher(ros::NodeHandle &n);
    void set_gripper_velocity(
        double x, double theta, double z,
        double yaw, double pitch, double roll);
    void set_tracks_command(double track_left, double track_right);
    void set_flippers_command(double flippers_rear, double flippers_front);
    void set_base_pose(double x, double y, double theta);
    void publish_all();

    bool send_gripper_command(std::string pose_name);
private:
    // Tracks
    ros::Publisher tracks_command_pub;
    geometry_msgs::Twist tracks_command_msg;

    // Flippers
    ros::Publisher flippers_front_command_pub, flippers_rear_command_pub;
    std_msgs::Float64 flippers_front_msg, flippers_rear_msg;

    // Gripper velocity
    ros::Publisher gripper_velocity_pub;
    std_msgs::Float64MultiArray gripper_velocity_msg;

    // Gripper commands (move to named pose)
    ros::ServiceClient gripper_command_service;
    arbie_msgs::GripperCommand gripper_command_msg;
};

#endif
