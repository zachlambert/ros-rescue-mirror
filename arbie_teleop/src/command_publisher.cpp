#include "command_publisher.h"

std_msgs::Float64MultiArray create_command_msg(std::size_t size)
{
    std_msgs::Float64MultiArray msg;
    msg.layout.data_offset = 0;
    msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
    msg.layout.dim[0].size = size;
    msg.layout.dim[0].stride = 1;
    msg.layout.dim[0].label = "joint";
    msg.data.resize(size);
    for (std::size_t i = 0; i < size; i++) msg.data[i] = 0;
    return msg;
}

CommandPublisher::CommandPublisher(ros::NodeHandle &n)
{
    // Arm command
    gripper_velocity_pub = n.advertise<std_msgs::Float64MultiArray>(
        "gripper_velocity", 1000
    );
    gripper_velocity_msg = create_command_msg(7); // Also includes gripper grasp velocity

    // Tracks command
    tracks_command_pub = n.advertise<geometry_msgs::Twist>(
        "base_controller/cmd_vel", 1000
    );

    // Flippers commands
    flippers_front_command_pub = n.advertise<std_msgs::Float64>(
        "flippers_front_controller/command", 1000
    );
    flippers_rear_command_pub = n.advertise<std_msgs::Float64>(
        "flippers_rear_controller/command", 1000
    );
    // Navigation command
    nav_pub = n.advertise<std_msgs::Bool>(
        "navigation_command", 1000
    );

    // Gripper command service
    manipulation_command_service = n.serviceClient<arbie_msgs::ManipulationCommand>(
        "manipulation_command"
    );

    // Calibration service
    calibrate_service = n.serviceClient<std_srvs::Trigger>(
        "calibrate"
    );
}

void CommandPublisher::set_gripper_velocity(double x, double theta, double z, double yaw, double pitch, double roll, double gripper)
{
    gripper_velocity_msg.data[0] = x;
    gripper_velocity_msg.data[1] = theta;
    gripper_velocity_msg.data[2] = z;
    gripper_velocity_msg.data[3] = yaw;
    gripper_velocity_msg.data[4] = pitch;
    gripper_velocity_msg.data[5] = roll;
    gripper_velocity_msg.data[6] = gripper;
}

void CommandPublisher::set_tracks_command(double linear_velocity, double angular_velocity)
{
    tracks_command_msg.linear.x = linear_velocity;
    tracks_command_msg.angular.z = angular_velocity;
}

void CommandPublisher::set_flippers_command(double flippers_rear, double flippers_front)
{
    flippers_front_msg.data = flippers_front;
    flippers_rear_msg.data = flippers_rear;
}

void CommandPublisher::navigation_command(bool navigation_mode)
{
    nav_pub_msg.data = navigation_mode;
}

void CommandPublisher::publish_all()
{
    gripper_velocity_pub.publish(gripper_velocity_msg);
    tracks_command_pub.publish(tracks_command_msg);
    flippers_front_command_pub.publish(flippers_front_msg);
    flippers_rear_command_pub.publish(flippers_rear_msg);
    nav_pub.publish(nav_pub_msg);
}

bool CommandPublisher::send_manipulation_command(
    const std::string &command, const std::string &argument)
{
    manipulation_command_msg.request.command = command;
    manipulation_command_msg.request.argument = argument;
    if (manipulation_command_service.call(manipulation_command_msg)) {
        ROS_INFO("Gripper command status: %d", manipulation_command_msg.response.success);
        return manipulation_command_msg.response.success;
    }
    ROS_INFO("Gripper command failed");
    return false;
}

bool CommandPublisher::calibrate()
{
    if (calibrate_service.call(calibrate_msg)) {
        ROS_INFO("Calibrate status: %d", calibrate_msg.response.success);
        return calibrate_msg.response.success;
    }
    ROS_INFO("Calibrate command failed");
    return false;
}
