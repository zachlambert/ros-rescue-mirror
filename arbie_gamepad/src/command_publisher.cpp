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
    gripper_velocity_msg = create_command_msg(6);

    // Tracks command
    tracks_command_pub = n.advertise<geometry_msgs::Twist>(
        "tracks_velocity_controller/cmd_vel", 1000
    );

    // Flippers command
    flippers_command_pub = n.advertise<std_msgs::Float64MultiArray>(
        "flippers_velocity_controller/command", 1000
    );
    flippers_command_msg = create_command_msg(2);

    // Gripper command service
    gripper_command_service = n.serviceClient<arbie_msgs::GripperCommand>(
        "gripper_command"
    );
}

void CommandPublisher::set_gripper_velocity(double x, double theta, double z, double yaw, double pitch, double roll)
{
    gripper_velocity_msg.data[0] = x;
    gripper_velocity_msg.data[1] = theta;
    gripper_velocity_msg.data[2] = z;
    gripper_velocity_msg.data[3] = yaw;
    gripper_velocity_msg.data[4] = pitch;
    gripper_velocity_msg.data[5] = roll;
}

void CommandPublisher::set_tracks_command(double linear_velocity, double angular_velocity)
{
    tracks_command_msg.linear.x = linear_velocity;
    tracks_command_msg.angular.z = angular_velocity;
}

void CommandPublisher::set_flippers_command(double flippers_rear, double flippers_front)
{
    flippers_command_msg.data[0] = flippers_rear;
    flippers_command_msg.data[1] = flippers_front;
}

void CommandPublisher::publish_all()
{
    gripper_velocity_pub.publish(gripper_velocity_msg);
    tracks_command_pub.publish(tracks_command_msg);
    flippers_command_pub.publish(flippers_command_msg);
}

bool CommandPublisher::send_gripper_command(std::string pose_name)
{
    gripper_command_msg.request.pose_name = pose_name;
    if (gripper_command_service.call(gripper_command_msg)) {
        ROS_INFO("Gripper command status: %d", gripper_command_msg.response.success);
        return gripper_command_msg.response.success;
    }
    ROS_INFO("Gripper command failed");
    return false;
}
