#ifndef RESCUE_CONTROL_CONTROL_JOINT_HANDLE_GROUP_H
#define RESCUE_CONTROL_CONTROL_JOINT_HANDLE_GROUP_H

#include <ros/ros.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <arbie_msgs/ReadHardware.h>
#include <arbie_msgs/WriteHardware.h>

class JointHandleGroup {
public:
    JointHandleGroup(
        ros::NodeHandle &n,
        std::vector<std::string> joint_names,
        std::string component_name);

    void register_with_interface(
        hardware_interface::JointStateInterface &state_interface,
        hardware_interface::PositionJointInterface &pos_interface);
    void register_with_interface(
        hardware_interface::JointStateInterface &state_interface,
        hardware_interface::VelocityJointInterface &vel_interface);

    void read();
    void write();
private:
    std::size_t N;
    std::vector<std::string> joint_names;
    double *pos;
    double *vel;
    double *eff;
    double *cmd;

    ros::ServiceClient read_client;
    arbie_msgs::ReadHardware read_msg;
    ros::ServiceClient write_client;
    arbie_msgs::WriteHardware write_msg;
};

#endif
