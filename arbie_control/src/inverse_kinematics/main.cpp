#include "ros/ros.h"
#include "kinematics_handler.h"
#include "controller_manager_msgs/SwitchController.h"
#include "rescue_control/GripperCommand.h"

class Node {
public:
    Node(ros::NodeHandle &n):
        kinematics_handler(n)
    {
        switch_controllers_client =
            n.serviceClient<controller_manager_msgs::SwitchController>(
                "controller_manager/switch_controller");

        load_position_msg.request.start_controllers.push_back(
            "arm_position_controller");
        load_position_msg.request.stop_controllers.push_back(
            "arm_trajectory_controller");
        load_position_msg.request.strictness = 1;

        load_trajectory_msg.request.start_controllers.push_back(
            "arm_trajectory_controller");
        load_trajectory_msg.request.stop_controllers.push_back(
            "arm_position_controller");
        load_trajectory_msg.request.strictness = 1;

        switch_controllers_client.call(load_position_msg);

        gripper_command_service = n.advertiseService(
            "gripper_command", &Node::gripper_command_callback, this);

        loop_timer = n.createTimer(
            ros::Duration(1.0/20),
            &Node::loop,
            this
        );
    }

    void loop(const ros::TimerEvent &timer)
    {
        kinematics_handler.loop(timer);
    }

    bool gripper_command_callback(
        rescue_control::GripperCommand::Request &req,
        rescue_control::GripperCommand::Response &res)
    {
        switch_controllers_client.call(load_trajectory_msg);
        if (load_trajectory_msg.response.ok) {
            res.success = kinematics_handler.gripper_command(req.pose_name);
        }
        switch_controllers_client.call(load_position_msg);
        return true;
    }
private:
    KinematicsHandler kinematics_handler;
    ros::ServiceClient switch_controllers_client;
    controller_manager_msgs::SwitchController load_position_msg;
    controller_manager_msgs::SwitchController load_trajectory_msg;
    ros::ServiceServer gripper_command_service;
    ros::Timer loop_timer;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "inverse_kinematics");
    ros::NodeHandle n;
    Node node(n);

    ros::AsyncSpinner spinner(4);
    spinner.start();
    ros::waitForShutdown();
    return 0;
}
