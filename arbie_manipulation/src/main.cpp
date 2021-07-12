#include "ros/ros.h"
#include "kinematics_handler.h"
#include "controller_manager_msgs/SwitchController.h"
#include "arbie_msgs/ManipulationCommand.h"
#include <moveit/move_group_interface/move_group_interface.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>

enum class ControlMode {
    VELOCITY, // Gripper velocity
    MASTER    // Master arm joint angles
};

class Node {
public:
    Node(ros::NodeHandle &n):
        kinematics_handler(n),
        move_group("arm")
    {
        // Initialise subscribers

        joint_states_sub = n.subscribe(
            "joint_states",
            1,
            &Node::joint_states_callback,
            this
        );

        gripper_velocity_sub = n.subscribe(
            "gripper_velocity",
            1,
            &Node::gripper_velocity_callback,
            this
        );

        master_angles_sub = n.subscribe(
            "master_angles",
            1,
            &Node::master_angles_callback,
            this
        );

        // Initialise publishers and their messages
        
        arm_command_pub = n.advertise<std_msgs::Float64MultiArray>(
            "arm_position_controller/command",
            10
        );
        arm_command_msg.layout.data_offset = 0;
        arm_command_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
        arm_command_msg.layout.dim[0].size = 6;
        arm_command_msg.layout.dim[0].stride = 1;
        arm_command_msg.layout.dim[0].label = "joint";
        arm_command_msg.data.resize(6);

        gripper_command_pub = n.advertise<std_msgs::Float64>(
            "gripper_position_controller/command",
            10
        );

        // Initialise service clients for swapping arm controllers

        switch_controllers_client =
            n.serviceClient<controller_manager_msgs::SwitchController>(
                "controller_manager/switch_controller");

        load_position_msg.request.start_controllers.push_back("arm_position_controller");
        load_position_msg.request.start_controllers.push_back("gripper_position_controller");
        load_position_msg.request.stop_controllers.push_back("arm_trajectory_controller");
        load_position_msg.request.stop_controllers.push_back("gripper_trajectory_controller");
        load_position_msg.request.strictness = 2;

        load_trajectory_msg.request.start_controllers.push_back("arm_trajectory_controller");
        load_trajectory_msg.request.start_controllers.push_back("gripper_trajectory_controller");
        load_trajectory_msg.request.stop_controllers.push_back("arm_position_controller");
        load_trajectory_msg.request.stop_controllers.push_back("gripper_position_controller");
        load_trajectory_msg.request.strictness = 2;

        // Initialise service server for receiving commands

        manipulation_command_service = n.advertiseService(
            "manipulation_command", &Node::manipulation_command_callback, this);

        // Initialise kinematics handler and loop

        loop_timer = n.createTimer(
            ros::Duration(1.0/50),
            &Node::loop,
            this
        );

        control_mode = ControlMode::VELOCITY;
    }

    void joint_states_callback(const sensor_msgs::JointState &joint_states)
    {
        kinematics_handler.set_joint_states(joint_states);
    }

    void gripper_velocity_callback(const std_msgs::Float64MultiArray &gripper_velocity)
    {
        if (control_mode == ControlMode::VELOCITY) {
            kinematics_handler.set_gripper_velocity(gripper_velocity);
        }
    }

    void master_angles_callback(const std_msgs::Float64MultiArray &master_angles)
    {
        if (control_mode == ControlMode::MASTER) {
            kinematics_handler.set_master_angles(master_angles);
        }
    }

    void loop(const ros::TimerEvent &timer)
    {
        double dt = ros::Duration(timer.current_real - timer.last_real).toSec();

        if (control_mode == ControlMode::VELOCITY) {
            kinematics_handler.loop_velocity(dt);
        } else { // ControlMode::MASTER
            kinematics_handler.loop_master(dt);
        }

        if (!kinematics_handler.are_joint_positions_valid()) return;

        std_msgs::Float64 gripper_command_msg;
        kinematics_handler.copy_joints_to(arm_command_msg, gripper_command_msg);
        arm_command_pub.publish(arm_command_msg);
        gripper_command_pub.publish(gripper_command_msg);
    }

    bool manipulation_command_callback(
        arbie_msgs::ManipulationCommand::Request &req,
        arbie_msgs::ManipulationCommand::Response &res)
    {
        ROS_INFO("RECEIVED MANIPULATION COMMAND");
        if (req.command == "named_target") {
            move_group.setNamedTarget(req.argument);
            res.success = plan_and_execute_trajectory();
            kinematics_handler.invalidate_joint_positions();
        } else if (req.command == "control_mode") {
            if (req.argument == "velocity") {
                control_mode = ControlMode::VELOCITY;
            } else if (req.argument == "master") {
                control_mode = ControlMode::MASTER;
            }
            res.success = true;
        }
        return true;
    }

private:
    bool plan_and_execute_trajectory() {
        ROS_INFO("Planning and executing trajectory");
        bool success = false;
        moveit::planning_interface::MoveGroupInterface::Plan plan;

        // Assumes move_group has already had its target set
        if (move_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS) {

            // Plan has been loaded into the plan variable.
            // Could manually execute this plan, but instead use move_group.move()
            // This requires trajectory controllers, to swap to these first

            switch_controllers_client.call(load_trajectory_msg);
            if (load_trajectory_msg.response.ok) {
                // Execute trajectory (BLOCKS CURRENT THREAD)
                ROS_INFO("Start moving");
                move_group.move();
                ROS_INFO("Finished moving");
                success = true;
            }

            // Switch back to position controller
            switch_controllers_client.call(load_position_msg);
        }
        return success;
    }

    ControlMode control_mode;

    // Updates joint targets in response to a target gripper velocity
    KinematicsHandler kinematics_handler;

    // Plans and executes trajectories
    moveit::planning_interface::MoveGroupInterface move_group;

    ros::Subscriber joint_states_sub;
    ros::Subscriber gripper_velocity_sub;
    ros::Subscriber master_angles_sub;

    ros::Publisher arm_command_pub; // Command joint positions for arm angles
    std_msgs::Float64MultiArray arm_command_msg;
    ros::Publisher gripper_command_pub;

    // Timer for kinematics_handler loop
    ros::Timer loop_timer;

    // Services for switching between direct joint controllers and
    // action server for executing trajectories
    ros::ServiceClient switch_controllers_client;
    controller_manager_msgs::SwitchController load_position_msg;
    controller_manager_msgs::SwitchController load_trajectory_msg;
    ros::ServiceServer manipulation_command_service;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "manipulation");
    ros::NodeHandle n;
    Node node(n);

    ros::AsyncSpinner spinner(4);
    spinner.start();
    ros::waitForShutdown();
    return 0;
}
