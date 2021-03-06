#ifndef RESCUE_CONTROL_CONTROL_INVERSE_KINEMATICS_H
#define RESCUE_CONTROL_CONTROL_INVERSE_KINEMATICS_H

#include <unordered_map>
#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float64MultiArray.h"
#include "geometry_msgs/Pose.h"
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/collision_detection/collision_tools.h>
#include "tf2_eigen/tf2_eigen.h"
#include <Eigen/Geometry>

class KinematicsHandler{
public:
    KinematicsHandler(ros::NodeHandle& n);

    void set_joint_states(const sensor_msgs::JointState &joint_states_msg);
    void set_gripper_velocity(const std_msgs::Float64MultiArray &velocity_msg);
    void set_master_angles(const std_msgs::Float64MultiArray &master_angles);

    void loop_velocity(double dt);
    void loop_master(double dt);

    void invalidate_joint_positions(){ joint_positions_valid = false; }
    bool are_joint_positions_valid(){ return joint_positions_valid; }

    const std::vector<double> &get_joint_positions(){ return joint_positions; }
    void copy_joints_to(std_msgs::Float64MultiArray &arm_command_msg, std_msgs::Float64 &gripper_command_msg);

private:
    // Validate the current state of kinematic_state
    bool validate_state();

    // Objects for loading robot model, accessing robot model, storing robot state
    // and accessing the joints for the arm group
    robot_model_loader::RobotModelLoader robot_model_loader;
    robot_model::RobotModelPtr robot_model;
    robot_state::RobotStatePtr robot_state;
    robot_state::JointModelGroup* arm_model_group;
    robot_state::JointModelGroup* gripper_model_group;

    // Objects for checking collisions
    collision_detection::CollisionRequest c_req;
    collision_detection::CollisionResult c_res;
    planning_scene::PlanningScene* g_planning_scene;

    // Publishers and subscribers
    ros::Subscriber joint_state_sub;
    ros::Subscriber velocity_sub;
    ros::Publisher arm_pub;

    // Joint positions, which are passed to the joint controller commands
    std::vector<double> joint_positions;
    std::vector<double> actual_joint_positions;
    double gripper_joint_position; // Keep this separate
    double actual_gripper_joint_position;
    bool joint_positions_valid;
    std::unordered_map<std::string, std::size_t> arm_joint_index;

    // Angles received from the master arm
    std::vector<double> master_angles;

    // Last received gripper_velocity command
    Eigen::VectorXd gripper_velocity; // 6-dimensional vector = [angular, linear]
    double gripper_joint_velocity; // For the grasping joints
    Eigen::Vector3d ee_reference_point; // Point in end effector frame to find jacobian for
};

#endif
