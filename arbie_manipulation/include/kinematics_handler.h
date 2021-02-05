#ifndef RESCUE_CONTROL_CONTROL_INVERSE_KINEMATICS_H
#define RESCUE_CONTROL_CONTROL_INVERSE_KINEMATICS_H

#include "ros/ros.h"
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

    const std::vector<double> &get_joint_positions();
    void reset_joint_positions();

private:
    // Validate the current state of kinematic_state
    bool validate_state();

    // Objects for loading robot model, accessing robot model, storing robot state
    // and accessing the joints for the arm group
    robot_model_loader::RobotModelLoader robot_model_loader;
    robot_model::RobotModelPtr robot_model;
    robot_state::RobotStatePtr robot_state;
    robot_state::JointModelGroup* joint_model_group;

    // Objects for checking collisions
    collision_detection::CollisionRequest c_req;
    collision_detection::CollisionResult c_res;
    planning_scene::PlanningScene* g_planning_scene;

    // Publishers and subscribers
    ros::Subscriber joint_state_sub;
    ros::Subscriber velocity_sub;
    ros::Publisher arm_pub;

    // Joint positions, velocities and gripper velocity
    std::vector<double> joint_positions;
    std::vector<double> joint_velocities;
    sensor_msgs::JointState joint_states;
    Eigen::VectorXd gripper_velocity; // 6-dimensional vector = [angular, linear]
};

#endif
