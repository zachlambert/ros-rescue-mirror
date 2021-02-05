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

    // Receive and save actual joint states
    void joint_state_callback(sensor_msgs::JointState joint_state_msg);
    // Receive and save target gripper velocity
    void velocity_callback(std_msgs::Float64MultiArray velocity_msg);
    // Loop for updating position command
    void loop(const ros::TimerEvent &timer);

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

    sensor_msgs::JointState joint_state_actual;
    bool joints_updated;

    std::vector<double> gripper_velocity;
    bool velocity_is_zero;

    std_msgs::Float64MultiArray arm_msg;

    // For handling inverse kinematics near r=0
    static constexpr double R_THRESHOLD = 0.05;
    double effective_arm_length;

    // For moving to a pose target
};

#endif
