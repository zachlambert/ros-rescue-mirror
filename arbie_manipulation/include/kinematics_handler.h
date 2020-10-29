#ifndef RESCUE_CONTROL_CONTROL_INVERSE_KINEMATICS_H
#define RESCUE_CONTROL_CONTROL_INVERSE_KINEMATICS_H

#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"
#include "geometry_msgs/Pose.h"
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/collision_detection/collision_tools.h>
#include "tf2_eigen/tf2_eigen.h"
#include <Eigen/Geometry>

struct ArmState {
    std::vector<double> gripper_coords;
    std::vector<double> joint_angles;
    std::size_t fail_count;
    ArmState(): gripper_coords(6),joint_angles(6) {}
};
struct ArmStateBuffer {
    static constexpr std::size_t BUFFER_SIZE = 20;
    std::size_t fail_threshold;

    static constexpr double TRIAL_STEP_MULTIPLIER = 2.0;
    std::vector<double> trial_gripper_coords;

    ArmState buffer[BUFFER_SIZE];
    int head;
    int tail;

    ArmStateBuffer(std::size_t fail_threshold = 10):
        fail_threshold(fail_threshold), trial_gripper_coords(6), head(0),tail(0)
    {}

    bool empty() {
        return head == tail;
    }
    ArmState& current() {
        // Only valid if not empty
        return buffer[(tail-1) % BUFFER_SIZE];
    }
    ArmState& next() {
        return buffer[tail];
    }
    void increment() { 
        tail = (tail+1) % BUFFER_SIZE;
        if (head == tail) {
            head = (head+1) % BUFFER_SIZE;
        }
    }
    void decrement() {
        if (!empty())
            tail = (tail-1) % BUFFER_SIZE;
    }
    void reset() {
        head = tail;
    }
};

class KinematicsHandler{
public:
    KinematicsHandler(ros::NodeHandle& n);

    void joint_state_callback(sensor_msgs::JointState joint_state_msg);
    void velocity_callback(std_msgs::Float64MultiArray velocity_msg);
    void loop(const ros::TimerEvent &timer);
    
    bool gripper_command(const std::string &pose_name);

private:
    void initialise_arm_state();
    void gripper_pose_to_coords(
        const Eigen::Isometry3d &gripper_pose,
        std::vector<double> &gripper_coords);
    Eigen::Isometry3d gripper_coords_to_pose(
        const std::vector<double> &gripper_coords);

    bool ik_constraint(
        robot_state::RobotState *robot_state,
        const robot_state::JointModelGroup *joint_group,
        const double *joint_angles);

    bool validate_velocity(double dt);
    bool validate_state(const Eigen::Isometry3d &gripper_pose);
    void update_position(double dt);
    void velocity_failed();

    robot_model_loader::RobotModelLoader robot_model_loader;
    robot_model::RobotModelPtr kinematic_model;
    robot_state::RobotStatePtr kinematic_state;
    robot_state::JointModelGroup* joint_model_group;

    collision_detection::CollisionRequest c_req;
    collision_detection::CollisionResult c_res;
    planning_scene::PlanningScene* g_planning_scene;

    ArmStateBuffer arm_state_buffer;

    ros::Subscriber joint_state_sub;
    sensor_msgs::JointState joint_state_actual;
    bool joints_updated;

    ros::Subscriber velocity_sub;
    std::vector<double> gripper_velocity;
    bool velocity_is_zero;

    ros::Publisher arm_pub;
    std_msgs::Float64MultiArray arm_msg;

    // For handling inverse kinematics near r=0
    static constexpr double R_THRESHOLD = 0.05;
    double effective_arm_length;

    // For moving to a pose target
    moveit::planning_interface::MoveGroupInterface move_group;
};

#endif
