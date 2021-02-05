#include "kinematics_handler.h"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <stdlib.h>
#include <math.h>
#include <string>

/*
void quaternion_to_euler(
        Eigen::Quaterniond &q,
        double *roll, double *pitch, double *yaw)
{
    *roll = atan2(
        2*(q.x()*q.w() + q.y()*q.z()),
        1 - 2*(pow(q.z(), 2) + pow(q.w(), 2)));
    *pitch = asin(2*(q.x()*q.z() - q.w()*q.y()));
    *yaw = atan2(
        2*(q.x()*q.y() + q.z()*q.w()),
        1 - 2*(pow(q.y(), 2) + pow(q.z(), 2)));
}

Eigen::Isometry3d KinematicsHandler::gripper_coords_to_pose(
    const std::vector<double> &gripper_coords)
{
    Eigen::Translation3d translation(
        cos(gripper_coords[1]) * gripper_coords[0],
        sin(gripper_coords[1]) * gripper_coords[0],
        gripper_coords[2]
    );
    Eigen::AngleAxisd roll(gripper_coords[3], Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitch(gripper_coords[4], Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yaw(gripper_coords[1] + gripper_coords[5], Eigen::Vector3d::UnitZ());
    Eigen::Isometry3d gripper_pose_relative = translation * yaw * pitch * roll;
    Eigen::Translation3d arm_base_translation(
        robot_state->getFrameTransform("arm_base_link").translation());

    Eigen::Isometry3d gripper_pose_absolute =
        arm_base_translation * gripper_pose_relative;

    return gripper_pose_absolute;
}

void KinematicsHandler::gripper_pose_to_coords(
    const Eigen::Isometry3d &gripper_pose_absolute,
    std::vector<double> &gripper_coords)
{
    Eigen::Translation3d arm_base_translation(
        robot_state->getFrameTransform("arm_base_link").translation());
    Eigen::Isometry3d gripper_pose_relative =
        arm_base_translation.inverse() * gripper_pose_absolute;

    // Extract the translation and rotation components
    Eigen::Translation3d gripper_translation(gripper_pose_relative.translation());
    Eigen::Quaterniond gripper_quaternion(gripper_pose_relative.rotation());

    double r = hypot(gripper_translation.x(), gripper_translation.y());
    double theta = atan2(gripper_translation.y(), gripper_translation.x());
    gripper_coords[0] = r;
    gripper_coords[1] = theta;
    gripper_coords[2] = gripper_translation.z();

    // Convert the quaternion to euler angles, and store in the last
    // 3 components of the gripper coordinates
    quaternion_to_euler(
        gripper_quaternion,
        &gripper_coords[3], &gripper_coords[4], &gripper_coords[5]
    );
}
*/


KinematicsHandler::KinematicsHandler(ros::NodeHandle& n):
        robot_model_loader("robot_description"),
        robot_model(robot_model_loader.getModel()),
        robot_state(new robot_state::RobotState(robot_model)),
        joint_model_group(robot_model->getJointModelGroup("arm")),
        c_req(),
        c_res(),
        gripper_velocity(6),
        velocity_is_zero(true),
        move_group("arm")
{
    // Setup collision detection
    c_req.group_name = "arm";
    c_req.contacts = true;
    c_req.max_contacts = 100;
    c_req.max_contacts_per_pair = 5;
    c_req.verbose = false;
    g_planning_scene = new planning_scene::PlanningScene(robot_model);

    // Setup subscriber for joint_states - for initialising arm state and
    // checking that the arm keeps up with the command angles
    joints_updated = false;
    joint_state_sub = n.subscribe(
        "joint_states",
        1,
        &KinematicsHandler::joint_state_callback,
        this
    );

    // Setup subscriber for gripper_velocity
    velocity_sub = n.subscribe(
        "gripper_velocity",
        1,
        &KinematicsHandler::velocity_callback,
        this
    );
}

void KinematicsHandler::joint_state_callback(
    sensor_msgs::JointState joint_state_msg)
{
    joint_state_actual = joint_state_msg;

    // TODO: Tidy the block below
    robot_state->setJointPositions(
        "flippers_front_joint", &joint_state_msg.position[3]);
    robot_state->setJointPositions(
        "flippers_rear_joint", &joint_state_msg.position[4]);
    // robot_state->setJointPositions(
    //     "track_left", &joint_state_msg.position[5]);
    // robot_state->setJointPositions(
    //     "track_right", &joint_state_msg.position[6]);

    if (!joints_updated) {
        // TODO: Copy joint_state_msg positions for arm joints
        joints_updated = true;
    }
}

void KinematicsHandler::velocity_callback(
    std_msgs::Float64MultiArray velocity_msg)
{
    copy(
        velocity_msg.data.cbegin(), velocity_msg.data.cend(),
        gripper_velocity.begin()
    );
    for (std::size_t i = 0; i < gripper_velocity.size(); i++) {
        if (gripper_velocity[i] != 0) {
            velocity_is_zero = false;
            return;
        }
    }
    velocity_is_zero = true;
}

void KinematicsHandler::loop(const ros::TimerEvent &timer)
{
    double dt = ros::Duration(timer.current_real - timer.last_real).toSec();
    if (!joints_updated || velocity_is_zero) return;

    // Assume dt is small enough that a single step is accurate
    // ie: theta_next = theta + dt * theta_vel
    // (instead of interpolating)

    // TODO: Make this a private variable, which gets updated each loop and
    // initialised by joint_state_callback when necessary
    std::vector<double> joint_pos(6);
    // TODO: Compare joint_pos with actual joint states. If these lag behind
    // the target joint states too much, adjust targets

    robot_state->setJointGroupPositions(joint_model_group, joint_pos);

    Eigen::VectorXd ee_vel, joint_vel;
    // TODO: Make this a private variable which gets updated with gripper velocity callback
    // TODO: Transform from polar coordinates to cartesian coordinates
    ee_vel << 0, 0, 0, 0, 0, 0;

    Eigen::MatrixXd jacobian;
    Eigen::Vector3d reference_point_position(0, 0, 0);
    robot_state->getJacobian(
        joint_model_group,
        robot_state->getLinkModel(joint_model_group->getLinkModelNames().back()),
        reference_point_position,
        jacobian
    );
    joint_vel = jacobian.inverse() * ee_vel;

    // Check that velocity limits aren't exceeded
    robot_state->setJointGroupVelocities(joint_model_group, joint_vel.data());
    for (auto &joint_model: joint_model_group->getJointModels()) {
        if (!robot_state->satisfiesVelocityBounds(joint_model)) {
            ROS_INFO("Doesn't satisfy velocity bounds.");
            return;
        }
    }

    // trial_joint_pos = joint_pos + dt * joint_vel
    // joint_pos will be updated to these values if the current velocity
    // doesn't cause a collision
    std::vector<double> trial_joint_pos(joint_pos.size());
    for (std::size_t i = 0; i < trial_joint_pos.size(); i++) {
        trial_joint_pos[i] = joint_pos[i] + joint_vel[i] * dt;
    }

    // To check if the current velocity causes a collision, move with this
    // velocity for a given period of time T, then check if the final state
    // is valid.

    std::vector<double> query_joint_pos = trial_joint_pos;

    double t = dt;
    const double T = 0.25;
    while (t < T) {
        robot_state->setJointGroupPositions(joint_model_group, query_joint_pos);
        robot_state->getJacobian(
            joint_model_group,
            robot_state->getLinkModel(joint_model_group->getLinkModelNames().back()),
            reference_point_position,
            jacobian
        );
        joint_vel = jacobian.inverse() * ee_vel;
        for (std::size_t i = 0; i < query_joint_pos.size(); i++) {
            query_joint_pos[i] += joint_vel[i] * dt;
        }
        t += dt;
    }
    robot_state->setJointGroupPositions(joint_model_group, query_joint_pos);
    if (!validate_state()) {
        ROS_INFO("Velocity failed");
        return;
    }

    joint_pos = trial_joint_pos;
    // TODO: Publish joint commands
}

bool KinematicsHandler::validate_state()
{
    // validates the current state of robot_state, so assumes
    // robot_state has already been set with positions and velocities

    if(!robot_state->satisfiesBounds()){
        ROS_INFO("IK solution found but exceeds joint limits");
        return false;
    }

    g_planning_scene->checkCollision(c_req, c_res, *robot_state);
    if(c_res.collision){
        ROS_INFO("IK found but in collision state");
        std::size_t i = 0;
        for (auto it = c_res.contacts.cbegin(); it != c_res.contacts.cend(); it++) {
            ROS_INFO(
                "Contact %lu: %s, %s",
                i, it->first.first.c_str(), it->first.second.c_str()
            );
            i++;
        }
        c_res.clear();
        return false;
    }
    return true;
}

bool KinematicsHandler::gripper_command(const std::string &pose_name)
{
    ROS_INFO("GRIPPER COMMAND: %s", pose_name.c_str());
    move_group.setNamedTarget(pose_name);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (move_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS) {
        move_group.move();
        // Reset state history, and invalidate current state, so wait
        // until the state gets updated with actual angles
        joints_updated = false;
        return true;
    }
    return false;
}
