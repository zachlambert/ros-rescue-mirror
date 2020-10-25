#include "kinematics_handler.h"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <stdlib.h>
#include <math.h>
#include <string>


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
        kinematic_state->getFrameTransform("arm_base_1").translation());

    Eigen::Isometry3d gripper_pose_absolute =
        arm_base_translation * gripper_pose_relative;

    return gripper_pose_absolute;
}

void KinematicsHandler::gripper_pose_to_coords(
    const Eigen::Isometry3d &gripper_pose_absolute,
    std::vector<double> &gripper_coords)
{
    Eigen::Translation3d arm_base_translation(
        kinematic_state->getFrameTransform("arm_base_1").translation());
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


KinematicsHandler::KinematicsHandler(ros::NodeHandle& n):
        robot_model_loader("robot_description"),
        kinematic_model(robot_model_loader.getModel()),
        kinematic_state(new robot_state::RobotState(kinematic_model)),
        joint_model_group(kinematic_model->getJointModelGroup("arm")),
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
    g_planning_scene = new planning_scene::PlanningScene(kinematic_model);

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

    // Setup publisher and message for the arm angles
    arm_msg.layout.data_offset = 0;
    arm_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
    arm_msg.layout.dim[0].size = 6;
    arm_msg.layout.dim[0].stride = 1;
    arm_msg.layout.dim[0].label = "joint";
    arm_msg.data.resize(6);
    arm_pub = n.advertise<std_msgs::Float64MultiArray>(
        "arm_position_controller/command",
        1000
    );

    // May need to use the below to do IK near the base position.
    // Alternatively, may choose to use moveit to return to home.
    effective_arm_length =
        kinematic_model->getLinkModel("arm_upper_1")
        ->getJointOriginTransform().translation().norm();
}

void KinematicsHandler::initialise_arm_state()
{
    // Won't update until it receives the first joint_state message
    for (std::size_t i = 0; i < joint_state_actual.name.size(); i++) {
        kinematic_state->setJointPositions(
            joint_state_actual.name[i], &joint_state_actual.position[i]
        );
    }

    // Tidy this up at some point, there should be a way to just access
    // the joint angles for the arm group from the kinematic state
    static const std::size_t arm_joint_indexes[] = {0, 1, 2, 7, 8, 9};
    for (std::size_t i = 0; i < 6; i++) {
        arm_state_buffer.next().joint_angles[i] =
            joint_state_actual.position[arm_joint_indexes[i]];
    }

    // Get the relative gripper pose
    Eigen::Isometry3d gripper_pose_absolute =
        kinematic_state->getFrameTransform("gripper_1");

    // Clear arm state buffer
    arm_state_buffer.reset();

    // Convert this gripper pose to euler coordinates
    gripper_pose_to_coords(
        gripper_pose_absolute,
        arm_state_buffer.next().gripper_coords
    );
    arm_state_buffer.next().fail_count = 0;

    arm_state_buffer.increment();
}

void KinematicsHandler::joint_state_callback(
    sensor_msgs::JointState joint_state_msg)
{
    joint_state_actual = joint_state_msg;
    if (!joints_updated) {
        initialise_arm_state();
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

    // Don't update until joints have been updated with actual values
    if (validate_velocity(dt)) {
        update_position(dt);
    } else {
        ROS_INFO("Velocity failed");
        velocity_failed();
    }
}

bool KinematicsHandler::validate_velocity(double dt)
{
    // Compute the gripper coordinates to test, should be further than the
    // actual next coordinates, to avoid getting too close to a collision state
    for (std::size_t i = 0; i < 6; i++) {
        arm_state_buffer.trial_gripper_coords[i] =
            arm_state_buffer.current().gripper_coords[i]
            + dt * gripper_velocity[i]
                 * arm_state_buffer.TRIAL_STEP_MULTIPLIER;
    }

    Eigen::Isometry3d gripper_pose = gripper_coords_to_pose(
        arm_state_buffer.trial_gripper_coords);

    return validate_state(gripper_pose);
}

void KinematicsHandler::update_position(double dt)
{
    // The above state was calculated using the trial coordinates
    // Now, need to calculate and verify the actual next state
    // If the trial state is valid, the next state is almost always
    // going to be valid, but should still check in the rare case that
    // it isn't

    std::vector<double> &current_gripper_coords =
        arm_state_buffer.current().gripper_coords;
    std::vector<double> &next_gripper_coords =
        arm_state_buffer.next().gripper_coords;
    for (std::size_t i = 0; i < 6; i++) {
        next_gripper_coords[i] = (current_gripper_coords[i]
            + dt * gripper_velocity[i]);
        // Normalise to [-pi, pi]
        while (next_gripper_coords[i] < -M_PI) next_gripper_coords[i] += 2*M_PI;
        while (next_gripper_coords[i] > M_PI) next_gripper_coords[i] -= 2*M_PI;
    }

    Eigen::Isometry3d gripper_pose = gripper_coords_to_pose(next_gripper_coords);

    // If the velocity is valid, it is very likely that the next valid too,
    // but should still check
    if (!validate_state(gripper_pose)) {
        return;
    }

    kinematic_state->copyJointGroupPositions(
        joint_model_group, arm_msg.data);
    arm_pub.publish(arm_msg);

    copy(
        arm_msg.data.cbegin(), arm_msg.data.cend(),
        arm_state_buffer.next().joint_angles.begin()
    );

    arm_state_buffer.current().fail_count = 0;
    arm_state_buffer.increment();
}

void KinematicsHandler::velocity_failed()
{
    arm_state_buffer.current().fail_count++;

    if (arm_state_buffer.current().fail_count > arm_state_buffer.fail_threshold) {
        // Although the current state is valid, for some reason, it is stuck and
        // no nearby states are valid. Therefore, move back to a known valid state.
        ROS_INFO("Current state exceeded fail count");

        arm_state_buffer.decrement();

        // If there are no valid states left then it is stuck.
        // Shouldn't get here, but need to make sure.
        if (arm_state_buffer.empty()) {
            arm_state_buffer.current().fail_count = 0;
            joints_updated = false;
            return;
        }

        // Load the previous state
        copy(
            arm_state_buffer.current().joint_angles.cbegin(),
            arm_state_buffer.current().joint_angles.cend(),
            arm_msg.data.begin()
        );
        arm_pub.publish(arm_msg);
    }
}

bool KinematicsHandler::ik_constraint(
    robot_state::RobotState *robot_state,
    const robot_state::JointModelGroup *joint_group,
    const double *joint_angles)
{
    static const double MAX_ANGLE_CHANGE = M_PI/4;
    double angle_dif;
    for (std::size_t i = 0; i < 6; i++) {
        angle_dif = fabs(joint_angles[i] - arm_state_buffer.current().joint_angles[i]);
        if (angle_dif > MAX_ANGLE_CHANGE && angle_dif < 2*M_PI - MAX_ANGLE_CHANGE) {
            return false;
        }
    }
    return true;
}

bool KinematicsHandler::validate_state(const Eigen::Isometry3d &gripper_pose)
{
    static const float IK_TIMEOUT = 0.2;

    static const moveit::core::GroupStateValidityCallbackFn constraint =
        boost::bind(&KinematicsHandler::ik_constraint, this, _1, _2, _3);

    bool found_ik = kinematic_state->setFromIK(
        joint_model_group, gripper_pose, IK_TIMEOUT, constraint
    );
    if (!found_ik) {
        ROS_INFO("IK solution not found");
        return false;
    }
    if(!kinematic_state->satisfiesBounds()){
        ROS_INFO("IK solution found but exceeds joint limits");
        return false;
    }

    g_planning_scene->checkCollision(c_req, c_res, *kinematic_state);
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
        arm_state_buffer.reset();
        joints_updated = false;
        return true;
    }
    return false;
}
