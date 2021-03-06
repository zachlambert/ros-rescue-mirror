#include "kinematics_handler.h"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <stdlib.h>
#include <math.h>
#include <string>

void quaternion_to_euler(
        const Eigen::Quaterniond &q,
        double *yaw, double *pitch, double *roll)
{
    *roll = atan2(
        2*(q.x()*q.w() + q.y()*q.z()),
        1 - 2*(pow(q.z(), 2) + pow(q.w(), 2)));
    *pitch = asin(2*(q.x()*q.z() - q.w()*q.y()));
    *yaw = atan2(
        2*(q.x()*q.y() + q.z()*q.w()),
        1 - 2*(pow(q.y(), 2) + pow(q.z(), 2)));
}

void rotation_matrix_to_euler(
        const Eigen::MatrixXd &R,
        double *yaw, double *pitch, double *roll)
{
    // Note: Yaw and roll are invalid if pitch = pi/2 or -pi/2
    *pitch = atan2(-R(2, 0), hypot(R(0, 0), R(1, 0)));
    *yaw = atan2(R(1, 0), R(0, 0));
    *roll = atan2(R(2, 1), R(2, 2));
}

KinematicsHandler::KinematicsHandler(ros::NodeHandle& n):
        robot_model_loader("robot_description"),
        robot_model(robot_model_loader.getModel()),
        robot_state(new robot_state::RobotState(robot_model)),
        arm_model_group(robot_model->getJointModelGroup("arm")),
        gripper_model_group(robot_model->getJointModelGroup("gripper")),
        c_req(),
        c_res()
{
    // Setup collision detection
    c_req.group_name = "arm";
    c_req.contacts = true;
    c_req.max_contacts = 100;
    c_req.max_contacts_per_pair = 5;
    c_req.verbose = false;
    g_planning_scene = new planning_scene::PlanningScene(robot_model);

    gripper_velocity = Eigen::VectorXd(6);
    gripper_velocity.fill(0);
    ee_reference_point = Eigen::Vector3d(0, 0, 0.1);

    joint_positions.resize(6);
    std::fill(joint_positions.begin(), joint_positions.end(), 0);
    actual_joint_positions.resize(6);
    std::fill(actual_joint_positions.begin(), actual_joint_positions.end(), 0);
    joint_positions_valid = false;

    gripper_joint_position = 0;
    gripper_joint_velocity = 0;

    std::size_t i = 0;
    for (const std::string &s: arm_model_group->getActiveJointModelNames()) {
        arm_joint_index[s] = i;
        i++;
    }
}

void KinematicsHandler::set_joint_states(const sensor_msgs::JointState &joint_states)
{
    for (std::size_t i = 0; i < joint_states.name.size(); i++) {
        if (arm_model_group->hasJointModel(joint_states.name[i])) {
            // If the joint is in the arm model, copy the value to the actual_joint_positions
            // vector, which is compared with the commanded joint positions
            std::size_t joint_i = arm_joint_index[joint_states.name[i]];
            actual_joint_positions[joint_i] = joint_states.position[i];
        } if (gripper_model_group->hasJointModel(joint_states.name[i])) {
            actual_gripper_joint_position = joint_states.position[i];
        } else {
            // Otherwise, update hte robot state with this joint, to use for
            // self-collision detection.
            robot_state->setJointPositions(
                joint_states.name[i], &joint_states.position[i]);
        }
    }

    // Already have valid joint positions, don't need to set from actual_joint_positions
    if (joint_positions_valid) return;

    auto elapsed = ros::Time::now()-joint_states.header.stamp;
    // joint_states_msg is too old, don't use
    if (elapsed.toSec() > 0.05) return;

    std::cout << "ELAPSED = " << elapsed << std::endl;
    std::cout << joint_states << std::endl;
    joint_positions = actual_joint_positions;
    gripper_joint_position = actual_gripper_joint_position;

    joint_positions_valid = true;
}

void KinematicsHandler::set_gripper_velocity(const std_msgs::Float64MultiArray &gripper_velocity_in)
{
    // The generated robot model uses the following reference frames:
    // Forward = Z (Usually X)
    // Left = X (Usually Y)
    // Up = Y (Usually Z)

    // Define a matrix U, with column vectors = unit vectors of reference frames
    // eg: Column 1 = X = Usually Y = (0, 1, 0)
    Eigen::Matrix3d U;
    U << 0, 0, 1,
         1, 0, 0,
         0, 1, 0;

    // Also need a matrix U that transforms homogeneous matrices
    Eigen::Matrix4d Uh;
    Uh << 0, 0, 1, 0,
          1, 0, 0, 0,
          0, 1, 0, 0,
          0, 0, 0, 1;

    // Velocities need to be expressed in terms of the robot reference frames
    // If v = velocity in (x, y, z)
    // and v2 = velocity in (u1, u2, u3) (in robot reference frame)
    // v2 = U^T * v
    // v = U * v2

    // gripper_velocity provided as:
    // [ r_dot, theta_dot, z_dot, yaw_dot, pitch_dot, roll_dot ]
    // - Linear velocity in cylindrical coordinates
    // - Angular velocity in euler angles

    Eigen::Vector3d cylindrical_velocity(
        gripper_velocity_in.data[0], gripper_velocity_in.data[1], gripper_velocity_in.data[2]);

    Eigen::Vector3d euler_velocity(
        gripper_velocity_in.data[3],
        gripper_velocity_in.data[4],
        gripper_velocity_in.data[5]
    );

    // this->gripper_velocity vector has the form:
    // [ vx, vy, vz, wx, wy, wz]
    // - Linear velocity and angular velocity in cartesian coordinates
    Eigen::Vector3d linear_velocity, angular_velocity;

    Eigen::Isometry3d origin_body_trans = robot_state->getFrameTransform("body_link");
    Eigen::Isometry3d origin_base_trans = robot_state->getFrameTransform("arm_base_link");
    Eigen::Isometry3d origin_ee_trans = robot_state->getFrameTransform("gripper_base_link");

    // Add the transformation from the ee_reference_point to the frame origin
    origin_ee_trans = origin_ee_trans * Eigen::Translation3d(ee_reference_point);

    // 1) Want rotation from body -> arm_base
    Eigen::Matrix3d base_rotation = (origin_body_trans.inverse() * origin_base_trans).rotation();
    // 2) Want rigid-body transformation from arm_base -> gripper (end effector)
    Eigen::Isometry3d ee_trans = (origin_base_trans.inverse() * origin_ee_trans);

    // Frame transformations need to also be changed, with:
    // A' = U*A*U^T (or Uh for full transformation)
    base_rotation = U*base_rotation.matrix()*U.transpose();
    ee_trans = Uh*ee_trans*Uh.transpose();

    // Set linear velocity

    Eigen::Vector3d ee_position = ee_trans.translation();
    double r = hypot(ee_position(0), ee_position(1));
    // Set linear velocity such that it's aligoed with the forward/left axis
    linear_velocity(0) = cylindrical_velocity(0);
    linear_velocity(1) = r * cylindrical_velocity(1);
    linear_velocity(2) = cylindrical_velocity(2);

    // Rotate this such that linear_velocity(0) -> Direction from arm_base -> ee
    linear_velocity = base_rotation * linear_velocity;

    // Set angular velocity

    double yaw, pitch, roll;
    rotation_matrix_to_euler(ee_trans.rotation(), &yaw, &pitch, &roll);
    // std::cout << "Yaw = " << yaw << std::endl;
    // std::cout << "Pitch = " << pitch << std::endl;
    // std::cout << "Roll = " << roll << std::endl;

    // angular_velocity = A * euler_velocity
    Eigen::Matrix3d A;
    A << -sin(pitch),            0,          1,
          cos(pitch)*sin(roll),  cos(roll),  0,
          cos(pitch)*cos(roll), -sin(roll), 0;
    angular_velocity = A * euler_velocity;
    // At this point, angular velocity is defined in the end effector
    // reference frame.
    // Therefore, need to refer to the body frame
    angular_velocity = base_rotation * ee_trans.rotation() * angular_velocity;

    // Add theta velocity to angular_velocity(2)
    angular_velocity(2) += cylindrical_velocity(1);

    gripper_velocity.block<3, 1>(0, 0) = U.transpose() * linear_velocity;
    gripper_velocity.block<3, 1>(3, 0) = U.transpose() * angular_velocity;

    // Also get the gripper joint velocity
    gripper_joint_velocity = gripper_velocity_in.data[6];
}

void KinematicsHandler::set_master_angles(const std_msgs::Float64MultiArray &master_angles_msg)
{
    master_angles.resize(6);
    for (std::size_t i = 0; i < master_angles_msg.data.size(); i++) {
        master_angles[i] = master_angles_msg.data[i];
    }
}

void KinematicsHandler::loop_velocity(double dt)
{
    if (!joint_positions_valid) return;

    for (std::size_t i = 0; i < joint_positions.size(); i++) {
        if (std::fabs(joint_positions[i] - actual_joint_positions[i]) > 0.2) {
            joint_positions[i] = actual_joint_positions[i];
        }
    }
    if (std::fabs(gripper_joint_position - actual_gripper_joint_position) < 0.2) {
        gripper_joint_position = actual_gripper_joint_position;
    }

    robot_state->setJointGroupPositions(arm_model_group, joint_positions);

    Eigen::VectorXd joint_velocities;

    Eigen::MatrixXd jacobian;
    // Reference position in end effector frame
    robot_state->getJacobian(
        arm_model_group,
        robot_state->getLinkModel("gripper_base_link"),
        ee_reference_point,
        jacobian
    );
    joint_velocities = jacobian.inverse() * gripper_velocity;

    // Check that velocity limits aren't exceeded
    robot_state->setJointGroupVelocities(arm_model_group, joint_velocities.data());
    for (auto &joint_model: arm_model_group->getJointModels()) {
        if (!robot_state->satisfiesVelocityBounds(joint_model)) {
            // ROS_INFO("Doesn't satisfy velocity bounds.");
            // return;
        }
    }

    // trial_joint_pos = joint_pos + dt * joint_vel
    // joint_pos will be updated to these values if the current velocity
    // doesn't cause a collision
    std::vector<double> trial_joint_positions(joint_positions.size());
    for (std::size_t i = 0; i < trial_joint_positions.size(); i++) {
        trial_joint_positions[i] = joint_positions[i] + joint_velocities[i] * dt;
    }

    double trial_gripper_joint_position = gripper_joint_position + gripper_joint_velocity * dt;

    // To check if the current velocity causes a collision, move with this
    // velocity for a given period of time T, then check if the final state
    // is valid.

    std::vector<double> query_joint_positions = trial_joint_positions;
    double query_gripper_joint_position = trial_gripper_joint_position;

    double t = dt;
    const double T = 0.25;
    while (t < T) {
        robot_state->setJointGroupPositions(arm_model_group, query_joint_positions);
        robot_state->getJacobian(
            arm_model_group,
            robot_state->getLinkModel(arm_model_group->getLinkModelNames().back()),
            ee_reference_point,
            jacobian
        );
        joint_velocities = jacobian.inverse() * gripper_velocity;
        for (std::size_t i = 0; i < query_joint_positions.size(); i++) {
            query_joint_positions[i] += joint_velocities[i] * dt;
        }
        query_gripper_joint_position += gripper_joint_velocity * dt;
        t += dt;
    }
    robot_state->setJointGroupPositions(arm_model_group, query_joint_positions);

    // Set gripper joint positions. Should update mimic joints too.
    std::vector<double> query_gripper_joint_positions = { query_gripper_joint_position };
    robot_state->setJointGroupPositions(gripper_model_group, query_gripper_joint_positions);

    if (!validate_state()) {
        ROS_INFO("Velocity failed");
        return;
    }

    joint_positions = trial_joint_positions;
    gripper_joint_position = trial_gripper_joint_position;
}

void KinematicsHandler::loop_master(double dt)
{
    static constexpr double time_constant = 1.0;
    static constexpr double snap_angle = 0.1;
    static constexpr double max_speed = 1.5;
    double difference;
    for (std::size_t i = 0; i < joint_positions.size(); i++) {
        difference = master_angles[i] - joint_positions[i];
        if (fabs(difference) < snap_angle) {
            joint_positions[i] = master_angles[i];
        } else {
            double vel = difference/time_constant;
            if (fabs(vel) > max_speed) vel = (vel>0 ? max_speed : -max_speed);
            joint_positions[i] += dt * vel;
        }
    }
}

void KinematicsHandler::copy_joints_to(std_msgs::Float64MultiArray &arm_command_msg, std_msgs::Float64 &gripper_command_msg)
{
    std::copy(joint_positions.begin(), joint_positions.end(), arm_command_msg.data.begin());
    gripper_command_msg.data = gripper_joint_position;
}

bool KinematicsHandler::validate_state()
{
    // validates the current state of robot_state, so assumes
    // robot_state has already been set with positions and velocities

    for (const auto &j: arm_model_group->getJointModels()) {
        if (!robot_state->satisfiesPositionBounds(j)) {
            ROS_INFO("%s outside position bounds", j->getName().c_str());
            // return false;
        }
        if (!robot_state->satisfiesVelocityBounds(j)) {
            ROS_INFO("%s outside velocity bounds", j->getName().c_str());
            ROS_INFO("Type = %s", j->getTypeName().c_str());
            return false;
        }
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
