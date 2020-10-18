
#include "handler.h"
#include <stdlib.h>
#include <math.h>
#include <string>

void convertToIntuitiveAngles(std::vector<double>& angles){
    for(int i=0; i<angles.size(); i++){
        angles[i] = angles[i] * (180.0/M_PI);
    }
    angles[1] = -angles[1];
}

void convertToModelAngles(std::vector<double>& angles){
    for(int i=0; i<angles.size(); i++){
        angles[i] = angles[i] * (M_PI/180.0);
    }
    angles[1] = -angles[1];
}

void poseMsgToEigen(const geometry_msgs::Pose& pose, Eigen::Isometry3d& e){
    e = Eigen::Translation3d(pose.position.x,
                             pose.position.y,
                             pose.position.z)*
        Eigen::Quaterniond(pose.orientation.w,
                           pose.orientation.x,
                           pose.orientation.y,
                           pose.orientation.z);
}

void eigenToPoseMsg(const Eigen::Isometry3d& e, geometry_msgs::Pose& pose){
    pose.position.x = e.translation().x();
    pose.position.y = e.translation().y();
    pose.position.z = e.translation().z();
    Eigen::Quaterniond q(e.linear());
    pose.orientation.x = q.x();
    pose.orientation.y = q.y();
    pose.orientation.z = q.z();
    pose.orientation.w = q.w();
}

std_msgs::Float32MultiArray generateAnglesMessage(int size){
    std_msgs::Float32MultiArray msg;
    msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
    msg.layout.dim[0].label = "joint";
    msg.layout.dim[0].size = size;
    msg.layout.dim[0].stride = 1;
    msg.data.clear();
    for(int i=0; i<size; i++){
        msg.data.push_back(0);
    }
    return msg;
}

KinematicsHandler::KinematicsHandler(ros::NodeHandle& n):
        robot_model_loader("robot_description"),
        kinematic_model(robot_model_loader.getModel()),
        kinematic_state(new robot_state::RobotState(kinematic_model)),
        joint_model_group(kinematic_model->getJointModelGroup("arm")),
        c_req(), c_res(), g_planning_scene(0){

    kinematic_state->setToDefaultValues();

    c_req.group_name = "arm";
    c_req.contacts = true;
    c_req.max_contacts = 100;
    c_req.max_contacts_per_pair = 5;
    c_req.verbose = false;

    g_planning_scene = new planning_scene::PlanningScene(kinematic_model);

    arm_angles_msg = generateAnglesMessage(3);
    wrist_angles_msg = generateAnglesMessage(3);
}


bool KinematicsHandler::poseToAngles(
        inverse_kinematics::PoseToAngles::Request& req,
        inverse_kinematics::PoseToAngles::Response& res){
    Eigen::Isometry3d end_effector_state;
    const geometry_msgs::Pose pose(req.pose);
    poseMsgToEigen(pose, end_effector_state);
    
    float timeout=0.1;
    bool found_ik = kinematic_state->setFromIK(joint_model_group, end_effector_state, timeout);

    if(found_ik){
        res.valid = validateState();
        if(res.valid){
            loadAnglesMsg();
            res.arm_angles = arm_angles_msg;
            res.wrist_angles = wrist_angles_msg;
        }
    }else{
        ROS_INFO("IK not found.");
        res.valid = false;
    }
    return true;
}

bool KinematicsHandler::anglesToPose(
        inverse_kinematics::AnglesToPose::Request& req,
        inverse_kinematics::AnglesToPose::Response& res){
    std::vector<std::string> joint_names = {
        "arm1", "arm2", "arm3", "wrist1", "wrist2", "wrist3"
    };
    std::vector<double> joint_values(6);
    for(int i=0; i<3; i++){
        joint_values[i] = req.arm_angles.data[i];
    }
    for(int i=0; i<3; i++){
        joint_values[i+3] = req.wrist_angles.data[i];
    }
    convertToModelAngles(joint_values);

    kinematic_state->setVariablePositions(joint_names, joint_values);
    res.valid = validateState();
    if(res.valid){
        loadPoseMsg();
        res.pose = pose_msg;
    }
    return true;
}

void KinematicsHandler::loadAnglesMsg(){
    std::vector<double> joint_values;
    kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);

    convertToIntuitiveAngles(joint_values); 

    arm_angles_msg.data.clear();
    for(int i=0; i<3; i++){
        arm_angles_msg.data.push_back(joint_values[i]);
    }

    wrist_angles_msg.data.clear();
    for(int i=0; i<3; i++){
        wrist_angles_msg.data.push_back(joint_values[i+3]);
    }
}

void KinematicsHandler::loadPoseMsg(){
    const Eigen::Isometry3d& end_effector_state =
        kinematic_state->getGlobalLinkTransform("gripper_1");
    eigenToPoseMsg(end_effector_state, pose_msg);
}

bool KinematicsHandler::validateState(){
    if(!kinematic_state->satisfiesBounds()){
        ROS_INFO("IK found but exceeds joint limits");
        return false;
    }else if(checkCollision()){
        ROS_INFO("IK found but in collision state");
        return false;
    }
    return true;
}

bool KinematicsHandler::checkCollision(){
    g_planning_scene->checkCollision(c_req, c_res, *kinematic_state);
    bool result = c_res.collision;
    c_res.clear();
    return result;
}
