
#include "handler.h"
#include <stdlib.h>
#include <math.h>

bool largeAngleChange(const std::vector<double>& joints1,
                      const std::vector<double>& joints2){
    static const double ANGLE_THRESHOLD = M_PI/2;
    if(joints1.size() != joints2.size()){
        ROS_INFO("Comparing vectors of different sizes.");
        return false;
    }
    for(int i=0; i<joints1.size(); i++){
        if(abs(joints1[i] - joints2[i]) > ANGLE_THRESHOLD){
            return true;
        }
    }
    return false;
}

void poseMsgToEigen(const geometry_msgs::Pose &m, Eigen::Isometry3d& e){
    e = Eigen::Translation3d(m.position.x,
                             m.position.y,
                             m.position.z)*
        Eigen::Quaterniond(m.orientation.w,
                           m.orientation.x,
                           m.orientation.y,
                           m.orientation.z);
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

    angles_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
    angles_msg.layout.dim[0].label = "joint";
    angles_msg.layout.dim[0].size = 6;
    angles_msg.layout.dim[0].stride = 1;
    angles_msg.data.clear();
    for(int i=0; i<6; i++){
        angles_msg.data.push_back(0);
    }
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
            res.angles = angles_msg;
        }
    }else{
        ROS_INFO("IK not found.");
        res.valid = false;
    }
    return true;
}

bool KinematicsHandler::checkAngles(
        inverse_kinematics::CheckAngles::Request& req,
        inverse_kinematics::CheckAngles::Response& res){
    std::vector<double> joint_values(6);
    for(int i=0; i<6; i++){
        joint_values[i] = req.angles.data[i];
    }
    kinematic_state->setVariablePositions(joint_values);
    res.valid = validateState();
    return true;
}

void KinematicsHandler::loadAnglesMsg(){
    std::vector<double> joint_values;
    kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
    angles_msg.data.clear();
    for(int i=0; i<6; i++){
        angles_msg.data.push_back(joint_values[i]);
    }
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
