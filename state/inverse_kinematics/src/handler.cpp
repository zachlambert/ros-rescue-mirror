
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

std::vector<double> convertAnglesToActual(const std::vector<double>& ros_angles){
    std::vector<double> actual_angles(6);
    actual_angles[0] = (180/M_PI)*ros_angles[0];
    actual_angles[1] = -(180/M_PI)*ros_angles[1];
    actual_angles[2] = (180/M_PI)*ros_angles[2];
    actual_angles[3] = (180/M_PI)*ros_angles[3];
    actual_angles[4] = (180/M_PI)*ros_angles[4];
    actual_angles[5] = (180/M_PI)*ros_angles[5];
    return actual_angles;
}

UpdateHandler::UpdateHandler(ros::NodeHandle& n):
        robot_model_loader("robot_description"),
        kinematic_model(robot_model_loader.getModel()),
        kinematic_state(new robot_state::RobotState(kinematic_model)),
        joint_model_group(kinematic_model->getJointModelGroup("arm")),
        c_req(), c_res(), g_planning_scene(0){

    kinematic_state->setToDefaultValues();

    arm_demand_pub = n.advertise<std_msgs::Float32MultiArray>(
        "/arm_demand_angles", 1);
    wrist_demand_pub = n.advertise<std_msgs::Float32MultiArray>(
        "/wrist_demand_angles", 1);

    arm_demand_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
    arm_demand_msg.layout.dim[0].label = "joint";
    arm_demand_msg.layout.dim[0].size = 3;
    arm_demand_msg.layout.dim[0].stride = 1;
    arm_demand_msg.data.clear();
    for(int i=0; i<3; i++){
        arm_demand_msg.data.push_back(0);
    }

    wrist_demand_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
    wrist_demand_msg.layout.dim[0].label = "joint";
    wrist_demand_msg.layout.dim[0].size = 3;
    wrist_demand_msg.layout.dim[0].stride = 1;
    wrist_demand_msg.data.clear();
    for(int i=0; i<3; i++){
        wrist_demand_msg.data.push_back(0);
    }

    target_pose_sub = n.subscribe(
        "/target_pose", 1000, &UpdateHandler::targetPoseCallback, this);

    c_req.group_name = "arm";
    c_req.contacts = true;
    c_req.max_contacts = 100;
    c_req.max_contacts_per_pair = 5;
    c_req.verbose = false;

    g_planning_scene = new planning_scene::PlanningScene(kinematic_model);

    geometry_msgs::Pose initial_pose;
    initial_pose.position.x = 0.6;
    initial_pose.position.z = 0.5;
    Eigen::Isometry3d end_effector_state;
    poseMsgToEigen(initial_pose, end_effector_state);
    bool found_ik = kinematic_state->setFromIK(joint_model_group, end_effector_state, 0.1);
    if(!found_ik){
        ROS_INFO("Failed to go to initial pose.");
    }else{
        std::vector<double> joint_values;
        kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
        publishAngles(joint_values);
    }
}


void UpdateHandler::targetPoseCallback(const geometry_msgs::Pose::ConstPtr& msg){
    Eigen::Isometry3d end_effector_state;

    const geometry_msgs::Pose pose(*msg);
    poseMsgToEigen(pose, end_effector_state);

    std::vector<double> prev_joint_values;
    kinematic_state->copyJointGroupPositions(joint_model_group, prev_joint_values);

    std::vector<double> joint_values;
    
    float timeout=0.1;
    bool found_ik = kinematic_state->setFromIK(joint_model_group, end_effector_state, timeout);

    if(found_ik){
        kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
        if(!kinematic_state->satisfiesBounds()){
            ROS_INFO("IK found but exceeds joint limits");
            kinematic_state->setVariablePositions(prev_joint_values); 
        }else if(checkCollision()){
            ROS_INFO("IK found but in collision state");
            kinematic_state->setVariablePositions(prev_joint_values); 
        }else if(largeAngleChange(prev_joint_values, joint_values)){
            ROS_INFO("IK found but angle changes are too large");
            kinematic_state->setVariablePositions(prev_joint_values); 
        }else{
            publishAngles(joint_values);
        }
    }else{
        ROS_INFO("No IK found");
    }
}

bool UpdateHandler::checkCollision(){
    g_planning_scene->checkCollision(c_req, c_res, *kinematic_state);
    bool result = c_res.collision;
    c_res.clear();
    return result;
}

void UpdateHandler::publishAngles(const std::vector<double>& joint_values){
    // Need to convert angles from ros angles to actual angles
    std::vector<double> actual_angles = convertAnglesToActual(joint_values);

    for(int i=0; i<3; i++){
        arm_demand_msg.data[i] = actual_angles[i];
    }
    for(int i=0; i<3; i++){
        wrist_demand_msg.data[i] = actual_angles[i+3];
    }
    arm_demand_pub.publish(arm_demand_msg);
    wrist_demand_pub.publish(wrist_demand_msg);
}
