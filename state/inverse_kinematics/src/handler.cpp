
#include "handler.h"

void poseMsgToEigen(const geometry_msgs::Pose &m, Eigen::Isometry3d& e){
    e = Eigen::Translation3d(m.position.x,
                             m.position.y,
                             m.position.z)*
        Eigen::Quaterniond(m.orientation.w,
                           m.orientation.x,
                           m.orientation.y,
                           m.orientation.z);
}

UpdateHandler::UpdateHandler(ros::NodeHandle& n):
        robot_model_loader("robot_description"),
        kinematic_model(robot_model_loader.getModel()),
        kinematic_state(new robot_state::RobotState(kinematic_model)),
        joint_model_group(kinematic_model->getJointModelGroup("arm")){

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

}


void UpdateHandler::targetPoseCallback(const geometry_msgs::Pose::ConstPtr& msg){
    Eigen::Isometry3d end_effector_state;

    const geometry_msgs::Pose pose(*msg);
    poseMsgToEigen(pose, end_effector_state);

    std::vector<double> joint_values;
    
    float timeout=0.1;
    bool found_ik = kinematic_state->setFromIK(joint_model_group, end_effector_state, timeout);

    if(found_ik){
        kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
        if(kinematic_state->satisfiesBounds()){
            for(int i=0; i<3; i++){
                arm_demand_msg.data[i] = joint_values[i];
            }
            for(int i=0; i<3; i++){
                wrist_demand_msg.data[i] = joint_values[i+3];
            }
            arm_demand_pub.publish(arm_demand_msg);
            wrist_demand_pub.publish(wrist_demand_msg);
        }else{
            ROS_INFO("IK found but exceeds joint limits");
        }
    }else{
        ROS_INFO("No IK found");
    }
}
