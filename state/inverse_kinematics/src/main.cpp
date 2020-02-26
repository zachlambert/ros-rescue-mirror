
#include "ros/ros.h"
#include "handler.h"

int main(int argc, char** argv){
    ros::init(argc, argv, "inverse_kinematics");
    ros::NodeHandle n;

    KinematicsHandler handler(n);
    ros::ServiceServer service1 = n.advertiseService(
        "pose_to_angles", &KinematicsHandler::poseToAngles, &handler);
    ros::ServiceServer service2 = n.advertiseService(
        "check_angles", &KinematicsHandler::checkAngles, &handler);
    ros::spin();
    return 0;
}
