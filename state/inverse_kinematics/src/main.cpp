
#include "ros/ros.h"
#include "handler.h"


int main(int argc, char** argv){
    ros::init(argc, argv, "inverse_kinematics");
    ros::NodeHandle n;
    UpdateHandler updateHandler(n);
    ros::spin();
    return 0;
}
