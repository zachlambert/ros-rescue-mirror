#include <ros/ros.h>
#include "dxl_control/xl430.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "dynamixel_controller");
    ros::NodeHandle n;

    CommHandler commHandler("/dev/ttyUSB0");
    XL430VelocityController arm1(commHandler, CommHandler::PROTOCOL_1, 1);
    XL430VelocityController arm2(commHandler, CommHandler::PROTOCOL_1, 2);
    XL430VelocityController arm3(commHandler, CommHandler::PROTOCOL_1, 3);

    arm2.enable();
    arm2.writeGoalVelocity(1);
    for (int i = 0; i < 10; i++) {
        ROS_INFO("Arm2: %f", arm2.readPosition());
        ros::Duration(0.1).sleep();
    }
    arm2.writeGoalVelocity(0);
    ros::Duration(1).sleep();
    arm2.disable();

    return 0;
}
