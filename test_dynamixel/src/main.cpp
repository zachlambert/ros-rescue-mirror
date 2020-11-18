#include <ros/ros.h>
#include "dxl_control/xl430.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "dynamixel_controller");
    ros::NodeHandle n;

    dxl::CommHandler commHandler("/dev/ttyUSB0");
    dxl::xl430::VelocityController arm1(commHandler, commHandler.PROTOCOL_1, 1);
    dxl::xl430::VelocityController arm2(commHandler, commHandler.PROTOCOL_1, 2);
    dxl::xl430::VelocityController arm3(commHandler, commHandler.PROTOCOL_1, 3);

    arm2.enable();
    arm2.writeGoalVelocity(-0.5);
    for (int i = 0; i < 10; i++) {
        ROS_INFO("Arm2: %f %f %f", arm2.readPosition(), arm2.readVelocity(), arm2.readLoad());
        ros::Duration(0.2).sleep();
    }
    arm2.writeGoalVelocity(0);
    arm2.disable();

    return 0;
}
