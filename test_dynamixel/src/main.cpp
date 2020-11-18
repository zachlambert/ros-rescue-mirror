#include <ros/ros.h>
#include "dxl/xl430.h"
#include "dxl/ax12a.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "dynamixel_controller");
    ros::NodeHandle n;

    dxl::CommHandler commHandler("/dev/ttyUSB0");
    dxl::xl430::VelocityController arm2(commHandler, commHandler.PROTOCOL_1, 2);
    dxl::ax12a::JointController wristRoll(commHandler, commHandler.PROTOCOL_1, 6);

    wristRoll.writeComplianceSlope(255);

    arm2.enable();
    wristRoll.enable();

    arm2.writeGoalVelocity(0.5);
    for (int i = 0; i < 10; i++) {
        wristRoll.writeGoalPosition(i*5 * M_PI / 180);
        ros::Duration(0.2).sleep();
        ROS_INFO("Arm2: %f %f %f", arm2.readPosition(), arm2.readVelocity(), arm2.readLoad());
        ROS_INFO("WristRoll: %f %f", wristRoll.readPosition(), wristRoll.readVelocity());
    }
    arm2.writeGoalVelocity(0);
    arm2.disable();
    wristRoll.writeGoalPosition(0);
    ros::Duration(0.5).sleep();
    wristRoll.disable();

    return 0;
}
