#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include "dxl/ax12a.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "benchmark_ax12a");
    ros::NodeHandle n;
    std::string port = "/dev/ttyUSB0";
    int baud_rate = 1000000;
    int ax12a_id = 6;

    dxl::CommHandler comm_handler(port, baud_rate);
    if (!comm_handler.connect()) {
        std::cerr << "Failed to open serial port" << std::endl;
        return 1;
    }

    dxl::ax12a::JointController controller(
        comm_handler,
        dxl::CommHandler::PROTOCOL_1,
        ax12a_id
    );
    controller.enable();

    std::size_t N = 100;
    ros::Time start_time;
    double time_us;

    double cmd;
    controller.readPosition(cmd);
    start_time = ros::Time::now();
    for (std::size_t i = 0; i < N; i++) {
        controller.writeGoalPosition(cmd);
    }
    time_us = (ros::Time::now() - start_time).toSec() * 1e6;
    ROS_INFO("Time per write = %f", time_us/N);

    double value;
    start_time = ros::Time::now();
    for (std::size_t i = 0; i < N; i++) {
        controller.readPosition(value);
    }
    time_us = (ros::Time::now() - start_time).toSec() * 1e6;
    ROS_INFO("Time per pos read = %f", time_us/N);

    start_time = ros::Time::now();
    for (std::size_t i = 0; i < N; i++) {
        controller.readVelocity(value);
    }
    time_us = (ros::Time::now() - start_time).toSec() * 1e6;
    ROS_INFO("Time per vel read = %f", time_us/N);

    start_time = ros::Time::now();
    for (std::size_t i = 0; i < N; i++) {
        controller.readLoad(value);
    }
    time_us = (ros::Time::now() - start_time).toSec() * 1e6;
    ROS_INFO("Time per load read = %f", time_us/N);
}
