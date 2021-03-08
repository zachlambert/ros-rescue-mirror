#include <ros/ros.h>
#include "handle/dxl.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_ax12a_pair");
    ros::NodeHandle n;
    std::string port;
    int baud_rate;
    double cmd_vel;
    int id1 = 4;
    int id2 = 5;

    ros::NodeHandle n_local("~");
    n_local.param("port", port, std::string("/dev/ttyUSB0"));
    n_local.param("baud_rate", baud_rate, 1000000);
    n_local.param("vel", cmd_vel, 0.5);

    std::cout << "Port = " << port << std::endl;
    std::cout << "Baud rate = " << baud_rate << std::endl;
    std::cout << "Vel = " << cmd_vel << std::endl;
    std::cout << "Type y to continue: ";
    std::string input;
    std::cin >> input;
    if (input != "y") {
        return 0;
    }

    if (std::fabs(cmd_vel) > 0.5) {
        std::cerr << "Use a smaller joint velocity" << std::endl;
        return 1;
    }

    dxl::CommHandler comm_handler(port, baud_rate);
    if (!comm_handler.connect()) {
        ROS_ERROR("Failed to open serial port");
        return 1;
    }

    dxl::ax12a::JointController controller1(
        comm_handler, dxl::CommHandler::PROTOCOL_1, id1);
    dxl::ax12a::JointController controller2(
        comm_handler, dxl::CommHandler::PROTOCOL_1, id2);

    handle::ax12a::PositionPair::Config config;
    config.scale1 = -1;
    config.scale2 = 1;

    handle::Interfaces interfaces;
    handle::ax12a::PositionPair handle(
        "wrist_pitch_joint", interfaces, controller1, controller2, config);

    double cmd, pos, vel, eff;
    handle.read(cmd, vel, eff);

    double t = 0;
    double dt;
    ros::Time prev_time = ros::Time::now();
    ros::Time current_time;
    while (t < 1) {
        // Read current state, expect a delay for reading
        handle.read(pos, vel, eff);
        std::cout << "t = " << t << std::endl;
        std::cout << "Pos = " << pos << std::endl;
        std::cout << "Vel = " << vel << std::endl;
        std::cout << "Load = " << eff << std::endl;
        // Measure elapsed time since last write
        current_time = ros::Time::now();
        dt = (current_time - prev_time).toSec();
        prev_time = current_time;
        // Write next cmd position
        cmd += cmd_vel*dt;
        t += dt;
        handle.write(cmd);
    }
    return 0;
}
