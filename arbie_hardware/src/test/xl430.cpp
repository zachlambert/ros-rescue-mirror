#include <ros/ros.h>
#include "dxl/xl430.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_xl430");
    ros::NodeHandle n;
    std::string port;
    int baud_rate;
    double cmd_vel;
    int id;
    bool write_tx_only;
    double extra_delay_ms;

    ros::NodeHandle n_local("~");
    n_local.param("port", port, std::string("/dev/ttyUSB0"));
    n_local.param("baud_rate", baud_rate, 1000000);
    n_local.param("id", id, 2); // Default to 3rd arm joint
    n_local.param("vel", cmd_vel, 0.5);
    n_local.param("write_tx_only", write_tx_only, false);
    n_local.param("delay", extra_delay_ms, 0.0);

    std::cout << "Port = " << port << std::endl;
    std::cout << "Baud rate = " << baud_rate << std::endl;
    std::cout << "Id = " << id << std::endl;
    std::cout << "Vel = " << cmd_vel << std::endl;
    std::cout << "Write tx only = " << write_tx_only << std::endl;
    std::cout << "Type y to continue: ";
    std::string input;
    std::cin >> input;
    if (input != "y") {
        return 0;
    }

    if (std::fabs(cmd_vel) > 25) {
        std::cerr << "Use a smaller joint velocity" << std::endl;
        return 1;
    }

    dxl::CommHandler comm_handler(port, baud_rate);
    if (!comm_handler.connect()) {
        ROS_ERROR("Failed to open serial port");
        return 1;
    }

    dxl::xl430::ExtendedPositionController controller(
       comm_handler, dxl::CommHandler::PROTOCOL_1, id, write_tx_only);
    controller.enable();

    double cmd;
    controller.readPosition(cmd);

    double t = 0;
    double dt;
    ros::Time prev_time = ros::Time::now();
    ros::Time current_time;
    double pos, vel, eff;
    while (t < 1) {
        // Read current state, expect a delay for reading
        controller.readPosition(pos);
        controller.readVelocity(vel);
        controller.readLoad(eff);
        ros::Duration(extra_delay_ms/1000).sleep();
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
        controller.writeGoalPosition(cmd);
        std::cout << "Cmd = " << cmd << std::endl;
    }
}
