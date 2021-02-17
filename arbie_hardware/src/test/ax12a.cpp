
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include "dxl/ax12a.h"

class Node {
public:
    Node(ros::NodeHandle &n, dxl::CommHandler &comm_handler, int id):
        controller(comm_handler, dxl::CommHandler::PROTOCOL_1, id)
    {
        command_sub = n.subscribe(
            "ax12a_command", 1, &Node::command_callback, this);
        controller.enable();
        controller.writeComplianceSlope(128);
    }

    void command_callback(const std_msgs::Float64 &msg)
    {
        controller.writeGoalPosition(msg.data);
        double pos, vel;
        controller.readPosition(pos);
        controller.readVelocity(vel);
        std::cout << "Pos = " << pos << std::endl;
        std::cout << "Vel = " << vel << std::endl;
    }
private:
    dxl::ax12a::JointController controller;
    ros::Subscriber command_sub;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_ax12a");
    ros::NodeHandle n;
    std::string port = "/dev/ttyUSB0";
    int baud_rate = 57600;

    if (argc != 2) {
        std::cerr << "Must provide 1 argument for ax12a id" << std::endl;
        return 1;
    }

    int id = atoi(argv[1]);
    // 0 if an invalid integer
    // Also avoid negative integers
    if (id <= 0) {
        std::cerr << "Argument is not a valid integer" << std::endl;
        return 1;
    }

    dxl::CommHandler comm_handler(port, baud_rate);
    if (!comm_handler.connect()) {
        std::cerr << "Failed to open serial port" << std::endl;
        return 1;
    }

    std::cout << "Connecting to id " << id << std::endl;
    Node node(n, comm_handler, id);
    ros::spin();
}
