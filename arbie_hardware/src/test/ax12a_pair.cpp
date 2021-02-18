
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include "handle/dxl.h"

/* This node creates a handle to a pair of ax12a joint controllers for
 * testing.
 * When used with ros_control, the Interfaces object is used for connecting
 * the handle to, such that ros can call the read and write functions.
 * Here, it's just a placeholder, but isn't used.
 * Instead, we just call the read and write functions manually to check
 * they work.
 */

class Node {
public:
    Node(ros::NodeHandle &n, dxl::CommHandler &comm_handler, int id1, int id2):
        controller1(comm_handler, dxl::CommHandler::PROTOCOL_1, id1),
        controller2(comm_handler, dxl::CommHandler::PROTOCOL_1, id2),
        interfaces()
    {
        handle::ax12a::PositionPair::Config config;
        config.scale1 = 1;
        config.scale2 = -1;
        pair_handle = std::make_unique<handle::ax12a::PositionPair>(
            "joiwt_name", interfaces, controller1, controller2, config
        );

        command_sub = n.subscribe(
            "ax12a_pair_command", 1, &Node::command_callback, this);
    }

    void command_callback(const std_msgs::Float64 &msg)
    {
        double pos, vel, eff;
        pair_handle->read(pos, vel, eff);
        pair_handle->write(msg.data);
    }

private:
    dxl::ax12a::JointController controller1;
    dxl::ax12a::JointController controller2;
    handle::Interfaces interfaces;
    std::unique_ptr<handle::ax12a::PositionPair> pair_handle;
    ros::Subscriber command_sub;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_ax12a");
    ros::NodeHandle n;
    std::string port = "/dev/ttyUSB0";
    int baud_rate = 57600;

    if (argc != 3) {
        std::cerr << "Must provide 2 argument for 2 ax12a ids" << std::endl;
        std::cerr << "For pitch joint: 4 and 5" << std::endl;
        return 1;
    }

    int id1 = atoi(argv[1]);
    int id2 = atoi(argv[2]);
    // 0 if an invalid integer
    // Also avoid negative integers
    if (id1 <= 0 || id2 <= 0) {
        std::cerr << "Arguments are not valid integers" << std::endl;
        return 1;
    }

    dxl::CommHandler comm_handler(port, baud_rate);
    if (!comm_handler.connect()) {
        std::cerr << "Failed to open serial port" << std::endl;
        return 1;
    }

    std::cout << "Connecting to ids " << id1 << " and " << id2 << std::endl;
    Node node(n, comm_handler, id1, id2);
    ros::spin();
}
