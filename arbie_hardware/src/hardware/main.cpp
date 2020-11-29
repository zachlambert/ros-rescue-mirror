#include <array>

#include "ros/ros.h"
#include "ros/callback_queue.h"
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <controller_manager/controller_manager.h>

#include "handle/dxl.h"

class Hardware: public hardware_interface::RobotHW {
public:
    Hardware(ros::NodeHandle &n, std::string port):
        commHandler(port)
    {
        handles.push_back(handle::make(
            "arm1",
            interfaces,
            dxl::xl430::VelocityController(commHandler, commHandler.PROTOCOL_1, 1))
        );
        handles.push_back(handle::make(
            "arm2",
            interfaces,
            dxl::xl430::VelocityController(commHandler, commHandler.PROTOCOL_1, 2))
        );
        handles.push_back(handle::make(
            "arm3",
            interfaces,
            dxl::xl430::VelocityController(commHandler, commHandler.PROTOCOL_1, 3))
        );
        handles.push_back(handle::make(
            "wrist_yaw",
            interfaces,
            dxl::ax12a::JointController(commHandler, commHandler.PROTOCOL_1, 6))
        );
        registerInterface(&interfaces.state);
        registerInterface(&interfaces.pos);
        registerInterface(&interfaces.vel);
        registerInterface(&interfaces.eff);
    }

    void read()
    {
        for (auto &handle: handles) {
            handle->read();
        }
    }

    void write()
    {
        for (auto &handle: handles) {
            handle->write();
        }
    }

private:
    dxl::CommHandler commHandler;
    handle::Interfaces interfaces;
    std::vector<std::unique_ptr<handle::Handle>> handles;
};


class Node {
public:
    Node(ros::NodeHandle& n, const std::string &port): hw(n, port), cm(&hw, n)
    {
        loop_timer = n.createTimer(
            ros::Duration(1.0/50),
            &Node::loop,
            this
        );
    }

    void loop(const ros::TimerEvent &timer)
    {
        hw.read();
        cm.update(
            ros::Time::now(),
            ros::Duration(timer.current_real - timer.last_real)
        );
        hw.write();
    }
private:
    Hardware hw;
    controller_manager::ControllerManager cm;
    ros::Timer loop_timer;
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "control");
    ros::NodeHandle n;
    std::string port = "/dev/ttyUSB0";
    Node node(n, port);

    ros::AsyncSpinner spinner(2);
    spinner.start();
    ros::waitForShutdown();
    return 0;
}
