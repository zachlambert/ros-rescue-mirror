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
        // TODO: Get IDs from param server

        // Arm
        handles.push_back(std::make_unique<handle::xl430::Velocity>(
            "arm_1_joint",
            interfaces,
            dxl::xl430::VelocityController(commHandler, commHandler.PROTOCOL_1, 1)
        ));
        handles.push_back(std::make_unique<handle::xl430::Velocity>(
            "arm_2_joint",
            interfaces,
            dxl::xl430::VelocityController(commHandler, commHandler.PROTOCOL_1, 2)
        ));
        handles.push_back(std::make_unique<handle::xl430::Velocity>(
            "arm_3_joint",
            interfaces,
            dxl::xl430::VelocityController(commHandler, commHandler.PROTOCOL_1, 3)
        ));

        // Wrist
        handles.push_back(std::make_unique<handle::ax12a::PositionPair>(
            "wrist_pitch_joint",
            interfaces,
            dxl::ax12a::JointController(commHandler, commHandler.PROTOCOL_1, 4),
            dxl::ax12a::JointController(commHandler, commHandler.PROTOCOL_1, 5),
            0, 0, -1, 1
        ));
        handles.push_back(std::make_unique<handle::ax12a::Position>(
            "wrist_yaw_joint",
            interfaces,
            dxl::ax12a::JointController(commHandler, commHandler.PROTOCOL_1, 6)
        ));
        handles.push_back(std::make_unique<handle::ax12a::Position>(
            "wrist_roll_joint",
            interfaces,
            dxl::ax12a::JointController(commHandler, commHandler.PROTOCOL_1, 7)
        ));

        // Gripper
        handles.push_back(std::make_unique<handle::ax12a::PositionPair>(
            "gripper_joint",
            interfaces,
            dxl::ax12a::JointController(commHandler, commHandler.PROTOCOL_1, 8),
            dxl::ax12a::JointController(commHandler, commHandler.PROTOCOL_1, 9),
            0.348, -0.317, -1, 1
        ));

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
