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
    Hardware(ros::NodeHandle &n, std::string dxl_port):
        commHandler(dxl_port)
    {
        // TODO: Get IDs from param server
        if (!commHandler.connect()) {
            ROS_ERROR("Failed to open serial port for dynamixels.");
        }

        // Arm
        handles.push_back(std::make_unique<handle::xl430::Position>(
            "arm_1_joint",
            interfaces,
            dxl::xl430::ExtendedPositionController(commHandler, commHandler.PROTOCOL_1, 1),
            89/14
        ));

        handles.push_back(std::make_unique<handle::xl430::Position>(
            "arm_2_joint",
            interfaces,
            dxl::xl430::ExtendedPositionController(commHandler, commHandler.PROTOCOL_1, 2),
            25,
            8
        ));
        arm_2_handle = dynamic_cast<handle::xl430::Position*>(handles.back().get());

        handles.push_back(std::make_unique<handle::xl430::Position>(
            "arm_3_joint",
            interfaces,
            dxl::xl430::ExtendedPositionController(commHandler, commHandler.PROTOCOL_1, 3),
            25,
            40,
            -0.11 // found empirically - ie: try different values
        ));
        arm_3_handle = dynamic_cast<handle::xl430::Position*>(handles.back().get());

        // Wrist
        handles.push_back(std::make_unique<handle::ax12a::PositionPair>(
            "wrist_pitch_joint",
            interfaces,
            dxl::ax12a::JointController(commHandler, commHandler.PROTOCOL_1, 4),
            dxl::ax12a::JointController(commHandler, commHandler.PROTOCOL_1, 5),
            0, 0, -1, 1
        ));
        handles.back()->write();
        wrist_pitch_handle = dynamic_cast<handle::ax12a::PositionPair*>(handles.back().get());

        handles.push_back(std::make_unique<handle::ax12a::Position>(
            "wrist_yaw_joint",
            interfaces,
            dxl::ax12a::JointController(commHandler, commHandler.PROTOCOL_1, 6)
        ));
        handles.back()->write();

        handles.push_back(std::make_unique<handle::ax12a::Position>(
            "wrist_roll_joint",
            interfaces,
            dxl::ax12a::JointController(commHandler, commHandler.PROTOCOL_1, 7)
        ));
        handles.back()->write();

        // Gripper
        handles.push_back(std::make_unique<handle::ax12a::PositionPair>(
            "gripper_joint",
            interfaces,
            dxl::ax12a::JointController(commHandler, commHandler.PROTOCOL_1, 8),
            dxl::ax12a::JointController(commHandler, commHandler.PROTOCOL_1, 9),
            0.348, -0.317, -1, 1
        ));

        // Odrives
        // Assume the odrives node has been started, with the services:

        handles.push_back(std::make_unique<handle::Service>(
            "flippers_front_joint",
            interfaces,
            handle::Type::VEL,
            "flippers/front",
            n
        ));

        handles.push_back(std::make_unique<handle::Service>(
            "flippers_rear_joint",
            interfaces,
            handle::Type::VEL,
            "flippers/rear",
            n
        ));

        handles.push_back(std::make_unique<handle::Service>(
            "tracks_left_joint",
            interfaces,
            handle::Type::VEL,
            "tracks/left",
            n
        ));

        handles.push_back(std::make_unique<handle::Service>(
            "tracks_right_joint",
            interfaces,
            handle::Type::VEL,
            "tracks/right",
            n
        ));

        // Camera (for now, placeholder handle)

        handles.push_back(std::make_unique<handle::PosDummy>(
            "camera_tilt_joint", interfaces
        ));

        registerInterface(&interfaces.state);
        registerInterface(&interfaces.pos);
        registerInterface(&interfaces.vel);
        registerInterface(&interfaces.eff);

        std::cout << "FINISHED REGISTERING" << std::endl;

        // Calibrate

        if (arm_2_handle->is_connected() && arm_3_handle->is_connected()
            & wrist_pitch_handle->is_connected()) {

            ros::Duration(1).sleep();

            arm_2_handle->calibrate();
            // Move arm_2 back up a bit
            // Move at 1 rad/s, for 1.2 radians, at 0.1s timesteps
            for (int i = 0; i < 12; i++) {
                arm_2_handle->move(0.1);
                ros::Duration(0.1).sleep();
            }
            ros::Duration(2.5).sleep();

            // Move wrist out the way temporarily, while calibrating arm_3
            // Move at 1 rads, for -0.8 radians, at 0.1s timesteps
            for (int i = 0; i < 8; i++) {
                arm_2_handle->move(-0.1);
                ros::Duration(0.1).sleep();
            }
            ros::Duration(1).sleep();

            arm_3_handle->calibrate();
            // Move arm_3 back up a bit
            arm_3_handle->write(0.24);
            ros::Duration(3).sleep();

            // Move wrist pitch down again
            // Move at 1 rads, for 0.8 radians, at 0.1s timesteps
            for (int i = 0; i < 8; i++) {
                arm_2_handle->move(0.1);
                ros::Duration(0.1).sleep();
            }
            // In case not back at 0 exactly
            wrist_pitch_handle->write(0);
            ros::Duration(1).sleep();
        }
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

    // For calibration - have no ownership
    handle::xl430::Position *arm_2_handle, *arm_3_handle;
    handle::ax12a::PositionPair *wrist_pitch_handle;
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

    ros::AsyncSpinner spinner(4);
    spinner.start();
    ros::waitForShutdown();
    return 0;
}
