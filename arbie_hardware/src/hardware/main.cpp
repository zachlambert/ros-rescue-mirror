#include <array>
#include <mutex>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <controller_manager/controller_manager.h>
#include <std_srvs/Trigger.h>

#include "handle/dxl.h"

class Hardware: public hardware_interface::RobotHW {
public:
    Hardware(ros::NodeHandle &n, const std::string &dxl_port):
        commHandler(dxl_port), calibrated(false) {}

    bool initialise(ros::NodeHandle &n)
    {
        ROS_INFO("Initialising hardware");

        // TODO: Get IDs from param server
        int arm_1_id = 1;
        int arm_2_id = 2;
        int arm_3_id = 3;
        int wrist_pitch_1_id = 4;
        int wrist_pitch_2_id = 5;
        int wrist_yaw_id = 6;
        int wrist_roll_id = 7;
        int gripper_1_id = 8;
        int gripper_2_id = 9;

        if (!commHandler.connect()) {
            ROS_ERROR("Failed to open serial port for dynamixels.");
            return false;
        }
        // Arm

        handle::xl430::Position::Config arm_1_config;
        arm_1_config.scale = 89.0/14;
        handles.push_back(std::make_unique<handle::xl430::Position>(
            "arm_1_joint",
            interfaces,
            dxl::xl430::ExtendedPositionController(
                commHandler, commHandler.PROTOCOL_1, arm_1_id),
            arm_1_config
        ));
        // Get the last added handle, and cast to a standard pointer, to get
        // access to the arm_1 handle for calibration
        arm_1_handle = dynamic_cast<handle::xl430::Position*>(handles.back().get());

        handle::xl430::Position::Config arm_2_config;
        arm_2_config.scale = 25;
        arm_2_config.eff2_threshold = 8;
        handles.push_back(std::make_unique<handle::xl430::Position>(
            "arm_2_joint",
            interfaces,
            dxl::xl430::ExtendedPositionController(
                commHandler, commHandler.PROTOCOL_1, arm_2_id),
            arm_2_config
        ));
        arm_2_handle = dynamic_cast<handle::xl430::Position*>(handles.back().get());

        handle::xl430::Position::Config arm_3_config;
        arm_3_config.scale = 25;
        arm_3_config.eff2_threshold = 40;
        // arm_3_config.zero_pos = -0.11; // Found by testing to see which works best
        handles.push_back(std::make_unique<handle::xl430::Position>(
            "arm_3_joint",
            interfaces,
            dxl::xl430::ExtendedPositionController(
                commHandler, commHandler.PROTOCOL_1, arm_3_id),
            arm_3_config
        ));
        arm_3_handle = dynamic_cast<handle::xl430::Position*>(handles.back().get());

        // Wrist
        handle::ax12a::PositionPair::Config wrist_pitch_config;
        wrist_pitch_config.origin1 = 0;
        wrist_pitch_config.origin2 = 0;
        wrist_pitch_config.scale1 = -1;
        wrist_pitch_config.scale2 = 1;
        handles.push_back(std::make_unique<handle::ax12a::PositionPair>(
            "wrist_pitch_joint",
            interfaces,
            dxl::ax12a::JointController(
                commHandler, commHandler.PROTOCOL_1, wrist_pitch_1_id),
            dxl::ax12a::JointController(
                commHandler, commHandler.PROTOCOL_1, wrist_pitch_2_id),
            wrist_pitch_config
        ));
        // Write to the last handle added, to set to the 0 position
        handles.back()->write();
        wrist_pitch_handle = dynamic_cast<handle::ax12a::PositionPair*>(handles.back().get());

        handles.push_back(std::make_unique<handle::ax12a::Position>(
            "wrist_yaw_joint",
            interfaces,
            dxl::ax12a::JointController(
                commHandler, commHandler.PROTOCOL_1, wrist_yaw_id)
        ));
        handles.back()->write();

        handles.push_back(std::make_unique<handle::ax12a::Position>(
            "wrist_roll_joint",
            interfaces,
            dxl::ax12a::JointController(
                commHandler, commHandler.PROTOCOL_1, wrist_roll_id)
        ));
        handles.back()->write();

        // Gripper
        handle::ax12a::PositionPair::Config gripper_config;
        gripper_config.origin1 = 0.348;
        gripper_config.origin2 = -0.317;
        gripper_config.scale1 = -1;
        gripper_config.scale2 = 1;
        gripper_config.enforce_matching = false;
        handles.push_back(std::make_unique<handle::ax12a::PositionPair>(
            "gripper_joint",
            interfaces,
            dxl::ax12a::JointController(commHandler, commHandler.PROTOCOL_1, gripper_1_id),
            dxl::ax12a::JointController(commHandler, commHandler.PROTOCOL_1, gripper_2_id),
            gripper_config
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

        ROS_INFO("Finished initialising hardware");
        hardware_mutex.unlock();
        return true;
    }

    bool calibrate() {
        if (!arm_2_handle->is_connected()) return false;
        if (!arm_3_handle->is_connected()) return false;
        if (!wrist_pitch_handle->is_connected()) return false;

        ROS_INFO("Calibration waiting for mutex to unlock");
        hardware_mutex.lock();
        ROS_INFO("Calibrating arm");

        // For now, just register current position as the origin.
        // TODO: Add proper calibration for arm_1
        arm_1_handle->set_as_origin();

        // TEMPORARY
        arm_2_handle->set_as_origin();
        arm_3_handle->set_as_origin();
        calibrated = true;
        hardware_mutex.unlock();
        return true;

        ros::Duration(1).sleep();

        arm_2_handle->calibrate();
        // Move arm_2 back up a bit
        arm_2_handle->move(0.1, 0.2);
        ros::Duration(2.5).sleep();

        // Move wrist out the way temporarily, while calibrating arm_3
        wrist_pitch_handle->move(-0.5, 0.4);
        ros::Duration(1).sleep();

        arm_3_handle->calibrate();
        // Move arm_3 back up a bit
        arm_3_handle->move(0.2, 0.2);
        ros::Duration(2.5).sleep();

        // Move wrist pitch down again
        wrist_pitch_handle->move(0.5, 0.4);
        wrist_pitch_handle->write(0);
        ros::Duration(1).sleep();

        ROS_INFO("Finished calibrating arm");
        calibrated = true;
        hardware_mutex.unlock();
        return true;
    }

    void read()
    {
        if (!hardware_mutex.try_lock()) return;
        for (auto &handle: handles) {
            handle->read();
        }
        hardware_mutex.unlock();
    }

    void write()
    {
        if (!calibrated) return;
        if (!hardware_mutex.try_lock()) return;
        for (auto &handle: handles) {
            handle->write();
        }
        hardware_mutex.unlock();
    }

private:
    dxl::CommHandler commHandler;
    handle::Interfaces interfaces;
    std::vector<std::unique_ptr<handle::Handle>> handles;

    std::mutex hardware_mutex;
    bool calibrated;
    // For calibration - have no ownership
    handle::xl430::Position *arm_1_handle, *arm_2_handle, *arm_3_handle;
    handle::ax12a::PositionPair *wrist_pitch_handle;
};


class Node {
public:
    Node(ros::NodeHandle& n, const std::string &port):
        hw(n, port),
        cm(&hw, n)
    {
        loop_timer = n.createTimer(
            ros::Duration(1.0/50),
            &Node::loop,
            this
        );
        successful = hw.initialise(n);
        if (!successful) {
            ROS_ERROR("Failed to initialise hardware");
        }

        // calibrate_server = n.advertiseService(
        //     "calibrate", &Node::calibrate_callback, this
        // );
        hw.calibrate();
    }

    void loop(const ros::TimerEvent &timer)
    {
        if (!successful) return;

        hw.read();
        cm.update(
            ros::Time::now(),
            ros::Duration(timer.current_real - timer.last_real)
        );
        hw.write();
    }

    bool calibrate_callback(
        std_srvs::TriggerRequest &req,
        std_srvs::TriggerResponse &res)
    {
        res.success = hw.calibrate();
        return true;
    }

private:
    Hardware hw;
    controller_manager::ControllerManager cm;
    ros::Timer loop_timer;
    ros::ServiceServer calibrate_server;
    bool successful;
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
