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
    Hardware(): calibrated(false) {}

    bool initialise(ros::NodeHandle &n)
    {
        ROS_INFO("Initialising hardware");

        handles.push_back(std::make_unique<handle::PosDummy>(
            "arm_1_joint", interfaces
        ));
        handles.push_back(std::make_unique<handle::PosDummy>(
            "arm_2_joint", interfaces
        ));
        handles.push_back(std::make_unique<handle::PosDummy>(
            "arm_3_joint", interfaces
        ));
        handles.push_back(std::make_unique<handle::PosDummy>(
            "wrist_pitch_joint", interfaces
        ));
        handles.push_back(std::make_unique<handle::PosDummy>(
            "wrist_yaw_joint", interfaces
        ));
        handles.push_back(std::make_unique<handle::PosDummy>(
            "wrist_roll_joint", interfaces
        ));

        handles.push_back(std::make_unique<handle::PosDummy>(
            "gripper_joint", interfaces
        ));

        handles.push_back(std::make_unique<handle::VelDummy>(
            "flippers_front_joint", interfaces
        ));
        handles.push_back(std::make_unique<handle::VelDummy>(
            "flippers_rear_joint", interfaces
        ));
        handles.push_back(std::make_unique<handle::VelDummy>(
            "tracks_left_joint", interfaces
        ));
        handles.push_back(std::make_unique<handle::VelDummy>(
            "tracks_right_joint", interfaces
        ));

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
        hardware_mutex.lock();
        ROS_INFO("Calibrating hardware");
        ros::Duration(1.0).sleep();
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
    handle::Interfaces interfaces;
    std::vector<std::unique_ptr<handle::Handle>> handles;
    std::mutex hardware_mutex;
    bool calibrated;
};


class Node {
public:
    Node(ros::NodeHandle& n, const std::string &port):
        hw(),
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

        calibrate_server = n.advertiseService(
            "calibrate", &Node::calibrate_callback, this
        );
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

    ros::AsyncSpinner spinner(6);
    spinner.start();
    ros::waitForShutdown();
    return 0;
}
