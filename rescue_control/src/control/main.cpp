#include "ros/ros.h"
#include "ros/callback_queue.h"
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <controller_manager/controller_manager.h>

#include "joint_handle_group.h"

class RescueHardware : public hardware_interface::RobotHW {
public:
    RescueHardware(ros::NodeHandle &n):
        joint_state_interface(),
        joint_pos_interface(),
        joint_vel_interface(),
        arm_handle(n, {"arm1", "arm2", "arm3"}, "arm"),
        wrist_handle(n,  {"wrist1", "wrist2", "wrist3"}, "wrist"),
        tracks_handle(n, {"track_left", "track_right"}, "tracks"),
        flippers_handle(n, {"flippers_rear", "flippers_front"}, "flippers")
    {
        // Position interfaces
        arm_handle.register_with_interface(
            joint_state_interface, joint_pos_interface);
        wrist_handle.register_with_interface(
            joint_state_interface, joint_pos_interface);

        // Velocity interfaces
        flippers_handle.register_with_interface(
            joint_state_interface, joint_vel_interface);
        tracks_handle.register_with_interface(
            joint_state_interface, joint_vel_interface);

        registerInterface(&joint_state_interface);
        registerInterface(&joint_pos_interface);
        registerInterface(&joint_vel_interface);
    }

    void read()
    {
        arm_handle.read();
        wrist_handle.read();
        tracks_handle.read();
        flippers_handle.read();
    }

    void write()
    {
        arm_handle.write();
        wrist_handle.write();
        tracks_handle.write();
        flippers_handle.write();
    }

private:
    hardware_interface::JointStateInterface joint_state_interface;
    hardware_interface::PositionJointInterface joint_pos_interface;
    hardware_interface::VelocityJointInterface joint_vel_interface;

    JointHandleGroup arm_handle;
    JointHandleGroup wrist_handle;
    JointHandleGroup tracks_handle;
    JointHandleGroup flippers_handle;
};


class Node {
public:
    Node(ros::NodeHandle& n): rescue_hw(n), cm(&rescue_hw, n)
    {
        loop_timer = n.createTimer(
            ros::Duration(1.0/20),
            &Node::loop,
            this
        );
    }

    void loop(const ros::TimerEvent &timer)
    {
        rescue_hw.read();
        cm.update(
            ros::Time::now(),
            ros::Duration(timer.current_real - timer.last_real)
        );
        rescue_hw.write();
    }
private:
    RescueHardware rescue_hw;
    controller_manager::ControllerManager cm;
    ros::Timer loop_timer;
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "control");
    ros::NodeHandle n;
    Node node(n);

    ros::AsyncSpinner spinner(4);
    spinner.start();
    ros::waitForShutdown();
    return 0;
}
