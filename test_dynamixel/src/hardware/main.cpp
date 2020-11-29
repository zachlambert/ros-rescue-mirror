#include <array>

#include "ros/ros.h"
#include "ros/callback_queue.h"
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <controller_manager/controller_manager.h>

#include "dxl/xl430.h"
#include "dxl/ax12a.h"

struct HardwareInterface {
    hardware_interface::JointStateInterface state;
    hardware_interface::PositionJointInterface pos;
    hardware_interface::VelocityJointInterface vel;
    hardware_interface::EffortJointInterface eff;
};

class HardwareHandle {
protected:
    enum Type {
        POS,
        VEL,
        EFF
    };
public:
    HardwareHandle(const std::string &name, HardwareInterface &interface, Type type): pos(0), vel(0), eff(0), cmd(0)
    {
        interface.state.registerHandle(hardware_interface::JointStateHandle(
                name, &pos, &vel, &eff
        ));
        switch (type) {
            case Type::POS:
                interface.pos.registerHandle(hardware_interface::JointHandle(
                        interface.state.getHandle(name), &cmd
                ));
                break;
            case Type::VEL:
                interface.vel.registerHandle(hardware_interface::JointHandle(
                        interface.state.getHandle(name), &cmd
                ));
                break;
            case Type::EFF:
                interface.eff.registerHandle(hardware_interface::JointHandle(
                        interface.state.getHandle(name), &cmd
                ));
                break;
        } }
    virtual ~HardwareHandle() {}
    void write() {
        write(cmd);
    }
    virtual void write(double cmd) = 0;

    void read() {
        read(pos, vel, cmd);
    }
    virtual void read(double &pos, double &vel, double &eff) = 0;
protected:
    double pos, vel, eff, cmd;
};

class HardwareHandle_Xl430: public HardwareHandle {
public:
    HardwareHandle_Xl430(
        const std::string &name,
        HardwareInterface &interface,
        dxl::xl430::VelocityController controller):
            HardwareHandle(name, interface, HardwareHandle::VEL),
            controller(controller) {
        controller.disable(); // If already enabled
        controller.enable();
    }
    ~HardwareHandle_Xl430(){
        controller.disable();
    }
    void write(double cmd) {
        controller.writeGoalVelocity(cmd);
    }
    void read(double &pos, double &vel, double &eff) {
        pos = controller.readPosition();
        vel = controller.readVelocity();
        eff = controller.readLoad();
    }
private:
    dxl::xl430::VelocityController controller;
};

class HardwareHandle_Ax12a: public HardwareHandle {
public:
    HardwareHandle_Ax12a(
        const std::string &name,
        HardwareInterface &interface,
        dxl::ax12a::JointController controller):
            HardwareHandle(name, interface, HardwareHandle::POS),
            controller(controller) {
        controller.disable(); // If already enabled
        controller.enable();
    }
    ~HardwareHandle_Ax12a() {
        controller.disable();
    }
    void write(double cmd) {
        controller.writeGoalPosition(cmd);
    }
    void read(double &pos, double &vel, double &eff) {
        pos = controller.readPosition();
        vel = controller.readVelocity();
        // eff not implemented
    }
private:
    dxl::ax12a::JointController controller;
};

class Hardware: public hardware_interface::RobotHW {
public:
    Hardware(ros::NodeHandle &n, std::string port):
        commHandler(port)
    {
        hardware.push_back(std::make_unique<HardwareHandle_Xl430>(
            "arm1",
            interface,
            dxl::xl430::VelocityController(commHandler, commHandler.PROTOCOL_1, 1))
        );
        hardware.push_back(std::make_unique<HardwareHandle_Xl430>(
            "arm2",
            interface,
            dxl::xl430::VelocityController(commHandler, commHandler.PROTOCOL_1, 2))
        );
        hardware.push_back(std::make_unique<HardwareHandle_Xl430>(
            "arm3",
            interface,
            dxl::xl430::VelocityController(commHandler, commHandler.PROTOCOL_1, 3))
        );
        hardware.push_back(std::make_unique<HardwareHandle_Ax12a>(
            "wrist_yaw",
            interface,
            dxl::ax12a::JointController(commHandler, commHandler.PROTOCOL_1, 6))
        );
        registerInterface(&interface.state);
        registerInterface(&interface.pos);
        registerInterface(&interface.vel);
        registerInterface(&interface.eff);
    }

    void read()
    {
        for (auto &hw: hardware) {
            hw->read();
        }
    }

    void write()
    {
        for (auto &hw: hardware) {
            hw->write();
        }
    }

private:
    dxl::CommHandler commHandler;
    HardwareInterface interface;
    std::vector<std::unique_ptr<HardwareHandle>> hardware;
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
