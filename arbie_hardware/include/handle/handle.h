#ifndef HANDLE_HANDLE_H
#define HANDLE_HANDLE_H

#include <sstream>

#include "ros/ros.h"

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include "arbie_msgs/ReadHardware.h"
#include "arbie_msgs/WriteHardware.h"

namespace handle {

struct Interfaces {
    hardware_interface::JointStateInterface state;
    hardware_interface::PositionJointInterface pos;
    hardware_interface::VelocityJointInterface vel;
    hardware_interface::EffortJointInterface eff;
};

enum class Type {
    POS,
    VEL,
    EFF
};

class Handle {
public:

    Handle(const std::string &name, Interfaces &interface, Type type):
        pos(0), vel(0), eff(0), cmd(0)
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
        }
    }
    virtual ~Handle() {}
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

class Service: public Handle {
public:
    Service(
            const std::string &name,
            Interfaces &interface,
            Type handle_type,
            const std::string &service_name,
            ros::NodeHandle &n):
        Handle(name, interface, handle_type)
    {
        std::stringstream write_name(service_name);
        write_name << "/write";
        write_client = n.serviceClient<arbie_msgs::WriteHardware>(write_name.str());

        std::stringstream read_name(service_name);
        read_name << "/read";
        read_client = n.serviceClient<arbie_msgs::ReadHardware>(read_name.str());
    }

    void write(double cmd)
    {
        write_msg.request.cmd.data = cmd;
        write_client.call(write_msg);
    }

    void read(double &pos, double &vel, double &eff)
    {
        if(read_client.call(read_msg) && read_msg.response.success) {
            pos = read_msg.response.pos.data;
            vel = read_msg.response.vel.data;
            eff = read_msg.response.eff.data;
        }
    }

private:
    ros::ServiceClient write_client;
    arbie_msgs::WriteHardware write_msg;
    ros::ServiceClient read_client;
    arbie_msgs::ReadHardware read_msg;
};

} // namespace handle

#endif
