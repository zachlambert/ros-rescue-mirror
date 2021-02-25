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

    Handle(const std::string &name, Interfaces &interface, Type type);
    virtual ~Handle() {}

    void write() {
        // write(cmd);
        std::cout << "CMD = " << cmd << std::endl;
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
        ros::NodeHandle &n);
    void write(double cmd);
    void read(double &pos, double &vel, double &eff);

private:
    ros::ServiceClient write_client;
    arbie_msgs::WriteHardware write_msg;
    ros::ServiceClient read_client;
    arbie_msgs::ReadHardware read_msg;
};


class PosDummy: public Handle {
public:
    PosDummy(const std::string &name, Interfaces &interface):
        Handle(name, interface, Type::POS)
    {}

    void write(double cmd) { 
        pos = cmd;
    }
    void read(double &pos, double &vel, double &eff) {
        pos = this->pos;
        vel = 0;
        eff = 0;
    }

private:
    double pos;
};


} // namespace handle

#endif
