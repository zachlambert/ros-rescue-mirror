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
        write(cmd);
    }
    virtual void write(double cmd) = 0;

    void read() {
        read(pos, vel, cmd);
    }
    virtual void read(double &pos, double &vel, double &eff) = 0;
protected:
    // Used in write(double cmd) if the handle wants to verify that
    // there isn't a large jump between the current position and next command
    double get_current_pos()const { return pos; }
    // Used for logging
    const std::string &get_name()const { return name; }
private:
    std::string name;
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
        Handle(name, interface, Type::POS), fake_pos(0)
    {}

    void write(double cmd) { 
        fake_pos = cmd;
    }
    void read(double &pos, double &vel, double &eff) {
        pos = fake_pos;
        vel = 0;
        eff = 0;
    }

private:
    double fake_pos;
};

class VelDummy: public Handle {
public:
    VelDummy(const std::string &name, Interfaces &interface):
        Handle(name, interface, Type::VEL), fake_pos(0), fake_vel(0)
    {
        write_time = ros::Time::now();
    }

    void write(double cmd) { 
        fake_vel = cmd;
        fake_pos += fake_vel*(ros::Time::now() - write_time).toSec();
        write_time = ros::Time::now();
    }
    void read(double &pos, double &vel, double &eff) {
        pos = fake_pos;
        vel = fake_vel;
        eff = 0;
    }

private:
    double fake_vel, fake_pos;
    ros::Time write_time;
};



} // namespace handle

#endif
