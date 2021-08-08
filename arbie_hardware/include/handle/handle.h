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


class PosDummy: public Handle {
public:
    PosDummy(
        const std::string &name,
        Interfaces &interface,
        double write_delay_ms=0, double read_delay_ms=0):
            Handle(name, interface, Type::POS),
            write_delay_ms(write_delay_ms), read_delay_ms(read_delay_ms),
            fake_pos(0)
    {}

    void write(double cmd) {
        fake_pos = cmd;
        ros::Duration(write_delay_ms/1000).sleep();
    }
    void read(double &pos, double &vel, double &eff) {
        pos = fake_pos;
        vel = 0;
        eff = 0;
        ros::Duration(read_delay_ms/1000).sleep();
    }

private:
    double write_delay_ms, read_delay_ms;
    double fake_pos;
};

} // namespace handle

#endif
