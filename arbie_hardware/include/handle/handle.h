#ifndef HANDLE_HANDLE_H
#define HANDLE_HANDLE_H

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>

namespace handle {

struct Interfaces {
    hardware_interface::JointStateInterface state;
    hardware_interface::PositionJointInterface pos;
    hardware_interface::VelocityJointInterface vel;
    hardware_interface::EffortJointInterface eff;
};

class Handle {
protected:
    enum Type {
        POS,
        VEL,
        EFF
    };
public:
    Handle(const std::string &name, Interfaces &interface, Type type): pos(0), vel(0), eff(0), cmd(0)
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

} // namespace handle

#endif
