#ifndef HANDLE_DXL_H
#define HANDLE_DXL_H

#include "handle/handle.h"

#include "dxl/xl430.h"
#include "dxl/ax12a.h"

namespace handle {

namespace xl430 {

class Velocity: public Handle {
public:
    Velocity(
        const std::string &name,
        Interfaces &interface,
        dxl::xl430::VelocityController controller):
            Handle(name, interface, Handle::VEL),
            controller(controller) {
        controller.disable(); // If already enabled
        controller.enable();
    }
    ~Velocity(){
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

class Effort: public Handle {
public:
    Effort(
        const std::string &name,
        Interfaces &interface,
        dxl::xl430::PwmController controller):
            Handle(name, interface, Handle::EFF),
            controller(controller)
    {
        controller.disable(); // If already enabled
        controller.enable();
    }
    ~Effort(){
        controller.disable();
    }
    void write(double cmd) {
        controller.writeGoalPwm(cmd);
    }
    void read(double &pos, double &vel, double &eff) {
        pos = controller.readPosition();
        vel = controller.readVelocity();
        eff = controller.readLoad();
    }
private:
    dxl::xl430::PwmController controller;
};

} // namespace xl430

namespace ax12a {

class Position: public Handle {
public:
    Position(
        const std::string &name,
        Interfaces &interface,
        dxl::ax12a::JointController controller):
            Handle(name, interface, Handle::POS),
            controller(controller)
    {
        controller.disable(); // If already enabled
        controller.enable();
    }
    ~Position() {
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

class PositionPair: public Handle {
public:
    PositionPair(
        const std::string &name,
        Interfaces &interface,
        dxl::ax12a::JointController controller1,
        dxl::ax12a::JointController controller2,
        double origin1=0, double origin2=0, double scale1=1, double scale2=1):
            Handle(name, interface, Handle::POS),
            controller1(controller1),
            controller2(controller2),
            origin1(origin1), origin2(origin2), scale1(scale1), scale2(scale2)
    {
        controller1.disable();
        controller2.disable();
        controller1.enable();
        controller2.enable();
    }
    ~PositionPair() {
        controller1.disable();
        controller2.disable();
    }
    void write(double cmd) {
        controller1.writeGoalPosition(origin1 + cmd*scale1);
        controller2.writeGoalPosition(origin2 + cmd*scale2);
    }
    void read(double &pos, double &vel, double &eff) {
        // Assume positions are consistent
        pos = (controller1.readPosition() - origin1)/scale1;
        vel = controller1.readVelocity()/scale1;
        // If not consistent, disable controller
        double pos2 = (controller2.readPosition() - origin2)/scale2;
        if (fabs(pos2 - pos) < pos_diff_allowance) {
            // controller1.disable();
            // controller2.disable();
            ROS_INFO("Pos1 = %f, pos2 = %f", pos, pos2);
        }
        // eff not implemented
    }
private:
    dxl::ax12a::JointController controller1, controller2;
    static constexpr double pos_diff_allowance = 0.05;
    double origin1, origin2, scale1, scale2;
};

} // namespace ax12a

} // namespace handle

#endif
