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
            controller(controller) {
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
            controller(controller) {
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

} // namespace ax12a

std::unique_ptr<Handle> make(const std::string &name, Interfaces &interfaces, dxl::xl430::VelocityController controller) {
    return std::make_unique<xl430::Velocity>(name, interfaces, controller);
}

std::unique_ptr<Handle> make(const std::string &name, Interfaces &interfaces, dxl::xl430::PwmController controller) {
    return std::make_unique<xl430::Effort>(name, interfaces, controller);
}

std::unique_ptr<Handle> make(const std::string &name, Interfaces &interfaces, dxl::ax12a::JointController controller) {
    return std::make_unique<ax12a::Position>(name, interfaces, controller);
}

} // namespace handle

#endif
