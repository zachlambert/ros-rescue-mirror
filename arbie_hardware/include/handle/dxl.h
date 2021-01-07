#ifndef HANDLE_DXL_H
#define HANDLE_DXL_H

#include "handle/handle.h"

#include "dxl/xl430.h"
#include "dxl/ax12a.h"

#include <numeric>

namespace handle {

namespace xl430 {

class Velocity: public Handle {
public:
    Velocity(
        const std::string &name,
        Interfaces &interface,
        dxl::xl430::VelocityController controller,
        double scale=1,
        double eff2_threshold=2,
        double zero_pos=0):
            Handle(name, interface, Type::VEL),
            controller(controller),
            scale(scale),
            eff2_threshold(eff2_threshold),
            zero_pos(zero_pos){

        controller.disable(); // If already enabled
        controller.enable();
        controller.readPosition(origin);
    }
    ~Velocity(){
        controller.disable();
    }

    void calibrate() {
        ROS_INFO("CALIBRATING");
        write(-0.04);
        ros::Duration(0.25).sleep();
        static constexpr std::size_t N = 10;
        std::array<double, N> eff2;
        double result;

        for (std::size_t i = 0; i < N; i++) {
            controller.readLoad(eff2[i]);
            eff2[i] = pow(result, 2);
            ros::Duration(0.1).sleep();
        }
        double mean_eff2;
        std::size_t i = 0;
        do {
            controller.readLoad(result);
            eff2[i] = pow(result, 2);
            i = (i+1)%N;
            mean_eff2 = std::accumulate(eff2.begin(), eff2.end(), 0.0f)/N;
            ROS_INFO("Load2: %f", mean_eff2);
            ros::Duration(0.1).sleep();
        } while(mean_eff2 < eff2_threshold);
        write(0);
        controller.readPosition(origin);
    }

    void write(double cmd) {
        controller.writeGoalVelocity(cmd*scale);
    }
    void read(double &pos, double &vel, double &eff) {
        // The controller reads back the position, velocity, effort
        // of the motor.
        // However, if there is a gearbox, this needs to be converted
        // to the actual joint position and velocity.

        double pos_reading;
        controller.readPosition(pos_reading);
        pos = zero_pos + (pos_reading - origin)/scale;

        double vel_reading;
        controller.readVelocity(vel_reading);
        vel = vel_reading/scale;

        controller.readLoad(eff); // Don't convert, just use as is
    }
private:
    dxl::xl430::VelocityController controller;
    double origin, zero_pos, scale;
    double eff2_threshold;
};

class Effort: public Handle {
public:
    Effort(
        const std::string &name,
        Interfaces &interface,
        dxl::xl430::PwmController controller):
            Handle(name, interface, Type::EFF),
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
        // TODO: Currently, this handle doesn't take into account
        // the motor->joint transmission.
        // Currently this handle isn't being used, but add in the
        // gear ratio stuff if it is needed.
        pos = controller.readPosition(pos);
        vel = controller.readVelocity(vel);
        eff = controller.readLoad(eff);
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
            Handle(name, interface, Type::POS),
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
        controller.readPosition(pos);
        controller.readVelocity(vel);
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
            Handle(name, interface, Type::POS),
            controller1(controller1),
            controller2(controller2),
            origin1(origin1), origin2(origin2), scale1(scale1), scale2(scale2),
            disabled(false)
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
        if (!disabled) {
            controller1.writeGoalPosition(origin1 + cmd*scale1);
            controller2.writeGoalPosition(origin2 + cmd*scale2);
        }
    }
    void read(double &pos, double &vel, double &eff) {
        // With a PositionPair controller, both positions have
        // the same cmd, pos, vel
        // The scale and origin determine how these correspond
        // to the actual cmd, pos, vel of both joints

        // Assume positions and velocities are consistent
        // so only read one joint
        double pos_reading;
        controller1.readPosition(pos_reading);
        pos = (pos_reading - origin1)/scale1;

        double vel_reading;
        controller1.readVelocity(vel_reading);
        vel = vel_reading/scale1;

        // If positions not consistent, disable controller
        double pos2_reading, pos2;
        controller2.readPosition(pos2_reading);
        pos2 = (pos2_reading - origin2)/scale2;

        if (fabs(pos2 - pos) > pos_diff_allowance) {
            // controller1.disable();
            // controller2.disable();
            // disabled = true;
            ROS_ERROR("Positions don't match: %f, %f", pos, pos2);
        }
        // eff not implemented
    }
private:
    dxl::ax12a::JointController controller1, controller2;
    static constexpr double pos_diff_allowance = 0.15;
    double origin1, origin2, scale1, scale2;
    bool disabled;
};

} // namespace ax12a

} // namespace handle

#endif
