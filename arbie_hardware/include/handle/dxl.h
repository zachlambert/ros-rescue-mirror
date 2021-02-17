#ifndef HANDLE_DXL_H
#define HANDLE_DXL_H

#include "handle/handle.h"

#include "dxl/xl430.h"
#include "dxl/ax12a.h"

#include <numeric>

namespace handle {

/* For the xl430 controllers, since it needs to support gear reductions, the
 * handles need calibration.
 * With all controllers type (except non-extended position controller), the
 * position reading covers a large number of revolutions.
 *
 * Define an "origin" for each joint, which defines the value of the
 * motor angle, for a joint angle = 0.
 * - zero_pos = Joint position at the origin
 * - origin = Motor position at the origin
 * - scale = scaling between (joint_pos - zero_pos) and (motor_pos - origin)
 * Such that:
 * - motor_pos = origin + scale*(joint_pos - zero_pos)
 * - joint_pos = zero_pos + (motor_pos - origin)/scale
 * - motor_vel = scale*joint_vel
 * - joint_vel = motor_vel/scale
 *
 * The calibration process for a joint moves the arm to the origin,
 * where it sets the origin (by reading the motor position), and knows
 * what the joint position should be
 */

namespace xl430 {

class Position: public Handle {
public:
    Position(
        const std::string &name,
        Interfaces &interface,
        dxl::xl430::ExtendedPositionController controller,
        double scale=1,
        double eff2_threshold=2,
        double zero_pos=0):
            Handle(name, interface, Type::POS),
            controller(controller),
            origin(0),
            scale(scale),
            zero_pos(zero_pos),
            eff2_threshold(eff2_threshold)
    {
        connected = controller.disable(); // If already enabled
        if (connected) {
            controller.enable();
            controller.readPosition(origin);
        } else {
            std::cerr << "Joint " << name << " not connected." << std::endl;
        }
    }

    ~Position()
    {
        controller.disable();
    }

    void calibrate() {
        if (!connected) return;

        // Copied calibration method for the velocity controller, but
        // replaced command velocity with equivalent increases in command
        // position.
        // Should work in principle but haven't tested. Having steps in
        // command position instead of a continuous velocity might make affect
        // the load measurements, in which decreasing step time might help

        ROS_INFO("CALIBRATING");

        double cmd_vel = -0.04;

        static constexpr std::size_t N = 10;
        std::array<double, N> eff2;
        double result;
        for (std::size_t i = 0; i < N; i++) {
            controller.readLoad(eff2[i]);
            eff2[i] = pow(result, 2);
            move(cmd_vel*0.1);
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
            move(cmd_vel*0.1);
            ros::Duration(0.1).sleep();
        } while(mean_eff2 < eff2_threshold);

        // Now read the current motor angle into the origin variable.
        // Now, a cmd_pos = zero_pos, will give a motor_pos = origin
        controller.readPosition(origin);
    }

    // Used for calibration
    void move(double cmd_increment)
    {
        if (!connected) return;
        double pos;
        controller.readPosition(pos);
        pos += cmd_increment*scale;
        controller.writeGoalPosition(pos);
    }

    void write(double cmd)
    {
        if (!connected) return;
        controller.writeGoalPosition(origin + scale*(cmd-zero_pos));
    }

    void read(double &pos, double &vel, double &eff)
    {
        if (!connected) return;

        double pos_reading;
        controller.readPosition(pos_reading);
        pos = zero_pos + (pos_reading - origin)/scale;

        double vel_reading;
        controller.readVelocity(vel_reading);
        vel = vel_reading/scale;

        controller.readLoad(eff); // Don't convert, just use as is
    }
    bool is_connected()const { return connected; }
private:
    bool connected;
    dxl::xl430::ExtendedPositionController controller;
    double origin, zero_pos, scale;
    double eff2_threshold;
};

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
            origin(0),
            scale(scale),
            zero_pos(zero_pos),
            eff2_threshold(eff2_threshold)
    {
        connected = controller.disable(); // If already enabled
        if (connected) {
            controller.enable();
            controller.readPosition(origin);
        } else {
            std::cerr << "Joint " << name << " not connected." << std::endl;
        }
    }

    ~Velocity()
    {
        controller.disable();
    }

    void calibrate() {
        if (!connected) return;

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

    void write(double cmd)
    {
        if (!connected) return;
        controller.writeGoalVelocity(cmd*scale);
    }

    void read(double &pos, double &vel, double &eff)
    {
        if (!connected) return;

        double pos_reading;
        controller.readPosition(pos_reading);
        pos = zero_pos + (pos_reading - origin)/scale;

        double vel_reading;
        controller.readVelocity(vel_reading);
        vel = vel_reading/scale;

        controller.readLoad(eff); // Don't convert, just use as is
    }
    bool is_connected()const { return connected; }
private:
    bool connected;
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
        connected = controller.disable(); // If already enabled
        if (connected) {
            controller.enable();
        } else {
            std::cerr << "Joint " << name << " not connected." << std::endl;
        }
    }

    ~Effort(){
        controller.disable();
    }

    void write(double cmd) {
        if (!connected) return;
        controller.writeGoalPwm(cmd);
    }

    void read(double &pos, double &vel, double &eff) {
        if (!connected) return;
        // TODO: Currently, this handle doesn't take into account
        // the motor->joint transmission.
        // Currently this handle isn't being used, but add in the
        // gear ratio stuff if it is needed.
        pos = controller.readPosition(pos);
        vel = controller.readVelocity(vel);
        eff = controller.readLoad(eff);
    }
    bool is_connected()const { return connected; }
private:
    bool connected;
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
        connected = controller.disable(); // If already enabled
        if (connected) {
            controller.writeComplianceSlope(128);
            controller.enable();
        } else {
            std::cerr << "Joint " << name << " not connected." << std::endl;
        }
    }
    ~Position() {
        controller.disable();
    }
    void write(double cmd) {
        if (!connected) return;
        controller.writeGoalPosition(cmd);
    }
    void read(double &pos, double &vel, double &eff) {
        if (!connected) return;
        controller.readPosition(pos);
        controller.readVelocity(vel);
        // eff not implemented
    }
    bool is_connected()const { return connected; }
private:
    bool connected;
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
        connected = controller1.disable();
        connected &= controller2.disable();
        if (connected) {
        controller1.enable();
        controller2.enable();
        } else {
            std::cerr << "Joint " << name << " not connected." << std::endl;
        }
    }

    ~PositionPair()
    {
        controller1.disable();
        controller2.disable();
    }

    void write(double cmd)
    {
        if (!connected || disabled) return;
        controller1.writeGoalPosition(origin1 + cmd*scale1);
        controller2.writeGoalPosition(origin2 + cmd*scale2);
    }

    void read(double &pos, double &vel, double &eff)
    {
        if (!connected) return;
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
    bool is_connected()const { return connected; }
private:
    bool connected;
    dxl::ax12a::JointController controller1, controller2;
    static constexpr double pos_diff_allowance = 0.15;
    double origin1, origin2, scale1, scale2;
    bool disabled;
};

} // namespace ax12a

} // namespace handle

#endif
