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
    struct Config {
        double scale;
        double eff2_threshold;
        double zero_pos;
        double max_pos_change;
        Config():
            scale(1), eff2_threshold(2), zero_pos(0),
            max_pos_change(0.3) {}
            // 0.3 rad ~= 17 deg, somewhat arbitrary
    };

    Position(
        const std::string &name,
        Interfaces &interface,
        dxl::xl430::ExtendedPositionController controller,
        Config config = Config());
    ~Position();

    void write(double cmd);
    void read(double &pos, double &vel, double &eff);
    void move(double change, double speed, double dt=0.01);
    void calibrate(double cmd_vel=-0.04);
    void set_as_origin();
    bool is_connected()const { return connected; }

private:
    bool connected;
    dxl::xl430::ExtendedPositionController controller;
    Config config;
    double origin;
};


class Velocity: public Handle {
public:
    struct Config {
        double scale;
        double eff2_threshold;
        double zero_pos;
        Config(): scale(1), eff2_threshold(2), zero_pos(0) {}
    };

    Velocity(
        const std::string &name,
        Interfaces &interface,
        dxl::xl430::VelocityController controller,
        Config config = Config());
    ~Velocity();

    void write(double cmd);
    void read(double &pos, double &vel, double &eff);
    void calibrate(double cmd_vel=-0.04);
    bool is_connected()const { return connected; }

private:
    bool connected;
    dxl::xl430::VelocityController controller;
    Config config;
    double origin;
};


} // namespace xl430

namespace ax12a {

class Position: public Handle {
public:
    struct Config {
        double max_pos_change;
        Config(): max_pos_change(0.3) {}
    };
    Position(
        const std::string &name,
        Interfaces &interface,
        dxl::ax12a::JointController controller,
        Config config = Config());
    ~Position();
    void write(double cmd);
    void read(double &pos, double &vel, double &eff);
    void move(double change, double speed, double dt=0.01);
    bool is_connected()const { return connected; }

private:
    bool connected;
    dxl::ax12a::JointController controller;
    Config config;
};

class PositionPair: public Handle {
public:
    struct Config {
        double origin1;
        double origin2;
        double scale1;
        double scale2;
        double pos_diff_allowance;
        bool enforce_matching;
        double max_pos_change;
        Config():
            origin1(0), origin2(0), scale1(1), scale2(1),
            pos_diff_allowance(0.15),
            enforce_matching(true),
            max_pos_change(0.3)
        {}
    };

    PositionPair(
        const std::string &name,
        Interfaces &interface,
        dxl::ax12a::JointController controller1,
        dxl::ax12a::JointController controller2,
        Config config = Config());
    ~PositionPair();
    void write(double cmd);
    void read(double &pos, double &vel, double &eff);
    void move(double change, double speed, double dt=0.01);
    bool is_connected()const { return connected; }

private:
    bool connected;
    dxl::ax12a::JointController controller1, controller2;
    Config config;
    bool disabled;
};

} // namespace ax12a

} // namespace handle

#endif
