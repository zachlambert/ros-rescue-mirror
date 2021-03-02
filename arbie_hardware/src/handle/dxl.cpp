#include "handle/dxl.h"

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


// ===== xl430::Position =====

Position::Position(
    const std::string &name,
    Interfaces &interface,
    dxl::xl430::ExtendedPositionController controller,
    Config config):
        Handle(name, interface, Type::POS),
        controller(controller),
        config(config),
        origin(0)
{
    connected = controller.disable(); // If already enabled
    if (connected) {
        controller.enable();
        controller.readPosition(origin);
    } else {
        ROS_ERROR("%s: Controller not connected", get_name().c_str());
    }
}

Position::~Position()
{
    controller.disable();
}

void Position::calibrate(double cmd_vel) {
    if (!connected) return;
    ROS_INFO("%s: Calibrating XL430 position controller", get_name().c_str());

    // * important *
    // The write and read functions use the origin variable, which is
    // changed within this function.
    // If you read a position, change the origin, and write the same position,
    // it will cause a jump in position.
    // Therefore:
    //   Only change the origin variable right at the end

    double cmd, _v, _e; // Vel and eff readings not used
    read(cmd, _v, _e);

    // Use a moving average of N samples of effort squared
    // At dt=0.01, N=50, this is the average over 0.5 seconds
    static constexpr std::size_t N = 50;
    std::array<double, N> eff2;
    double result;
    double dt = 0.01;

    // Start moving, and get an initial set of measurements.
    // Don't measure the mean eff2 before moving, since the load experienced
    // while moving is different.
    for (std::size_t i = 0; i < N; i++) {
        controller.readLoad(eff2[i]);
        eff2[i] = pow(result, 2);
        cmd += cmd_vel * dt;
        write(cmd);
        ros::Duration(dt).sleep();
    }

    double mean_eff2;
    std::size_t i = 0;
    do {
        controller.readLoad(result);
        eff2[i] = pow(result, 2);
        i = (i+1)%N;
        mean_eff2 = std::accumulate(eff2.begin(), eff2.end(), 0.0f)/N;
        ROS_INFO("%s: Load^2: %f", get_name().c_str(), mean_eff2);
        cmd += cmd_vel * dt;
        write(cmd);
        ros::Duration(dt).sleep();
    } while(mean_eff2 < config.eff2_threshold);

    // Now read the current motor angle into the origin variable.
    // Now, a cmd_pos = zero_pos, will give a motor_pos = origin
    // eg: If zero_pos = 0, writing cmd_pos = 0 won't cause any change
    controller.readPosition(origin);
}

void Position::set_as_origin()
{
    controller.readPosition(origin);
}

void Position::move(double change, double speed, double dt)
{
    if (!connected) return;
    double cmd, final_cmd, _v, _e;
    read(cmd, _v, _e);
    final_cmd = cmd + change;
    const double T = change/speed;
    double t = 0;
    while (t < T) {
        cmd += speed * dt;
        write(cmd);
        t += dt;
        ros::Duration(dt).sleep();
    }
    write(final_cmd);
}

void Position::write(double cmd)
{
    if (!connected) return;
    // if (std::fabs(cmd - get_current_pos()) > config.max_pos_change) {
    //     ROS_ERROR("%s: Attempted to move from %f to %f, exceeds maximum change %f",
    //         get_name().c_str(), get_current_pos(), cmd, config.max_pos_change);
    //     return;
    // }
    controller.writeGoalPosition(origin + config.scale*(cmd-config.zero_pos));
}

void Position::read(double &pos, double &vel, double &eff)
{
    if (!connected) return;

    double pos_reading;
    controller.readPosition(pos_reading);
    pos = config.zero_pos + (pos_reading - origin)/config.scale;

    double vel_reading;
    controller.readVelocity(vel_reading);
    vel = vel_reading/config.scale;

    controller.readLoad(eff); // Don't convert, just use as is
}


// ===== xl430::Velocity =====

Velocity::Velocity(
    const std::string &name,
    Interfaces &interface,
    dxl::xl430::VelocityController controller,
    Config config):
        Handle(name, interface, Type::VEL),
        controller(controller),
        config(config),
        origin(0)
{
    connected = controller.disable(); // If already enabled
    if (connected) {
        controller.enable();
        controller.readPosition(origin);
    } else {
        ROS_ERROR("%s: Controller not connected", get_name().c_str());
    }
}

Velocity::~Velocity()
{
    controller.disable();
}

void Velocity::calibrate(double cmd_vel) {
    if (!connected) return;

    ROS_INFO("%s: Calibrating XL430 Velocity Controller", get_name().c_str());
    write(cmd_vel);
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
        ROS_INFO("Load^2: %f", mean_eff2);
        ros::Duration(0.1).sleep();
    } while(mean_eff2 < config.eff2_threshold);
    write(0);
    controller.readPosition(origin);
}

void Velocity::write(double cmd)
{
    if (!connected) return;
    controller.writeGoalVelocity(cmd*config.scale);
}

void Velocity::read(double &pos, double &vel, double &eff)
{
    if (!connected) return;

    double pos_reading;
    controller.readPosition(pos_reading);
    pos = config.zero_pos + (pos_reading - origin)/config.scale;

    double vel_reading;
    controller.readVelocity(vel_reading);
    vel = vel_reading/config.scale;

    controller.readLoad(eff); // Don't convert, just use as is
}

} // namespace xl430


namespace ax12a {

Position::Position(
    const std::string &name,
    Interfaces &interface,
    dxl::ax12a::JointController controller,
    Config config):
        Handle(name, interface, Type::POS),
        controller(controller),
        config(config)
{
    connected = controller.disable(); // If already enabled
    if (connected) {
        controller.enable();
    } else {
        ROS_ERROR("%s: Controller not connected", get_name().c_str());
    }
}
Position::~Position() {
    controller.disable();
}

void Position::write(double cmd) {
    if (!connected) return;
    // if (std::fabs(cmd - get_current_pos()) > config.max_pos_change) {
    //     ROS_ERROR("%s: Attempted to move from %f to %f, exceeds maximum change %f",
    //         get_name().c_str(), get_current_pos(), cmd, config.max_pos_change);
    //     return;
    // }
    controller.writeGoalPosition(cmd);
}

void Position::read(double &pos, double &vel, double &eff) {
    if (!connected) return;
    controller.readPosition(pos);
    controller.readVelocity(vel);
    controller.readLoad(eff);
}

void Position::move(double change, double speed, double dt)
{
    if (!connected) return;
    double cmd, final_cmd, _v, _e;
    read(cmd, _v, _e);
    final_cmd = cmd + change;
    const double T = change/speed;
    double t = 0;
    while (t < T) {
        cmd += speed * dt;
        write(cmd);
        t += dt;
        ros::Duration(dt).sleep();
    }
    write(final_cmd);
}


PositionPair::PositionPair(
    const std::string &name,
    Interfaces &interface,
    dxl::ax12a::JointController controller1,
    dxl::ax12a::JointController controller2,
    Config config):
        Handle(name, interface, Type::POS),
        controller1(controller1),
        controller2(controller2),
        config(config),
        disabled(false)
{
    connected = controller1.disable();
    connected &= controller2.disable();
    if (connected) {
        controller1.enable();
        controller2.enable();
    } else {
        ROS_ERROR("%s: Controller not connected", get_name().c_str());
    }
}

PositionPair::~PositionPair()
{
    controller1.disable();
    controller2.disable();
}

void PositionPair::write(double cmd)
{
    if (!connected || disabled) return;
    // if (std::fabs(cmd - get_current_pos()) > config.max_pos_change) {
    //     ROS_ERROR("%s: Attempted to move from %f to %f, exceeds maximum change %f",
    //         get_name().c_str(), get_current_pos(), cmd, config.max_pos_change);
    //     return;
    // }
    controller1.writeGoalPosition(config.origin1 + cmd*config.scale1);
    controller2.writeGoalPosition(config.origin2 + cmd*config.scale2);
}

void PositionPair::read(double &pos, double &vel, double &eff)
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
    pos = (pos_reading - config.origin1)/config.scale1;

    double vel_reading;
    controller1.readVelocity(vel_reading);
    vel = vel_reading/config.scale1;

    if (!config.enforce_matching) return;

    // If positions not consistent, disable controller
    double pos2_reading, pos2;
    controller2.readPosition(pos2_reading);
    pos2 = (pos2_reading - config.origin2)/config.scale2;

    if (fabs(pos2 - pos) > config.pos_diff_allowance) {
        controller1.disable();
        controller2.disable();
        disabled = true;
        ROS_ERROR("%s: Positions don't match: %f, %f", get_name().c_str(), pos, pos2);
    }
    // eff not implemented
}

// Used for calibration
void PositionPair::move(double change, double speed, double dt)
{
    if (!connected) return;
    double pos, vel, eff;
    read(pos, vel, eff);

    double cmd = pos;
    double final_cmd = pos + change;
    const double T = change/speed;
    double t = 0;
    while (t < T) {
        cmd += speed * dt;
        write(cmd);
        read(pos, vel, eff); // Make sure positions don't deviate
        t += dt;
        ros::Duration(dt).sleep();
    }
    write(final_cmd);
}

} // namespace ax12a

} // namespace handle
