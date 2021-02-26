#ifndef DXL_XL430_H
#define DXL_XL430_H

#include "dxl/dxl.h"
#include <math.h>

namespace dxl {
namespace xl430 {


class BaseController: public dxl::BaseController {
protected:
    static constexpr uint32_t ADDR_OPERATING_MODE = 11;
    static constexpr uint32_t ADDR_TORQUE_ENABLE = 64;
    static constexpr uint32_t ADDR_PRESENT_POSITION = 132;
    static constexpr uint32_t ADDR_PRESENT_VELOCITY = 128;
    static constexpr uint32_t ADDR_PRESENT_LOAD = 126;

public:
    BaseController(
        CommHandler &commHandler,
        CommHandler::Protocol protocol,
        uint32_t id):
            dxl::BaseController(commHandler, protocol, id) {}

    bool enable() {
        return write1Byte(ADDR_TORQUE_ENABLE, 1);
    }
    bool disable() {
        return write1Byte(ADDR_TORQUE_ENABLE, 0);
    }
    bool readPosition(double &result);
    double readVelocity(double &result);
    bool readLoad(double &result);
};


class ExtendedPositionController: public BaseController {
    static constexpr uint32_t ADDR_GOAL_POSITION = 116;
public:
    ExtendedPositionController(
        CommHandler &commHandler,
        CommHandler::Protocol protocol,
        uint32_t id);
    bool writeGoalPosition(double pos);
};


class VelocityController: public BaseController {
    static constexpr uint32_t ADDR_GOAL_VELOCITY = 104;
public:
    VelocityController(
        CommHandler &commHandler,
        CommHandler::Protocol protocol,
        uint32_t id);
    bool writeGoalVelocity(double velocity);
};


class PwmController: public BaseController {
    static constexpr uint32_t ADDR_GOAL_PWM = 100;
public:
    PwmController(
        CommHandler &commHandler,
        CommHandler::Protocol protocol,
        uint32_t id);
    bool writeGoalPwm(double pwm);
};

} // namespace xl430
} // namespace dxl

#endif
