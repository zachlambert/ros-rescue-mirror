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

    void enable()
    {
        write1Byte(ADDR_TORQUE_ENABLE, 1);
    }

    void disable()
    {
        write1Byte(ADDR_TORQUE_ENABLE, 0);
    }

    double readPosition()
    {
        int32_t value;
        read4Byte(ADDR_PRESENT_POSITION, (uint32_t*)&value);
        return ((double)value / 4096) * 2*M_PI;
    }

    double readVelocity()
    {
        int32_t value;
        read4Byte(ADDR_PRESENT_VELOCITY, (uint32_t*)&value);
        return (double)value * 0.02398;
    }

    double readLoad()
    {
        int16_t value;
        read2Byte(ADDR_PRESENT_LOAD, (uint16_t*)&value);
        return (double)value * 0.1;
    }
};

class VelocityController: public BaseController {
    static constexpr uint32_t ADDR_GOAL_VELOCITY = 104;

public:
    VelocityController(
        CommHandler &commHandler,
        CommHandler::Protocol protocol,
        uint32_t id):
            BaseController(commHandler, protocol, id)
    {
        write1Byte(ADDR_OPERATING_MODE, 1);
    }

    void writeGoalVelocity(double velocity)
    {
        uint32_t value = velocity / 0.02398;
        write4Byte(ADDR_GOAL_VELOCITY, value);
    }
};

class PwmController: public BaseController {
    static constexpr uint32_t ADDR_GOAL_PWM = 100;

public:
    PwmController(
        CommHandler &commHandler,
        CommHandler::Protocol protocol,
        uint32_t id):
            BaseController(commHandler, protocol, id)
    {
        write1Byte(ADDR_OPERATING_MODE, 16);
    }

    void writeGoalPwm(double pwm)
    {
        // pwm: -1 -> 1
        // value: -885 -> 885 signed
        int16_t value = 885*pwm;
        write2Byte(100, pwm);
    }
};

class ExtendedPositionController: public BaseController {
    static constexpr uint32_t ADDR_GOAL_POSITION = 116;

public:
    ExtendedPositionController(
        CommHandler &commHandler,
        CommHandler::Protocol protocol,
        uint32_t id):
            BaseController(commHandler, protocol, id)
    {
        write1Byte(ADDR_OPERATING_MODE, 4);
        initial_pos = readPosition();
        std::cout << "Initial pos: " << initial_pos << std::endl;
    }

    void writeGoalPosition(double pos)
    {
        uint32_t value = 4096 * pos/(2*M_PI);
        write4Byte(ADDR_GOAL_POSITION, value);
    }

private:
    double initial_pos;
};

} // namespace xl430
} // namespace dxl

#endif
