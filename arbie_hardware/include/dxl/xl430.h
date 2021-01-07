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

    bool enable()
    {
        return write1Byte(ADDR_TORQUE_ENABLE, 1);
    }

    bool disable()
    {
        return write1Byte(ADDR_TORQUE_ENABLE, 0);
    }

    bool readPosition(double &result)
    {
        int32_t value;
        if (read4Byte(ADDR_PRESENT_POSITION, (uint32_t*)&value)) {
            result = ((double)value / 4096) * 2*M_PI;
            return true;
        } else {
            result = 0;
            return false;
        }
    }

    double readVelocity(double &result)
    {
        int32_t value;
        if (read4Byte(ADDR_PRESENT_VELOCITY, (uint32_t*)&value)) {
            result = (double)value * 0.02398;
            return true;
        } else {
            result = 0;
            return false;
        }
    }

    bool readLoad(double &result)
    {
        int16_t value;
        if (read2Byte(ADDR_PRESENT_LOAD, (uint16_t*)&value)) {
            result = (double)value * 0.1;
            return true;
        } else {
            result = 0;
            return false;
        }
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

    bool writeGoalVelocity(double velocity)
    {
        uint32_t value = velocity / 0.02398;
        return write4Byte(ADDR_GOAL_VELOCITY, value);
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

    bool writeGoalPwm(double pwm)
    {
        // pwm: -1 -> 1
        // value: -885 -> 885 signed
        int16_t value = 885*pwm;
        return write2Byte(100, pwm);
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
        readPosition(initial_pos);
        std::cout << "Initial pos: " << initial_pos << std::endl;
    }

    bool writeGoalPosition(double pos)
    {
        uint32_t value = 4096 * pos/(2*M_PI);
        return write4Byte(ADDR_GOAL_POSITION, value);
    }

private:
    double initial_pos;
};

} // namespace xl430
} // namespace dxl

#endif
