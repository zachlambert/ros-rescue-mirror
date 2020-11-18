#ifndef DXL_CONTROL_XL430
#define DXL_CONTROL_XL430

#include "dxl_control/base_controller.h"
#include <math.h>

class XL430Controller: public BaseController {
protected:
    static constexpr uint32_t ADDR_OPERATING_MODE = 11;
    static constexpr uint32_t ADDR_TORQUE_ENABLE = 64;
    static constexpr uint32_t ADDR_PRESENT_POSITION = 132;
    static constexpr uint32_t ADDR_PRESENT_VELOCITY = 128;
    // static constexpr uint32_t ADDR_GOAL_POSITION = 116;

public:
    XL430Controller(
        CommHandler &commHandler,
        CommHandler::Protocol protocol,
        uint32_t id):
            BaseController(commHandler, protocol, id) {}

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
        uint32_t value;
        read4Byte(ADDR_PRESENT_POSITION, &value);
        return ((double)value / 4096) * 2*M_PI;
    }

    double readVelocity()
    {
        uint32_t value;
        read4Byte(ADDR_PRESENT_VELOCITY, &value);
        return (double)value * 0.02398;
    }

};

class XL430VelocityController: public XL430Controller {
    static constexpr uint32_t ADDR_GOAL_VELOCITY = 104;

public:
    XL430VelocityController(
        CommHandler &commHandler,
        CommHandler::Protocol protocol,
        uint32_t id):
            XL430Controller(commHandler, protocol, id)
    {
        write1Byte(ADDR_OPERATING_MODE, 1);
    }

    void writeGoalVelocity(double velocity)
    {
        uint32_t value = velocity / 0.02398;
        write4Byte(ADDR_GOAL_VELOCITY, value);
    }
};

#endif
