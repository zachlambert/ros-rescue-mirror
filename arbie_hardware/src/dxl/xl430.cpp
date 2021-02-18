#include "dxl/xl430.h"

namespace dxl {
namespace xl430 {

// ===== xl430::BaseController =====

bool BaseController::readPosition(double &result)
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

double BaseController::readVelocity(double &result)
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

bool BaseController::readLoad(double &result)
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


// ===== xl430::ExtendedPositionController =====

ExtendedPositionController::ExtendedPositionController(
    CommHandler &commHandler,
    CommHandler::Protocol protocol,
    uint32_t id):
        BaseController(commHandler, protocol, id)
{
    write1Byte(ADDR_OPERATING_MODE, 4);
    // readPosition(initial_pos);
    // std::cout << "Initial pos: " << initial_pos << std::endl;
}

bool ExtendedPositionController::writeGoalPosition(double pos)
{
    uint32_t value = 4096 * pos/(2*M_PI);
    return write4Byte(ADDR_GOAL_POSITION, value);
}


// ===== xl430::VelocityController =====

VelocityController::VelocityController(
    CommHandler &commHandler,
    CommHandler::Protocol protocol,
    uint32_t id):
        BaseController(commHandler, protocol, id)
{
    write1Byte(ADDR_OPERATING_MODE, 1);
}

bool VelocityController::writeGoalVelocity(double velocity)
{
    uint32_t value = velocity / 0.02398;
    return write4Byte(ADDR_GOAL_VELOCITY, value);
}


// ===== xl430::PwmController =====

PwmController::PwmController(
    CommHandler &commHandler,
    CommHandler::Protocol protocol,
    uint32_t id):
        BaseController(commHandler, protocol, id)
{
    write1Byte(ADDR_OPERATING_MODE, 16);
}

bool PwmController::writeGoalPwm(double pwm)
{
    // pwm: -1 -> 1
    // value: -885 -> 885 signed
    int16_t value = 885*pwm;
    return write2Byte(100, pwm);
}

} // namespace xl430
} // namespace dxl
