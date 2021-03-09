#include "dxl/xl430.h"

namespace dxl {
namespace xl430 {

// ===== xl430::BaseController =====

BaseController::BaseController(
    CommHandler &commHandler,
    CommHandler::Protocol protocol,
    uint32_t id,
    bool write_tx_only):
        dxl::BaseController(commHandler, protocol, id),
        write_tx_only(write_tx_only)
{
    // Shutdown in the following situations:
    // - Bit 5 = Overload error (persistent loda > maximum)
    // - Bit 4 = Electrical shock error (shock on circuit or insufficient power)
    // - Bit 3 = Motor encoder error
    // - Bit 2 = Overheating error
    // - Bit 0 = Input voltage error
    // (ie: all the available flags)
    uint8_t flags = (1<<5) | (1<<4) | (1<<3) | (1<<2) | (1<<0);
    write1Byte(ADDR_SHUTDOWN, 52); // TEMP: Writing default value 52

    if (write_tx_only) {
        // Only return status packet on read and ping commands
        write1Byte(ADDR_STATUS_RETURN_LEVEL, 1);
    } else {
        // Always return status packet
        write1Byte(ADDR_STATUS_RETURN_LEVEL, 2);
        std::cout << "Writing status packet level 2" << std::endl;
    }
}

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

bool BaseController::readVelocity(double &result)
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
    uint32_t id,
    bool write_tx_only):
        BaseController(commHandler, protocol, id, write_tx_only)
{
    write1Byte(ADDR_OPERATING_MODE, 4);
}

bool ExtendedPositionController::writeGoalPosition(double pos)
{
    uint32_t value = 4096 * pos/(2*M_PI);
    return write4Byte(ADDR_GOAL_POSITION, value, write_tx_only);
}


// ===== xl430::VelocityController =====

VelocityController::VelocityController(
    CommHandler &commHandler,
    CommHandler::Protocol protocol,
    uint32_t id,
    bool write_tx_only):
        BaseController(commHandler, protocol, id, write_tx_only)
{
    write1Byte(ADDR_OPERATING_MODE, 1);
}

bool VelocityController::writeGoalVelocity(double velocity)
{
    uint32_t value = velocity / 0.02398;
    return write4Byte(ADDR_GOAL_VELOCITY, value, write_tx_only);
}


// ===== xl430::PwmController =====

PwmController::PwmController(
    CommHandler &commHandler,
    CommHandler::Protocol protocol,
    uint32_t id,
    bool write_tx_only):
        BaseController(commHandler, protocol, id, write_tx_only)
{
    write1Byte(ADDR_OPERATING_MODE, 16);
}

bool PwmController::writeGoalPwm(double pwm)
{
    // pwm: -1 -> 1
    // value: -885 -> 885 signed
    int16_t value = 885*pwm;
    return write2Byte(100, pwm, write_tx_only);
}

} // namespace xl430
} // namespace dxl
