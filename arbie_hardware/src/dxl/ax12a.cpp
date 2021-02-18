#include "dxl/ax12a.h"

namespace dxl {
namespace ax12a {

JointController::JointController(
    CommHandler &commHandler,
    CommHandler::Protocol protocol,
    uint32_t id,
    Config config):
        BaseController(commHandler, protocol, id)
{
    // Ensure lower and upper angle limits are valid
    if (config.lower_angle_limit_degrees < -150)
        config.lower_angle_limit_degrees = -150;
    if (config.upper_angle_limit_degrees > 150)
        config.upper_angle_limit_degrees = 150;
    if (config.lower_angle_limit_degrees > config.upper_angle_limit_degrees) {
        config.lower_angle_limit_degrees = -150;
        config.upper_angle_limit_degrees = 150;
    }

    uint16_t cw_angle_limit_value = floor(
        (config.lower_angle_limit_degrees + 150)/300.1 * 1024
    );
    uint16_t ccw_angle_limit_value = floor(
        (config.upper_angle_limit_degrees + 150)/300.1 * 1024
    );
    write2Byte(ADDR_CW_ANGLE_LIMIT, cw_angle_limit_value);
    write2Byte(ADDR_CCW_ANGLE_LIMIT, ccw_angle_limit_value);

    writeComplianceSlope(config.compliance_slope);
}

bool JointController::writeGoalPosition(double angle)
{
    angle *= 180/M_PI;
    if (angle > 150) angle = 150;
    if (angle < -150) angle = -150;
    uint16_t value = floor((angle + 150)/300 * 1024);
    return write2Byte(ADDR_GOAL_POSITION, value);
}

bool JointController::readPosition(double &result)
{
    uint16_t value;
    if (read2Byte(ADDR_PRESENT_POSITION, &value)) {
        result = (((double)value - 512) / 1024) * 300 * M_PI/180;
        return true;
    } else {
        result = 0;
        return false;
    }
}

bool JointController::readVelocity(double &result)
{
    uint16_t value;
    if (read2Byte(ADDR_PRESENT_VELOCITY, &value)) {
        if (value < 1024) {
            result =  ((double)value / 1024) * 0.01162;
        } else {
            result = -((double)(value-1024) / 1024) * 0.01162;
        }
        return true;
    } else {
        return false;
    }
}

bool JointController::writeComplianceSlope(uint8_t slope)
{
    // Slope value must be even
    uint8_t value = slope - slope%2;
    if (!write1Byte(ADDR_CW_COMPLIANCE_SLOPE, value))
        return false;
    if (!write1Byte(ADDR_CCW_COMPLIANCE_SLOPE, value))
        return false;
    return true;
}

} // namespace ax12a
} // namespace dxl