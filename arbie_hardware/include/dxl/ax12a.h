#ifndef DXL_AX12A_H
#define DXL_AX12A_H

#include "dxl/dxl.h"
#include <math.h>

namespace dxl {

namespace ax12a {

class BaseController: public dxl::BaseController {
protected:
    static constexpr uint32_t ADDR_CW_ANGLE_LIMIT = 6;
    static constexpr uint32_t ADDR_CCW_ANGLE_LIMIT = 8;
    static constexpr uint32_t ADDR_TORQUE_ENABLE = 24;

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
};

class JointController: public BaseController{
    static constexpr uint32_t ADDR_CW_COMPLIANCE_SLOPE = 28;
    static constexpr uint32_t ADDR_CCW_COMPLIANCE_SLOPE = 29;
    static constexpr uint32_t ADDR_GOAL_POSITION = 30;
    static constexpr uint32_t ADDR_PRESENT_POSITION = 36;
    static constexpr uint32_t ADDR_PRESENT_VELOCITY = 38;
    static constexpr uint32_t ADDR_PRESENT_LOAD = 40;

public:
    JointController(
        CommHandler &commHandler,
        CommHandler::Protocol protocol,
        uint32_t id,
        double lower_angle_limit_degrees=-150,
        double upper_angle_limit_degrees=150):
            BaseController(commHandler, protocol, id)
    {
        if (lower_angle_limit_degrees < -150) lower_angle_limit_degrees = -150;
        if (upper_angle_limit_degrees > 150) upper_angle_limit_degrees = 150;
        if (lower_angle_limit_degrees > upper_angle_limit_degrees) {
            lower_angle_limit_degrees = -150;
            upper_angle_limit_degrees = 150;
        }

        uint16_t cw_angle_limit_value = floor((lower_angle_limit_degrees + 150)/300.1 * 1024);
        uint16_t ccw_angle_limit_value = floor((upper_angle_limit_degrees + 150)/300.1 * 1024);
        write2Byte(ADDR_CW_ANGLE_LIMIT, cw_angle_limit_value);
        write2Byte(ADDR_CCW_ANGLE_LIMIT, ccw_angle_limit_value);
    }

    bool writeGoalPosition(double angle)
    {
        // Should be given in radians, but convert to degrees here for simplicity
        angle *= 180/M_PI;
        if (angle > 150) angle = 150;
        if (angle < -150) angle = -150;
        uint16_t value = floor((angle + 150)/300 * 1024);
        return write2Byte(ADDR_GOAL_POSITION, value);
    }

    bool readPosition(double &result)
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

    bool readVelocity(double &result)
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

    bool writeComplianceSlope(uint8_t slope)
    {
        // Slope value must be even
        uint8_t value = slope - slope%2;
        if (!write1Byte(ADDR_CW_COMPLIANCE_SLOPE, value))
            return false;
        if (!write1Byte(ADDR_CCW_COMPLIANCE_SLOPE, value))
            return false;
        return true;
    }
};

} // namespace ax12a
} // namespace dxl

#endif
