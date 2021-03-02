#ifndef DXL_AX12A_H
#define DXL_AX12A_H

#include "dxl/dxl.h"
#include <math.h>

namespace dxl {
namespace ax12a {


class BaseController: public dxl::BaseController {
protected:
    // The operating mode (Joint vs Wheel) is determined
    // by the values of the angle limits.
    // If both = 0, (no limits), goes into wheel mode
    // If neither = 0 (both have valid limits), goes into joint mode.
    static constexpr uint32_t ADDR_CW_ANGLE_LIMIT = 6;
    static constexpr uint32_t ADDR_CCW_ANGLE_LIMIT = 8;
    static constexpr uint32_t ADDR_TORQUE_ENABLE = 24;

public:
    BaseController(
        CommHandler &commHandler,
        CommHandler::Protocol protocol,
        uint32_t id):
            dxl::BaseController(commHandler, protocol, id) {}

    bool enable() {
        return write1Byte(ADDR_TORQUE_ENABLE, 1);
    }
    bool disable(){
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
    static constexpr uint32_t ADDR_TORQUE_LIMIT = 34;
    static constexpr uint32_t ADDR_ALARM_LED = 18;
    static constexpr uint32_t ADDR_SHUTDOWN = 18;

public:
    struct Config {
        double lower_angle_limit_degrees;
        double upper_angle_limit_degrees;
        int compliance_slope;
        double torque_limit; // percentage of max
        Config():
            lower_angle_limit_degrees(-150),
            upper_angle_limit_degrees(150),
            compliance_slope(128),
            torque_limit(50)
        {}
    };

    JointController(
        CommHandler &commHandler,
        CommHandler::Protocol protocol,
        uint32_t id,
        Config config = Config());

    bool writeGoalPosition(double angle);
    bool readPosition(double &result);
    bool readVelocity(double &result);
    bool readLoad(double &result);

private:
    bool writeComplianceSlope(uint8_t slope);
    bool writeTorqueLimit(double percent);
};

} // namespace ax12a
} // namespace dxl

#endif
