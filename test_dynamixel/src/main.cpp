#include <ros/ros.h>
#include <dynamixel_sdk/dynamixel_sdk.h>

class Dynamixel {
public:
    Dynamixel(
        dynamixel::PortHandler *portHandler,
        dynamixel::PacketHandler *packetHandler,
        uint32_t id):
            id(id), portHandler(portHandler), packetHandler(packetHandler) {}

    virtual ~Dynamixel() {}

protected:
    bool write1Byte(uint32_t addr, uint8_t value)
    {
        uint8_t dxl_error = 0;
        return checkResult(
            packetHandler->write1ByteTxRx(portHandler, id, addr, value, &dxl_error),
            dxl_error);
    }
    bool write2Byte(uint32_t addr, uint16_t value)
    {
        uint8_t dxl_error = 0;
        return checkResult(
            packetHandler->write2ByteTxRx(portHandler, id, addr, value, &dxl_error),
            dxl_error);
    }
    bool write4Byte(uint32_t addr, uint32_t value)
    {
        uint8_t dxl_error = 0;
        return checkResult(
            packetHandler->write4ByteTxRx(portHandler, id, addr, value, &dxl_error),
            dxl_error);
    }
    bool read1Byte(uint32_t addr, uint8_t *output)
    {
        uint8_t dxl_error = 0;
        return checkResult(
            packetHandler->read1ByteTxRx(portHandler, id, addr, output, &dxl_error),
            dxl_error);
    }
    bool read2Byte(uint32_t addr, uint16_t *output)
    {
        uint8_t dxl_error = 0;
        return checkResult(
            packetHandler->read2ByteTxRx(portHandler, id, addr, output, &dxl_error),
            dxl_error);
    }
    bool read4Byte(uint32_t addr, uint32_t *output)
    {
        uint8_t dxl_error = 0;
        return checkResult(
            packetHandler->read4ByteTxRx(portHandler, id, addr, output, &dxl_error),
            dxl_error);
    }

private:
    uint32_t id;

    bool checkResult(int dxl_comm_result, uint8_t dxl_error)
    {
        if (dxl_comm_result != COMM_SUCCESS) {
            ROS_ERROR("%s", packetHandler->getTxRxResult(dxl_comm_result));
            return false;
        } else if (dxl_error != 0) {
            ROS_ERROR("%s", packetHandler->getRxPacketError(dxl_error));
            return false;
        }
        return true;
    }

    dynamixel::PortHandler *portHandler;
    dynamixel::PacketHandler *packetHandler;
};

class DynamixelXL430: public Dynamixel {
protected:
    static constexpr uint32_t ADDR_OPERATING_MODE = 11;
    static constexpr uint32_t ADDR_TORQUE_ENABLE = 64;
    static constexpr uint32_t ADDR_PRESENT_POSITION = 132;
    static constexpr uint32_t ADDR_PRESENT_VELOCITY = 128;
    // static constexpr uint32_t ADDR_GOAL_POSITION = 116;

public:
    DynamixelXL430(
        dynamixel::PortHandler *portHandler,
        dynamixel::PacketHandler *packetHandler,
        uint32_t id):
            Dynamixel(portHandler, packetHandler, id) {}

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

class DynamixelXL430Velocity: public DynamixelXL430 {
    static constexpr uint32_t ADDR_GOAL_VELOCITY = 104;

public:
    DynamixelXL430Velocity(
        dynamixel::PortHandler *portHandler,
        dynamixel::PacketHandler *packetHandler,
        uint32_t id):
            DynamixelXL430(portHandler, packetHandler, id)
    {
        write1Byte(ADDR_OPERATING_MODE, 1);
    }

    void writeGoalVelocity(double velocity)
    {
        uint32_t value = velocity / 0.02398;
        write4Byte(ADDR_GOAL_VELOCITY, value);
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "dynamixel_controller");
    ros::NodeHandle n;

    dynamixel::PortHandler *portHandler =
        dynamixel::PortHandler::getPortHandler("/dev/ttyUSB0");

    dynamixel::PacketHandler *packetHandler1 =
        dynamixel::PacketHandler::getPacketHandler(1);
    dynamixel::PacketHandler *packetHandler2 =
        dynamixel::PacketHandler::getPacketHandler(2);

    if (!portHandler->openPort()) {
        ROS_INFO("Failed to open the port");
        return 1;
    }

    if (!portHandler->setBaudRate(57600)) {
        ROS_INFO("Failed to set baudrate");
        return 1;
    }

    // TODO: Load this from yaml file
    std::map<std::string, std::map<std::string, uint32_t>> dynamixels;
    dynamixels["arm_1"]["ID"] = 1;
    dynamixels["arm_1"]["Operating_Mode"] = 1;
    dynamixels["arm_2"]["ID"] = 2;
    dynamixels["arm_2"]["Operating_Mode"] = 1;
    dynamixels["arm_3"]["ID"] = 3;
    dynamixels["arm_3"]["Operating_Mode"] = 1;
    // dynamixel_properties["wrist_tilt_right"].push_back({"ID", 1});
    // TODO

    DynamixelXL430Velocity d1(portHandler, packetHandler1, 1);

    d1.enable();
    d1.writeGoalVelocity(1);
    for (int i = 0; i < 10; i++) {
        ROS_INFO("Angle: %f", d1.readPosition());
        ros::Duration(0.2).sleep();
    }
    d1.writeGoalVelocity(0);
    d1.disable();

    return 0;
}
