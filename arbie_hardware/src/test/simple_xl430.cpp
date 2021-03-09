#include <ros/ros.h>
#include <dynamixel_sdk/dynamixel_sdk.h>

void check_result(uint8_t dxl_comm_result, int dxl_error, dynamixel::PacketHandler *packet_handler)
{
    if (dxl_comm_result != COMM_SUCCESS) {
        std::cerr << packet_handler->getTxRxResult(dxl_comm_result) << std::endl;
    } else if (dxl_error != 0) {
        std::cerr << packet_handler->getRxPacketError(dxl_error) << std::endl;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_simple_xl430");
    ros::NodeHandle n;

    std::string port = "/dev/ttyUSB0";
    int baud_rate = 1000000;
    int id = 3;

    dynamixel::PortHandler *port_handler =
        dynamixel::PortHandler::getPortHandler(port.c_str());
    dynamixel::PacketHandler *packet_handler =
        dynamixel::PacketHandler::getPacketHandler(1);

    if (!port_handler->openPort()) {
        std::cerr << "Failed to connect to serial port" << std::endl;
        return 1;
    }
    if (!port_handler->setBaudRate(baud_rate)) {
        std::cerr << "Failed to set baud rate on serial port" << std::endl;
        return 1;
    }

    constexpr uint32_t ADDR_TORQUE_ENABLE = 64;

    uint8_t dxl_error = 0;
    int result = packet_handler->write1ByteTxRx(
        port_handler, id, ADDR_TORQUE_ENABLE, 1, &dxl_error);
    check_result(result, dxl_error, packet_handler);

    dxl_error = 0;
    result = packet_handler->write1ByteTxRx(
        port_handler, id, ADDR_TORQUE_ENABLE, 0, &dxl_error);
    check_result(result, dxl_error, packet_handler);

    return 0;
}
