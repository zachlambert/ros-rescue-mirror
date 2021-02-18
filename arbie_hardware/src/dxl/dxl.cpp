#include "dxl/dxl.h"

namespace dxl {

// ===== CommHandler =====

CommHandler::CommHandler(const std::string &port, int baud_rate):
    baud_rate(baud_rate)
{
    portHandler = dynamixel::PortHandler::getPortHandler(port.c_str());
    packetHandler1 = dynamixel::PacketHandler::getPacketHandler(1);
    packetHandler2 = dynamixel::PacketHandler::getPacketHandler(2);
}

bool CommHandler::connect()
{
    if (!portHandler->openPort()) {
        std::cerr << "Failed to connect to serial port for dynamixels" << std::endl;
        return false;
    }
    if (!portHandler->setBaudRate(baud_rate)) {
        std::cerr << "Failed to set baud rate on serial port for dynamixels" << std::endl;
        return false;
    }
    return true;
}


// ===== BaseController =====

bool BaseController::write1Byte(uint32_t addr, uint8_t value)
{
    uint8_t dxl_error = 0;
    return checkResult(
        packetHandler->write1ByteTxRx(portHandler, id, addr, value, &dxl_error),
        dxl_error);
}

bool BaseController::write2Byte(uint32_t addr, uint16_t value)
{
    uint8_t dxl_error = 0;
    return checkResult(
        packetHandler->write2ByteTxRx(portHandler, id, addr, value, &dxl_error),
        dxl_error);
}

bool BaseController::write4Byte(uint32_t addr, uint32_t value)
{
    uint8_t dxl_error = 0;
    return checkResult(
        packetHandler->write4ByteTxRx(portHandler, id, addr, value, &dxl_error),
        dxl_error);
}

bool BaseController::read1Byte(uint32_t addr, uint8_t *output)
{
    uint8_t dxl_error = 0;
    return checkResult(
        packetHandler->read1ByteTxRx(portHandler, id, addr, output, &dxl_error),
        dxl_error);
}

bool BaseController::read2Byte(uint32_t addr, uint16_t *output)
{
    uint8_t dxl_error = 0;
    return checkResult(
        packetHandler->read2ByteTxRx(portHandler, id, addr, output, &dxl_error),
        dxl_error);
}

bool BaseController::read4Byte(uint32_t addr, uint32_t *output)
{
    uint8_t dxl_error = 0;
    return checkResult(
        packetHandler->read4ByteTxRx(portHandler, id, addr, output, &dxl_error),
        dxl_error);
}

bool BaseController::checkResult(int dxl_comm_result, uint8_t dxl_error)
{
    if (dxl_comm_result != COMM_SUCCESS) {
        std::cerr << packetHandler->getTxRxResult(dxl_comm_result) << std::endl;
        return false;
    } else if (dxl_error != 0) {
        std::cerr << packetHandler->getRxPacketError(dxl_error) << std::endl;
        return false;
    }
    return true;
}

} // namespace dxl
