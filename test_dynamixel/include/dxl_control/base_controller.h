#ifndef DXL_CONTROL_BASE_CONTROLLER_H
#define DXL_CONTROL_BASE_CONTROLLER_H

#include <iostream>
#include <dynamixel_sdk/dynamixel_sdk.h>

class CommHandler {
public:
    CommHandler(const std::string &port, int default_baud_rate = 57600)
    {
        portHandler = dynamixel::PortHandler::getPortHandler(port.c_str());

        packetHandler1 = dynamixel::PacketHandler::getPacketHandler(1);
        packetHandler2 = dynamixel::PacketHandler::getPacketHandler(2);

        if (!portHandler->openPort()) {
            std::cerr << "Failed to open the port" << std::endl;
        }

        setBaudRate(default_baud_rate);
    }

    void setBaudRate(int baud_rate)
    {
        if (!portHandler->setBaudRate(baud_rate)) {
            std::cerr << "Failed to set baudrate" << std::endl;
        }
    }

    enum Protocol {
        PROTOCOL_1,
        PROTOCOL_2
    };

    dynamixel::PortHandler *getPortHandler(){ return portHandler; }

    dynamixel::PacketHandler *getPacketHandler(Protocol protocol){
        if (protocol == PROTOCOL_1) {
            return packetHandler1;
        } else {
            return packetHandler2;
        }
    }

private:
    dynamixel::PortHandler *portHandler;
    dynamixel::PacketHandler *packetHandler1;
    dynamixel::PacketHandler *packetHandler2;
};

class BaseController {
public:
    BaseController(
        CommHandler &commHandler,
        CommHandler::Protocol protocol,
        uint32_t id):
            id(id),
            portHandler(commHandler.getPortHandler()),
            packetHandler(commHandler.getPacketHandler(protocol)){}

    virtual ~BaseController() {}

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
            std::cerr << packetHandler->getTxRxResult(dxl_comm_result) << std::endl;
            return false;
        } else if (dxl_error != 0) {
            std::cerr << packetHandler->getRxPacketError(dxl_error) << std::endl;
            return false;
        }
        return true;
    }

    dynamixel::PortHandler *portHandler;
    dynamixel::PacketHandler *packetHandler;
};

#endif
