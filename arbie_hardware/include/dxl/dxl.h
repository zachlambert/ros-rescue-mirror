#ifndef DXL_DXL_H
#define DXL_DXL_H

#include <iostream>
#include <dynamixel_sdk/dynamixel_sdk.h>

namespace dxl {

class CommHandler {
public:
    enum Protocol {
        PROTOCOL_1,
        PROTOCOL_2
    };

    CommHandler(const std::string &port, int baud_rate = 1000000);
    bool connect();
    dynamixel::PortHandler *getPortHandler(){ return portHandler; }
    dynamixel::PacketHandler *getPacketHandler(Protocol protocol) {
        return (protocol == PROTOCOL_1 ? packetHandler1 : packetHandler2);
    }

private:
    int baud_rate;
    dynamixel::PortHandler *portHandler;
    dynamixel::PacketHandler *packetHandler1;
    dynamixel::PacketHandler *packetHandler2;
};

class BaseController {
public:
    BaseController(CommHandler &commHandler, CommHandler::Protocol protocol, uint32_t id):
        id(id),
        portHandler(commHandler.getPortHandler()),
        packetHandler(commHandler.getPacketHandler(protocol))
    {}
    virtual ~BaseController() {}

protected:
    bool write1Byte(uint32_t addr, uint8_t value, bool tx_only=false);
    bool write2Byte(uint32_t addr, uint16_t value, bool tx_only=false);
    bool write4Byte(uint32_t addr, uint32_t value, bool tx_only=false);
    bool read1Byte(uint32_t addr, uint8_t *output);
    bool read2Byte(uint32_t addr, uint16_t *output);
    bool read4Byte(uint32_t addr, uint32_t *output);

    uint32_t id;
private:
    bool checkResult(int dxl_comm_result, uint8_t dxl_error);

    dynamixel::PortHandler *portHandler;
    dynamixel::PacketHandler *packetHandler;
};

} // namespace dxl

#endif
