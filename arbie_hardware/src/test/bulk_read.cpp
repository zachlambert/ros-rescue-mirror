#include <ros/ros.h>
#include <dynamixel_sdk/dynamixel_sdk.h>

namespace xl430 {
    static constexpr uint32_t ADDR_TORQUE_ENABLE = 64;
    static constexpr uint32_t ADDR_PRESENT_POSITION = 132;
    static constexpr uint32_t ADDR_PRESENT_VELOCITY = 128;
    static constexpr uint32_t ADDR_PRESENT_LOAD = 126;
}
namespace ax12a {
    constexpr uint32_t ADDR_TORQUE_ENABLE = 24;
    constexpr uint32_t ADDR_PRESENT_POSITION = 36;
    constexpr uint32_t ADDR_PRESENT_VELOCITY = 38;
    constexpr uint32_t ADDR_PRESENT_LOAD = 40;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "benchmark_ax12a");
    ros::NodeHandle n;
    std::string port = "/dev/ttyUSB0";
    int baud_rate = 1000000;
    std::vector<int> xl430_ids = { 1, 2, 3 };
    std::vector<int> ax12a_ids = { 4, 5, 6, 7, 8, 9 };

    dynamixel::PortHandler *port_handler =
        dynamixel::PortHandler::getPortHandler(port.c_str());;
    dynamixel::PacketHandler *packet_handler =
        dynamixel::PacketHandler::getPacketHandler(1);

    if (!port_handler->openPort()) {
        std::cerr << "Failed to connect to serial port" << std::endl;
        return 1;
    }
    if (!port_handler->setBaudRate(baud_rate)) {
        std::cerr << "Failed to set baud rate" << std::endl;
        return 1;
    }

    uint8_t error;
    for (std::size_t i = 0; i < xl430_ids.size(); i++) {
        packet_handler->write1ByteTxRx(
            port_handler, ax12a_ids[i], xl430::ADDR_TORQUE_ENABLE, 1, &error);
    }
    for (std::size_t i = 0; i < ax12a_ids.size(); i++) {
        packet_handler->write1ByteTxRx(
            port_handler, ax12a_ids[i], ax12a::ADDR_TORQUE_ENABLE, 1, &error);
    }

    // Create a bulk read object and load id/address/size elements
    dynamixel::GroupBulkRead bulk_read(port_handler, packet_handler);
    for (std::size_t i = 0; i < xl430_ids.size(); i++) {
        bulk_read.addParam(xl430_ids[i], xl430::ADDR_PRESENT_POSITION, 4);
        bulk_read.addParam(xl430_ids[i], xl430::ADDR_PRESENT_VELOCITY, 4);
        bulk_read.addParam(xl430_ids[i], xl430::ADDR_PRESENT_LOAD, 2);
    }
    for (std::size_t i = 0; i < ax12a_ids.size(); i++) {
        bulk_read.addParam(ax12a_ids[i], ax12a::ADDR_PRESENT_POSITION, 2);
        bulk_read.addParam(ax12a_ids[i], ax12a::ADDR_PRESENT_VELOCITY, 2);
        bulk_read.addParam(ax12a_ids[i], ax12a::ADDR_PRESENT_LOAD, 2);
    }

    int dxl_comm_result = bulk_read.txRxPacket();
    if (dxl_comm_result != COMM_SUCCESS) {
        std::cout << "Bulk read failed" << std::endl;
        std::cout << packet_handler->getTxRxResult(dxl_comm_result) << std::endl;
        return 1;
    }

    // Test a single bulk read and make sure the results seem sensible
    uint32_t data;
    for (std::size_t i = 0; i < xl430_ids.size(); i++) {
        data = bulk_read.getData(xl430_ids[i], xl430::ADDR_PRESENT_POSITION, 4);
        std::cout << "xl430 " << i << " position = " << data << std::endl;
        data = bulk_read.getData(xl430_ids[i], xl430::ADDR_PRESENT_VELOCITY, 4);
        std::cout << "xl430 " << i << " velocity = " << data << std::endl;
        data = bulk_read.getData(xl430_ids[i], xl430::ADDR_PRESENT_LOAD, 2);
        std::cout << "xl430 " << i << " load = " << data << std::endl;
    }
    for (std::size_t i = 0; i < ax12a_ids.size(); i++) {
        data = bulk_read.getData(ax12a_ids[i], ax12a::ADDR_PRESENT_POSITION, 2);
        std::cout << "ax12a " << i << " position = " << data << std::endl;
        data = bulk_read.getData(ax12a_ids[i], ax12a::ADDR_PRESENT_VELOCITY, 2);
        std::cout << "ax12a " << i << " velocity = " << data << std::endl;
        data = bulk_read.getData(ax12a_ids[i], ax12a::ADDR_PRESENT_LOAD, 2);
        std::cout << "ax12a " << i << " load = " << data << std::endl;
    }

    // Now time a number of bulk reads
    int num_reads = 3 * (xl430_ids.size() + ax12a_ids.size());
    constexpr std::size_t N = 100;
    ros::Time start_time = ros::Time::now();
    for (std::size_t i = 0; i < N; i++) {
        bulk_read.txRxPacket();
    }
    double time_ms = (ros::Time::now() - start_time).toSec() * 1e3;
    std::cout << "Average time for all reads = " << time_ms/N << " ms" << std::endl;
}
