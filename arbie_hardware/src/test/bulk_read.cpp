#include <ros/ros.h>
#include <dynamixel_sdk/dynamixel_sdk.h>

namespace xl430 {
    constexpr uint32_t ADDR_TORQUE_ENABLE = 64;
    constexpr uint32_t ADDR_PRESENT_POSITION = 132;
    constexpr uint32_t ADDR_PRESENT_VELOCITY = 128;
    constexpr uint32_t ADDR_PRESENT_LOAD = 126;
    constexpr uint32_t BULK_ADDR = 126;
    constexpr int BULK_LEN = 2+4+4;
}
namespace ax12a {
    constexpr uint32_t ADDR_TORQUE_ENABLE = 24;
    constexpr uint32_t ADDR_PRESENT_POSITION = 36;
    constexpr uint32_t ADDR_PRESENT_VELOCITY = 38;
    constexpr uint32_t ADDR_PRESENT_LOAD = 40;
    constexpr uint32_t BULK_ADDR = 36;
    constexpr uint32_t BULK_LEN = 2+2+2;
}

bool check_result(int dxl_comm_result, uint8_t error, dynamixel::PacketHandler *packet_handler)
{
    if (dxl_comm_result != COMM_SUCCESS) {
        std::cerr << packet_handler->getTxRxResult(dxl_comm_result) << std::endl;
        return false;
    } else if (error != 0) {
        std::cerr << packet_handler->getRxPacketError(error) << std::endl;
        return false;
    }
    return true;
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

    std::cout << "Writing torque enable" << std::endl;

    int dxl_comm_result;
    uint8_t error;
    for (std::size_t i = 0; i < xl430_ids.size(); i++) {
        dxl_comm_result = packet_handler->write1ByteTxRx(
            port_handler, xl430_ids[i], xl430::ADDR_TORQUE_ENABLE, 1, &error);
        if (!check_result(dxl_comm_result, error, packet_handler)) return 1;
    }
    for (std::size_t i = 0; i < ax12a_ids.size(); i++) {
        dxl_comm_result = packet_handler->write1ByteTxRx(
            port_handler, ax12a_ids[i], ax12a::ADDR_TORQUE_ENABLE, 1, &error);
        if (!check_result(dxl_comm_result, error, packet_handler)) return 1;
    }

    std::cout << "Creating bulk read object" << std::endl;

    // Create a bulk read object and load id/address/size elements
    dynamixel::GroupBulkRead bulk_read(port_handler, packet_handler);
    for (std::size_t i = 0; i < xl430_ids.size(); i++) {
        if (!bulk_read.addParam(xl430_ids[i], xl430::BULK_ADDR, xl430::BULK_LEN)) {
            std::cerr << "Failed to add param" << std::endl;
        }
    }
    for (std::size_t i = 0; i < ax12a_ids.size(); i++) {
        if (!bulk_read.addParam(ax12a_ids[i], ax12a::BULK_ADDR, ax12a::BULK_LEN)) {
            std::cerr << "Failed to add param" << std::endl;
        }
    }

    std::cout << "Sending bulk read packet" << std::endl;

    dxl_comm_result = bulk_read.txRxPacket();
    if (!check_result(dxl_comm_result, error, packet_handler)) {
        std::cerr << "Bulk read failed" << std::endl;
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

    std::cout << "Timing bulk read" << std::endl;

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
