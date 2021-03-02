#include "handle/handle.h"

namespace handle {


// ===== Handle =====

Handle::Handle(const std::string &name, Interfaces &interface, Type type):
    name(name), pos(0), vel(0), eff(0), cmd(0)
{
    interface.state.registerHandle(hardware_interface::JointStateHandle(
            name, &pos, &vel, &eff
    ));
    switch (type) {
        case Type::POS:
            interface.pos.registerHandle(hardware_interface::JointHandle(
                    interface.state.getHandle(name), &cmd
            ));
            break;
        case Type::VEL:
            interface.vel.registerHandle(hardware_interface::JointHandle(
                    interface.state.getHandle(name), &cmd
            ));
            break;
        case Type::EFF:
            interface.eff.registerHandle(hardware_interface::JointHandle(
                    interface.state.getHandle(name), &cmd
            ));
            break;
    }
}


// ===== Service =====

Service::Service(
        const std::string &name,
        Interfaces &interface,
        Type handle_type,
        const std::string &service_name,
        ros::NodeHandle &n):
    Handle(name, interface, handle_type)
{
    std::stringstream write_name;
    write_name << service_name << "/write";
    write_client = n.serviceClient<arbie_msgs::WriteHardware>(write_name.str());

    std::stringstream read_name;
    read_name << service_name << "/read";
    read_client = n.serviceClient<arbie_msgs::ReadHardware>(read_name.str());
}

void Service::write(double cmd)
{
    write_msg.request.cmd.data = cmd;
    write_client.call(write_msg);
}

void Service::read(double &pos, double &vel, double &eff)
{
    if(read_client.call(read_msg) && read_msg.response.success) {
        pos = read_msg.response.pos.data;
        vel = read_msg.response.vel.data;
        eff = read_msg.response.eff.data;
    }
}

} // namespace handle
