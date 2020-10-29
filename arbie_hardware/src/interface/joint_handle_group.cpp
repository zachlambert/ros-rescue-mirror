#include "joint_handle_group.h"
#include <sstream>


JointHandleGroup::JointHandleGroup(
    ros::NodeHandle &n,
    std::vector<std::string> joint_names,
    std::string component_name):
        N(joint_names.size()),
        joint_names(joint_names)
{
    pos = new double[N];
    vel = new double[N];
    eff = new double[N];
    cmd = new double[N];
    // Ensure they are all initialised to zero
    memset(pos, 0, N*sizeof(double));
    memset(vel, 0, N*sizeof(double));
    memset(eff, 0, N*sizeof(double));
    memset(cmd, 0, N*sizeof(double));

    std::stringstream read_service_name;
    read_service_name << "hardware/" << component_name << "/read";
    read_client = n.serviceClient<arbie_msgs::ReadHardware>(
            read_service_name.str()
    );

    write_msg.request.cmd.layout.dim.push_back(std_msgs::MultiArrayDimension());
    write_msg.request.cmd.layout.dim[0].size = N;
    write_msg.request.cmd.layout.dim[0].stride = 1;
    write_msg.request.cmd.layout.dim[0].label = "joint";
    write_msg.request.cmd.data.resize(N);

    std::stringstream write_service_name;
    write_service_name << "hardware/" << component_name << "/write";
    write_client = n.serviceClient<arbie_msgs::WriteHardware>(
        write_service_name.str()
    );
}

void JointHandleGroup::register_with_interface(
    hardware_interface::JointStateInterface &state_interface,
    hardware_interface::PositionJointInterface &pos_interface)
{
    for (std::size_t i = 0; i < N; i++) {
        state_interface.registerHandle(hardware_interface::JointStateHandle(
            joint_names[i], &pos[i], &vel[i], &eff[i]
        ));
        pos_interface.registerHandle(hardware_interface::JointHandle(
            state_interface.getHandle(joint_names[i]), &cmd[i]
        ));
    }
}

void JointHandleGroup::register_with_interface(
    hardware_interface::JointStateInterface &state_interface,
    hardware_interface::VelocityJointInterface &vel_interface)
{
    for (std::size_t i = 0; i < N; i++) {
        state_interface.registerHandle(hardware_interface::JointStateHandle(
            joint_names[i], &pos[i], &vel[i], &eff[i]
        ));
        vel_interface.registerHandle(hardware_interface::JointHandle(
            state_interface.getHandle(joint_names[i]), &cmd[i]
        ));
    }
}

void JointHandleGroup::read()
{
    if (read_client.call(read_msg) && read_msg.response.success) {
        copy(
            std::begin(read_msg.response.pos.data),
            std::end(read_msg.response.pos.data),
            pos
        );
        copy(
            std::begin(read_msg.response.vel.data),
            std::end(read_msg.response.vel.data),
            vel
        );
        copy(
            std::begin(read_msg.response.eff.data),
            std::end(read_msg.response.eff.data),
            eff
        );
    }
}

void JointHandleGroup::write()
{
    copy(
        cmd,
        cmd + N,
        std::begin(write_msg.request.cmd.data)
    );
    write_client.call(write_msg);
}
