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

} // namespace handle
