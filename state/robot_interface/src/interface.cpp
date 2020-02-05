
#include "interface.h"
#include <ros/console.h>
#include "std_msgs/MultiArrayDimension.h"

RobotInterface::RobotInterface(ros::NodeHandle& n)
{ 
    // Register state interface: interface for reading arm angles
    
    hardware_interface::JointStateHandle state_handle_1("arm1", &pos[0], &vel[0], &eff[0]);
    hardware_interface::JointStateHandle state_handle_2("arm2", &pos[1], &vel[1], &eff[1]);
    hardware_interface::JointStateHandle state_handle_3("arm3", &pos[2], &vel[2], &eff[2]);
    hardware_interface::JointStateHandle state_handle_4("wrist1", &pos[3], &vel[3], &eff[3]);
    hardware_interface::JointStateHandle state_handle_5("wrist2", &pos[4], &vel[4], &eff[4]);
    hardware_interface::JointStateHandle state_handle_6("wrist3", &pos[5], &vel[5], &eff[5]);

    jnt_state_interface.registerHandle(state_handle_1);
    jnt_state_interface.registerHandle(state_handle_2);
    jnt_state_interface.registerHandle(state_handle_3);
    jnt_state_interface.registerHandle(state_handle_4);
    jnt_state_interface.registerHandle(state_handle_5);
    jnt_state_interface.registerHandle(state_handle_6);

    registerInterface(&jnt_state_interface);

    // Register pos interface: interface for setting arm angles
    
    hardware_interface::JointHandle pos_handle_1(
        jnt_state_interface.getHandle("arm1"), &cmd[0]);
    hardware_interface::JointHandle pos_handle_2(
        jnt_state_interface.getHandle("arm2"), &cmd[1]);
    hardware_interface::JointHandle pos_handle_3(
        jnt_state_interface.getHandle("arm3"), &cmd[2]);
    hardware_interface::JointHandle pos_handle_4(
        jnt_state_interface.getHandle("wrist1"), &cmd[3]);
    hardware_interface::JointHandle pos_handle_5(
        jnt_state_interface.getHandle("wrist2"), &cmd[4]);
    hardware_interface::JointHandle pos_handle_6(
        jnt_state_interface.getHandle("wrist3"), &cmd[5]);
    
    jnt_pos_interface.registerHandle(pos_handle_1);
    jnt_pos_interface.registerHandle(pos_handle_2);
    jnt_pos_interface.registerHandle(pos_handle_3);
    jnt_pos_interface.registerHandle(pos_handle_4);
    jnt_pos_interface.registerHandle(pos_handle_5);
    jnt_pos_interface.registerHandle(pos_handle_6);

    registerInterface(&jnt_pos_interface);
}

void RobotInterface::read(const double* actual_angles)
{
   for(int i=0; i<6; i++){
       pos[i] = actual_angles[i];
   }
}

void RobotInterface::write(double* demand_angles)const
{
   for(int i=0; i<6; i++){
       demand_angles[i] = cmd[i];
   }
}
