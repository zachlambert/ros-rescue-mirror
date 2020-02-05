
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include "std_msgs/Float32MultiArray.h"
#include <controller_manager/controller_manager.h>


class RobotInterface : public hardware_interface::RobotHW
{
public:
  RobotInterface(ros::NodeHandle& n);
  // Read from actual_angles, write to pos[6]
  void read(const double* actual_angles);
  // Write to demand_angles, read from cmd[6]
  void write(double* demand_angles)const;

protected:

  hardware_interface::JointStateInterface jnt_state_interface;
  hardware_interface::PositionJointInterface jnt_pos_interface;
  double cmd[6];
  double pos[6];
  double vel[6];
  double eff[6];
};
