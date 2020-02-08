
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include "std_msgs/Float32MultiArray.h"
#include <controller_manager/controller_manager.h>


class RobotInterface : public hardware_interface::RobotHW
{
public:
  RobotInterface(ros::NodeHandle& n);
  void update();

  double cmd[6];

protected:

  hardware_interface::JointStateInterface jnt_state_interface;
  hardware_interface::PositionJointInterface jnt_pos_interface;
  double pos[8];
  double vel[8];
  double eff[8];
};
