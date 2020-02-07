
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include "std_msgs/Float32MultiArray.h"
#include <controller_manager/controller_manager.h>


class RobotInterface : public hardware_interface::RobotHW
{
public:
  RobotInterface(ros::NodeHandle& n);
  // Read from arm_actual, wrist_actual, flipper_actual. Write to pos[8]
  void read(const double* arm_actual,
            const double* wrist_actual,
            const double* flipper_actual);
  // Write to arm_demand, wrist_demand. Read from cmd[6]
  void write(double* arm_demand,
             double* wrist_demand)const;

protected:

  hardware_interface::JointStateInterface jnt_state_interface;
  hardware_interface::PositionJointInterface jnt_pos_interface;
  double cmd[6];
  double pos[8];
  double vel[8];
  double eff[8];
};
