
#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include "geometry_msgs/Pose.h"
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/collision_detection/collision_tools.h>
#include "tf2_eigen/tf2_eigen.h"
#include <Eigen/Geometry>

#include "inverse_kinematics/PoseToAngles.h"
#include "inverse_kinematics/CheckAngles.h"

class KinematicsHandler{
public:
    KinematicsHandler(ros::NodeHandle& n);
    ~KinematicsHandler(){}
    
    bool poseToAngles(inverse_kinematics::PoseToAngles::Request&,
                      inverse_kinematics::PoseToAngles::Response&);
    bool checkAngles(inverse_kinematics::CheckAngles::Request&,
                     inverse_kinematics::CheckAngles::Response&);

private:

    void loadAnglesMsg();
    bool validateState();
    bool checkCollision();

    std_msgs::Float32MultiArray angles_msg;

    robot_model_loader::RobotModelLoader robot_model_loader;
    robot_model::RobotModelPtr kinematic_model;
    robot_state::RobotStatePtr kinematic_state;
    robot_state::JointModelGroup* joint_model_group;

    collision_detection::CollisionRequest c_req;
    collision_detection::CollisionResult c_res;
    planning_scene::PlanningScene* g_planning_scene;
};
