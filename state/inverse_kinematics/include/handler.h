
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

class UpdateHandler{
public:
    UpdateHandler(ros::NodeHandle& n);
    ~UpdateHandler(){}
    void targetPoseCallback(const geometry_msgs::Pose::ConstPtr& target_pose);

private:

    bool checkCollision();
    void publishAngles(const std::vector<double>& joint_values);

    ros::Publisher arm_demand_pub;
    ros::Publisher wrist_demand_pub;
    ros::Subscriber target_pose_sub;

    std_msgs::Float32MultiArray arm_demand_msg;
    std_msgs::Float32MultiArray wrist_demand_msg;

    robot_model_loader::RobotModelLoader robot_model_loader;
    robot_model::RobotModelPtr kinematic_model;
    robot_state::RobotStatePtr kinematic_state;
    robot_state::JointModelGroup* joint_model_group;

    collision_detection::CollisionRequest c_req;
    collision_detection::CollisionResult c_res;
    planning_scene::PlanningScene* g_planning_scene;
};
