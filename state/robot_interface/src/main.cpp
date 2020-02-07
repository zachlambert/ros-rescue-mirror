#include "interface.h"

#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/MultiArrayDimension.h"
#include <controller_manager/controller_manager.h>
#include <ros/callback_queue.h>
#include <stdio.h>


// What does this node do?
// 1. Sets up functions for writing and reading actual joint angles
// 2. Creates an RobotInterface object, which:
//    - Creates a JointState interface, which reads joint angles and
//      publishes them to the JointState topic.
//    - Creates JointPosition interfaces, which writes the joint angles
// 3. Creates a control manager, which allows controllers to use this
//    RobotInterface to command the joints and read them.


class RobotHandler{
public:
    RobotHandler(ros::NodeHandle& n);
    void update(const ros::TimerEvent& timer);
    void arm_callback(const std_msgs::Float32MultiArray::ConstPtr& arm_read_msg);
    void wrist_callback(const std_msgs::Float32MultiArray::ConstPtr& wrist_read_msg);
    void flipper_callback(const std_msgs::Float32MultiArray::ConstPtr& flipper_read_msg);

private:
    RobotInterface robot;
    boost::shared_ptr<controller_manager::ControllerManager> cm;

    double arm_demand[3];
    double arm_actual[3];
    double wrist_demand[3];
    double wrist_actual[3];
    double flipper_actual[2];

    ros::Publisher arm_demand_pub;
    ros::Subscriber arm_actual_sub;
    std_msgs::Float32MultiArray arm_demand_msg;

    ros::Publisher wrist_demand_pub;
    ros::Subscriber wrist_actual_sub;
    std_msgs::Float32MultiArray wrist_demand_msg;

    ros::Subscriber flipper_actual_sub;

    ros::Duration elapsed_time;
};

RobotHandler::RobotHandler(ros::NodeHandle& n):
        robot(n), arm_demand_msg(), wrist_demand_msg(), elapsed_time(){
    cm.reset(new controller_manager::ControllerManager(&robot, n));

    arm_demand_pub = n.advertise<std_msgs::Float32MultiArray>(
        "/arm_demand_angles", 1);
    arm_actual_sub = n.subscribe(
        "/arm_demand_angles", 1000, &RobotHandler::arm_callback, this); 

    arm_demand_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
    arm_demand_msg.layout.dim[0].label = "joint";
    arm_demand_msg.layout.dim[0].size = 3;
    arm_demand_msg.layout.dim[0].stride = 1;

    wrist_demand_pub = n.advertise<std_msgs::Float32MultiArray>(
        "/wrist_demand_angles", 1);
    wrist_actual_sub = n.subscribe(
        "/wrist_demand_angles", 1000, &RobotHandler::wrist_callback, this); 

    wrist_demand_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
    wrist_demand_msg.layout.dim[0].label = "joint";
    wrist_demand_msg.layout.dim[0].size = 3;
    wrist_demand_msg.layout.dim[0].stride = 1;

    flipper_actual_sub = n.subscribe(
        "/flipper/positions", 1000, &RobotHandler::flipper_callback, this);
}

void RobotHandler::update(const ros::TimerEvent& timer){
    elapsed_time = ros::Duration(timer.current_real - timer.last_real);

    robot.read(arm_actual, wrist_actual, flipper_actual);
    cm->update(ros::Time::now(), elapsed_time);
    robot.write(arm_demand, wrist_demand);

    arm_demand_msg.data.clear();
    for(int i=0; i<3; i++){
        arm_demand_msg.data.push_back(arm_demand[i]);
    }
    arm_demand_pub.publish(arm_demand_msg);

    wrist_demand_msg.data.clear();
    for(int i=0; i<3; i++){
        wrist_demand_msg.data.push_back(wrist_demand[i]);
    }
    wrist_demand_pub.publish(wrist_demand_msg);
}

void RobotHandler::arm_callback(const std_msgs::Float32MultiArray::ConstPtr& arm_read_msg){
    for(int i=0; i<3; i++){
        arm_actual[i] = arm_read_msg->data[i];
    }
}

void RobotHandler::wrist_callback(const std_msgs::Float32MultiArray::ConstPtr& wrist_read_msg){
    for(int i=0; i<3; i++){
        wrist_actual[i] = wrist_read_msg->data[i];
    }
}

void RobotHandler::flipper_callback(const std_msgs::Float32MultiArray::ConstPtr& flipper_read_msg){
    for(int i=0; i<2; i++){
        flipper_actual[i] = flipper_read_msg->data[i];
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "robot_interface");
    ros::NodeHandle n;

    ros::CallbackQueue ros_queue;
    n.setCallbackQueue(&ros_queue);

    RobotHandler handler(n);
    ros::Duration update_period(1.0/20);
    ros::Timer timer = n.createTimer(update_period, &RobotHandler::update, &handler);

    ROS_INFO("Here");

    ros::MultiThreadedSpinner spinner(0);
    spinner.spin(&ros_queue);
    return 0;
}
