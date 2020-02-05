#include "interface.h"

#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/MultiArrayDimension.h"
#include <controller_manager/controller_manager.h>
#include <ros/console.h>
#include <ros/callback_queue.h>


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
    void callback(const std_msgs::Float32MultiArray::ConstPtr& read_msg);

private:
    RobotInterface robot;
    boost::shared_ptr<controller_manager::ControllerManager> cm;

    double demand_angles[6];
    double read_angles[6];

    ros::Publisher demand_pub;
    ros::Subscriber read_sub;
    std_msgs::Float32MultiArray demand_msg;
    ros::Duration elapsed_time;
};

RobotHandler::RobotHandler(ros::NodeHandle& n):
        robot(n), demand_msg(), elapsed_time(){
    cm.reset(new controller_manager::ControllerManager(&robot, n));

    demand_pub = n.advertise<std_msgs::Float32MultiArray>("arm/angles", 1);
    read_sub = n.subscribe("arm/angles", 1000, &RobotHandler::callback, this); 

    demand_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
    demand_msg.layout.dim[0].label = "pivot";
    demand_msg.layout.dim[0].size = 6;
    demand_msg.layout.dim[0].stride = 1;
}

void RobotHandler::update(const ros::TimerEvent& timer){
    elapsed_time = ros::Duration(timer.current_real - timer.last_real);

    robot.read(read_angles);
    cm->update(ros::Time::now(), elapsed_time);
    robot.write(demand_angles);

    demand_msg.data.clear();
    for(int i=0; i<6; i++){
        demand_msg.data.push_back(demand_angles[i]);
    }
    demand_pub.publish(demand_msg);
}

void RobotHandler::callback(const std_msgs::Float32MultiArray::ConstPtr& read_msg){
    for(int i=0; i<6; i++){
        read_angles[i] = read_msg->data[i];
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
