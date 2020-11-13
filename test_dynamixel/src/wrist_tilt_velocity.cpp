#include <ros/ros.h>
#include <dynamixel_workbench_msgs/DynamixelCommand.h>
#include <boost/thread.hpp>
#include <boost/chrono.hpp>
#include <sensor_msgs/JointState.h>
#include <math.h>

class Node {
public:
    Node(ros::NodeHandle &n) {

    }
    void write_velocity(int velocity) {
        if (collision > 0 && velocity > 0) {
            return;
        } else if (collision < 0 && velocity < 0){
            return;
        } else if (abs(velocity) > 1023) {
            return;
        }
        int command = velocity>0 ? velocity : 1024+velocity;
    }
    void joint_states_callback(sensor_msgs::JointState &msg) {

    }
private:
    int collision; // 0 = no collision, 1 = collision for positive vel, -1 = ...
    static constexpr double lower_limit = 73*(M_PI/180);
    static constexpr double upper_limit = 205*(M_PI/180);
};

void write_dynamixel(
    ros::ServiceClient &dynamixel_command,
    dynamixel_workbench_msgs::DynamixelCommand &msg)
{
    if (!dynamixel_command.call(msg) || !msg.response.comm_result) {
        std::cout << "Failure" << std::endl;
    } else {
        std::cout << "Success" << std::endl;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "control");
    ros::NodeHandle n;
    ros::ServiceClient dynamixel_command =
        n.serviceClient<dynamixel_workbench_msgs::DynamixelCommand>(
            "/dynamixel/dynamixel_command"
    );

    dynamixel_workbench_msgs::DynamixelCommand right_msg;
    right_msg.request.id = 4;
    right_msg.request.addr_name = "Moving_Speed";

    int velocity, velocity_command;

    dynamixel_workbench_msgs::DynamixelCommand left_msg;
    left_msg.request.id = 5;
    left_msg.request.addr_name = "Goal_Position";

    int offset;
    for (int i = 0; i <= 10; i++) {
        offset = (25 - (i-5)*(i-5))*10;
        std::cout << offset << std::endl;

        right_msg.request.value = 512-offset;
        left_msg.request.value = 512+offset;

        boost::thread right_t(write_dynamixel, dynamixel_command, right_msg);
        boost::thread left_t(write_dynamixel, dynamixel_command, left_msg);
        right_t.join();
        left_t.join();

        //boost::this_thread::sleep_for(boost::chrono::milliseconds(100));
    }

    return 0;
}
