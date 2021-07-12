#include <sstream>
#include <iostream>
#include "ros/ros.h"
#include "joystick_listener.h"
#include "command_processor.h"

bool goal_reached = false;
bool navigation_mode = false;

class NavMessage {

	public:
		double x;
		double y;
		double orientation;
		void set_values(double x0, double y0, double orientation0);
};

void NavMessage::set_values(double x0, double y0, double orientation0){
	x = x0;
	y = y0;
	orientation = orientation0;
}

//Response to goal-reached activity
void Respond_1(const std_msgs::Bool msg)
{
	goal_reached = msg.data;
}

//Response to navigation-mode-change activity
void Respond_2(const std_msgs::Bool msg)
{
	navigation_mode = msg.data;
}



int main(int argc, char **argv)
{
	NavMessage navmessage;
	navmessage.set_values(0,0,0);

	int loop_count = 0;

	ros::init(argc,argv,"navigation");
	ros::NodeHandle n;

	CommandPublisher command_publisher(n);
    JoystickListener joystick_listener(n);

	ros::Subscriber goal_reached_sub = n.subscribe("/goal_reached", 1000, Respond_1);
    ros::Subscriber navigation_switch_sub = n.subscribe("/navigation_command", 1000, Respond_2);
	ROS_INFO("all good");

	ros::Rate loop_rate(50);
	while(ros::ok()){
		if(navigation_mode){
			loop_count += 1;
			
			navmessage.x = navmessage.x - 0.01*joystick_listener.query_axis(JoyAxis::LEFT_HORIZONTAL);
			navmessage.y = navmessage.y + 0.01*joystick_listener.query_axis(JoyAxis::LEFT_VERTICAL);
			navmessage.orientation = navmessage.orientation + 0.03*joystick_listener.query_axis(JoyAxis::RIGHT_HORIZONTAL);
			command_publisher.set_cursor_pose(navmessage.x, navmessage.y, navmessage.orientation);
			if (loop_count%5==0){
				command_publisher.cursor_publish();
				loop_count = 0;
			}

			if (joystick_listener.query_button_state(JoyButton::A) == JoyButtonState::PRESSED){
				command_publisher.goal_publish();
			}
		}
		

		ros::spinOnce();
		loop_rate.sleep();

	}
	goal_reached = false;
	return 0;
}
