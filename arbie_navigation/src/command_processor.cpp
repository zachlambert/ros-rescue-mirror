#include "command_processor.h"

geometry_msgs::PoseStamped create_cursor_msg()
{
    geometry_msgs::PoseStamped cursor_msg;
  	cursor_msg.header.seq = 1;
	cursor_msg.header.stamp = ros::Time::now();
	cursor_msg.header.frame_id = "map";
  	cursor_msg.pose.position.x = 0.0;
	cursor_msg.pose.position.y = 0.0;
	cursor_msg.pose.position.z = 0.0;
	cursor_msg.pose.orientation.x = 0.0;
  	cursor_msg.pose.orientation.y = 0.0;
  	cursor_msg.pose.orientation.z = 0.0;
  	cursor_msg.pose.orientation.w = 0.0;
    return cursor_msg;
}

CommandPublisher::CommandPublisher(ros::NodeHandle &n)
{

    //Cursor command
    cursor_pose_pub = n.advertise<geometry_msgs::PoseStamped>(
	    "navigation/goal",1000
    );
    cursor_msg = create_cursor_msg();

    goal_pose_pub = n.advertise<geometry_msgs::PoseStamped>(
        "move_base_simple/goal", 1000
    );
}

void CommandPublisher::set_cursor_pose(double x, double y, double orientation)
{

  	cursor_msg.header.seq = 1;
	cursor_msg.header.stamp = ros::Time::now();
	cursor_msg.header.frame_id = "map";
  	cursor_msg.pose.position.x = x;
	cursor_msg.pose.position.y = y;
	cursor_msg.pose.position.z = 0.0;
	cursor_msg.pose.orientation.x = 0.0;
  	cursor_msg.pose.orientation.y = 0.0;
  	cursor_msg.pose.orientation.z = 1.0;
  	cursor_msg.pose.orientation.w = orientation;
}


void CommandPublisher::cursor_publish()
{
    cursor_pose_pub.publish(cursor_msg);
}

void CommandPublisher::goal_publish()
{
	goal_pose_pub.publish(cursor_msg);
}

