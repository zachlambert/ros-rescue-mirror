#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

class Node {
public:
    Node(ros::NodeHandle &n)
    {
        // Base pose transform
        base_transform.header.frame_id = "world";
        base_transform.child_frame_id = "base_link";
        base_transform.transform.rotation.w = 1;
        base_transform.header.stamp = ros::Time::now();

        // Odometry subscriber
        odom_sub = n.subscribe(
            "tracks_velocity_controller/odom",
            1,
            &Node::odometry_callback,
            this
        );
    }

    void odometry_callback(nav_msgs::Odometry msg)
    {
        base_transform.transform.translation.x = msg.pose.pose.position.x;
        base_transform.transform.translation.y = msg.pose.pose.position.y;
        base_transform.transform.rotation.z = msg.pose.pose.orientation.z;
        base_transform.transform.rotation.w = msg.pose.pose.orientation.w;
    }

    void update(void)
    {
        base_transform.header.stamp = ros::Time::now();
        tf_broadcaster.sendTransform(base_transform);
    }

private:
    ros::Subscriber odom_sub;
    tf2_ros::TransformBroadcaster tf_broadcaster;
    geometry_msgs::TransformStamped base_transform;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "localisation_placeholder");
    ros::NodeHandle n;
    Node node(n);
    ros::Rate loop_rate(10);
    while (ros::ok()) {
        node.update();
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
