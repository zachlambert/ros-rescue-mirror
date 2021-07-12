#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include <iostream>
#include <sstream>
#include "command_publisher.h"
#include "joystick_listener.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "main");
    ros::NodeHandle n;

    CommandPublisher command_publisher(n);
    JoystickListener joystick_listener(n);

    command_publisher.set_tracks_command(0, 0);
    command_publisher.set_flippers_command(1.5, 0);

    bool arm_mode = false;

    ros::Rate loop_rate(50);
    while (ros::ok()) {
        if (joystick_listener.query_button_state(JoyButton::Y) == JoyButtonState::PRESSED) {
            arm_mode = !arm_mode;
        }
<<<<<<< HEAD
        if (!arm_mode) {
=======
	if (joystick_listener.query_button_state(JoyButton::B) == JoyButtonState::PRESSED) {
            navigation_mode = !navigation_mode;
	    command_publisher.navigation_command(navigation_mode);
        }
        if (!arm_mode && !navigation_mode) {
>>>>>>> parent of 211b2e5... navigation package
            double linear_vel = -0.5*joystick_listener.query_axis(JoyAxis::LEFT_VERTICAL);
            double angular_vel = -3*joystick_listener.query_axis(JoyAxis::LEFT_HORIZONTAL);
            command_publisher.set_tracks_command(linear_vel, angular_vel);
            double rear_flippers = joystick_listener.query_axis(JoyAxis::RIGHT_VERTICAL);
            double front_flippers = joystick_listener.query_axis(JoyAxis::RIGHT_HORIZONTAL);
            command_publisher.set_flippers_command(rear_flippers, front_flippers);
        } else {
            double yaw_vel = 0, pitch_vel = 0, roll_vel = 0, gripper_vel = 0;
            if (joystick_listener.query_button_value(JoyButton::LB)) {
                roll_vel = -1.5*joystick_listener.query_axis(JoyAxis::RIGHT_HORIZONTAL);
                gripper_vel = 1.0*joystick_listener.query_axis(JoyAxis::RIGHT_VERTICAL);
            } else {
                yaw_vel = -0.25*joystick_listener.query_axis(JoyAxis::RIGHT_HORIZONTAL);
                pitch_vel = -0.25*joystick_listener.query_axis(JoyAxis::RIGHT_VERTICAL);
            }

            double r_vel = -0.1*joystick_listener.query_axis(JoyAxis::LEFT_VERTICAL);
            double theta_vel = -0.2*joystick_listener.query_axis(JoyAxis::LEFT_HORIZONTAL);
            double z_vel = 0.1 * (
                joystick_listener.query_axis(JoyAxis::RT)
                - joystick_listener.query_axis(JoyAxis::LT));

            command_publisher.set_gripper_velocity(
                r_vel, theta_vel, z_vel,
                yaw_vel, pitch_vel, roll_vel,
                gripper_vel
            );

            if (joystick_listener.query_button_state(JoyButton::A) == JoyButtonState::PRESSED) {
                command_publisher.send_gripper_command("named_target", "home");
            }
            if (joystick_listener.query_button_state(JoyButton::X) == JoyButtonState::PRESSED) {
                command_publisher.send_gripper_command("named_target", "ready");
            }
        }

        command_publisher.publish_all();
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
