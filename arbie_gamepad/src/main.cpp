#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
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
        if (!arm_mode) {
            command_publisher.set_tracks_command(
                joystick_listener.query_axis(JoyAxis::LEFT_VERTICAL),
                joystick_listener.query_axis(JoyAxis::LEFT_HORIZONTAL));
            command_publisher.set_flippers_command(
                joystick_listener.query_axis(JoyAxis::RIGHT_VERTICAL),
                joystick_listener.query_axis(JoyAxis::RIGHT_HORIZONTAL));
        } else {
            command_publisher.set_gripper_velocity(
                joystick_listener.query_axis(JoyAxis::LEFT_VERTICAL),
                joystick_listener.query_axis(JoyAxis::LEFT_HORIZONTAL),
                0.5 * (joystick_listener.query_axis(JoyAxis::LT)
                 - joystick_listener.query_axis(JoyAxis::RT)),
                3 * (joystick_listener.query_button_value(JoyButton::LB)
                 - joystick_listener.query_button_value(JoyButton::RB)),
                joystick_listener.query_axis(JoyAxis::RIGHT_VERTICAL),
                joystick_listener.query_axis(JoyAxis::RIGHT_HORIZONTAL)
            );

            if (joystick_listener.query_button_state(JoyButton::A) == JoyButtonState::PRESSED) {
                command_publisher.send_gripper_command("home");
            }
            if (joystick_listener.query_button_state(JoyButton::X) == JoyButtonState::PRESSED) {
                command_publisher.send_gripper_command("ready");
            }
        }

        command_publisher.publish_all();
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
