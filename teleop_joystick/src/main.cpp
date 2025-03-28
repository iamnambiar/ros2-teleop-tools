#include "rclcpp/rclcpp.hpp"
#include "teleop_joystick/teleop_joystick.hpp"

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<teleop_tools::TeleopJoystick>());
    rclcpp::shutdown();
    return 0;
}