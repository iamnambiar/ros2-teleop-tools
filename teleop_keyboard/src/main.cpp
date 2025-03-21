#include "teleop_keyboard/teleop_keyboard.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<teleop_tools::TeleopKeyboard>());
    rclcpp::shutdown();
    return 0;
}