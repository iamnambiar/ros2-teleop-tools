#include "teleop_keyboard/teleop_keyboard.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

namespace teleop_tools {
TeleopKeyboard::TeleopKeyboard() : rclcpp::Node("teleop_keyboard") {
  cmd_pub_ =
      this->create_publisher<geometry_msgs::msg::TwistStamped>("key_vel", 10);
  PrintInstructions();
  timer_ = this->create_wall_timer(std::chrono::milliseconds(100),
                                   std::bind(&TeleopKeyboard::run, this));
}

TeleopKeyboard::~TeleopKeyboard() { disableRawMode(); }

void TeleopKeyboard::enableRawMode() {
  tcgetattr(STDIN_FILENO, &old_termios_);
  new_termios_ = old_termios_;
  new_termios_.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &new_termios_);
  int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
  fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);
}

void TeleopKeyboard::disableRawMode() {
  tcsetattr(STDIN_FILENO, TCSANOW, &old_termios_);
}

void TeleopKeyboard::PrintInstructions() {
  std::cout << "\nTeleoperation using Arrow Keys:\n";
  std::cout << "  Up Arrow        : Move Forward" << '\n';
  std::cout << "  Down Arrow      : Move Backward" << '\n';
  std::cout << "  Left Arrow      : Turn Left" << '\n';
  std::cout << "  Right Arrow     : Turn Right" << '\n';
  std::cout << "  Space           : Stop" << '\n';
  std::cout << "Press 'q' to exit.\n";
}

void TeleopKeyboard::run() {
  auto twist_message = geometry_msgs::msg::TwistStamped();
  auto ch = getchar();
  if (ch == 27) {
    if (getchar() == '[') {
      switch (getchar()) {
      case 'A':
        twist_message.twist.linear.x = 1.0;
        break;
      case 'B':
        twist_message.twist.linear.x = -1.0;
        break;
      case 'C':
        twist_message.twist.angular.z = -1.0;
        break;
      case 'D':
        twist_message.twist.angular.z = 1.0;
        break;
      default:
        twist_message.twist.linear.x = 0.0;
        twist_message.twist.angular.z = 0.0;
        break;
      }
    }
  } else if (ch == 'q') {
    rclcpp::shutdown();
  }
  twist_message.header.stamp = this->now();
  cmd_pub_->publish(twist_message);
}
} // namespace teleop_tools