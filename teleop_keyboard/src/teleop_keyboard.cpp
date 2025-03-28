#include "teleop_keyboard/teleop_keyboard.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <vector>

namespace teleop_tools {
TeleopKeyboard::TeleopKeyboard()
    : rclcpp::Node("teleop_keyboard"), useSmoothAcceleration_(false),
      velocity_({0.0, 0.0}) {
  cmd_pub_ =
      this->create_publisher<geometry_msgs::msg::TwistStamped>("key_vel", 10);
  printInstructions();
  this->enableRawMode();
  timer_ = this->create_wall_timer(std::chrono::milliseconds(50),
                                   std::bind(&TeleopKeyboard::run, this));
  status_timer_ =
      this->create_wall_timer(std::chrono::milliseconds(500),
                              std::bind(&TeleopKeyboard::printStatus, this));
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
  std::cout << '\n';
}

void TeleopKeyboard::printInstructions() {
  std::cout << "\nTeleoperation using Arrow Keys:\n";
  std::cout << "  Up Arrow        : Move Forward\n";
  std::cout << "  Down Arrow      : Move Backward\n";
  std::cout << "  Left Arrow      : Turn Left\n";
  std::cout << "  Right Arrow     : Turn Right\n";
  std::cout << "  Space           : Stop\n";
  std::cout << "Press 'm' to toggle acceleration mode (Smooth/Instant).\n";
  std::cout << "Press 'q' to exit.\n";
}

void TeleopKeyboard::printStatus() {
  std::cout << std::setprecision(2) << std::fixed
            << "\rCurrent Twist: Linear X: " << velocity_[0]
            << " Angular Z: " << velocity_[1]
            << " Mode: " << (useSmoothAcceleration_ ? "Smooth--" : "Instant")
            << "----------" << std::flush;
}

void TeleopKeyboard::run() {
  auto ch = getchar();
  std::vector<double> targetVelocity = {0.0, 0.0};
  const double max_speed = 1.0;
  const double max_acceleration = 0.05;
  if (ch == 27) {
    if (getchar() == '[') {
      switch (getchar()) {
      case 'A':
        targetVelocity[0] = max_speed;
        break;
      case 'B':
        targetVelocity[0] = -max_speed;
        break;
      case 'C':
        targetVelocity[1] = -max_speed;
        break;
      case 'D':
        targetVelocity[1] = max_speed;
        break;
      default:
        break;
      }
    }
  } else if (ch == 'm') {
    useSmoothAcceleration_ = !useSmoothAcceleration_;
  } else if (ch == 'q') {
    rclcpp::shutdown();
  }

  if (useSmoothAcceleration_) {
    updateVelocity(velocity_, targetVelocity, max_acceleration);
  } else {
    velocity_ = targetVelocity;
  }
  auto twist_message = geometry_msgs::msg::TwistStamped();
  twist_message.header.stamp = this->now();
  twist_message.twist.linear.x = velocity_[0];
  twist_message.twist.angular.z = velocity_[1];
  cmd_pub_->publish(twist_message);
  tcflush(STDIN_FILENO, TCIFLUSH);
}

void TeleopKeyboard::updateVelocity(std::vector<double> &velocity,
                                    const std::vector<double> &target,
                                    double step) {
  for (size_t i = 0; i < velocity.size(); ++i) {
    if (velocity[i] < target[i]) {
      velocity[i] = std::min(velocity[i] + step, target[i]);
    } else if (velocity[i] > target[i]) {
      velocity[i] = std::max(velocity[i] - step, target[i]);
    }
  }
}

} // namespace teleop_tools