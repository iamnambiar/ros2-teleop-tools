#ifndef TELEOP_KEYBOARD_HPP
#define TELEOP_KEYBOARD_HPP

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include <termios.h>

/**
 * @brief A ROS2 node for smooth teleoperation using keyboard arrow keys.
 *
 * This node reads arrow keys from the terminal and gradually adjusts
 * the velocity of the robot to avoid jerky motions.
 */

namespace teleop_tools {
class TeleopKeyboard : public rclcpp::Node {
public:
  TeleopKeyboard();
  ~TeleopKeyboard();
  void PrintInstructions();

private:
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  struct termios old_termios_; // Original termios settings
  struct termios new_termios_;

  void enableRawMode();
  void disableRawMode();

  void run();
};
} // namespace teleop_tools

#endif // TELEOP_KEYBOARD_HPP