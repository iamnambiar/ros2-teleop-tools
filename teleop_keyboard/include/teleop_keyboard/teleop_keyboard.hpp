#ifndef TELEOP_KEYBOARD_HPP
#define TELEOP_KEYBOARD_HPP

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include <termios.h>
#include <vector>

/**
 * @class TeleopKeyboard
 * @brief A ROS 2 node for teleoperation using keyboard input.
 *
 * The TeleopKeyboard class provides functionality to control a robot by
 * publishing velocity commands based on keyboard input.
 *
 * @note This class uses the ROS 2 rclcpp library for node functionality and
 * publishes TwistStamped messages to control the robot.
 */

namespace teleop_tools {
class TeleopKeyboard : public rclcpp::Node {
public:
  /**
   * @brief Constructor for the TeleopKeyboard class.
   *
   * Initializes the ROS 2 node, sets up publishers and timers, and configures
   * terminal settings.
   */
  TeleopKeyboard();

  /**
   * @brief Destructor for the TeleopKeyboard class.
   *
   * Restores the original terminal settings and cleans up resources.
   */
  ~TeleopKeyboard();

private:
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr
      cmd_pub_;                        ///< Publisher for velocity commands.
  rclcpp::TimerBase::SharedPtr timer_; ///< Timer for periodic updates.
  rclcpp::TimerBase::SharedPtr status_timer_;

  struct termios
      old_termios_; ///< Original terminal settings, saved for restoration.
  struct termios new_termios_; ///< Modified terminal settings for raw mode.

  bool useSmoothAcceleration_;
  std::vector<double> velocity_;

  /**
   * @brief Enables raw mode for the terminal.
   *
   * Configures the terminal to raw mode to capture key presses without
   * buffering.
   */
  void enableRawMode();

  /**
   * @brief Disables raw mode for the terminal.
   *
   * Restores the terminal to its original settings.
   */
  void disableRawMode();

  /**
   * @brief Main loop for processing keyboard input and publishing commands.
   *
   * This method captures key presses, interprets them as velocity commands, and
   * publishes the commands to the appropriate ROS 2 topic.
   */
  void run();

  /**
   * @brief Smoothly updates the velocity towards the target value.
   *
   * This method adjusts the given velocity by incrementing or decrementing it
   * towards the target value using the specified step size. It ensures a
   * gradual change in velocity for smoother transitions.
   *
   * @param velocity Reference to the current velocity to be updated.
   * @param target The target velocity to approach.
   * @param step The step size to increment or decrement the velocity.
   */
  void updateVelocity(std::vector<double> &velocity, const std::vector<double> &target, double step);

  /**
   * @brief Prints instructions for using the teleoperation keyboard.
   *
   * This method outputs the key mappings and usage instructions to the console.
   */
  void printInstructions();

  void printStatus();
};

} // namespace teleop_tools

#endif // TELEOP_KEYBOARD_HPP