#ifndef TELEOP_JOYSTICK_HPP
#define TELEOP_JOYSTICK_HPP

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "sensor_msgs/msg/joy.hpp"

namespace teleop_tools {
class TeleopJoystick : public rclcpp::Node {
public:
  TeleopJoystick();
  ~TeleopJoystick();

private:
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  int axis_linear_;
  int axis_angular_;

  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg);
};
} // namespace teleop_tools

#endif // TELEOP_JOYSTICK_HPP