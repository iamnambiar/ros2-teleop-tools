#include "teleop_joystick/teleop_joystick.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "sensor_msgs/msg/joy.hpp"

namespace teleop_tools {
TeleopJoystick::TeleopJoystick() : rclcpp::Node("teleop_joystick") {
  this->declare_parameter("axis_linear", 1);
  this->declare_parameter("axis_angular", 0);

  axis_linear_ = this->get_parameter("axis_linear").as_int();
  axis_angular_ = this->get_parameter("axis_angular").as_int();

  cmd_pub_ =
      this->create_publisher<geometry_msgs::msg::TwistStamped>("/key_vel", 10);
  joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "/joy", 10,
      std::bind(&TeleopJoystick::joy_callback, this, std::placeholders::_1));
}
TeleopJoystick::~TeleopJoystick() {}
void TeleopJoystick::joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {
  auto twist_msg = geometry_msgs::msg::TwistStamped();
  twist_msg.header.stamp = this->get_clock()->now();
  twist_msg.twist.linear.x = msg->axes[axis_linear_];
  twist_msg.twist.angular.z = msg->axes[axis_angular_];

  cmd_pub_->publish(twist_msg);
}
} // namespace teleop_tools