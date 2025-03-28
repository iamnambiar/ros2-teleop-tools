# ros2-teleop-tools

This repository provides a collection of teleoperation tools for controlling robots in ROS2 using different input devices.

## Features

- **teleop_keyboard**: Control robots via keyboard inputs.
- **teleop_joystick**: Control robots using a joystick. 

## Installation and Build Instructions

### Prerequisites  

Ensure you have ROS 2 installed on your system. If not, follow the official ROS 2 installation guide: [ROS 2 Installation](https://docs.ros.org/en/ros2_documentation/humble/Installation.html).  

### Clone Repository  

```bash
cd ~/ros2_ws/src
git clone https://github.com/iamnambiar/ros2-teleop-tools.git
cd ~/ros2_ws
```

### Build

```bash
source /opt/ros/${ROS-DISTRO}/setup.bash
colcon build --packages-select teleop_tools
source install/setup.bash
```

### Running the Teleop Nodes

- **Keyboard Teleoperation**
```bash
ros2 run teleop_keyboard teleop_keyboard_node
```

- **Joystick Teleoperation**
```bash
ros2 run teleop_joystick teleop_joystick_node
```

## License
This project is licensed under the MIT License. See the [LICENSE](./LICENSE) file for details