from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[{'deadzone': 0.1}]
        ),
        Node(
            package='teleop_joystick',
            executable='teleop_joystick_node',
            name='teleop_joystick_node',
            remappings=[('key_vel', 'cmd_vel')]
        )
    ])
