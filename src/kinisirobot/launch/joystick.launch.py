# Example launch file content
from launch import LaunchDescription
from launch_ros.actions import Node

joy_node = Node(
    package='joy_linux',
    executable='joy_linux_node',
    name='joy_linux_node'
)

# https://index.ros.org/p/teleop_twist_joy/github-ros2-teleop_twist_joy/
teleop_twist_joy = Node(
    package='teleop_twist_joy',
    executable='teleop_node',
    name='teleop_twist_joy_node',
    parameters=[{'axis_linear': {'x': 1, 'y': 3, 'z': -1},
                 'scale_linear': {'x': 0.5, 'y': 0.5, 'z': 0.0},
                 'axis_angular': {'yaw': 0, 'pitch': -1, 'roll': -1},
                 'scale_angular': {'yaw': 1.2, 'pitch': 0.0, 'roll': 0.0}}],
)
    

def generate_launch_description():
    return LaunchDescription([
        joy_node,
        teleop_twist_joy
    ])