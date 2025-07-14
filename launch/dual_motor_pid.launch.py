from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='control_motor',
            executable='simple_dual_motor_control',
            name='simple_dual_motor_control',
            output='screen'
        ),
        Node(
            package='control_motor',
            executable='pid_motor_control',
            name='pid_motor_control',
            output='screen'
        ),
    ])
