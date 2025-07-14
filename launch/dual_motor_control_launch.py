from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='control_motor',
            executable='encoder_publisher',
            name='encoder_publisher',
            output='screen',
        ),
        Node(
            package='control_motor',
            executable='dual_motor_pid_node',
            name='dual_motor_pid_nod',
            output='screen',
        ),
    ])
