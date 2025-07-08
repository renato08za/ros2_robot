from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='control_motor',
            executable='encoders_publisher_node',
            name='encoder_publisher'
        ),
        Node(
            package='control_motor',
            executable='simple_motor_control_node',
            name='motor_control'
        ),
        Node(
            package='control_motor',
            executable='velocity_computation_node',
            name='velocity_computation'
        ),
    ])
