from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='control_motor',
            executable='encoder_publisher',
            name='encoder_publisher',
            output='screen'
        ),
        Node(
            package='control_motor',
            executable='velocity_computation',
            name='velocity_computation',
            output='screen'
        ),
        Node(
            package='control_motor',
            executable='control_motor',
            name='control_motor',
            output='screen'
        ),
    ])
