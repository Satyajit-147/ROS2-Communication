from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='number_square_package',
            executable='number_publisher',
            name='number_publisher',
            output='screen'
        ),
        Node(
            package='number_square_package',
            executable='square_subscriber',
            name='square_subscriber',
            output='screen'
        )
    ])
