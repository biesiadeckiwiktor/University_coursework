from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            executable='turtlesim_node'
        ),
        Node(
            package='task3',
            executable='spawner'
        ),
        Node(
            package='task3',
            executable='prey_node',
            output='screen'
        ),
        Node(
            package='task3',
            executable='predator_node',
            output='screen'
        )
    ])

