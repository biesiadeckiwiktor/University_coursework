from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            executable='turtlesim_node'
        ),
        Node(
            package='task1',
            executable='spawner'
        ),
        Node(
            package='task1',
            executable='prey_node',
            output = 'screen'
        ),
        Node(
            package='task1',
            executable='predator_node',
            output = 'screen'
        )

    ])