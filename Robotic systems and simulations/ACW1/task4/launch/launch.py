from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            executable='turtlesim_node'
        ),
        Node(
            package='task4',
            executable='spawner'
        ),
        Node(
            package='task4',
            executable='prey_node',
            output='screen'
        ),
        Node(
            package='task4',
            executable='predator_node',
            name='predator1_node',
            parameters=[{'turtle_name': 'predator1'}],
            output='screen'
        ),
        Node(
            package='task4',
            executable='predator_node',
            name='predator2_node',
            parameters=[{'turtle_name': 'predator2'}],
            output='screen'
        ),
        Node(
            package='task4',
            executable='predator_node',
            name='predator3_node',
            parameters=[{'turtle_name': 'predator3'}],
            output='screen'
        ),
    ])
