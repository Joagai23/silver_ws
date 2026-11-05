from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='dht11',
            executable='dht11_node',
            name='dht11_node',
            parameters=[{'read_interval': 5.0}],
            output='screen',
        )
    ])