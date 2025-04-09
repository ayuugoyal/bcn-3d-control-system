from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='rosbridge_websocket',
            parameters=[
                {'port': 9090},
                {'address': '0.0.0.0'},  # Allow connections from any IP
                {'use_compression': False},
                {'delay_between_messages': 0.0},
                {'max_message_size': 10000000}
            ],
            output='screen'
        )
    ])
