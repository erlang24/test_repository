from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='chcnav_driver',
            executable='chanav_node_bz',
            name='chanav_node_bz',
            parameters=[
                {'port': '/dev/ttyUSB0'},
                {'baud_rate': 115200}
            ]
        )
    ])
