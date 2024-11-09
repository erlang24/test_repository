from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    # 发送紧急车辆故障
    jinjicheliang = Node(
        package = 'position_determination',
        namespace = 'simple_v2x',
        executable = 'send_emergency_vehicles_node',
        name = 'send_emergency_vehicles_node',
        output='screen',
    )

    ld.add_action(jinjicheliang)

    return ld