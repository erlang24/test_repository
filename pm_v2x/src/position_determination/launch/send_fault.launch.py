from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    # 发送紧急车辆故障
    yichangcheliang = Node(
        package = 'position_determination',
        namespace = 'simple_v2x',
        executable = 'send_fault_alarm_node',
        name = 'send_fault_alarm_node',
        output='screen',
    )

    ld.add_action(yichangcheliang)

    return ld