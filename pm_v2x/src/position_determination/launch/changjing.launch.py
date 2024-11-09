from launch import LaunchDescription
from launch_ros.actions import Node

# 场景启动

def generate_launch_description():
    ld = LaunchDescription()

    # 前向碰撞预警
    QianXiangPengZhuang = Node(
        package = 'position_determination',
        namespace = 'simple_v2x',
        executable = 'vehicle_position_node',
        name = 'vehicle_position_node',
        output='screen',
    )
    # 盲区碰撞预警
    MangQu = Node(
        package = 'position_determination',
        namespace = 'simple_v2x',
        executable = 'blind_spot_warning_node',
        name = 'blind_spot_warning_node',
        output='screen',
    )
    # 紧急车辆
    JingJiCheLiang = Node(
        package = 'position_determination',
        namespace = 'simple_v2x',
        executable = 'emergency_vehicles_node',
        name = 'vehicle_emergency_node',
        output='screen',
    )
    # 异常车辆
    YiChangCheLiang = Node(
        package = 'position_determination',
        namespace = 'simple_v2x',
        executable = 'fault_alarm_receiver_node',
        name = 'fault_alarm_receiver_node',
        output='screen',
    )
    # 交叉路口碰撞预警
    JiaoChaLuKou = Node(
        package = 'position_determination',
        namespace = 'simple_v2x',
        executable = 'intersection_collision_warning_node',
        name = 'intersection_collision_warning_node',
        output='screen',
    )
    # 绿波车速引导
    LvBoCheSu = Node(
        package = 'position_determination',
        namespace = 'simple_v2x',
        executable = 'green_wave_node',
        name = 'green_wave_node',
        output='screen',
    )

    ld.add_action(QianXiangPengZhuang)
    ld.add_action(MangQu)
    ld.add_action(JingJiCheLiang)
    ld.add_action(YiChangCheLiang)
    ld.add_action(JiaoChaLuKou)
    ld.add_action(LvBoCheSu)

    return ld