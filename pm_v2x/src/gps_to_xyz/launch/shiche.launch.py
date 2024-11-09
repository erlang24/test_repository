from launch import LaunchDescription
from launch_ros.actions import Node

# 使用华测GPS-朗逸

def generate_launch_description():
    ld = LaunchDescription()

    # 监听车辆节点
    gps_listener_node = Node(
        package='gps_to_xyz',
        namespace='simple_v2x',
        executable='gps_listener_gpchd_node', # 可执行文件名
        name='gpchd_gps_listener_node', # 节点名
        output='screen',
    )
    # 发送透传节点 
    udp_transmitter_node = Node(
        package='gps_to_xyz',
        namespace='simple_v2x',
        executable='udp_transmitter_node',
        name='udp_transmitter_node',
        output='screen', 
    )
    # 坐标转换节点
    gps_to_xyz_node = Node(
        package='gps_to_xyz',
        namespace='simple_v2x',
        executable='gps_to_xyz_node',
        name='gps_to_xyz_node',
    )
    # obu处理透传数据节点
    # obu_message_processor_node = Node(
    #     package='gps_to_xyz',
    #     namespace='simple_v2x',
    #     executable='obu_message_processor_node',
    #     name='obu_message_processor_node',
    #     output='screen',
    # )
    # 接收透传节点
    udp_receiver_node = Node(
        package='gps_to_xyz',
        namespace='simple_v2x',
        executable='udp_receiver_node',
        name='udp_receiver_node',
        output='screen',
    )

    # =====================================

    decode_node = Node(
        package = 'obu_json',
        namespace = 'simple_v2x',
        executable = 'decode_node',
        name = 'decode_node',
        output='screen',
    )

    encode_node = Node(
        package = 'obu_json',
        namespace = 'simple_v2x',
        executable = 'encode_node',
        name = 'encode_node',
        output='screen',
    )

    # 接收透传解析 map json数据节点
    # obu_handle_node = Node(
    #     package = 'obu_json',
    #     namespace = 'simple_v2x',
    #     executable = 'obu_handle_node',
    #     name = 'obu_handle_node',
    # )

    # ===================================

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

    ld.add_action(gps_listener_node)
    ld.add_action(udp_transmitter_node) 
    ld.add_action(gps_to_xyz_node)
    # ld.add_action(obu_message_processor_node)
    ld.add_action(udp_receiver_node)

    ld.add_action(decode_node)
    ld.add_action(encode_node)
    # ld.add_action(obu_handle_node)

    ld.add_action(QianXiangPengZhuang)
    ld.add_action(MangQu)
    ld.add_action(JingJiCheLiang)
    ld.add_action(YiChangCheLiang)
    ld.add_action(JiaoChaLuKou)
    ld.add_action(LvBoCheSu)

    return ld