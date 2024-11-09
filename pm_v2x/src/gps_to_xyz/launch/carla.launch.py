from launch import LaunchDescription    # launch文件的描述类
from launch_ros.actions import Node     # 节点启动的描述类
  
def generate_launch_description():  
    """  
    生成ROS 2的launch描述。  
    这个描述定义了如何启动一个或多个ROS 2节点。  
    """  
    ld = LaunchDescription()  
      
    example_node1 = Node(
        package='gps_to_xyz',
        namespace='GPS2XYZ',
        executable='listen_gpstcp_to_udp_node',
        name='listen_gpstcp_to_udp_node',
        output='screen',
    )
    
    # 创建一个ROS 2节点描述 
    example_node2 = Node(
        package='gps_to_xyz',
        namespace='GPS2XYZ',
        executable='udp_receiver_node',
        name='udp_receiver_node',
        output='screen',
    )

    example_node3 = Node(  
        package='gps_to_xyz',                   # 节点所在的功能包名  
        namespace='GPS2XYZ',                           # 命名空间，用于避免同名节点的冲突  
        executable='gps_to_xyz_node',           # 表示要运行的可执行文件名，应该指向一个可执行文件，而不是一个.py脚本。如果你有一个Python脚本，你可能需要在你的CMakeLists.txt或package.xml中设置一个entry_point，然后在这里引用它。  
        name='gps_to_xyz_node',                 # 节点名，在ROS图(graph)中使用的名称  
        # parameters=[{'parameter-name': 'parameter-value'}],        # 传递给节点的参数列表，确保value是字符串或已定义的变量  
        # arguments=['--xxx', 'xxx_value', '--yyy', 'yyy_value'],    # 启动参数列表，注意参数和值之间用空格分隔  
        output='screen',                        # 将节点的输出打印到屏幕  
        # remappings=[                          # 重映射规则列表  
        #     ('/xxx/topic-new', '/xxx/topic-old'),  # 重映射规则，将旧话题映射到新话题  
        # ],  
    )
    # 如果有第二个节点，则按照相同的方式定义  
    example_node4 = Node(
        package='gps_to_xyz',
        namespace='GPS2XYZ',
        executable='vehicle_position_node',
        name='vehicle_position_node',
        output='screen',
        # ... 其他节点配置 ...  
    )

    # 将节点添加到launch描述中  
    ld.add_action(example_node1)  
    ld.add_action(example_node2)  
    ld.add_action(example_node3)
    ld.add_action(example_node4)
  
    # 返回完整的launch描述  
    return ld