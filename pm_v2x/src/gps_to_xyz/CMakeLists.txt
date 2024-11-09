cmake_minimum_required(VERSION 3.8)
project(gps_to_xyz)

# 默认使用 C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "/usr/share/cmake/geographiclib/")


# 查找依赖项
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(GeographicLib REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(gps_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

# find_package(serial REQUIRED)

# 查找 msg_msg 包
# find_package(msg_msg REQUIRED)
find_package(msg_interfaces REQUIRED)
find_package(raw_sensor_msgs REQUIRED)

# find_package(nlohmann_json REQUIRED)


# include_directories(
#   /home/erlang/shared_dir/ros2pkg/src/chcnav/src/hc_msg_parser_node/device_connector/serial_common/serial/include
#   # /tmp/serial/include
# )

# 添加可执行文件
add_executable(gps_listener_gpchd_node src/gps_listener_gpchd_node.cpp)
ament_target_dependencies(gps_listener_gpchd_node rclcpp std_msgs gps_msgs msg_interfaces geometry_msgs)

add_executable(gps_listener_rtk_node src/gps_listener_rtk_node.cpp)
ament_target_dependencies(gps_listener_rtk_node rclcpp gps_msgs geometry_msgs std_msgs raw_sensor_msgs)

add_executable(listen_gpstcp_to_udp_node src/listen_gpstcp_to_udp_node.cpp)
ament_target_dependencies(listen_gpstcp_to_udp_node rclcpp sensor_msgs geometry_msgs )

add_executable(gps_to_xyz_node src/gps_to_xyz_node.cpp)
ament_target_dependencies(gps_to_xyz_node rclcpp std_msgs GeographicLib sensor_msgs geometry_msgs Eigen3 gps_msgs )

add_executable(udp_receiver_node src/udp_receiver_node.cpp)
ament_target_dependencies(udp_receiver_node rclcpp std_msgs sensor_msgs geometry_msgs )

add_executable(gps_listener_node src/gps_listener_node.cpp)
ament_target_dependencies(gps_listener_node rclcpp sensor_msgs geometry_msgs gps_msgs )


add_executable(udp_transmitter_node src/udp_transmitter_node.cpp)
ament_target_dependencies(udp_transmitter_node rclcpp std_msgs )

add_executable(obu_message_processor_node src/obu_message_processor_node.cpp)
ament_target_dependencies(obu_message_processor_node rclcpp std_msgs sensor_msgs geometry_msgs )

# 安装可执行文件
install(TARGETS
  gps_listener_gpchd_node
  gps_listener_rtk_node
  listen_gpstcp_to_udp_node
  gps_to_xyz_node
  udp_receiver_node
  gps_listener_node

  udp_transmitter_node
  obu_message_processor_node
  DESTINATION lib/${PROJECT_NAME}
)

# 测试依赖项
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}/
)

# 导出依赖项
ament_export_dependencies(rosidl_default_runtime)
ament_export_dependencies(msg_interfaces)

ament_package()