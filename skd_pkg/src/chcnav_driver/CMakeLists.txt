cmake_minimum_required(VERSION 3.5)
project(chcnav_driver)

set(CMAKE_BUILD_TYPE Debug)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(serial REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(raw_sensor_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)


# 添加可执行文件
add_executable(chanav_node_bz src/chanav_node_bz.cpp)

# 添加依赖关系
ament_target_dependencies(chanav_node_bz
  rclcpp
  geometry_msgs
  nav_msgs
  serial
  sensor_msgs
  std_msgs
  tf2_geometry_msgs
  tf2
  raw_sensor_msgs
)

# 安装可执行文件
install(TARGETS chanav_node_bz
  DESTINATION lib/${PROJECT_NAME}
)

# 安装 launch 文件
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# 安装 include 文件
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}/
)

# 包含 ament 相关信息
ament_package()


