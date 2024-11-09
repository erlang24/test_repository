cmake_minimum_required(VERSION 3.8)
project(position_determination)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED) 
find_package(nlohmann_json REQUIRED)
find_package(OpenSSL REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(gps_msgs REQUIRED)

add_executable(vehicle_position_node src/vehicle_position_node.cpp)
ament_target_dependencies(vehicle_position_node rclcpp geometry_msgs tf2_geometry_msgs )

add_executable(fault_alarm_receiver_node src/fault_alarm_receiver_node.cpp)
ament_target_dependencies(fault_alarm_receiver_node rclcpp std_msgs geometry_msgs tf2_geometry_msgs )

add_executable(send_fault_alarm_node src/send_fault_alarm_node.cpp)
ament_target_dependencies(send_fault_alarm_node rclcpp std_msgs)

add_executable(send_emergency_vehicles_node src/send_emergency_vehicles_node.cpp)
ament_target_dependencies(send_emergency_vehicles_node rclcpp std_msgs)

add_executable(emergency_vehicles_node src/emergency_vehicles_node.cpp)
ament_target_dependencies(emergency_vehicles_node rclcpp std_msgs geometry_msgs tf2_geometry_msgs)

add_executable(receive_handle_topic_json_node src/receive_handle_topic_json_node.cpp)
ament_target_dependencies(receive_handle_topic_json_node rclcpp std_msgs nlohmann_json OpenSSL) 

add_executable(blind_spot_warning_node src/blind_spot_warning_node.cpp)
ament_target_dependencies(blind_spot_warning_node rclcpp std_msgs sensor_msgs geometry_msgs tf2_geometry_msgs)

add_executable(intersection_collision_warning_node src/intersection_collision_warning_node.cpp)
ament_target_dependencies(intersection_collision_warning_node rclcpp std_msgs sensor_msgs tf2_geometry_msgs gps_msgs)

add_executable(green_wave_node src/green_wave_node.cpp)
ament_target_dependencies(green_wave_node rclcpp std_msgs sensor_msgs geometry_msgs nlohmann_json gps_msgs)

target_link_libraries(receive_handle_topic_json_node OpenSSL::Crypto)

install(TARGETS
  vehicle_position_node
  fault_alarm_receiver_node
  send_fault_alarm_node
  send_emergency_vehicles_node
  emergency_vehicles_node
  receive_handle_topic_json_node
  blind_spot_warning_node
  intersection_collision_warning_node
  green_wave_node
  
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
