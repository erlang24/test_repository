// 发布紧急车辆信息

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <chrono> // 包含时间库

using namespace std::chrono_literals;

class SendEmergencyVehicles : public rclcpp::Node
{
public:
  SendEmergencyVehicles()
  : Node("send_emergency_vehicles_node")
  {
    udp_publisher_ = this->create_publisher<std_msgs::msg::String>("/vehicle_emergency", 10);
    timer_ = this->create_wall_timer(1s, std::bind(&SendEmergencyVehicles::publish_message, this));
  }

private:
  void publish_message()
  {
    auto message = std_msgs::msg::String();
    message.data = "紧急车辆"; 
    RCLCPP_INFO(this->get_logger(), "Publishing message: %s", message.data.c_str());
    udp_publisher_->publish(message);
  }

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr udp_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SendEmergencyVehicles>());
  rclcpp::shutdown();
  return 0;
}