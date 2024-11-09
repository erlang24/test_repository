#include <memory>
#include <string>
#include <iostream>
#include <asio.hpp>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"

class UdpTransmitterNode : public rclcpp::Node {
public:
    UdpTransmitterNode()
      : Node("udp_transmitter_node"),
        remote_ip_("192.168.62.199"),      
        remote_port_(30300),
        udp_socket_(io_context_) {


        // udp_subscription_ = this->create_subscription<std_msgs::msg::String>(
        //     "/send_to_udp_topic1", 10, std::bind(&UdpTransmitterNode::udp_callback, this, std::placeholders::_1));


        udp_subscription_ = this->create_subscription<std_msgs::msg::UInt8MultiArray>(
            "/encoded_bsm_data", 10, std::bind(&UdpTransmitterNode::udp_callback, this, std::placeholders::_1));

        try {
            udp_socket_.open(asio::ip::udp::v4());
        } catch (std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Error initializing UDP socket: %s", e.what());
            return;
        }
    }

private:
    void udp_callback(const std_msgs::msg::UInt8MultiArray::SharedPtr msg) {
        send_to_obu(msg->data);
    }

    void send_to_obu(const std::vector<uint8_t> &message) {
        try {
            asio::ip::udp::endpoint remote_endpoint(asio::ip::make_address(remote_ip_), remote_port_);
            udp_socket_.send_to(asio::buffer(message), remote_endpoint);
            RCLCPP_INFO(this->get_logger(), "Sent encoded BSM data to OBU");
        } catch (std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Error sending data to OBU: %s", e.what());
        }
    }

    rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr udp_subscription_;
    std::string remote_ip_;
    unsigned short remote_port_;
    asio::io_context io_context_;
    asio::ip::udp::socket udp_socket_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<UdpTransmitterNode>());
    rclcpp::shutdown();
    return 0;
}



