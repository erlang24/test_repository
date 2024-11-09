#include <asio.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>

class UdpReceiverNode : public rclcpp::Node {
public:
    UdpReceiverNode()
        : Node("udpreceivernode"),
          local_ip_("192.168.62.224"),
          local_port_(30301),
          udp_socket_(io_context_) {

        udp_array_publisher_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>("/Uint8_encode", 10);

        // 尝试绑定到UDP端口，直到成功
        while (!bind_udp_socket()) {
            RCLCPP_WARN(this->get_logger(), "Waiting for UDP connection on %s:%u...", local_ip_.c_str(), local_port_);
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }

        start_receive();
        io_thread_ = std::thread([this]() { io_context_.run(); });
    }

    ~UdpReceiverNode() {
        io_context_.stop();
        if (io_thread_.joinable()) {
            io_thread_.join();
        }
    }

private:
    bool bind_udp_socket() {
        try {
            asio::ip::udp::endpoint local_endpoint(asio::ip::make_address(local_ip_), local_port_);
            udp_socket_.open(local_endpoint.protocol());
            udp_socket_.bind(local_endpoint);
            RCLCPP_INFO(this->get_logger(), "Successfully bound to UDP %s:%u", local_ip_.c_str(), local_port_);
            return true;
        } catch (const std::system_error& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to bind UDP socket: %s", e.what());
            return false;
        }
    }

    void start_receive() {
        udp_socket_.async_receive_from(
            asio::buffer(receive_buffer_), sender_endpoint_,
            [this](std::error_code ec, std::size_t bytes_received) {
                if (!ec && bytes_received > 0) {
                    std_msgs::msg::UInt8MultiArray msg;
                    msg.data.resize(bytes_received);
                    std::copy(receive_buffer_.data(), receive_buffer_.data() + bytes_received, msg.data.begin());

                    RCLCPP_INFO(this->get_logger(), "Received %zu bytes of data", bytes_received);
                    udp_array_publisher_->publish(msg);
                } else {
                    RCLCPP_ERROR(this->get_logger(), "Error receiving UDP data: %s", ec.message().c_str());
                }

                start_receive();
            });
    }

    rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr udp_array_publisher_;
    std::string local_ip_;
    unsigned short local_port_;
    asio::io_context io_context_;
    asio::ip::udp::socket udp_socket_;
    asio::ip::udp::endpoint sender_endpoint_;
    std::array<char, 8192> receive_buffer_;
    std::thread io_thread_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<UdpReceiverNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
