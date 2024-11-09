// 需要同时收到
/*
listen_gpstcp_to_udp_node这个节点功能：
1.可以成功监听gps消息（192.168.62.199/50210）
2.将监听到的GPS消息转换为经纬度、航向角和速度信息，并发布到/local_messages和/speed_messages主题上。
3.将监听到的GPS消息转换为经纬度、航向角和速度信息，通过透传（30300端口）发送给obu，UTF-8

发送的消息格式为：
Lat:36.7521,Lon:117.256,Heading:88.9,Speed:0.1852
*/




/*  !!!!!!这个程序是原始的，被拆分了 !!!!!!!! */ 




#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <iostream>
#include <asio.hpp>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

class GpsReceiverNode : public rclcpp::Node {
public:
    GpsReceiverNode()
      : Node("listen_gpstcp_to_udp_node"),
        remote_ip_("192.168.62.199"),       // 远程 IP 和端口号发送obu透传消息
        remote_port_(30300),
        source_host_("192.168.8.81"),     
        source_port_(50210),                // 监听gps端口号
        udp_socket_(io_context_) {

        gps_publisher_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("/local_messages_b", 10);
        speed_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/local_speed_b", 10);

        // 初始化用于向OBU发送数据的UDP套接字
        try {
            udp_socket_.open(asio::ip::udp::v4());
        } catch (std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Error initializing UDP socket: %s", e.what());
            return;
        }

        // 开始接收消息
        receive_messages();
    }

private:
    void publish_gps_data(double latitude, double longitude, double heading, double speed_kmh) {
        // Publish GPS data
        auto gps_msg = sensor_msgs::msg::NavSatFix();
        gps_msg.latitude = latitude;
        gps_msg.longitude = longitude;
        gps_msg.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
        gps_msg.position_covariance[0] = heading;  // 航向角存在了协方差矩阵中
        gps_msg.position_covariance[4] = 0.0;
        gps_msg.position_covariance[8] = 0.0;

        gps_msg.header.frame_id = "gps_frame";
        gps_msg.header.stamp = this->get_clock()->now();

        RCLCPP_INFO(this->get_logger(), "Publishing GPS data: Lat=%.6f, Long=%.6f, Heading=%.1f, Speed=%.1f km/h", latitude, longitude, heading, speed_kmh);
        gps_publisher_->publish(gps_msg);

        // 将速度转为 m/s 发布
        double speed_ms = speed_kmh * (1000.0 / 3600.0);
        auto speed_msg = geometry_msgs::msg::Twist();
        speed_msg.linear.x = speed_ms;
        RCLCPP_INFO(this->get_logger(), "Publishing speed: %.2f m/s", speed_ms);
        speed_publisher_->publish(speed_msg);

        // 通过UDP向OBU发送数据
        send_to_obu(latitude, longitude, heading, speed_kmh);
    }

    void send_to_obu(double latitude, double longitude, double heading, double speed) {
        try {
            // Format the message
            std::ostringstream oss;
            oss << "Lat:" << latitude << ",Lon:" << longitude << ",Heading:" << heading << ",Speed:" << speed << "\n";
            std::string message = oss.str();

            // Send the message over UDP
            asio::ip::udp::endpoint remote_endpoint(asio::ip::make_address(remote_ip_), remote_port_);
            udp_socket_.send_to(asio::buffer(message), remote_endpoint);

            RCLCPP_INFO(this->get_logger(), "Sent data to OBU: %s", message.c_str());
        } catch (std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Error sending data to OBU: %s", e.what());
        }
    }

    void receive_messages() {
        try {
            asio::io_context io_context;
            asio::ip::tcp::socket socket(io_context);
            asio::ip::tcp::endpoint endpoint(asio::ip::make_address(source_host_), source_port_);

            socket.connect(endpoint);
            RCLCPP_INFO(this->get_logger(), "Connected to %s:%d", source_host_.c_str(), source_port_);

            while (rclcpp::ok()) {
                char data[1024]; 
                asio::error_code error;
                size_t length = socket.read_some(asio::buffer(data), error);

                if (error == asio::error::eof) {
                    break; // Connection closed cleanly by peer.
                } else if (error) {
                    throw asio::system_error(error); // Some other error.
                }

                std::string data_str(data, length);
                std::istringstream iss(data_str);
                std::string line;
                while (std::getline(iss, line)) {
                    if (line.substr(0, 6) == "$GPRMC") {
                        std::vector<std::string> parts;
                        std::istringstream line_stream(line);
                        std::string part;
                        while (std::getline(line_stream, part, ',')) {
                            parts.push_back(part);
                        }

                        if (parts.size() >= 10 && parts[2] == "A") {
                            double latitude = std::stod(parts[3].substr(0, 2)) + std::stod(parts[3].substr(2)) / 60.0;
                            if (parts[4] == "S") latitude = -latitude;
                            double longitude = std::stod(parts[5].substr(0, 3)) + std::stod(parts[5].substr(3)) / 60.0;
                            if (parts[6] == "W") longitude = -longitude;
                            double speed = std::stod(parts[7]) * 1.852;  // Convert knots to km/h
                            double heading = std::stod(parts[8]);

                            publish_gps_data(latitude, longitude, heading, speed);
                        }
                    }
                }

                std::this_thread::sleep_for(10ms);
            }
        } catch (std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Error receiving data: %s", e.what());
        }
    }

    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr gps_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr speed_publisher_;
    std::string remote_ip_;
    unsigned short remote_port_;
    std::string source_host_;
    unsigned short source_port_;
    asio::io_context io_context_;
    asio::ip::udp::socket udp_socket_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GpsReceiverNode>());
    rclcpp::shutdown();
    return 0;
}