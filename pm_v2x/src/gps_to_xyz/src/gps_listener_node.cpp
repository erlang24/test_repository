#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <iostream>
#include <iomanip> 
#include <asio.hpp>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp" // 包含 NavSatFix 消息类型
#include "geometry_msgs/msg/twist.hpp" // 包含 Twist 消息类型
#include "std_msgs/msg/string.hpp"
#include "gps_msgs/msg/gps_fix.hpp"

using namespace std::chrono_literals; //  简化时间表示 std::chrono::seconds(2) 可以替换为 2s

class GpsListenerNode : public rclcpp::Node {
public:
    GpsListenerNode()
    : Node("gps_listener_node"),

      source_host_("192.168.8.84"),
    //   source_host_("192.168.62.199"),
      source_port_(50210) { 

        // 创建发布者，发布 GPSFix 和 Twist 消息
        gps_publisher_ = this->create_publisher<gps_msgs::msg::GPSFix>("/local_messages_b", 10);
        speed_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/local_speed_b", 10);
        udp_publisher_ = this->create_publisher<std_msgs::msg::String>("/send_to_udp_topic1", 10);
        UI_llahs_publisher_ = this->create_publisher<gps_msgs::msg::GPSFix>("/ui_datapublisher_llahs", 10);
        
        // 开始接收消息
        receive_messages();
    }

private:
    void publish_gps_data(double latitude, double longitude, double heading, double speed_kmh) {

        // 将度分秒格式的经纬度转换为十进制格式
        double decimalLatitude;
        double decimalLongitude;
        RCLCPP_INFO(this->get_logger(), "latitude: %lf, longitude: %lf", latitude, longitude);
        decimalLatitude = std::floor(latitude / 100) + (latitude - std::floor(latitude / 100) * 100) / 60;
        decimalLongitude = std::floor(longitude / 100) + (longitude - std::floor(longitude / 100) * 100) / 60;

        // 处理不同半球突变
        if (latitude < 3600) {
            decimalLatitude = -decimalLatitude;
            decimalLatitude = 36-(36.666667+decimalLatitude);
        }
        if (longitude < 11700){
            decimalLongitude = -decimalLongitude;
            decimalLongitude = 117-(117.666667+decimalLongitude);
        }

        RCLCPP_INFO(this->get_logger(), "decimalLatitude: %lf, decimalLongitude: %lf", decimalLatitude, decimalLongitude);


        /*============================  不需要进行gps的度分转换  ===============================*/

        // 发布 GPSFix(只需经纬度航向角) 到 gps_to_xyz 消息
        auto gps_msg = gps_msgs::msg::GPSFix();
        gps_msg.latitude = decimalLatitude; // 使用转换后的十进制经纬度
        gps_msg.longitude = decimalLongitude; // 使用转换后的十进制经纬度
        gps_msg.track = heading;
        gps_msg.header.frame_id = "gps_frame";
        gps_msg.header.stamp = this->get_clock()->now();
        RCLCPP_INFO(this->get_logger(), "Publishing GPS data: Lat=%.6f, Long=%.6f, Heading=%.1f, Speed=%.1f km/h", decimalLatitude, decimalLongitude, heading, speed_kmh);
        gps_publisher_->publish(gps_msg);


        // 发布 Twist 消息
        double speed_ms = speed_kmh * (1000.0 / 3600.0); // 将速度转换为 m/s
        auto speed_msg = geometry_msgs::msg::Twist();
        speed_msg.linear.x = speed_ms;
        RCLCPP_INFO(this->get_logger(), "Publishing speed: %.2f m/s", speed_ms);
        speed_publisher_->publish(speed_msg);

        // Publish data to UDP topic
        auto udp_msg = std_msgs::msg::String();
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(6);  // 设置小数点后保留6位
        oss << "Lat:" << decimalLatitude << ",Lon:" << decimalLongitude << ",Heading:" << heading << ",Speed:" << speed_kmh;
        udp_msg.data = oss.str();
        RCLCPP_INFO(this->get_logger(), "Publishing to UDP topic: %s", udp_msg.data.c_str());
        udp_publisher_->publish(udp_msg);

        // 发布到UI的 GPSFix 消息
        auto ui_gpsfix_msg = gps_msgs::msg::GPSFix();
        ui_gpsfix_msg.latitude = decimalLatitude;  // 使用转换后的十进制经纬度
        ui_gpsfix_msg.longitude = decimalLongitude;  // 使用转换后的十进制经纬度
        ui_gpsfix_msg.altitude = 0.0;  // 高度固定为0
        ui_gpsfix_msg.track = heading;  // 使用航向角
        ui_gpsfix_msg.speed = speed_kmh;  // 使用计算后的速度

        ui_gpsfix_msg.header.frame_id = "ui_gps_frame";
        ui_gpsfix_msg.header.stamp = this->get_clock()->now();

        RCLCPP_INFO(this->get_logger(), "Publishing UI GPSFix data: Lat=%.6f, Long=%.6f, Alt=%.1f, Heading=%.1f, Speed=%.2f m/s",
                    ui_gpsfix_msg.latitude, ui_gpsfix_msg.longitude, ui_gpsfix_msg.altitude, ui_gpsfix_msg.track, ui_gpsfix_msg.speed);
        UI_llahs_publisher_->publish(ui_gpsfix_msg);
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
                RCLCPP_INFO(this->get_logger(), "Received raw data: %s", std::string(data, length).c_str()); // 原始数据

                if (error == asio::error::eof) {
                    break; // Connection closed cleanly by peer.
                } else if (error) {
                    throw asio::system_error(error); // Some other error.
                }

                std::string data_str(data, length);
                std::istringstream iss(data_str);
                std::string line;
                while (std::getline(iss, line)) {
                    RCLCPP_INFO(this->get_logger(), "NMEA 0183 data: %s", line.c_str());  // 打印解析后的NMEA 0183数据
                    if (line.substr(0, 6) == "$GPRMC") {
                        std::vector<std::string> parts;
                        std::istringstream line_stream(line);
                        std::string part;
                        while (std::getline(line_stream, part, ',')) {
                            parts.push_back(part);
                        }
                        if (parts.size() >= 10 && parts[2] == "A") {
                            try {
                                // 现在只会是 N 和 E ，不一定是简单取负值
                                // Extract latitude
                                double latitude = std::stod(parts[3]); // 直接使用 std::stod
                                if (parts[4] == "S") latitude = -latitude;

                                // Extract longitude
                                double longitude = std::stod(parts[5]); // 直接使用 std::stod
                                if (parts[6] == "W") longitude = -longitude;

                                // Extract speed and heading
                                double speed = std::stod(parts[7]) * 1.852; // Convert knots to km/h
                                double heading = std::stod(parts[8]);

                                publish_gps_data(latitude, longitude, heading, speed);
                            } catch (const std::exception& e) {
                                RCLCPP_ERROR(this->get_logger(), "Error parsing NMEA 0183 data: %s", e.what());
                            }
                        }
                    }
                }

                std::this_thread::sleep_for(10ms);
            }
        } catch (std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Error receiving data: %s", e.what());
        }
    }

    rclcpp::Publisher<gps_msgs::msg::GPSFix>::SharedPtr gps_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr speed_publisher_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr udp_publisher_;
    rclcpp::Publisher<gps_msgs::msg::GPSFix>::SharedPtr UI_llahs_publisher_;

    std::string source_host_;
    unsigned short source_port_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GpsListenerNode>());
    rclcpp::shutdown();
    return 0;
}

// carla经纬度信息
// [INFO] [1722230064.998839832] [gps_listener_node]: Received raw data: $GPRMC,,A,-0.02515281938855196,N,-0.0664076174398905,E,0.0,-0.000579833984375120724,,,A*5D
// [INFO] [1722230064.998892471] [gps_listener_node]: NMEA 0183 data: $GPRMC,,A,-0.02515281938855196,N,-0.0664076174398905,E,0.0,-0.000579833984375120724,,,A*5D
// [INFO] [1722230064.998907251] [gps_listener_node]: Publishing GPS data: Lat=0.666247, Long=0.665560, Heading=-0.0, Speed=0.0 km/h

// obu经纬度信息
// [INFO] [1722230150.943899778] [gps_listener_node]: NMEA 0183 data: $GPRMC,051550.91,A,3645.130274,N,11715.347537,E,0.0,267.4,290724,,,A*5A
// [INFO] [1722230150.943958775] [gps_listener_node]: Publishing GPS data: Lat=36.752171, Long=117.255792, Heading=267.4, Speed=0.0 km/h
// [INFO] [1722230150.944020164] [gps_listener_node]: Publishing speed: 0.00 m/s
