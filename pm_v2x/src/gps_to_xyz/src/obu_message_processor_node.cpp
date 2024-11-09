#include <memory>
#include <string>
#include <sstream>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "geometry_msgs/msg/twist.hpp"

// ========================================
// ==  decode_node 节点已经将此节点替代 ======
// ========================================

class ObuMessageProcessorNode : public rclcpp::Node {
public:
    ObuMessageProcessorNode()
      : Node("obu_message_processor_node") {

        udp_string_subscription_ = this->create_subscription<std_msgs::msg::String>(
            "/receive_UDPstring_messages", 10,
            std::bind(&ObuMessageProcessorNode::process_obu_message, this, std::placeholders::_1)
        );

        obu_gps_publisher_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("/other_messages_b", 10);
        obu_speed_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/other_speed_b", 10);
        fault_alarm_publisher_ = this->create_publisher<std_msgs::msg::String>("/fault_alarm", 10); // 异常车辆
        emergency_vehicle_publisher_ = this->create_publisher<std_msgs::msg::String>("/emergency_vehicle", 10); // 紧急车辆

        RCLCPP_INFO(this->get_logger(), "ObuMessageProcessorNode initialized.");
    }

private:
    void process_obu_message(const std_msgs::msg::String::SharedPtr msg) {
        std::string message = msg->data;
        RCLCPP_INFO(this->get_logger(), "Processing OBU data: %s", message.c_str());

        // 解析 OBU 消息的代码
        double latitude = 0.0, longitude = 0.0, heading = 0.0, speed = 0.0;
        std::istringstream iss(message);
        std::string token;
        bool valid_data = false;
        bool gnss_data = false;
        std::string fault_alarm_message;
        std::string emergency_vehicle_message;

        while (std::getline(iss, token, ',')) {
            if (token.find("Lat:") == 0) {
                latitude = std::stod(token.substr(4));
                gnss_data = true;
                valid_data = true;
            } else if (token.find("Lon:") == 0) {
                longitude = std::stod(token.substr(4));
            } else if (token.find("Heading:") == 0) {
                heading = std::stod(token.substr(8));
            } else if (token.find("Speed:") == 0) {
                speed = std::stod(token.substr(6));
            // } else if (token.find("Fault:") == 0) {
            //     fault_alarm_message = token.substr(6); // 获取实际的故障报警消息
            //     RCLCPP_INFO(this->get_logger(), "Detected fault alarm: %s", fault_alarm_message.c_str());

            } else if (token.find("异常车辆") == 0) {
                valid_data = true;
                fault_alarm_message = "异常车辆"; // 获取实际的故障报警消息
                RCLCPP_INFO(this->get_logger(), "Detected state: %s", fault_alarm_message.c_str());
            }
            else if (token.find("紧急车辆") == 0) {
                valid_data = true;
                emergency_vehicle_message = "紧急车辆"; // 获取实际的故障报警消息
                RCLCPP_INFO(this->get_logger(), "Detected state: %s", emergency_vehicle_message.c_str());
            }
        }

        if (valid_data) {
            publish_obu_data(latitude, longitude, heading, speed, fault_alarm_message, emergency_vehicle_message, gnss_data);
        } else {
            RCLCPP_WARN(this->get_logger(), "Received invalid OBU data, skipping publish.");
        }
    }

    void publish_obu_data(double latitude, double longitude, double heading, double speed_kmh, const std::string& fault_alarm_message, const std::string& emergency_vehicle_message, bool gnss_data) {
        // 发布 OBU GPS 数据
        // std::cout<< "gnss_data : "<<gnss_data<<std::endl;
        if(gnss_data){
        /* gps_msg publishing */
            auto gps_msg = sensor_msgs::msg::NavSatFix();
            gps_msg.latitude = latitude;
            gps_msg.longitude = longitude;
            // 在处理其他车辆的时候没有用到其他车辆的航向角，可以继续使用NavSatFix类型进行发送，可以不发送heading (但是十字路口用到了他车的heading)
            gps_msg.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
            gps_msg.position_covariance[0] = 0.0;
            gps_msg.position_covariance[4] = 0.0;
            gps_msg.position_covariance[8] = heading; // 将 heading 存储在协方差矩阵中

            gps_msg.header.frame_id = "obu_gps_frame";
            gps_msg.header.stamp = this->get_clock()->now();

            RCLCPP_INFO(this->get_logger(), "Publishing OBU GPS data: Lat=%.6f, Long=%.6f, Heading=%.1f", latitude, longitude, heading);
            obu_gps_publisher_->publish(gps_msg); 
        /* speed publishing -- 将速度转换为 m/s 并发布 */
            double speed_ms = speed_kmh * (1000.0 / 3600.0);
            auto speed_msg = geometry_msgs::msg::Twist();
            speed_msg.linear.x = speed_ms;
            RCLCPP_INFO(this->get_logger(), "Publishing OBU speed: %.2f m/s", speed_ms);
            obu_speed_publisher_->publish(speed_msg);
            
        }
        else{
            RCLCPP_INFO(this->get_logger(), "No GNSS and Speed data received.");
        }

        // 如果有故障报警消息，则发布
        if (!fault_alarm_message.empty()) {
            auto fault_alarm_msg = std_msgs::msg::String();
            fault_alarm_msg.data = fault_alarm_message;
            RCLCPP_INFO(this->get_logger(), "Publishing fault alarm message: %s", fault_alarm_msg.data.c_str());
            fault_alarm_publisher_->publish(fault_alarm_msg);
        }
        else {
            RCLCPP_INFO(this->get_logger(), "No fault alarm message received.");
        }

        // 如果有紧急车辆消息，则发布
        if (!emergency_vehicle_message.empty()) {
            auto emergency_vehicle_msg = std_msgs::msg::String();
            emergency_vehicle_msg.data = emergency_vehicle_message;
            RCLCPP_INFO(this->get_logger(), "Publishing emergency vehicle message: %s", emergency_vehicle_msg.data.c_str());
            emergency_vehicle_publisher_->publish(emergency_vehicle_msg);
        }
        else {
            RCLCPP_INFO(this->get_logger(), "No emergency vehicle message received.");
        }
    }

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr udp_string_subscription_;
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr obu_gps_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr obu_speed_publisher_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr fault_alarm_publisher_; // 发布故障报警消息
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr emergency_vehicle_publisher_; // 发布紧急车辆消息
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ObuMessageProcessorNode>());
    rclcpp::shutdown();
    return 0;
}
