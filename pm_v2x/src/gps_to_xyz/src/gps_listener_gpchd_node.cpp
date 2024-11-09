#include <chrono>
#include <memory>
#include <string>
#include <iomanip>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "gps_msgs/msg/gps_fix.hpp"
// #include "sensor_msgs/msg/nav_sat_fix.hpp"  // 包含 NavSatFix 消息类型
#include "geometry_msgs/msg/twist.hpp" // 包含 Twist 消息类型
#include "msg_interfaces/msg/hcinspvatzcb.hpp"
// #include "msg_interfaces/msg/string.hpp"
// #include "msg_interfaces/msg/hc_sentence.hpp"
// #include "msg_interfaces/msg/hcrawimub.hpp" 
// #include "msg_interfaces/msg/int8_array.hpp"

/*
话题名：/chcnav/devpvt
类型：msg_interfaces::msg::Hcinspvatzcb
*/

class GpchdGpsListener : public rclcpp::Node{
public:
    GpchdGpsListener() : Node("gpchd_gps_listener_node"){
        rclcpp::QoS qos_profile(10);
        qos_profile.reliable();
        subscription_ = this->create_subscription<msg_interfaces::msg::Hcinspvatzcb>(
            "/chcnav/devpvt", qos_profile,
            std::bind(&GpchdGpsListener::gpchc_topic_callback, this, std::placeholders::_1));

        gps_publisher_ = this->create_publisher<gps_msgs::msg::GPSFix>("/local_messages_b", 10);
        speed_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/local_speed_b", 10);
        // udp_publisher_ = this->create_publisher<std_msgs::msg::String>("/send_to_udp_topic1", 10);
        UI_llahs_publisher_ = this->create_publisher<gps_msgs::msg::GPSFix>("/ui_datapublisher_llahs", 10);
    }

private:
    void gpchc_topic_callback(const msg_interfaces::msg::Hcinspvatzcb::SharedPtr msg) {

        double latitude = msg->latitude;
        double longitude = msg->longitude;
        double altitude = msg->altitude;
        double altitude0 = 0.0;
        double heading = msg->heading2;
        double speed_ms = msg->speed;
        double speed_kmh = speed_ms * 3.6;

        // 创建并发布 GPSFix 消息
        auto gps_msg = gps_msgs::msg::GPSFix();
        gps_msg.latitude = latitude;
        gps_msg.longitude = longitude;
        gps_msg.altitude = altitude0;
        gps_msg.track = heading;
        gps_msg.speed = speed_kmh;
        gps_publisher_->publish(gps_msg);
        UI_llahs_publisher_->publish(gps_msg);

        // 创建并发布 Twist 消息
        auto speed_msg = geometry_msgs::msg::Twist();
        speed_msg.linear.x = speed_kmh;
        speed_publisher_->publish(speed_msg);

        // Publish data to UDP topic 需要进行编码再发送出去，这个话题不需要了？
        // auto udp_msg = std_msgs::msg::String();
        // std::ostringstream oss;
        // oss << std::fixed << std::setprecision(7);  // 设置小数点后保留7位
        // oss << "Lat:" << latitude << ",Lon:" << longitude << ",Heading:" << heading << ",Speed:" << speed_kmh;
        // udp_msg.data = oss.str();
        // RCLCPP_INFO(this->get_logger(), "Publishing to UDP topic: %s", udp_msg.data.c_str());
        // udp_publisher_->publish(udp_msg);

        // // 打印 GPS 信息
        // RCLCPP_INFO(this->get_logger(), "Week: %d", msg->week);
        // RCLCPP_INFO(this->get_logger(), "Second: %f", msg->second);
        // RCLCPP_INFO(this->get_logger(), "Latitude: %f", msg->latitude);
        // RCLCPP_INFO(this->get_logger(), "Longitude: %f", msg->longitude);
        // RCLCPP_INFO(this->get_logger(), "Altitude: %f", msg->altitude);
        // RCLCPP_INFO(this->get_logger(), "Position StdDev: [%f, %f, %f]", msg->position_stdev[0], msg->position_stdev[1], msg->position_stdev[2]);
        // RCLCPP_INFO(this->get_logger(), "Undulation: %f", msg->undulation);

        // // 打印姿态信息
        // RCLCPP_INFO(this->get_logger(), "Roll: %f", msg->roll);
        // RCLCPP_INFO(this->get_logger(), "Pitch: %f", msg->pitch);
        // RCLCPP_INFO(this->get_logger(), "Yaw: %f", msg->yaw);
        // RCLCPP_INFO(this->get_logger(), "Euler StdDev: [%f, %f, %f]", msg->euler_stdev[0], msg->euler_stdev[1], msg->euler_stdev[2]);

        // // 打印速度和航向信息
        // RCLCPP_INFO(this->get_logger(), "Speed: %f", msg->speed);
        // RCLCPP_INFO(this->get_logger(), "Heading: %f", msg->heading);
        // RCLCPP_INFO(this->get_logger(), "Heading2: %f", msg->heading2);

        // // 打印速度向量信息
        // RCLCPP_INFO(this->get_logger(), "ENU Velocity: [E: %f, N: %f, U: %f]", 
        //              msg->enu_velocity.x, msg->enu_velocity.y, msg->enu_velocity.z);
        // RCLCPP_INFO(this->get_logger(), "ENU Velocity StdDev: [%f, %f, %f]", 
        //              msg->enu_velocity_stdev[0], msg->enu_velocity_stdev[1], msg->enu_velocity_stdev[2]);

        // // 打印车辆速度和加速度信息
        // RCLCPP_INFO(this->get_logger(), "Vehicle Angular Velocity: [X: %f, Y: %f, Z: %f]", 
        //              msg->vehicle_angular_velocity.x, msg->vehicle_angular_velocity.y, msg->vehicle_angular_velocity.z);
        // RCLCPP_INFO(this->get_logger(), "Vehicle Linear Velocity: [X: %f, Y: %f, Z: %f]", 
        //              msg->vehicle_linear_velocity.x, msg->vehicle_linear_velocity.y, msg->vehicle_linear_velocity.z);
        // RCLCPP_INFO(this->get_logger(), "Vehicle Linear Acceleration: [X: %f, Y: %f, Z: %f]", 
        //              msg->vehicle_linear_acceleration.x, msg->vehicle_linear_acceleration.y, msg->vehicle_linear_acceleration.z);

        // // 打印状态信息
        // RCLCPP_INFO(this->get_logger(), "Stat: [%d, %d]", msg->stat[0], msg->stat[1]);
        // RCLCPP_INFO(this->get_logger(), "Age: %f", msg->age);
        // RCLCPP_INFO(this->get_logger(), "Number of Satellites: %d, %d", msg->ns, msg->ns2);
        // RCLCPP_INFO(this->get_logger(), "Leaps: %d", msg->leaps);

        // // 打印精度因子信息
        // RCLCPP_INFO(this->get_logger(), "HDOP: %f", msg->hdop);
        // RCLCPP_INFO(this->get_logger(), "PDOP: %f", msg->pdop);
        // RCLCPP_INFO(this->get_logger(), "VDOP: %f", msg->vdop);
        // RCLCPP_INFO(this->get_logger(), "TDOP: %f", msg->tdop);
        // RCLCPP_INFO(this->get_logger(), "GDOP: %f", msg->gdop);

        // // 打印杆臂和角度信息
        // RCLCPP_INFO(this->get_logger(), "INS to GNSS Vector: [X: %f, Y: %f, Z: %f]", 
        //              msg->ins2gnss_vector.x, msg->ins2gnss_vector.y, msg->ins2gnss_vector.z);
        // RCLCPP_INFO(this->get_logger(), "INS to Body Angle: [X: %f, Y: %f, Z: %f]", 
        //              msg->ins2body_angle.x, msg->ins2body_angle.y, msg->ins2body_angle.z);
        // RCLCPP_INFO(this->get_logger(), "GNSS to Body Vector: [X: %f, Y: %f, Z: %f]", 
        //              msg->gnss2body_vector.x, msg->gnss2body_vector.y, msg->gnss2body_vector.z);
        // RCLCPP_INFO(this->get_logger(), "GNSS to Body Angle Z: %f", msg->gnss2body_angle_z);

        // // 打印警告和传感器使用标识
        // RCLCPP_INFO(this->get_logger(), "Warning: %d", msg->warning);
        // RCLCPP_INFO(this->get_logger(), "Sensor Used: %d", msg->sensor_used);
    }

    rclcpp::Subscription<msg_interfaces::msg::Hcinspvatzcb>::SharedPtr subscription_;

    rclcpp::Publisher<gps_msgs::msg::GPSFix>::SharedPtr gps_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr speed_publisher_;
    // rclcpp::Publisher<std_msgs::msg::String>::SharedPtr udp_publisher_;
    rclcpp::Publisher<gps_msgs::msg::GPSFix>::SharedPtr UI_llahs_publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GpchdGpsListener>());
  rclcpp::shutdown();
  return 0;
}
